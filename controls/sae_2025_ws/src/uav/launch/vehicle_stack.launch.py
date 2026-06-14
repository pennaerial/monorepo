#!/usr/bin/env python3
import json
import os
import re
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from vehicle_core.runtime.mission_spec import MissionSpec, mission_path_for_name
from vehicle_core.runtime.vision_loader import load_vision_class
from uav.utils import (
    camel_to_snake,
    find_folder_with_heuristic,
    get_airframe_details,
    vehicle_camera_map,
    vehicle_id_dict,
)


def _load_vehicle_config(context) -> dict:
    raw = LaunchConfiguration("vehicle_json").perform(context).strip()
    if not raw:
        raise ValueError("vehicle_stack.launch.py requires a non-empty vehicle_json.")
    config = json.loads(raw)
    if not isinstance(config, dict):
        raise ValueError("vehicle_json must decode to a JSON object.")
    return config


def _vehicle_namespace_for(vehicle_name: str) -> str:
    clean_name = str(vehicle_name).strip().strip("/")
    if not clean_name:
        raise ValueError("Vehicle stack requires a non-empty vehicle_name.")
    return clean_name


def _runtime_executable_for(mission_spec: MissionSpec) -> str:
    if mission_spec.is_uav:
        return "uav_mission"
    if mission_spec.is_payload:
        return "payload_mission"
    raise ValueError(f"Unsupported mission target '{mission_spec.target}'.")


def _resolve_mission_path(config: dict) -> str:
    mission_path = str(config.get("mission_path", "")).strip()
    if mission_path:
        return mission_path
    mission_name = str(config.get("mission_name", "")).strip()
    if mission_name:
        return mission_path_for_name(mission_name)
    raise ValueError("Vehicle stack requires either mission_name or mission_path.")


def _resolve_bool(config: dict, key: str, default: bool) -> bool:
    value = config.get(key, default)
    if isinstance(value, bool):
        return value
    if value is None:
        return bool(default)
    if isinstance(value, str):
        normalized = value.strip().lower()
        if normalized in {"1", "true", "yes", "on"}:
            return True
        if normalized in {"0", "false", "no", "off"}:
            return False
    raise ValueError(f"Vehicle stack expected boolean '{key}', received {value!r}.")


def _warn_logger(logger, message: str) -> None:
    if logger is None:
        return
    warn = getattr(logger, "warning", None) or getattr(logger, "warn", None)
    if callable(warn):
        warn(message)


def _resolve_force_camera(config: dict, *, logger=None) -> bool:
    has_force_camera = "force_camera" in config
    has_use_camera = "use_camera" in config

    if has_force_camera:
        force_camera = _resolve_bool(config, "force_camera", False)
        if has_use_camera:
            _warn_logger(
                logger,
                "Vehicle stack field 'use_camera' is deprecated and ignored because "
                "'force_camera' is also set.",
            )
        return force_camera

    if has_use_camera:
        _warn_logger(
            logger,
            "Vehicle stack field 'use_camera' is deprecated; use 'force_camera' instead.",
        )
        return _resolve_bool(config, "use_camera", False)

    return False


def _resolve_camera_input_transport(
    *,
    mission_spec: MissionSpec,
    sim: bool,
    configured_transport: str,
    rotate_degrees: float,
    preprocess_hook: str,
    logger=None,
) -> str:
    transport = str(configured_transport).strip() or ("raw" if sim else "compressed")
    if not mission_spec.is_payload or sim:
        return transport

    preprocess_active = abs(float(rotate_degrees)) > 1e-9 or bool(
        str(preprocess_hook).strip()
    )
    if transport.lower() != "compressed" or not preprocess_active:
        return transport

    if logger is not None:
        logger.info(
            "Payload camera preprocessing is enabled; using raw camera input "
            "instead of compressed to avoid JPEG decode/re-encode latency."
        )
    return "raw"


def _resolve_airframe_id(config: dict) -> int:
    if config.get("px4_airframe_id") is not None:
        return int(config["px4_airframe_id"])

    airframe_value = config.get("airframe", "quadcopter")
    try:
        return int(airframe_value)
    except (TypeError, ValueError):
        try:
            return vehicle_id_dict[str(airframe_value)]
        except KeyError as exc:
            raise ValueError(f"Unknown airframe name: {airframe_value}") from exc


def _resolve_uav_airframe(
    config: dict, px4_path: str
) -> tuple[AirframeClass, int, str]:
    airframe_id = _resolve_airframe_id(config)
    vehicle_class, model_name = get_airframe_details(px4_path, airframe_id)
    if not isinstance(vehicle_class, AirframeClass):
        raise ValueError(f"Unexpected airframe class: {vehicle_class!r}.")
    model = str(config.get("model", "")).strip() or model_name
    return vehicle_class, int(airframe_id), model


def _camera_contract_for(
    mission_spec: MissionSpec,
    *,
    vehicle_name: str,
    input_transport: str | None = None,
    rotate_degrees: float | None = None,
    preprocess_hook: str | None = None,
    camera_info_url: str | None = None,
) -> dict[str, object]:
    return {
        "vehicle_name": vehicle_name,
        "camera_namespace": _vehicle_namespace_for(vehicle_name),
        "image_topic": "camera",
        "camera_info_topic": "camera_info",
        "camera_service_name": "camera_data",
        "input_transport": input_transport or "",
        "input_raw_topic": "camera_source",
        "input_compressed_topic": "camera_source/compressed",
        "input_camera_info_topic": "camera_info_source",
        "rotate_degrees": 0.0 if rotate_degrees is None else rotate_degrees,
        "preprocess_hook": "" if preprocess_hook is None else preprocess_hook,
        "camera_info_url": "" if camera_info_url is None else camera_info_url,
        "mission_target": mission_spec.target,
    }


def _payload_camera_info_url_for(
    vehicle_name: str, *, camera_calibration_file: str = ""
) -> str:
    calibration_dir = (
        Path(get_package_share_directory("uav")) / "config" / "camera_calibrations"
    )
    requested_file = str(camera_calibration_file).strip()
    if requested_file:
        if Path(requested_file).name != requested_file:
            raise ValueError(
                "camera_calibration_file must be a filename inside the packaged "
                "uav camera_calibrations directory."
            )
        candidate = calibration_dir / requested_file
        if candidate.exists():
            return f"file://{candidate}"
        raise ValueError(
            f"Vehicle '{vehicle_name}' requested camera calibration file "
            f"'{requested_file}', but no packaged calibration exists at '{candidate}'."
        )

    m = re.search(r"_(\d+)$", vehicle_name)
    candidates = []
    if m:
        candidates.append(calibration_dir / f"camera_info_{m.group(1)}.yaml")
    candidates.append(calibration_dir / "payload_0_camera_info.yaml")
    for candidate in candidates:
        if candidate.exists():
            return f"file://{candidate}"
    return ""


def _sim_camera_bridge_actions(
    *,
    world_name: str,
    vehicle_name: str,
    sim_entity_name: str,
    mission_target: str,
) -> list:
    gz_image_topic = f"/world/{world_name}/model/{sim_entity_name}/link/camera_link/sensor/camera/image"
    gz_camera_info_topic = f"/world/{world_name}/model/{sim_entity_name}/link/camera_link/sensor/camera/camera_info"
    image_target = "camera_source" if mission_target == "payload" else "camera"
    camera_info_target = (
        "camera_info_source" if mission_target == "payload" else "camera_info"
    )
    return [
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[f"{gz_image_topic}@sensor_msgs/msg/Image[gz.msgs.Image"],
            remappings=[(gz_image_topic, image_target)],
            output="screen",
            name=f"{vehicle_name}_camera_bridge",
            namespace=_vehicle_namespace_for(vehicle_name),
        ),
        Node(
            package="ros_gz_bridge",
            executable="parameter_bridge",
            arguments=[
                f"{gz_camera_info_topic}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
            ],
            remappings=[(gz_camera_info_topic, camera_info_target)],
            output="screen",
            name=f"{vehicle_name}_camera_info_bridge",
            namespace=_vehicle_namespace_for(vehicle_name),
        ),
    ]


def _build_camera_actions(
    *,
    mission_spec: MissionSpec,
    camera_contract: dict[str, object],
    vision_nodes: list[str],
    sim: bool,
    sim_world_name: str,
    sim_entity_name: str,
    vision_debug: bool,
    debug_vision_node: bool,
    save_vision_milliseconds: int,
) -> list:
    save_vision = save_vision_milliseconds > 0
    actions = []
    vehicle_name = str(camera_contract["vehicle_name"])

    if sim:
        actions.extend(
            _sim_camera_bridge_actions(
                world_name=sim_world_name,
                vehicle_name=vehicle_name,
                sim_entity_name=sim_entity_name,
                mission_target=str(camera_contract["mission_target"]),
            )
        )
    else:
        v4l2_cmd = [
            "ros2",
            "run",
            "v4l2_camera",
            "v4l2_camera_node",
            "--ros-args",
            "-r",
            f"__ns:=/{camera_contract['vehicle_name']}",
            "-p",
            "image_size:=[640,480]",
            "-p",
            "output_encoding:=bgr8",
        ]
        if mission_spec.is_payload and str(camera_contract["camera_info_url"]).strip():
            v4l2_cmd.extend(
                ["-p", f"camera_info_url:={camera_contract['camera_info_url']}"]
            )
        v4l2_cmd.extend(
            [
                "--remap",
                (
                    f"image_raw:={camera_contract['input_raw_topic']}"
                    if mission_spec.is_payload
                    else f"image_raw:={camera_contract['image_topic']}"
                ),
                "--remap",
                (
                    f"image_raw/compressed:={camera_contract['input_compressed_topic']}"
                    if mission_spec.is_payload
                    else f"image_raw/compressed:={camera_contract['image_topic']}/compressed"
                ),
                "--remap",
                (
                    f"camera_info:={camera_contract['input_camera_info_topic']}"
                    if mission_spec.is_payload
                    else f"camera_info:={camera_contract['camera_info_topic']}"
                ),
            ]
        )
        actions.append(
            ExecuteProcess(
                cmd=v4l2_cmd,
                output="screen",
                name=f"{camera_contract['vehicle_name']}_camera_device",
            )
        )

    camera_parameters = {
        "vehicle_name": camera_contract["vehicle_name"],
        "image_topic": "",
        "camera_info_topic": "",
        "camera_service_name": "",
        "display": False,
        "debug": vision_debug,
        "save_vision_milliseconds": save_vision_milliseconds,
    }
    if mission_spec.is_payload:
        camera_parameters.update(
            {
                "input_transport": camera_contract["input_transport"],
                "input_raw_topic": camera_contract["input_raw_topic"],
                "input_compressed_topic": camera_contract["input_compressed_topic"],
                "input_camera_info_topic": camera_contract["input_camera_info_topic"],
                "rotate_degrees": camera_contract["rotate_degrees"],
                "preprocess_hook": camera_contract["preprocess_hook"],
            }
        )

    actions.append(
        Node(
            package="uav",
            executable="camera",
            namespace=vehicle_name,
            name=f"{vehicle_name}_camera",
            output="screen",
            parameters=[camera_parameters],
        )
    )

    preferred_image_transport = (
        "compressed"
        if mission_spec.is_payload
        and str(camera_contract["input_transport"]).strip().lower() == "compressed"
        else "raw"
    )

    for vision_node in vision_nodes:
        vision_class = load_vision_class(vision_node)
        executable = camel_to_snake(vision_class.__name__)
        actions.append(
            Node(
                package="uav",
                executable=executable,
                namespace=vehicle_name,
                name=f"{vehicle_name}_{executable}",
                output="screen",
                parameters=[
                    {
                        "vehicle_name": camera_contract["vehicle_name"],
                        "camera_service_name": "",
                        "use_camera_service": vision_class.__name__
                        != "PayloadAprilTagNode",
                        "preferred_image_transport": preferred_image_transport,
                        "debug": debug_vision_node,
                        "sim": sim,
                        "save_vision": save_vision,
                        "enable_failsafe_service": mission_spec.is_uav,
                        "failsafe_service_name": "mode_manager/failsafe"
                        if mission_spec.is_uav
                        else "",
                    }
                ],
            )
        )

    return actions


def _build_runtime_parameters(
    *,
    mission_path: str,
    mission_spec: MissionSpec,
    vehicle_name: str,
    auto_launch: bool,
    debug: bool,
    vision_debug: bool,
    servo_only: bool,
    vehicle_class_name: str | None,
    camera_mount_offsets: list[float],
) -> dict:
    parameters = {"mode_map": mission_path, "auto_launch": bool(auto_launch)}
    if mission_spec.is_payload:
        parameters["vehicle_name"] = vehicle_name
        parameters["vision_debug"] = bool(vision_debug)
        return parameters

    if vehicle_class_name is None:
        raise ValueError("UAV missions require a vehicle class.")
    if len(camera_mount_offsets) != 3:
        raise ValueError(
            f"camera_mount_offsets must have exactly 3 values. Received: {camera_mount_offsets}"
        )

    parameters.update(
        {
            "vehicle_name": vehicle_name,
            "debug": bool(debug),
            "servo_only": bool(servo_only),
            "vehicle_class": vehicle_class_name,
            "camera_mount_offsets": list(camera_mount_offsets),
        }
    )
    return parameters


def _payload_launch_action(*, vehicle_name: str, controller: str, sim_entity_name: str):
    launch_arguments = {"vehicle_name": vehicle_name}
    if controller:
        launch_arguments["controller"] = controller
    if sim_entity_name and sim_entity_name != vehicle_name:
        launch_arguments["sim_entity_name"] = sim_entity_name

    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("payload_controller"),
                "launch",
                "payload_controller.launch.py",
            )
        ),
        launch_arguments=launch_arguments.items(),
    )


def _udp_bridge_action(
    *,
    vehicle_name: str,
    udp_port: int,
    udp_all_ports: list,
    udp_topics: list,
    udp_broadcast_ip: str,
    udp_peer_ttl: float,
) -> Node:
    return Node(
        package="udp_bridge",
        executable="bridge_node",
        name="udp_bridge",
        namespace=vehicle_name,
        parameters=[
            {
                "my_port": udp_port,
                "all_ports": udp_all_ports,
                "broadcast_ip": udp_broadcast_ip,
                "topics": udp_topics,
                "peer_ttl": udp_peer_ttl,
            }
        ],
        output="screen",
    )


def _middleware_action(*, sim: bool, config: dict):
    if sim:
        port = int(config.get("middleware_port", 8888))
        cmd = ["MicroXRCEAgent", "udp4", "-p", str(port)]
    else:
        cmd = [
            "MicroXRCEAgent",
            "serial",
            "--dev",
            str(config.get("middleware_device", "/dev/serial0")),
            "-b",
            str(config.get("middleware_baud", "921600")),
        ]
    return ExecuteProcess(
        cmd=cmd,
        output="screen",
        name=f"{config['vehicle_name']}_middleware",
    )


def _px4_sitl_action(
    *,
    px4_path: str,
    world_name: str,
    sim_entity_name: str,
    autostart: int,
    px4_instance: int,
    vehicle_name: str,
):
    env_exports = [
        f"PX4_GZ_MODEL_NAME={sim_entity_name}",
        f"PX4_GZ_WORLD={world_name}",
        "PX4_GZ_STANDALONE=1",
        f"PX4_SYS_AUTOSTART={autostart}",
        f"PX4_UXRCE_DDS_NS={_vehicle_namespace_for(vehicle_name)}",
    ]

    cmd = [
        "bash",
        "-c",
        " ".join(
            [
                (
                    "until gz topic -l | grep -q "
                    f"'^/world/{world_name}/model/{sim_entity_name}/link/base_link/"
                    "sensor/imu_sensor/imu$'; "
                    "do sleep 0.2; done;"
                ),
                *env_exports,
                f"./build/px4_sitl_default/bin/px4 -i {int(px4_instance)}",
            ]
        ),
    ]
    return ExecuteProcess(
        cmd=cmd,
        cwd=px4_path,
        output="screen",
        name=f"{vehicle_name}_px4_sitl",
    )


def launch_setup(context, *args, **kwargs):
    logger = get_logger("vehicle_stack.launch")
    config = _load_vehicle_config(context)

    mission_path = _resolve_mission_path(config)
    mission_spec = MissionSpec.load(mission_path)
    runtime_executable = _runtime_executable_for(mission_spec)

    vehicle_name = str(config.get("vehicle_name", "")).strip()
    if not vehicle_name:
        raise ValueError("Vehicle stack requires a non-empty vehicle_name.")
    declared_kind = str(config.get("kind", "")).strip()
    if declared_kind and declared_kind != mission_spec.target:
        raise ValueError(
            f"Vehicle '{vehicle_name}' declared kind '{declared_kind}' "
            f"but mission target is '{mission_spec.target}'."
        )

    sim = _resolve_bool(config, "sim", False)
    auto_launch = _resolve_bool(config, "auto_launch", sim)
    debug = _resolve_bool(config, "debug", False)
    vision_debug = _resolve_bool(config, "vision_debug", False)
    debug_vision_node = _resolve_bool(config, "debug_vision_node", False)
    servo_only = _resolve_bool(config, "servo_only", False)
    launch_middleware = _resolve_bool(config, "launch_middleware", mission_spec.is_uav)
    launch_px4_sitl = _resolve_bool(
        config, "launch_px4_sitl", sim and mission_spec.is_uav
    )
    launch_payload_backend = _resolve_bool(
        config, "launch_payload_backend", mission_spec.is_payload
    )

    sim_entity_name = str(config.get("sim_entity_name", "")).strip() or vehicle_name
    sim_world_name = str(config.get("sim_world_name", "")).strip()
    save_vision_milliseconds = int(config.get("save_vision_milliseconds", 0))
    camera_rotate_degrees = float(
        config.get("camera_rotate_degrees", 0.0 if sim else 180.0)
    )
    camera_calibration_file = str(config.get("camera_calibration_file", "")).strip()
    camera_preprocess_hook = str(config.get("camera_preprocess_hook", "")).strip()
    camera_input_transport = _resolve_camera_input_transport(
        mission_spec=mission_spec,
        sim=sim,
        configured_transport=str(
            config.get("camera_input_transport", "raw" if sim else "compressed")
        ),
        rotate_degrees=camera_rotate_degrees,
        preprocess_hook=camera_preprocess_hook,
        logger=logger,
    )

    requires_camera = bool(
        getattr(mission_spec, "requires_camera", False)
    ) or _resolve_force_camera(config, logger=logger)

    px4_path = ""
    vehicle_class: AirframeClass | None = None
    autostart: int | None = None
    model = str(config.get("model", "")).strip()
    if mission_spec.is_uav:
        px4_path = find_folder_with_heuristic(
            "PX4-Autopilot", os.path.expanduser(str(config.get("px4_path", "")))
        )
        if not px4_path:
            raise ValueError("UAV vehicle stacks require a valid px4_path.")
        vehicle_class, autostart, model = _resolve_uav_airframe(config, px4_path)
        if requires_camera and model and not vehicle_camera_map.get(model, False):
            raise ValueError(
                f"The selected airframe/model '{model}' does not have a camera sensor configured."
            )
    if launch_px4_sitl and (not sim or not sim_world_name or autostart is None):
        raise ValueError(
            f"Vehicle '{vehicle_name}' cannot launch PX4 SITL "
            "without sim_world_name and a valid UAV airframe."
        )

    camera_mount_offsets = list(config.get("camera_mount_offsets", [0.0, 0.0, 0.0]))
    camera_contract = (
        _camera_contract_for(
            mission_spec,
            vehicle_name=vehicle_name,
            input_transport=camera_input_transport,
            rotate_degrees=camera_rotate_degrees,
            preprocess_hook=camera_preprocess_hook,
            camera_info_url=(
                _payload_camera_info_url_for(
                    vehicle_name,
                    camera_calibration_file=camera_calibration_file,
                )
                if mission_spec.is_payload and not sim
                else ""
            ),
        )
        if requires_camera
        else None
    )
    if (
        camera_contract is not None
        and mission_spec.is_payload
        and not sim
        and not str(camera_contract["camera_info_url"]).strip()
    ):
        logger.warning(
            "No packaged payload camera calibration file was found; "
            "v4l2_camera will fall back to its default camera_info behavior."
        )

    camera_actions = []
    if requires_camera:
        if camera_contract is None:
            raise ValueError(f"Vehicle '{vehicle_name}' requires a camera contract.")
        camera_actions = _build_camera_actions(
            mission_spec=mission_spec,
            camera_contract=camera_contract,
            vision_nodes=list(mission_spec.vision_nodes),
            sim=sim,
            sim_world_name=sim_world_name,
            sim_entity_name=sim_entity_name,
            vision_debug=vision_debug,
            debug_vision_node=debug_vision_node,
            save_vision_milliseconds=save_vision_milliseconds,
        )

    mission_node = Node(
        package="uav",
        executable=runtime_executable,
        namespace=vehicle_name,
        output="screen",
        parameters=[
            _build_runtime_parameters(
                mission_path=mission_path,
                mission_spec=mission_spec,
                vehicle_name=vehicle_name,
                auto_launch=auto_launch,
                debug=debug,
                vision_debug=vision_debug,
                servo_only=servo_only,
                vehicle_class_name=(
                    vehicle_class.name if vehicle_class is not None else None
                ),
                camera_mount_offsets=camera_mount_offsets,
            )
        ],
    )

    raw_udp_port = config.get(
        "udp_port"
    )  # None = auto-compute, 0 = disabled, >0 = explicit
    if raw_udp_port is None:
        m = re.search(r"_(\d+)$", vehicle_name)
        udp_port = (5000 + int(m.group(1))) if m else 0
    else:
        udp_port = int(raw_udp_port)
    udp_all_ports = list(config.get("udp_all_ports") or [5000, 5001, 5002, 5003])
    udp_topics = list(
        config.get("udp_topics")
        or ["heartbeat:payload_interfaces/msg/PayloadHeartbeat"]
    )
    udp_broadcast_ip = str(config.get("udp_broadcast_ip") or "10.42.0.255").strip()
    udp_peer_ttl = float(config.get("udp_peer_ttl") or 3.0)

    actions = []
    if mission_spec.is_payload and launch_payload_backend:
        payload_controller = "SimController" if sim else "GPIOController"
        actions.append(
            _payload_launch_action(
                vehicle_name=vehicle_name,
                controller=payload_controller,
                sim_entity_name=sim_entity_name,
            )
        )

    actions.extend(camera_actions)

    if mission_spec.is_payload and not sim and udp_port:
        logger.info(
            f"Launching UDP bridge for '{vehicle_name}' on port {udp_port} "
            f"(peers: {[p for p in udp_all_ports if p != udp_port]})."
        )
        actions.append(
            _udp_bridge_action(
                vehicle_name=vehicle_name,
                udp_port=udp_port,
                udp_all_ports=udp_all_ports,
                udp_topics=udp_topics,
                udp_broadcast_ip=udp_broadcast_ip,
                udp_peer_ttl=udp_peer_ttl,
            )
        )

    if mission_spec.is_uav and launch_middleware:
        actions.append(_middleware_action(sim=sim, config=config))

    if mission_spec.is_uav and launch_px4_sitl:
        if autostart is None:
            raise ValueError(
                f"Vehicle '{vehicle_name}' cannot launch PX4 SITL without a valid UAV airframe."
            )
        actions.append(
            _px4_sitl_action(
                px4_path=px4_path,
                world_name=sim_world_name,
                sim_entity_name=sim_entity_name,
                autostart=autostart,
                px4_instance=int(config.get("px4_instance", 0)),
                vehicle_name=vehicle_name,
            )
        )

    actions.append(mission_node)
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("vehicle_json", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
