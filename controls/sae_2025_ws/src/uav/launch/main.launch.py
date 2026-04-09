#!/usr/bin/env python3
import os
from pathlib import Path

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.logging import get_logger
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav.runtime.mission_spec import MissionSpec, mission_path_for_name
from uav.utils import (
    camel_to_snake,
    find_folder_with_heuristic,
    get_airframe_details,
    vehicle_camera_map,
    vehicle_id_dict,
)


def _load_launch_parameters(context) -> dict:
    params_path_override = LaunchConfiguration("params_file").perform(context).strip()
    if params_path_override:
        params_path = Path(os.path.expanduser(params_path_override)).resolve()
    else:
        params_path = Path(__file__).resolve().with_name("launch_params.yaml")
    with params_path.open("r", encoding="utf-8") as params_file:
        return yaml.safe_load(params_file) or {}


def _yaml_or_launch_string(context, name: str, config_value) -> str:
    override = LaunchConfiguration(name).perform(context).strip()
    if override:
        return override
    return "" if config_value is None else str(config_value).strip()


def _yaml_bool_value(config_value, *, name: str, default: bool) -> bool:
    if config_value is None:
        return bool(default)
    if isinstance(config_value, bool):
        return config_value
    raise ValueError(
        f"Launch parameter '{name}' must be a YAML boolean, received {config_value!r}."
    )


def _runtime_executable_for(mission_spec: MissionSpec) -> str:
    if mission_spec.is_uav:
        return "uav_mission"
    if mission_spec.is_payload:
        return "payload_mission"
    raise ValueError(f"Unsupported mission target '{mission_spec.target}'.")


def _sim_requires_px4_sitl(mission_spec: MissionSpec, *, sim: bool) -> bool:
    return bool(sim and mission_spec.is_uav)


def _gz_entity_name_for_model(model: str) -> str:
    sim_model_name = model[3:] if model.startswith("gz_") else model
    return f"{sim_model_name}_0"


def _camera_contract_for(
    mission_spec: MissionSpec,
    payload_name: str,
    *,
    input_transport: str | None = None,
    rotate_degrees: float | None = None,
    preprocess_hook: str | None = None,
    camera_info_url: str | None = None,
) -> dict[str, object]:
    if mission_spec.is_uav:
        vehicle_name = "uav"
        namespace = "/uav"
    else:
        if not payload_name:
            raise ValueError("Payload missions require an explicit payload_name.")
        vehicle_name = payload_name
        namespace = f"/{payload_name}"
    return {
        "vehicle_name": vehicle_name,
        "camera_namespace": namespace,
        "image_topic": f"{namespace}/camera",
        "camera_info_topic": f"{namespace}/camera_info",
        "camera_service_name": f"{namespace}/camera_data",
        "input_transport": input_transport or "",
        "input_raw_topic": f"{namespace}/camera_source",
        "input_compressed_topic": f"{namespace}/camera_source/compressed",
        "input_camera_info_topic": f"{namespace}/camera_info_source",
        "rotate_degrees": 0.0 if rotate_degrees is None else rotate_degrees,
        "preprocess_hook": "" if preprocess_hook is None else preprocess_hook,
        "camera_info_url": "" if camera_info_url is None else camera_info_url,
    }


def _payload_camera_info_url_for(payload_name: str) -> str:
    payload_share = Path(get_package_share_directory("payload"))
    candidates = [
        payload_share / "config" / f"{payload_name}_camera_info.yaml",
        payload_share / "config" / "payload_0_camera_info.yaml",
    ]
    for candidate in candidates:
        if candidate.exists():
            return f"file://{candidate}"
    return ""


def _build_runtime_parameters(
    mission_path: str,
    mission_spec: MissionSpec,
    *,
    auto_launch: bool,
    debug: bool,
    servo_only: bool,
    vehicle_class_name: str | None,
    payload_name: str,
    uav_camera_offsets: list[float],
) -> dict:
    parameters = {"mode_map": mission_path, "auto_launch": bool(auto_launch)}
    if mission_spec.is_payload:
        parameters["payload_name"] = payload_name
        return parameters

    if vehicle_class_name is None:
        raise ValueError("UAV missions require a vehicle class.")
    if len(uav_camera_offsets) != 3:
        raise ValueError(
            f"uav_camera_offsets must have exactly 3 values. Received: {uav_camera_offsets}"
        )

    parameters["debug"] = bool(debug)
    parameters["servo_only"] = bool(servo_only)
    parameters["vehicle_class"] = vehicle_class_name
    parameters["uav_camera_offsets"] = list(uav_camera_offsets)
    return parameters


def _build_camera_actions(
    *,
    mission_spec: MissionSpec,
    camera_contract: dict[str, object],
    vision_nodes: list[str],
    sim: bool,
    vision_debug: bool,
    save_vision_milliseconds: int,
) -> list:
    if not vision_nodes:
        return []

    save_vision = save_vision_milliseconds > 0
    actions = []

    if not sim:
        v4l2_cmd = [
            "ros2",
            "run",
            "v4l2_camera",
            "v4l2_camera_node",
            "--ros-args",
            "-p",
            "image_size:=[640,480]",
        ]
        if mission_spec.is_payload and str(camera_contract["camera_info_url"]).strip():
            v4l2_cmd.extend(
                [
                    "-p",
                    f"camera_info_url:={camera_contract['camera_info_url']}",
                ]
            )
        v4l2_cmd.extend(
            [
                "--remap",
                (
                    f"/image_raw:={camera_contract['input_raw_topic']}"
                    if mission_spec.is_payload
                    else f"/image_raw:={camera_contract['image_topic']}"
                ),
                "--remap",
                (
                    f"/image_raw/compressed:={camera_contract['input_compressed_topic']}"
                    if mission_spec.is_payload
                    else f"/image_raw/compressed:={camera_contract['image_topic']}/compressed"
                ),
                "--remap",
                (
                    f"/camera_info:={camera_contract['input_camera_info_topic']}"
                    if mission_spec.is_payload
                    else f"/camera_info:={camera_contract['camera_info_topic']}"
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

    actions.append(
        Node(
            package="uav",
            executable="camera",
            name=f"{camera_contract['vehicle_name']}_camera",
            output="screen",
            parameters=[
                {
                    "vehicle_name": camera_contract["vehicle_name"],
                    "image_topic": camera_contract["image_topic"],
                    "camera_info_topic": camera_contract["camera_info_topic"],
                    "camera_service_name": camera_contract["camera_service_name"],
                    **(
                        {
                            "input_transport": camera_contract["input_transport"],
                            "input_raw_topic": camera_contract["input_raw_topic"],
                            "input_compressed_topic": camera_contract[
                                "input_compressed_topic"
                            ],
                            "input_camera_info_topic": camera_contract[
                                "input_camera_info_topic"
                            ],
                            "rotate_degrees": camera_contract["rotate_degrees"],
                            "preprocess_hook": camera_contract["preprocess_hook"],
                        }
                        if mission_spec.is_payload
                        else {}
                    ),
                    "display": False,
                    "debug": vision_debug,
                    "save_vision_milliseconds": save_vision_milliseconds,
                }
            ],
        )
    )

    preferred_image_transport = (
        "compressed"
        if mission_spec.is_payload
        and str(camera_contract["input_transport"]).strip().lower() == "compressed"
        else "raw"
    )

    for vision_node in vision_nodes:
        use_camera_service = vision_node != "PayloadAprilTagNode"
        actions.append(
            Node(
                package="uav",
                executable=camel_to_snake(vision_node),
                name=f"{camera_contract['vehicle_name']}_{camel_to_snake(vision_node)}",
                output="screen",
                parameters=[
                    {
                        "vehicle_name": camera_contract["vehicle_name"],
                        "camera_namespace": camera_contract["camera_namespace"],
                        "camera_service_name": camera_contract["camera_service_name"],
                        "use_camera_service": use_camera_service,
                        "preferred_image_transport": preferred_image_transport,
                        "debug": vision_debug,
                        "sim": sim,
                        "save_vision": save_vision,
                        "enable_failsafe_service": mission_spec.is_uav,
                        "failsafe_service_name": "/mode_manager/failsafe"
                        if mission_spec.is_uav
                        else "",
                    }
                ],
            )
        )

    return actions


def launch_setup(context, *args, **kwargs):
    logger = get_logger("main.launch")
    params = _load_launch_parameters(context)

    mission_name = _yaml_or_launch_string(
        context, "mission_name", params.get("mission_name", "basic")
    )
    uav_debug = bool(params.get("uav_debug", False))
    vision_debug = bool(params.get("vision_debug", False))
    enable_vehicle_camera_pipeline = bool(
        params.get("enable_vehicle_camera_pipeline", True)
    )
    save_vision_milliseconds = int(params.get("save_vision_milliseconds", 0))
    servo_only = bool(params.get("servo_only", False))
    sim = bool(params.get("sim", False))
    auto_launch = _yaml_bool_value(
        params.get("auto_launch", True), name="auto_launch", default=True
    )
    payload_name = _yaml_or_launch_string(
        context, "payload_name", params.get("payload_name", "")
    )
    payload_controller = str(params.get("payload_controller", "")).strip()
    payload_camera_input_transport = str(
        params.get(
            "payload_camera_input_transport",
            "raw" if sim else "compressed",
        )
    ).strip()
    payload_camera_rotate_degrees = float(
        params.get("payload_camera_rotate_degrees", 0.0 if sim else 180.0)
    )
    payload_camera_preprocess_hook = str(
        params.get("payload_camera_preprocess_hook", "")
    ).strip()

    mission_path = mission_path_for_name(mission_name)
    mission_spec = MissionSpec.load(mission_path)
    runtime_executable = _runtime_executable_for(mission_spec)
    requires_vision = bool(mission_spec.vision_nodes)

    if mission_spec.is_payload and not payload_name:
        raise ValueError(
            f"Payload mission '{mission_name}' requires an explicit non-empty payload_name."
        )
    if requires_vision and not enable_vehicle_camera_pipeline:
        raise ValueError(
            f"Mission '{mission_name}' requires vision nodes {list(mission_spec.vision_nodes)}, "
            "but enable_vehicle_camera_pipeline is false."
        )

    camera_contract = (
        _camera_contract_for(
            mission_spec,
            payload_name,
            input_transport=payload_camera_input_transport,
            rotate_degrees=payload_camera_rotate_degrees,
            preprocess_hook=payload_camera_preprocess_hook,
            camera_info_url=(
                _payload_camera_info_url_for(payload_name)
                if mission_spec.is_payload and not sim
                else ""
            ),
        )
        if requires_vision
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
    camera_actions = (
        _build_camera_actions(
            mission_spec=mission_spec,
            camera_contract=camera_contract,
            vision_nodes=list(mission_spec.vision_nodes),
            sim=sim,
            vision_debug=vision_debug,
            save_vision_milliseconds=save_vision_milliseconds,
        )
        if requires_vision
        else []
    )

    uav_camera_offsets = params.get("uav_camera_offsets", [0.0, 0.0, 0.0])
    if mission_spec.is_uav and len(uav_camera_offsets) != 3:
        raise ValueError(
            f"uav_camera_offsets must have exactly 3 values. Received: {uav_camera_offsets}"
        )

    px4_path = None
    model = ""
    vehicle_class = None
    middleware = None
    px4_sitl = None
    autostart = None
    launch_px4_sitl = _sim_requires_px4_sitl(mission_spec, sim=sim)

    if mission_spec.is_uav or sim:
        px4_path = find_folder_with_heuristic(
            "PX4-Autopilot",
            os.path.expanduser(LaunchConfiguration("px4_path").perform(context)),
        )

    if mission_spec.is_uav or sim:
        airframe_id = params.get("airframe", "quadcopter")
        custom_airframe_model = params.get("custom_airframe_model", "")
        try:
            airframe_id = int(airframe_id)
        except ValueError:
            try:
                airframe_id = vehicle_id_dict[airframe_id]
            except KeyError as exc:
                raise ValueError(f"Unknown airframe name: {airframe_id}") from exc

        vehicle_class, model_name = get_airframe_details(px4_path, airframe_id)
        autostart = int(airframe_id)
        model = custom_airframe_model or model_name
        if (
            mission_spec.is_uav
            and requires_vision
            and not vehicle_camera_map.get(model, False)
        ):
            raise ValueError(
                f"The selected airframe ID {airframe_id} ({model}) does not have a camera sensor configured."
            )
    if mission_spec.is_uav:
        middleware = ExecuteProcess(
            cmd=["MicroXRCEAgent", "udp4", "-p", "8888"]
            if sim
            else ["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
            output="screen",
            name="middleware",
        )
    if mission_spec.is_uav:
        logger.info(
            f"Launching UAV mission '{mission_name}' with airframe ID {airframe_id}, using model {model}"
        )
    else:
        logger.info(
            f"Launching payload mission '{mission_name}'"
            + (f" in sim with vehicle model '{model}'" if sim and model else "")
        )

    mission_node = Node(
        package="uav",
        executable=runtime_executable,
        output="screen",
        parameters=[
            _build_runtime_parameters(
                mission_path,
                mission_spec,
                auto_launch=auto_launch,
                debug=uav_debug,
                servo_only=servo_only,
                vehicle_class_name=vehicle_class.name
                if vehicle_class is not None
                else None,
                payload_name=payload_name,
                uav_camera_offsets=uav_camera_offsets,
            )
        ],
    )
    sim_startup_started = {"value": False}

    if sim:
        from sim.constants import COMPETITION_NAMES, DEFAULT_COMPETITION, Competition
        from sim.utils import load_sim_launch_parameters, load_sim_parameters

        if launch_px4_sitl and (autostart is None or not model):
            raise ValueError(
                "UAV simulation launches require a valid airframe/model configuration."
            )

        sim_params = load_sim_launch_parameters()
        competition_num = sim_params.get("competition", DEFAULT_COMPETITION.value)
        try:
            competition_type = Competition(competition_num)
            competition = COMPETITION_NAMES[competition_type]
        except (ValueError, KeyError) as exc:
            valid_values = [entry.value for entry in Competition]
            raise ValueError(
                f"Invalid competition: {competition_num}. Must be one of {valid_values}"
            ) from exc

        mission_stage = str(sim_params.get("mission_stage", "")).strip()
        sim_stage_params, _ = load_sim_parameters(
            competition,
            logger,
            competition_name=competition,
            mission_stage=mission_stage,
        )
        world_params = sim_stage_params["world"]["params"]
        payload_names = sorted(
            name
            for name, value in world_params.items()
            if name.startswith("payload_") and value is not None
        )
        logger.info(f"Detected payloads from config: {payload_names}")

        if mission_spec.is_payload and payload_name not in payload_names:
            raise ValueError(
                f"Configured payload_name '{payload_name}' was not found in the selected sim world. "
                f"Available payloads: {payload_names}"
            )

        sim_launch_args = {
            "px4_path": px4_path,
            "model": model,
            "spawn_uav_model": str(bool(model)).lower(),
            "camera_vehicle_type": mission_spec.target if requires_vision else "",
            "camera_vehicle_name": camera_contract["vehicle_name"]
            if camera_contract is not None
            else "",
        }
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory("sim"), "launch", "sim.launch.py"
                )
            ),
            launch_arguments=sim_launch_args.items(),
        )

        startup_actions = []
        if launch_px4_sitl:
            model_entity_name = _gz_entity_name_for_model(model)
            px4_sitl = ExecuteProcess(
                cmd=[
                    "bash",
                    "-c",
                    " ".join(
                        [
                            (
                                "until gz topic -l | grep -q "
                                f"'^/world/{competition}/model/{model_entity_name}/link/base_link/sensor/imu_sensor/imu$'; "
                                "do sleep 0.2; done;"
                            ),
                            f"PX4_GZ_MODEL_NAME={model_entity_name}",
                            f"PX4_GZ_WORLD={competition}",
                            "PX4_GZ_STANDALONE=1",
                            f"PX4_SYS_AUTOSTART={autostart}",
                            "./build/px4_sitl_default/bin/px4",
                        ]
                    ),
                ],
                cwd=px4_path,
                output="screen",
                name="px4_sitl",
            )
            startup_actions.append(px4_sitl)

        if mission_spec.is_payload:
            startup_actions.append(
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        os.path.join(
                            get_package_share_directory("payload"),
                            "launch",
                            "payload.launch.py",
                        )
                    ),
                    launch_arguments={
                        "payload_name": payload_name,
                        "controller": "SimController",
                    }.items(),
                )
            )
        startup_actions.extend(camera_actions)
        if middleware is not None:
            startup_actions.append(middleware)

        startup_trigger = "World generation successful"

        def maybe_start_sim_runtime(event: ProcessIO):
            text = event.text.decode() if isinstance(event.text, bytes) else event.text
            if sim_startup_started["value"] or startup_trigger not in text:
                return None
            sim_startup_started["value"] = True
            return [LogInfo(msg="Gazebo process started."), *startup_actions]

        actions = [
            sim_launch,
            RegisterEventHandler(OnProcessIO(on_stdout=maybe_start_sim_runtime)),
            RegisterEventHandler(OnProcessIO(on_stderr=maybe_start_sim_runtime)),
            mission_node,
        ]
        return actions

    payload_launch_actions = []
    if mission_spec.is_payload:
        payload_launch_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("payload"),
                        "launch",
                        "payload.launch.py",
                    )
                ),
                launch_arguments={
                    "payload_name": payload_name,
                    "controller": payload_controller,
                }.items(),
            )
        )

    actions = [
        *payload_launch_actions,
        *camera_actions,
        *([middleware] if middleware is not None else []),
        mission_node,
    ]
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            DeclareLaunchArgument("params_file", default_value=""),
            DeclareLaunchArgument("mission_name", default_value=""),
            DeclareLaunchArgument("payload_name", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
