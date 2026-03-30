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

from uav.mission_spec import MissionSpec, mission_path_for_name
from uav.utils import (
    camel_to_snake,
    clean_text,
    find_folder_with_heuristic,
    get_airframe_details,
    vehicle_camera_map,
    vehicle_id_dict,
)


def _load_launch_parameters() -> dict:
    params_path = Path(__file__).resolve().with_name("launch_params.yaml")
    with params_path.open("r", encoding="utf-8") as params_file:
        return yaml.safe_load(params_file) or {}


def _yaml_or_launch_string(context, name: str, config_value) -> str:
    override = LaunchConfiguration(name).perform(context).strip()
    if override:
        return override
    return "" if config_value is None else str(config_value).strip()


def _runtime_executable_for(mission_spec: MissionSpec) -> str:
    if mission_spec.is_uav:
        return "uav_mission"
    if mission_spec.is_payload:
        return "payload_mission"
    raise ValueError(f"Unsupported mission target '{mission_spec.target}'.")


def _camera_contract_for(mission_spec: MissionSpec, payload_name: str) -> dict[str, str]:
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
    }


def _build_runtime_parameters(
    mission_path: str,
    mission_spec: MissionSpec,
    *,
    debug: bool,
    servo_only: bool,
    vehicle_class_name: str | None,
    payload_name: str,
    uav_camera_offsets: list[float],
) -> dict:
    parameters = {"mode_map": mission_path}
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
    camera_contract: dict[str, str],
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
        actions.append(
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "v4l2_camera",
                    "v4l2_camera_node",
                    "--ros-args",
                    "-p",
                    "image_size:=[640,480]",
                    "--remap",
                    f"/image_raw:={camera_contract['image_topic']}",
                    "--remap",
                    f"/camera_info:={camera_contract['camera_info_topic']}",
                ],
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
                    "display": False,
                    "debug": vision_debug,
                    "save_vision_milliseconds": save_vision_milliseconds,
                }
            ],
        )
    )

    for vision_node in vision_nodes:
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
                        "use_camera_service": True,
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
    params = _load_launch_parameters()

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
    auto_launch = bool(params.get("auto_launch", True))
    payload_name = _yaml_or_launch_string(
        context, "payload_name", params.get("payload_name", "")
    )
    payload_controller = str(params.get("payload_controller", "")).strip()

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
        _camera_contract_for(mission_spec, payload_name) if requires_vision else None
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
        if mission_spec.is_uav and requires_vision and not vehicle_camera_map.get(
            model, False
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
            + (f" in sim with backing airframe '{model}'" if sim else "")
        )

    mission_node = Node(
        package="uav",
        executable=runtime_executable,
        name="mission",
        output="screen",
        parameters=[
            _build_runtime_parameters(
                mission_path,
                mission_spec,
                debug=uav_debug,
                servo_only=servo_only,
                vehicle_class_name=vehicle_class.name if vehicle_class is not None else None,
                payload_name=payload_name,
                uav_camera_offsets=uav_camera_offsets,
            )
        ],
    )

    start_mission_trigger = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            "/mode_manager/start_mission",
            "std_srvs/srv/Trigger",
        ],
        output="screen",
        name="start_mission_trigger",
    )

    def make_io_handler(process_name: str):
        trigger = (
            "INFO  [commander] Ready for takeoff!"
            if process_name == "uav"
            else (
                "INFO  [uxrce_dds_client] synchronized with time offset"
                if sim
                else "session established"
            )
        )

        def handler(event: ProcessIO):
            text = clean_text(
                event.text.decode() if isinstance(event.text, bytes) else event.text
            )
            if trigger not in text:
                return None
            mission_ready_flags[process_name] = True
            if not mission_started["value"] and all(mission_ready_flags.values()):
                mission_started["value"] = True
                return [
                    LogInfo(msg="[launcher] Processes ready, starting mission"),
                    start_mission_trigger,
                ]
            return None

        return handler

    mission_ready_flags = {key: False for key in ("uav", "middleware")}
    mission_started = {"value": False}

    if sim:
        from sim.constants import COMPETITION_NAMES, DEFAULT_COMPETITION, Competition
        from sim.utils import load_sim_launch_parameters, load_sim_parameters

        if autostart is None or not model:
            raise ValueError(
                "Simulation-backed launches require a valid airframe/model configuration."
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

        vehicle_pose = world_params.get(
            "vehicle_pose", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        )
        vehicle_pose_str = ",".join(str(pose) for pose in vehicle_pose)

        sim_launch_args = {
            "px4_path": px4_path,
            "model": model,
            "camera_vehicle_type": mission_spec.target if requires_vision else "",
            "camera_vehicle_name": camera_contract["vehicle_name"]
            if camera_contract is not None
            else "",
        }
        sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("sim"), "launch", "sim.launch.py")
            ),
            launch_arguments=sim_launch_args.items(),
        )

        startup_actions = []
        px4_sitl = ExecuteProcess(
            cmd=[
                "bash",
                "-c",
                f"PX4_GZ_MODEL_POSE='{vehicle_pose_str}' PX4_GZ_WORLD={competition} PX4_GZ_STANDALONE=1 "
                f"PX4_SYS_AUTOSTART={autostart} PX4_SIM_MODEL={model} ./build/px4_sitl_default/bin/px4",
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

        actions = [
            sim_launch,
            RegisterEventHandler(
                OnProcessIO(
                    on_stderr=lambda event: (
                        [LogInfo(msg="Gazebo process started."), *startup_actions]
                        if b"Successfully generated world file:" in event.text
                        else None
                    )
                )
            ),
            mission_node,
        ]
        if auto_launch and mission_spec.is_uav:
            actions.extend(
                [
                    RegisterEventHandler(
                        OnProcessIO(
                            target_action=px4_sitl,
                            on_stdout=make_io_handler("middleware"),
                        )
                    ),
                    RegisterEventHandler(
                        OnProcessIO(
                            target_action=px4_sitl,
                            on_stdout=make_io_handler("uav"),
                        )
                    ),
                ]
            )
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
    if auto_launch and mission_spec.is_uav and middleware is not None:
        actions.append(
            RegisterEventHandler(
                OnProcessIO(
                    target_action=middleware,
                    on_stderr=make_io_handler("middleware"),
                )
            )
        )
    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("px4_path", default_value="~/PX4-Autopilot"),
            DeclareLaunchArgument("mission_name", default_value=""),
            DeclareLaunchArgument("payload_name", default_value=""),
            OpaqueFunction(function=launch_setup),
        ]
    )
