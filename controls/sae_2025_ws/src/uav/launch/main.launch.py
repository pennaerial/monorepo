#!/usr/bin/env python3
import os
import re
import subprocess
from launch import LaunchDescription
from launch.actions import (
    ExecuteProcess,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    DeclareLaunchArgument,
)
from launch.event_handlers import OnProcessIO
from launch.events.process import ProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from uav.utils import (
    vehicle_id_dict,
    vehicle_camera_map,
    get_airframe_details,
    find_folder_with_heuristic,
    load_launch_parameters,
    extract_vision_nodes,
    clean_text,
)
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.logging import get_logger


def _is_middleware_running():
    """Return True if MicroXRCEAgent is already running on this machine (e.g. port 8888 in sim)."""
    try:
        # Prefer port check for sim (UDP 8888); pgrep as fallback
        result = subprocess.run(
            ["pgrep", "-f", "MicroXRCEAgent"],
            capture_output=True,
            timeout=2,
        )
        return result.returncode == 0
    except (FileNotFoundError, subprocess.TimeoutExpired, Exception):
        return False


def launch_setup(context, *args, **kwargs):
    logger = get_logger("main.launch")
    logger.info("Loading launch parameters...")
    # Load launch parameters from the YAML file.
    params = load_launch_parameters()
    vehicle_id = int(LaunchConfiguration("vehicle_id").perform(context))
    if vehicle_id < 0:
        raise ValueError("vehicle_id must be >= 0")
    launch_sim_arg = LaunchConfiguration("launch_sim", default="auto").perform(context).lower()
    # "auto" = start sim for vehicle_id 0; "true" = always; "false" = never
    should_launch_sim = (
        launch_sim_arg == "true"
        or (launch_sim_arg == "auto" and vehicle_id == 0)
    )
    mission_name = params.get("mission_name", "basic")
    uav_debug = str(params.get("uav_debug", "false"))
    vision_debug = str(params.get("vision_debug", "false"))
    use_camera = str(params.get("use_camera", "true"))
    save_vision_milliseconds = int(params.get("save_vision_milliseconds", "0"))
    save_vision_bool = save_vision_milliseconds > 0
    servo_only = str(params.get("servo_only", "false"))

    sim_bool = str(params.get("sim", "false")).lower() == "true"
    run_mission_bool = str(params.get("run_mission", "true")).lower() == "true"
    vision_debug_bool = vision_debug.lower() == "true"
    use_camera_bool = use_camera.lower() == "true"

    """
    Airframe ID handling
    All PX4 supported IDs can be found here: https://docs.px4.io/main/en/airframes/airframe_reference
    However, IDs available for simulation can be found in PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
    """
    airframe_override = LaunchConfiguration("airframe", default="").perform(context)
    airframe_id = airframe_override if airframe_override else params.get("airframe", "quadcopter")
    try:
        # If an airframe ID is provided directly, use it
        airframe_id = int(airframe_id)
    except (ValueError, TypeError):
        try:
            # Otherwise, map preset vehicle name to airframe ID
            airframe_id = vehicle_id_dict[str(airframe_id)]
        except KeyError:
            raise ValueError(f"Unknown airframe name: {airframe_id}")

    custom_airframe_model = params.get("custom_airframe_model", "")
    camera_offsets = params.get("camera_offsets", [0, 0, 0])

    # Build the mission YAML file path using the mission name.
    YAML_PATH = os.path.join(
        os.getcwd(), "src", "uav", "uav", "missions", f"{mission_name}.yaml"
    )

    print("Building vision node actions...")
    # Build vision node actions.
    vision_nodes = []
    vision_node_actions = []
    if use_camera_bool:
        vision_node_actions.append(
            Node(
                package="uav",
                executable="camera",
                name="camera",
                output="screen",
                parameters=[
                    {
                        "vehicle_id": vehicle_id,
                        "debug": vision_debug_bool,
                        "save_vision_milliseconds": save_vision_milliseconds,
                    }
                ],
            )
        )

        for node in extract_vision_nodes(YAML_PATH):
            vision_nodes.append(node)
            # Convert CamelCase node names to snake_case executable names.
            s1 = re.sub("(.)([A-Z][a-z]+)", r"\1_\2", node)
            exe_name = re.sub("([a-z0-9])([A-Z])", r"\1_\2", s1).lower()
            vision_node_actions.append(
                Node(
                    package="uav",
                    executable=exe_name,
                    name=exe_name,
                    output="screen",
                    parameters=[
                        {
                            "debug": vision_debug_bool,
                            "sim": sim_bool,
                            "save_vision": save_vision_bool,
                            "vehicle_id": vehicle_id,
                        }
                    ],
                )
            )

        # Clear vision node actions if none are found.
        if len(vision_nodes) == 0 and not save_vision_bool:
            vision_node_actions = []

        if not sim_bool:
            vision_node_actions.insert(
                0,
                ExecuteProcess(
                    cmd=[
                        "ros2",
                        "run",
                        "v4l2_camera",
                        "v4l2_camera_node",
                        "--ros-args",
                        "-p",
                        "image_size:=[640,480]",
                        "--ros-args",
                        "--remap",
                        f"/image_raw:=/px4_{vehicle_id}/camera",
                        "--remap",
                        f"/camera_info:=/px4_{vehicle_id}/camera_info",
                    ],
                    output="screen",
                    name="cam2image",
                ),
            )

    # Define the middleware process (may be skipped if already running on device).
    middleware = ExecuteProcess(
        cmd=["MicroXRCEAgent", "udp4", "-p", "8888"]
        if sim_bool
        else ["MicroXRCEAgent", "serial", "--dev", "/dev/serial0", "-b", "921600"],
        output="screen",
        name="middleware",
    )
    middleware_already_running = _is_middleware_running()
    if middleware_already_running:
        logger = get_logger("main.launch")
        logger.info("MicroXRCEAgent already running on device; skipping start and ready wait.")

    # Define the PX4 SITL model, autostart, and vehicle class
    px4_path = find_folder_with_heuristic(
        "PX4-Autopilot",
        os.path.expanduser(LaunchConfiguration("px4_path").perform(context)),
    )
    vehicle_class, model_name = get_airframe_details(px4_path, airframe_id)
    autostart = int(airframe_id)
    model = custom_airframe_model or model_name
    if (not vehicle_camera_map.get(model, False)) and use_camera_bool:
        raise ValueError(
            f"The selected airframe ID {airframe_id} ({model}) does not have a camera sensor configured. Please choose a different airframe or add a camera to the model."
        )
    print(
        f"Launching a {vehicle_class.name} with airframe ID {airframe_id}, using model {model}"
    )

    camera_offsets_str = ",".join(str(offset) for offset in camera_offsets)
    mission_cmd = [
        "ros2",
        "run",
        "uav",
        "mission",
        uav_debug,
        YAML_PATH,
        servo_only,
        camera_offsets_str,
        vehicle_class.name,
        ",".join(vision_nodes),
        str(vehicle_id),
    ]
    mission = ExecuteProcess(
        cmd=mission_cmd, output="screen", emulate_tty=True, name="mission"
    )

    start_mission_trigger = ExecuteProcess(
        cmd=[
            "ros2",
            "service",
            "call",
            f"/mode_manager/vehicle_{vehicle_id}/start_mission",
            "std_srvs/srv/Trigger",
        ],
        output="screen",
        name="start_mission_trigger",
    )

    # Determine which processes need to be ready before starting mission.
    # Only wait for middleware if we are starting it (not already running on device).
    if sim_bool:
        wait_for_middleware = not middleware_already_running
        required_processes = ["uav"] + (["middleware"] if wait_for_middleware else [])
    else:
        required_processes = [] if middleware_already_running else ["middleware"]
    mission_ready_flags = {proc: False for proc in required_processes}
    mission_started = {"value": False}  # mutable so inner functions can modify

    def get_trigger(process_name):
        """Get the trigger string for a given process based on sim mode."""
        if process_name == "uav":
            return "INFO  [commander] Ready for takeoff!"
        elif process_name == "middleware":
            return (
                "INFO  [uxrce_dds_client] synchronized with time offset"
                if sim_bool
                else "session established"
            )
        else:
            raise ValueError(f"Invalid process name: {process_name}")

    def make_io_handler(process_name):
        """Create an IO handler for a specific process."""
        trigger = get_trigger(process_name)

        def handler(event: ProcessIO):
            text = clean_text(
                event.text.decode() if isinstance(event.text, bytes) else event.text
            )
            if trigger in text:
                mission_ready_flags[process_name] = True
                if not mission_started["value"] and all(mission_ready_flags.values()):
                    mission_started["value"] = True
                    return [
                        LogInfo(msg="[launcher] Processes ready, starting mission"),
                        start_mission_trigger,
                    ]
            return None

        return handler

    # Now, construct the actions list in a single step, depending on sim_bool
    if sim_bool:
        from sim.utils import load_sim_launch_parameters
        from sim.constants import Competition, COMPETITION_NAMES, DEFAULT_COMPETITION

        # Resolve world name from sim launch params (same source as sim.launch.py)
        sim_params = load_sim_launch_parameters()
        competition_num = sim_params.get("competition", DEFAULT_COMPETITION.value)
        try:
            competition_type = Competition(competition_num)
            competition = COMPETITION_NAMES[competition_type]
        except (ValueError, KeyError):
            valid_values = [e.value for e in Competition]
            raise ValueError(
                f"Invalid competition: {competition_num}. Must be one of {valid_values}"
            )
        logger.info(f"PX4_GZ_WORLD={competition} (vehicle_id={vehicle_id})")

        # vehicle_id is 0-based and matches the PX4 instance number directly.
        px4_instance = vehicle_id
        # Model pose offset for multi-vehicle (x, y in meters; see PX4 multi-vehicle doc)
        model_pose = f"0,{vehicle_id},0,0,0,0" if vehicle_id > 0 else "0,0,0,0,0,0"
        # First vehicle (vehicle_id=0) starts Gazebo and connects; others use STANDALONE and connect to existing server
        px4_sitl_cmd = (
            f"PX4_GZ_MODEL_POSE='{model_pose}' PX4_GZ_WORLD={competition} "
            f"PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART={autostart} PX4_SIM_MODEL={model} "
            f"./build/px4_sitl_default/bin/px4 -i {px4_instance}"
        )

        print(f"Starting PX4 SITL instance {px4_instance} (vehicle_id={vehicle_id})...")
        px4_sitl = ExecuteProcess(
            cmd=["bash", "-c", px4_sitl_cmd],
            cwd=px4_path,
            output="screen",
            name="px4_sitl",
        )

        if should_launch_sim:
            # This vehicle is responsible for starting Gazebo via sim.launch.py.
            sim_launch_args = {
                "model": model,
                "px4_path": px4_path,
                "vehicle_id": str(vehicle_id),
            }
            sim = IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("sim"), "launch", "sim.launch.py"
                    )
                ),
                launch_arguments=sim_launch_args.items(),
            )
            processes_after_sim_ready = [
                LogInfo(msg=f"Simulation world ready; starting vehicle_{vehicle_id} stack."),
                px4_sitl,
                *vision_node_actions,
            ]
            if not middleware_already_running:
                processes_after_sim_ready.append(middleware)
            actions = [
                sim,
                RegisterEventHandler(
                    OnProcessIO(
                        on_stderr=lambda event, procs=processes_after_sim_ready: (
                            procs if b"Successfully generated world file:" in event.text else None
                        )
                    )
                ),
                mission,
            ]
        else:
            # Additional vehicles still need their own camera bridges here because sim.launch.py
            # only creates the ROS<->GZ bridges for vehicle_id=0.
            # In GZ topics the model name has no "gz_" prefix (e.g. x500_mono_cam_down_2)
            model_name_in_world = f"{model[3:]}_{px4_instance}"
            gz_cam = f"/world/{competition}/model/{model_name_in_world}/link/camera_link/sensor/camera/image"
            gz_cam_info = f"/world/{competition}/model/{model_name_in_world}/link/camera_link/sensor/camera/camera_info"
            cam_prefix = f"/px4_{px4_instance}"
            bridge_camera = Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[f"{gz_cam}@sensor_msgs/msg/Image[gz.msgs.Image"],
                remappings=[(gz_cam, f"{cam_prefix}/camera")],
                output="screen",
                name="gz_ros_bridge_camera",
            )
            bridge_camera_info = Node(
                package="ros_gz_bridge",
                executable="parameter_bridge",
                arguments=[f"{gz_cam_info}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"],
                remappings=[(gz_cam_info, f"{cam_prefix}/camera_info")],
                output="screen",
                name="gz_ros_bridge_camera_info",
            )
            actions = [
                LogInfo(msg=f"Connecting vehicle_id={vehicle_id} to existing Gazebo (px4_{px4_instance})..."),
                bridge_camera,
                bridge_camera_info,
                px4_sitl,
                *vision_node_actions,
            ]
            if not middleware_already_running:
                actions.append(middleware)
            actions.append(mission)

        if run_mission_bool:
            handlers = [RegisterEventHandler(OnProcessIO(target_action=px4_sitl, on_stdout=make_io_handler("uav")))]
            if not middleware_already_running:
                handlers.append(
                    RegisterEventHandler(
                        OnProcessIO(
                            target_action=middleware,
                            on_stdout=make_io_handler("middleware"),
                        )
                    )
                )
            actions.extend(handlers)
    else:
        # Hardware: start vision, middleware (if not already running), mission
        actions = [
            *vision_node_actions,
            LogInfo(msg="Vision nodes started."),
        ]
        if not middleware_already_running:
            actions.append(middleware)
        actions.append(mission)
        if run_mission_bool and not middleware_already_running:
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
            DeclareLaunchArgument("px4_path", default_value="~/Tools-users/PX4-Autopilot"),
            DeclareLaunchArgument("vehicle_id", default_value="0"),
            DeclareLaunchArgument(
                "airframe",
                default_value="",
                description="Override vehicle type (airframe name or ID). Used by multi_uav.launch.py.",
            ),
            DeclareLaunchArgument(
                "launch_sim",
                default_value="auto",
                description="'auto' = start Gazebo for vehicle_id 0; 'true' = always; 'false' = never.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
