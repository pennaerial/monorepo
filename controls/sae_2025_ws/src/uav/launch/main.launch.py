#!/usr/bin/env python3
import os
import re
import platform
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessIO, OnProcessStart
from launch.events.process import ProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav.utils import vehicle_map, find_folder_with_heuristic, load_launch_parameters, extract_vision_nodes
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.logging import get_logger
from launch.logging import get_logger

def launch_setup(context, *args, **kwargs):
    logger = get_logger('main.launch')
    logger = get_logger('main.launch')
    # Load launch parameters from the YAML file.
    params = load_launch_parameters()
    mission_name = params.get('mission_name', 'basic')
    uav_debug = str(params.get('uav_debug', 'false'))
    vision_debug = str(params.get('vision_debug', 'false'))
    sim_bool = str(params.get('sim', 'false'))
    run_mission = str(params.get('run_mission', 'true'))
    vehicle_type = vehicle_map[params.get('vehicle_type', 0)]
    save_vision = str(params.get('save_vision', 'false'))
    camera_offsets = params.get('camera_offsets', [0, 0, 0])
    servo_only = str(params.get('servo_only', 'false'))

    # Convert debug and simulation flags to booleans.
    vision_debug_bool = vision_debug.lower() == 'true'
    sim_bool = sim_bool.lower() == 'true'
    run_mission_bool = run_mission.lower() == 'true'
    save_vision_bool = save_vision.lower() == 'true'
    
    # Build the mission YAML file path using the mission name.
    YAML_PATH = os.path.join(os.getcwd(), 'src', 'uav', 'uav', 'missions', f"{mission_name}.yaml")
    
    # Build vision node actions.
    vision_nodes = []
    vision_node_actions = [Node(
        package='uav',
        executable='camera',
        name='camera',
        output='screen'
    )]

    for node in extract_vision_nodes(YAML_PATH):
        vision_nodes.append(node)
        # Convert CamelCase node names to snake_case executable names.
        s1 = re.sub('(.)([A-Z][a-z]+)', r'\1_\2', node)
        exe_name = re.sub('([a-z0-9])([A-Z])', r'\1_\2', s1).lower()
        vision_node_actions.append(Node(
            package='uav',
            executable=exe_name,
            name=exe_name,
            output='screen',
            parameters=[{'debug': vision_debug_bool, 'sim': sim_bool, 'save_vision': save_vision_bool}],
        ))
    
    # Clear vision node actions if none are found.
    if len(vision_nodes) == 0:
        vision_node_actions = []
    
    if not sim_bool:
        vision_node_actions.insert(0, ExecuteProcess(
            cmd=['ros2', 'run', 'v4l2_camera', 'v4l2_camera_node', '--ros-args', '-p', 'image_size:=[640,480]', '--ros-args', '--remap', '/image_raw:=/camera'],
            output='screen',
            name='cam2image'
        ))
    
    # Define the middleware process.
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'] if sim_bool else ['MicroXRCEAgent', 'serial', '--dev', '/dev/serial0', '-b', '921600'],
        output='screen',
        name='middleware'
    )
    
    # Define the PX4 SITL model and autostart
    if vehicle_type == 'quadcopter':
        autostart = 4001
        model = 'gz_x500_mono_cam' # append '_down' for down-facing camera
    elif vehicle_type == 'tiltrotor_vtol':
        autostart = 4020
        model = 'gz_tiltrotor'
    elif vehicle_type == 'fixed_wing':
        autostart = 4003
        model = 'gz_rc_cessna'
    elif vehicle_type == 'standard_vtol':
        autostart = 4004
        model = 'gz_standard_vtol'
    else:
        raise ValueError(f"Invalid vehicle type: {vehicle_type}")

    
    gz_camera_topic_model = model[3:]  # remove 'gz_' prefix
    gz_camera_topic_model = model[3:]  # remove 'gz_' prefix

    arch = platform.machine().lower()
    if arch in ("x86_64", "amd64", "i386", "i686"):
        platform_type = "x86"
    elif arch in ("arm64", "aarch64", "armv7l", "arm"):
        platform_type = "arm"
    else:
        raise ValueError(f"Unknown architecture: {arch}")

    logger.debug(f"Running Architecture: {arch}")

    sae_ws_path = os.path.expanduser(os.getcwd())

    camera_offsets_str = ','.join(str(offset) for offset in camera_offsets)
    mission_cmd = ['ros2', 'run', 'uav', 'mission', uav_debug, YAML_PATH, servo_only, camera_offsets_str, ','.join(vision_nodes)]
    mission = ExecuteProcess(
        cmd=mission_cmd,
        output='screen',
        emulate_tty=True,
        name='mission'
    )
    mission_ready_flags = {"uav": False, "middleware": False}
    mission_started = {"value": False}  # mutable so inner functions can modify
    def make_io_handler(process_name):
        trigger = "INFO  [commander] Ready for takeoff!" if process_name == "uav" else "INFO  [uxrce_dds_client] time sync converged" if process_name == "middleware" else None
        if trigger is None:
            raise ValueError(f"Invalid process name: {process_name}")
        def clean_text(text):
            ansi_escape = re.compile(r'\x1b\[[0-9;]*m') # remove ANSI escape codes that give color in terminal
            return ansi_escape.sub('', text).strip()
        def handler(event: ProcessIO):
            text = clean_text(event.text.decode() if isinstance(event.text, bytes) else event.text)
            if trigger in text:
                mission_ready_flags[process_name] = True
                # Only when BOTH are ready do we launch spawn_world
                if not mission_started["value"] and all(mission_ready_flags.values()):
                    mission_started["value"] = True
                    return [
                        LogInfo(msg="[launcher] Both processes ready, starting mission"),
                        mission,
                    ]
            return None
        return handler
    # Now, construct the actions list in a single step, depending on sim_bool
    if sim_bool:
        # Find required paths.
        px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser(LaunchConfiguration('px4_path').perform(context)))

        # Prepare sim launch arguments with all simulation parameters
        sim_launch_args = {
            'px4_path': px4_path,
            'gz_camera_topic_model': gz_camera_topic_model
        }
        
        sim = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('sim'),
                    'launch',
                    'sim.launch.py'
                )
            ),
            launch_arguments=sim_launch_args.items()
        )

        px4_sitl = ExecuteProcess(
            cmd=['bash', '-c', f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART={autostart} PX4_SIM_MODEL={model} ./build/px4_sitl_default/bin/px4'],
            cwd=px4_path,
            output='screen',
            name='px4_sitl'
        )
        actions = [
            sim,
            RegisterEventHandler(
                    OnProcessIO(on_stderr=lambda event: (
                        [LogInfo(msg="Gazebo process started."), px4_sitl, *vision_node_actions, middleware] if b"Successfully generated world file:" in event.text else None
                    )
                )
            ),
            
            RegisterEventHandler(
                OnProcessIO(
                    target_action=px4_sitl,
                    on_stdout=make_io_handler("uav"),
                )
            ),
        ]
    else:
        # Hardware mode: start mission after middleware is ready
        actions = [
            *vision_node_actions,
            LogInfo(msg="Vision nodes started."),
            middleware,
        ]
    if run_mission_bool:
        actions.append(
                RegisterEventHandler(
                    OnProcessIO(
                        target_action=px4_sitl,
                        on_stdout=make_io_handler("middleware"),
                    )
                )
            )
    
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_path', default_value='~/PX4-Autopilot'),
        OpaqueFunction(function=launch_setup)
    ])
