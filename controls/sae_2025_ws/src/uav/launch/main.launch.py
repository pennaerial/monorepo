#!/usr/bin/env python3
import os
import re
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart, OnProcessIO
from launch.events.process import ProcessIO
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav.utils import vehicle_id_dict, vehicle_camera_map, Vehicle, get_airframe_details, find_folder_with_heuristic, load_launch_parameters, extract_vision_nodes, clean_text
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.logging import get_logger

def launch_setup(context, *args, **kwargs):
    logger = get_logger('main.launch')
    logger.info("Loading launch parameters...")
    # Load launch parameters from the YAML file.
    params = load_launch_parameters()
    mission_name = params.get('mission_name', 'basic')
    uav_debug = str(params.get('uav_debug', 'false'))
    vision_debug = str(params.get('vision_debug', 'false'))
    use_camera = str(params.get('use_camera', 'true'))
    save_vision = str(params.get('save_vision', 'false'))
    servo_only = str(params.get('servo_only', 'false'))
    
    sim_bool = str(params.get('sim', 'false')).lower() == 'true'
    run_mission_bool = str(params.get('run_mission', 'true')).lower() == 'true'
    vision_debug_bool = vision_debug.lower() == 'true'
    save_vision_bool = save_vision.lower() == 'true'
    use_camera_bool = use_camera.lower() == 'true'
    
    '''
    Airframe ID handling
    All PX4 supported IDs can be found here: https://docs.px4.io/main/en/airframes/airframe_reference
    However, IDs available for simulation can be found in PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes
    '''
    airframe_id = params.get('airframe', 'quadcopter')
    try:
        # If an airframe ID is provided directly, use it
        airframe_id = int(airframe_id) 
    except ValueError:
        try:
            # Otherwise, map preset vehicle name to airframe ID
            airframe_id = vehicle_id_dict[airframe_id] 
        except KeyError:
            raise ValueError(f"Unknown airframe name: {airframe_id}")

    custom_airframe_model = params.get('custom_airframe_model', '')
    camera_offsets = params.get('camera_offsets', [0, 0, 0])
    
    # Build the mission YAML file path using the mission name.
    YAML_PATH = os.path.join(os.getcwd(), 'src', 'uav', 'uav', 'missions', f"{mission_name}.yaml")
    
    # Build vision node actions.
    vision_nodes = []
    vision_node_actions = []
    if use_camera_bool:
        vision_node_actions.append(Node(
            package='uav',
            executable='camera',
            name='camera',
            output='screen'
        ))

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

    # Define the PX4 SITL model, autostart, and vehicle class
    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser(LaunchConfiguration('px4_path').perform(context)))
    vehicle_class, model_name = get_airframe_details(px4_path, airframe_id)
    autostart = int(airframe_id)
    model = custom_airframe_model or model_name
    if (not vehicle_camera_map.get(model, False)) and use_camera_bool: 
        raise ValueError(f"The selected airframe ID {airframe_id} ({model}) does not have a camera sensor configured. Please choose a different airframe or add a camera to the model.")
    print(f"Launching a {vehicle_class.name} with airframe ID {airframe_id}, using model {model}")

    camera_offsets_str = ','.join(str(offset) for offset in camera_offsets)
    mission_cmd = ['ros2', 'run', 'uav', 'mission', uav_debug, YAML_PATH, servo_only, camera_offsets_str, vehicle_class.name, ','.join(vision_nodes)]
    mission = ExecuteProcess(
        cmd=mission_cmd,
        output='screen',
        emulate_tty=True,
        name='mission'
    )

    # Determine which processes need to be ready before starting mission
    # In sim mode: need both uav (px4_sitl) and middleware
    # In hardware mode: only need middleware (uav is already running as flight controller)
    required_processes = ["uav", "middleware"] if sim_bool else ["middleware"]
    mission_ready_flags = {proc: False for proc in required_processes}
    mission_started = {"value": False}  # mutable so inner functions can modify
    
    def get_trigger(process_name):
        """Get the trigger string for a given process based on sim mode."""
        if process_name == "uav":
            return "INFO  [commander] Ready for takeoff!"
        elif process_name == "middleware":
            return "INFO  [uxrce_dds_client] time sync converged" if sim_bool else "session established"
        else:
            raise ValueError(f"Invalid process name: {process_name}")
    
    def make_io_handler(process_name):
        """Create an IO handler for a specific process."""
        trigger = get_trigger(process_name)
        
        def handler(event: ProcessIO):
            text = clean_text(event.text.decode() if isinstance(event.text, bytes) else event.text)
            if trigger in text:
                mission_ready_flags[process_name] = True
                if not mission_started["value"] and all(mission_ready_flags.values()):
                    mission_started["value"] = True
                    return [
                        LogInfo(msg="[launcher] Processes ready, starting mission"),
                        mission,
                    ]
            return None
        return handler
    
    # Now, construct the actions list in a single step, depending on sim_bool
    if sim_bool:
        # Prepare sim launch arguments with all simulation parameters
        sim_launch_args = {
            'px4_path': px4_path,
            'model': model
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
                    [LogInfo(msg="Gazebo process started."), px4_sitl, *vision_node_actions, middleware] 
                    if b"Successfully generated world file:" in event.text else None
                ))
            ),
        ]
        if run_mission_bool:
            actions.extend([
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
            ])
    else:
        actions = [
            *vision_node_actions,
            LogInfo(msg="Vision nodes started."),
            middleware,
        ]
        if run_mission_bool:
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
    return LaunchDescription([
        DeclareLaunchArgument('px4_path', default_value='~/PX4-Autopilot'),
        OpaqueFunction(function=launch_setup)
    ])
