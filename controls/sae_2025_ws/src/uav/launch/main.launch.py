#!/usr/bin/env python3
import os
import re
import platform
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, TimerAction, OpaqueFunction, RegisterEventHandler, DeclareLaunchArgument
from launch.event_handlers import OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from uav.utils import vehicle_id_dict, vehicle_camera_map, Vehicle, get_airframe_details, find_folder_with_heuristic, load_launch_parameters, extract_vision_nodes
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def launch_setup(context, *args, **kwargs):
    # Load launch parameters from the YAML file.
    print("Loading launch parameters...")
    params = load_launch_parameters()
    mission_name = params.get('mission_name', 'basic')
    uav_debug = str(params.get('uav_debug', 'false'))
    vision_debug = str(params.get('vision_debug', 'false'))
    sim_bool = str(params.get('sim', 'false'))
    run_mission = str(params.get('run_mission', 'true'))

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

    vehicle_class = params.get('vehicle_class', 'auto')
    custom_airframe_model = params.get('custom_airframe_model', '')
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
    
    print("Building vision node actions...")
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

    # Define the PX4 SITL model, autostart, and vehicle class
    if sim_bool:
        # In simulation mode, determine vehicle class and model from airframe ID
        px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser(LaunchConfiguration('px4_path').perform(context)))
        vehicle_class, model_name = get_airframe_details(px4_path, airframe_id)
        autostart = int(airframe_id)
        model = custom_airframe_model or model_name
        if not vehicle_camera_map.get(model, False):  # Remove 'gz_' prefix for model check
            raise ValueError(f"The selected airframe ID {airframe_id} ({model}) does not have a camera sensor configured. Please choose a different airframe or add a camera to the model.")
        print(f"Simulation mode: Launching a {vehicle_class.name} with airframe ID {airframe_id}, using model {model}")
    else:
        # In hardware mode, trust any PX4 supported airframe ID and use specified vehicle class
        model = "gz_x500"  # Use default model for hardware mode
        autostart = int(airframe_id)
        if vehicle_class == 'auto':
            raise ValueError("In hardware mode, please specify vehicle_class in launch_params.yaml")
        else:
            try:
                vehicle_class = Vehicle[vehicle_class.upper()]
            except KeyError:
                raise ValueError(f"Invalid vehicle_class: {vehicle_class}")
        print(f"Hardware mode: Launching a {vehicle_class.name} with airframe ID {airframe_id}")
    
    topic_model_name = model[3:]  # remove 'gz_' prefix

    arch = platform.machine().lower()
    if arch in ("x86_64", "amd64", "i386", "i686"):
        platform_type = "x86"
    elif arch in ("arm64", "aarch64", "armv7l", "arm"):
        platform_type = "arm"
    else:
        raise ValueError(f"Unknown architecture: {arch}")

    camera_topic_name = (
        "imager" if platform_type == "x86" else "camera"
    )  # windows -> 'imager'   mac -> 'camera'
    GZ_CAMERA_TOPIC = f"/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/image"
    GZ_CAMERA_INFO_TOPIC = f"/world/custom/model/{topic_model_name}_0/link/camera_link/sensor/{camera_topic_name}/camera_info"

    sae_ws_path = os.path.expanduser(os.getcwd())
    
    gz_ros_bridge_camera = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[f"{GZ_CAMERA_TOPIC}@sensor_msgs/msg/Image[gz.msgs.Image"],
        remappings=[(GZ_CAMERA_TOPIC, "/camera")],
        output="screen",
        name="gz_ros_bridge_camera",
        cwd=sae_ws_path,
    )

    gz_ros_bridge_camera_info = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            f"{GZ_CAMERA_INFO_TOPIC}@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo"
        ],
        remappings=[(GZ_CAMERA_INFO_TOPIC, "/camera_info")],
        output="screen",
        name="gz_ros_bridge_camera_info",
        cwd=sae_ws_path,
    )

    camera_offsets_str = ','.join(str(offset) for offset in camera_offsets)
    mission_cmd = ['ros2', 'run', 'uav', 'mission', uav_debug, YAML_PATH, servo_only, camera_offsets_str, vehicle_class.name, ','.join(vision_nodes)]
    mission = ExecuteProcess(
        cmd=mission_cmd,
        output='screen',
        emulate_tty=True,
        name='mission'
    )

    # Now, construct the actions list in a single step, depending on sim_bool
    if sim_bool:
        # Prepare sim launch arguments with all simulation parameters
        sim_launch_args = {
            'model': model,
            'px4_path': px4_path,
        }
        
        print("Including simulation launch description...")
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

        print("Starting PX4 SITL...")
        px4_sitl = ExecuteProcess(
            cmd=['bash', '-c', f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART={autostart} PX4_SIM_MODEL={model} ./build/px4_sitl_default/bin/px4'],
            cwd=px4_path,
            output='screen',
            name='px4_sitl'
        )
        actions = [
            sim,
            LogInfo(msg="Gazebo started."),
            *vision_node_actions,
            LogInfo(msg="Vision nodes started."),
            middleware,
            RegisterEventHandler(
                OnProcessStart(target_action=middleware, on_start=[LogInfo(msg="Middleware started."), px4_sitl])
            ),
            RegisterEventHandler(
                OnProcessStart(target_action=px4_sitl, on_start=[LogInfo(msg="PX4 SITL started."), TimerAction(period=15.0, actions=[mission] if run_mission_bool else [])])
            ),
        ]
    else:
        # Hardware mode: start mission after middleware is ready
        actions = [
            *vision_node_actions,
            LogInfo(msg="Vision nodes started."),
            middleware,
            RegisterEventHandler(
                OnProcessStart(
                    target_action=middleware,
                    on_start=[
                        LogInfo(msg="Hardware middleware ready."),
                        TimerAction(period=5.0, actions=[mission] if run_mission_bool else [])
                    ]
                )
            ),
        ]
    return actions

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('px4_path', default_value='~/PX4-Autopilot'),
        OpaqueFunction(function=launch_setup)
    ])
