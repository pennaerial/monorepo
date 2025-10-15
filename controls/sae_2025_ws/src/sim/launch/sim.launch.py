#!/usr/bin/env python3

import os
import yaml
from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler, TimerAction, OpaqueFunction
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
from sim.in_house.worldgen import generate_world


class CourseParams:
    """Data class to hold course parameters for worldgen.py"""
    def __init__(self, dlz, uav, num_hoops, max_dist, width=None, height=None, 
                 asc_start_height=None, des_start_height=None, spacing=None, ip_file=None, op_file=None):
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops
        self.max_dist = max_dist
        self.width = width
        self.height = height
        self.asc_start_height = asc_start_height
        self.des_start_height = des_start_height
        self.spacing = spacing
        self.ip_file = ip_file
        self.op_file = op_file


def find_folder_with_heuristic(folder_name, home_dir, keywords=('penn', 'air')):
    """Find folder using heuristic search."""
    immediate_dirs = [d for d in os.listdir(home_dir) if os.path.isdir(os.path.join(home_dir, d))]
    if folder_name in immediate_dirs:
        return os.path.join(home_dir, folder_name)
    for d in immediate_dirs:
        if any(kw.lower() in d.lower() for kw in keywords):
            result = find_folder(folder_name, os.path.join(home_dir, d))
            if result:
                return result
    return find_folder(folder_name, home_dir)


def find_folder(folder_name, search_path):
    """Find folder recursively."""
    for root, dirs, files in os.walk(search_path):
        if folder_name in dirs:
            return os.path.join(root, folder_name)
    return None


def load_launch_params():
    """Load parameters from launch_params.yaml file."""
    launch_file_dir = os.path.dirname(os.path.abspath(__file__))
    params_file = os.path.join(launch_file_dir, 'launch_params.yaml')
    
    try:
        with open(params_file, 'r') as f:
            params = yaml.safe_load(f)
        return params
    except FileNotFoundError:
        print(f"Warning: {params_file} not found, using default parameters")
        return get_default_params()


def get_default_params():
    """Default parameters if YAML file is not found."""
    return {
        'competition': {'type': 'in_house', 'name': 'slalom'},
        'course': {
            'type': 'slalom',
            'slalom': {
                'dlz': [10, 5, 0],
                'uav': [0, 0, 0],
                'num_hoops': 5,
                'max_dist': 10,
                'width': 3,
                'height': 4
            },
            'ascent': {
                'dlz': [10, 5, 0],
                'uav': [0, 0, 0],
                'num_hoops': 4,
                'max_dist': 8,
                'start_height': 2
            },
            'descent': {
                'dlz': [10, 5, 0],
                'uav': [0, 0, 0],
                'num_hoops': 4,
                'max_dist': 8,
                'start_height': 2
            }
        },
        'simulation': {
            'world_name': 'custom',
            'enable_scoring': True,
            'uav_model': 'gz_x500_mono_cam_down',
            'position_poll_rate': 10.0,
            'scoring_rate': 5.0,
            'scoring': {
                'hoop_tolerance': 1.5,
                'max_flight_time': 300,
                'points_per_hoop': 10,
                'penalty_per_second': 0.1
            }
        },
        'ros2': {
            'middleware_port': 8888,
            'enable_camera_bridge': True,
            'topics': {
                'camera': '/camera',
                'camera_info': '/camera_info',
                'vehicle_position': '/fmu/out/vehicle_local_position',
                'vehicle_attitude': '/fmu/out/vehicle_attitude',
                'scoring_results': '/scoring/results'
            }
        }
    }


def launch_setup(context, *args, **kwargs):
    """Setup launch configuration."""
    
    # Load parameters from YAML file
    params = load_launch_params()
    
    # Use YAML values directly
    competition_type = params['competition']['type']
    course_type = params['course']['type']
    enable_scoring = params['simulation'].get('enable_scoring', True)
    enable_camera_bridge = params['ros2'].get('enable_camera_bridge', True)
    num_hoops = params['course'][course_type]['num_hoops']
    max_dist = params['course'][course_type]['max_dist']
    
    # Extract course parameters
    course_params_raw = params['course'][course_type].copy()
    course_params_raw['num_hoops'] = num_hoops
    course_params_raw['max_dist'] = max_dist
    
    # Convert lists to tuples for compatibility
    course_params_dict = {
        'dlz': tuple(course_params_raw['dlz']),
        'uav': tuple(course_params_raw['uav']),
        'num_hoops': course_params_raw['num_hoops'],
        'max_dist': course_params_raw['max_dist']
    }
    
    # Add course-specific parameters
    if course_type == 'slalom':
        course_params_dict['width'] = course_params_raw['width']
        course_params_dict['height'] = course_params_raw['height']
    elif course_type in ['ascent', 'descent']:
        course_params_dict['start_height'] = course_params_raw['start_height']
    elif course_type == 'straight':
        course_params_dict['height'] = course_params_raw['height']
        course_params_dict['spacing'] = course_params_raw['spacing']
    
    # Extract simulation parameters
    sim_params = params['simulation']
    ros_params = params['ros2']
    
    print(f"Launching {params['competition']['type']} competition: {params['competition']['name']}")
    print(f"Course type: {course_type}")
    print(f"Course parameters: {course_params_dict}")
    
    # Generate world file using worldgen.py
    world_name = f"{params['competition']['type']}_{params['competition']['name']}"
    
    # Create CourseParams object for worldgen.py
    course_params_obj = CourseParams(
        dlz=course_params_dict['dlz'],
        uav=course_params_dict['uav'],
        num_hoops=course_params_dict['num_hoops'],
        max_dist=course_params_dict['max_dist'],
        width=course_params_dict.get('width'),
        height=course_params_dict.get('height'),
        asc_start_height=course_params_dict.get('start_height'),
        des_start_height=course_params_dict.get('start_height'),
        spacing=course_params_dict.get('spacing'),
        # CHANGE HARDCODED PATH LATER, BASE ON PARAMs
        ip_file="/home/avaniko/penn-air/monorepo/controls/sae_2025_ws/src/sim/sim/in_house/worlds/template.sdf", 
        op_file=os.path.expanduser(f"~/.simulation-gazebo/worlds/{world_name}.sdf")
    )
    
    # Ensure output directory exists
    os.makedirs(os.path.dirname(course_params_obj.op_file), exist_ok=True)
    
    # Generate world using existing worldgen.py
    try:
        generate_world(course_type, course_params_obj)
        print(f"Generated world file: {course_params_obj.op_file}")
    except Exception as e:
        print(f"Error generating world: {e}")
        raise
    
    # Find required paths
    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
    if not px4_path:
        raise RuntimeError("PX4-Autopilot directory not found")
    
    sae_ws_path = os.getcwd()
    
    # Define the middleware process
    print("start middleware")
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', str(ros_params['middleware_port'])],
        output='screen',
        name='middleware'
    )
    
    # Define the Gazebo process
    print("start gazebo")
    gazebo = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', f'--world={world_name}'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )
    
    # Define the PX4 SITL process
    print("start px4_sitl")
    px4_sitl = ExecuteProcess(
        cmd=['bash', '-c', f'PX4_GZ_STANDALONE=1 PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL={sim_params["uav_model"]} PX4_GZ_WORLD={world_name} ./build/px4_sitl_default/bin/px4'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )
    
    # Calculate hoop positions for scoring node
    hoop_positions = []
    if course_type == 'straight':
        # Calculate hoop positions for straight course
        num_hoops = course_params_dict['num_hoops']
        height = course_params_dict['height']
        spacing = course_params_dict['spacing']
        
        for i in range(num_hoops):
            x = 2.0 + (i * spacing)  # Start at x=2, then spacing apart
            y = 0.0
            z = height
            hoop_positions.extend([x, y, z])  # Flatten for ROS2 parameter array
    elif course_type == 'slalom':
        # Calculate hoop positions for slalom course
        num_hoops = course_params_dict['num_hoops']
        height = course_params_dict['height']
        width = course_params_dict['width']
        
        for i in range(num_hoops):
            x = 2.0 + (i * 2.0)  # 2m spacing
            y = (i % 2) * width - width/2  # Alternating left/right
            z = height
            hoop_positions.extend([x, y, z])
    elif course_type in ['ascent', 'descent']:
        # Calculate hoop positions for ascent/descent course
        num_hoops = course_params_dict['num_hoops']
        start_height = course_params_dict.get('start_height', 2.0)
        
        for i in range(num_hoops):
            x = 2.0 + (i * 2.0)  # 2m spacing
            y = 0.0
            if course_type == 'ascent':
                z = start_height + (i * 0.5)  # Increasing height
            else:  # descent
                z = start_height - (i * 0.5)  # Decreasing height
            hoop_positions.extend([x, y, z])
    
    print(f"Calculated {len(hoop_positions)//3} hoop positions for scoring: {hoop_positions}")
    
    # Define the scoring node (only if enabled)
    scoring_node = None
    if sim_params.get('enable_scoring', True):
        scoring_node = Node(
            package='sim',
            executable='scoring_node',
            name='scoring_node',
            output='screen',
            parameters=[{
                'competition_type': params['competition']['type'],
                'competition_name': params['competition']['name'],
                'course_type': course_type,
                'hoop_positions': hoop_positions,  # Add hoop positions
                'position_poll_rate': sim_params.get('position_poll_rate', 10.0),
                'scoring_rate': sim_params.get('scoring_rate', 5.0),
                'hoop_tolerance': sim_params.get('scoring', {}).get('hoop_tolerance', 1.5),
                'max_flight_time': sim_params.get('scoring', {}).get('max_flight_time', 300),
                'points_per_hoop': sim_params.get('scoring', {}).get('points_per_hoop', 10)
            }]
        )
    
    # Define the ROS-Gazebo bridge for camera topics (only if enabled)
    gz_ros_bridge_camera = None
    gz_ros_bridge_camera_info = None
    
    if ros_params.get('enable_camera_bridge', True):
        camera_topic = ros_params['topics']['camera']
        camera_info_topic = ros_params['topics']['camera_info']
        
        print("start gz_ros_bridge_camera")
        gz_ros_bridge_camera = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/image@sensor_msgs/msg/Image[gz.msgs.Image',
                '--ros-args', '--remap', f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/image:={camera_topic}'],
            output='screen',
            cwd=sae_ws_path,
            name='gz_ros_bridge_camera'
        )
        
        gz_ros_bridge_camera_info = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
                f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
                '--ros-args', '--remap', f'/world/{world_name}/model/{sim_params["uav_model"]}_0/link/camera_link/sensor/imager/camera_info:={camera_info_topic}'],
            output='screen',
            cwd=sae_ws_path,
            name='gz_ros_bridge_camera_info'
        )
    
    # Delayed scoring node start (only if scoring is enabled)
    delayed_scoring = None
    if scoring_node is not None:
        delayed_scoring = TimerAction(
            period=10.0,
            actions=[scoring_node]
        )
    
    # Build action list based on enabled features
    bridge_actions = []
    if gz_ros_bridge_camera is not None:
        bridge_actions.append(gz_ros_bridge_camera)
    if gz_ros_bridge_camera_info is not None:
        bridge_actions.append(gz_ros_bridge_camera_info)
    
    # Add delayed scoring if enabled
    if delayed_scoring is not None:
        bridge_actions.append(delayed_scoring)
    
    # Build and return the complete list of actions
    return [
        middleware,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[gazebo, LogInfo(msg="Middleware started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gazebo, on_start=[px4_sitl, LogInfo(msg="Gazebo started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=px4_sitl, on_start=bridge_actions + [LogInfo(msg="PX4 SITL started.")])
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])