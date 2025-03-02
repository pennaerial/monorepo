from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

HARDCODE_PATH = False

def generate_launch_description():
    # Adjust these paths according to your setup
    # look for PX4-Autopilot folder in your home directory

    def find_folder(folder_name, search_path):
        for root, dirs, files in os.walk(search_path):
            if folder_name in dirs:
                return os.path.join(root, folder_name)
        return None

    def find_folder_with_heuristic(folder_name, home_dir, keywords=('penn', 'air')):
        immediate_dirs = [d for d in os.listdir(home_dir) if os.path.isdir(os.path.join(home_dir, d))]
        if folder_name in immediate_dirs:
            return os.path.join(home_dir, folder_name)

        for d in immediate_dirs:
            if any(kw.lower() in d.lower() for kw in keywords):
                result = find_folder(folder_name, os.path.join(home_dir, d))
                if result:
                    return result

        return find_folder(folder_name, home_dir)

    px4_path = find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~')) if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot')
    # qgc_path = os.path.expanduser('~/pennair')  # Folder containing QGroundControl.AppImage
    sae_ws_path = os.getcwd()

    px4_sitl = ExecuteProcess(
            cmd=['bash', 'standalone_px4_cmd.sh'],
            cwd=px4_path,
            output='screen',
            name='px4_sitl'
        )
    
    gazebo = ExecuteProcess(
            cmd=['bash', 'standalone_gazebo_cmd.sh'],
            cwd=px4_path,
            output='screen',
            name='gazebo'
        )
    
    middleware = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen',
            name='middleware'
        )
    
    gz_ros_bridge = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
                 '/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
            cwd=sae_ws_path,
            name='gz_ros_bridge'
        )
    
    # camera_feed = Node(
    #         output='screen',
    #         package='uav',
    #         executable='camera_feed',
    #         name='temp',
    #     )
    
    # movement = Node(
    #         output='screen',
    #         package='uav',
    #         executable='global_position_offboard_control',
    #         name='global_position_offboard',
    #     )
    
    # camera_feed = Node(
    #     output='screen',
    #     package='uav',
    #     executable='vision_pipeline',
    #     name='vision_pipeline'
    # )

    # movement = Node(
    #     output='screen',
    #     package='uav',
    #     executable='flight',
    #     name='flight'
    # )

    movement = Node(
        output='screen',
        package='uav',
        executable='mission',
        name='mission'
    )
    
    return LaunchDescription([
        middleware,
        TimerAction(period=3.0, actions=[gazebo]),  # Start Gazebo after 5 seconds
        TimerAction(period=6.0, actions=[px4_sitl]),  # Start PX4 after 10 seconds
        TimerAction(period=9.0, actions=[gz_ros_bridge]),  # Start Bridge after 15 seconds
        # TimerAction(period=12.0, actions=[camera_feed]),  # Start Control Node last
        TimerAction(period=15.0, actions=[movement]),  # Start Control Node last
    ])