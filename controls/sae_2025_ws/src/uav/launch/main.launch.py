from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
import os

def generate_launch_description():
    # Adjust these paths according to your setup
    px4_path = os.path.expanduser('~/pennair/PX4-Autopilot')
    # qgc_path = os.path.expanduser('~/pennair')  # Folder containing QGroundControl.AppImage
    sae_ws_path = os.path.expanduser('~/pennair/monorepo/controls/sae_2025_ws')

    px4_sitl = ExecuteProcess(
            cmd=['bash', 'standalone_px4_cmd.sh'],
            cwd=px4_path,
            output='screen'
        )
    
    gazebo = ExecuteProcess(
            cmd=['bash', 'standalone_gazebo_cmd.sh'],
            cwd=px4_path,
            output='screen'
        )
    
    middleware = ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        )
    
    gz_ros_bridge = ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
                 '/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
            cwd=sae_ws_path
        )
    
    camera_feed = Node(
            output='screen',
            package='uav',
            executable='camera_feed',
            name='temp',
        )
    
    movement = Node(
            output='screen',
            package='uav',
            executable='global_position_offboard_control',
            name='global_position_offboard',
        )
    
    return LaunchDescription([
        middleware,
        TimerAction(period=3.0, actions=[gazebo]),  # Start Gazebo after 5 seconds
        TimerAction(period=6.0, actions=[px4_sitl]),  # Start PX4 after 10 seconds
        TimerAction(period=9.0, actions=[gz_ros_bridge]),  # Start Bridge after 15 seconds
        TimerAction(period=12.0, actions=[camera_feed]),  # Start Control Node last
        TimerAction(period=20.0, actions=[movement]),  # Start Control Node last
    ])