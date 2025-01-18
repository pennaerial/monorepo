from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    # Adjust these paths according to your setup
    px4_path = os.path.expanduser('~/pennair/PX4-Autopilot')
    qgc_path = os.path.expanduser('~/pennair')  # Folder containing QGroundControl.AppImage
    sae_ws_path = os.path.expanduser('~/pennair/monorepo/controls/sae_2025_ws')
    
    return LaunchDescription([
        # 1. Launch PX4 in standalone mode
        ExecuteProcess(
            cmd=['bash', 'standalone_gazebo_cmd.sh'],
            cwd=px4_path,
            output='screen'
        ),

        ExecuteProcess(
            cmd=['bash', 'standalone_px4_cmd.sh'],
            cwd=px4_path,
            output='screen'
        ),

        # 2. Launch Gazebo
        

        # 3. Start Micro XRCE Agent
        ExecuteProcess(
            cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
            output='screen'
        ),

        # # 4. Launch QGroundControl
        # ExecuteProcess(
        #     cmd=['./QGroundControl.AppImage'],
        #     cwd=qgc_path,
        #     output='screen'
        # ),

        # 5. Bridge the camera feed:
        # Make sure your environment is already sourced before running `ros2 launch`.
        ExecuteProcess(
            cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge', 
                 '/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
            output='screen',
            cwd=sae_ws_path
        ),

        Node(
            output='screen',
            package='uav',
            executable='camera_feed',
            name='temp',
        ),

        # Node(
        #     output='screen',
        #     package='uav',
        #     executable='altitude',
        #     name='altitude',
        # )
    ])