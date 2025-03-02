from launch import LaunchDescription
from launch.actions import ExecuteProcess, LogInfo, RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import os

HARDCODE_PATH = False

def generate_launch_description():
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

    px4_path = (find_folder_with_heuristic('PX4-Autopilot', os.path.expanduser('~'))
                if not HARDCODE_PATH else os.path.expanduser('~/PX4-Autopilot'))
    sae_ws_path = os.getcwd()

    # Define the processes
    middleware = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],
        output='screen',
        name='middleware'
    )

    gazebo = ExecuteProcess(
        cmd=['bash', 'standalone_gazebo_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='gazebo'
    )

    px4_sitl = ExecuteProcess(
        cmd=['bash', 'standalone_px4_cmd.sh'],
        cwd=px4_path,
        output='screen',
        name='px4_sitl'
    )

    gz_ros_bridge = ExecuteProcess(
        cmd=['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
             '/camera@sensor_msgs/msg/Image[gz.msgs.Image'],
        output='screen',
        cwd=sae_ws_path,
        name='gz_ros_bridge'
    )

    mission = ExecuteProcess(
        cmd=['ros2', 'run', 'uav', 'mission'],
        output='screen',
        emulate_tty=True,
        name='mission'
    )
    return LaunchDescription([
        middleware,
        RegisterEventHandler(
            OnProcessStart(target_action=middleware, on_start=[gazebo, LogInfo(msg="Middleware started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gazebo, on_start=[px4_sitl, LogInfo(msg="Gazebo started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=px4_sitl, on_start=[gz_ros_bridge, LogInfo(msg="PX4 SITL started.")])
        ),
        RegisterEventHandler(
            OnProcessStart(target_action=gz_ros_bridge, on_start=[mission, LogInfo(msg="gz_ros_bridge started.")])
        )
    ])