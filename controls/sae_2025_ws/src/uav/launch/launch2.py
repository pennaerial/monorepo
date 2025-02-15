from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
import os

def generate_launch_description():
    return LaunchDescription([
        # 1. Launch PX4 in standalone mod

        Node(
            output='screen',
            package='uav',
            executable='altitude',
            name='altitude',
        ),
    ])