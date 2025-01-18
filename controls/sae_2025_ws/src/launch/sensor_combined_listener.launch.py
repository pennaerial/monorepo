#!/usr/bin/env python

################################################################################
#
# Copyright (c) 2018-2022, PX4 Development Team. All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its contributors
# may be used to endorse or promote products derived from this software without
# specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
#
################################################################################

"""
Example to launch a sensor_combined listener node.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction

def generate_launch_description():


    # offboard_control_node = Node(
    #     package='px4_ros_com',
    #     executable='offboard_control.py',
    #     name='offboard',
    #     shell=True,
    #     output='screen'
    # )

    middleware_agent = ExecuteProcess(
        cmd=['MicroXRCEAgent', 'udp4', '-p', '8888'],  # Start the middleware agent
        name='micro_xrce_agent',
        output='screen'
    )


    px4_sitl = ExecuteProcess(
        cmd=['./build/px4_sitl_default/bin/px4'],
        cwd='/home/kaitianchao/PennAir/PX4-Autopilot',  # Adjust this to the location of your PX4 repo
        name='px4_sitl',
        output='screen',
        additional_env={
            'PX4_GZ_STANDALONE': '1', 
            'PX4_SYS_AUTOSTART': '4001',
            'PX4_SIM_MODEL': 'gz_x500_mono_cam_down',
            'PX4_GZ_WORLD': 'custom'
        }
    )

    # Gazebo simulation process
    gazebo_process = ExecuteProcess(
        cmd=['python3', 'Tools/simulation/gz/simulation-gazebo', '--world=custom'],
        cwd='/home/kaitianchao/PennAir/PX4-Autopilot',  # Adjust this to the location of your PX4 repo
        name='gazebo_process',
        output='screen'
    )

    global_position_offboard_control_node = Node(
        package='px4_ros_com',
        executable='global_position_offboard_control.py',
        name='global_position_offboard',
        shell=True,
        output='screen'
    )

    # ROS2-Gazebo bridge node
    bridge_node = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        arguments=[
            '/diff_drive/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
            '/diff_drive/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/diff_drive/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
        ]
    )



    # local_control_node = Node(
    #     package='px4_ros_com',
    #     executable='local_offboard_control.py',
    #     name='local_offboard',
    #     shell=True,
    #     output='screen'
    # )

    # velocity_offboard_control_node = Node(
    #     package='px4_ros_com',
    #     executable='velocity_offboard_control.py',
    #     name='velocity_offboard',
    #     shell=True,
    #     output='screen'
    # )


    # return LaunchDescription([
    #     #micro_ros_agent,
    #     # sensor_combined_listener_node,
    #     middleware_agent,
    #     gazebo_process,
    #     bridge_node,
    #     px4_sitl,
    #     global_position_offboard_control_node
    #     # velocity_offboard_control_node
    # ])
    return LaunchDescription([
        middleware_agent,
        # px4_sitl,
        # gazebo_process,
        # bridge_node,
        # global_position_offboard_control_node
        TimerAction(period=5.0, actions=[gazebo_process]),  # Start Gazebo after 5 seconds
        TimerAction(period=10.0, actions=[px4_sitl]),  # Start PX4 after 10 seconds
        TimerAction(period=15.0, actions=[bridge_node]),  # Start Bridge after 15 seconds
        TimerAction(period=20.0, actions=[global_position_offboard_control_node]),  # Start Control Node last
    ])
