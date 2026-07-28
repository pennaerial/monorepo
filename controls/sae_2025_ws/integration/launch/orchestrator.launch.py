"""Launches the orchestrator (the "computer") that phone/web clients talk to.

The orchestrator lives outside the colcon `src/` tree (it's a Node/TS
process, not a ROS package), but it's a real ROS2 participant (see the
integration rewrite plan's `rclnodejs` decision for mission RPC), so it's
started the same way as everything else in this fleet: `ros2 launch`.

Usage:
    ros2 launch orchestrator.launch.py
    ros2 launch orchestrator.launch.py serve_frontend:=true
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

_INTEGRATION_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))


def generate_launch_description() -> LaunchDescription:
    serve_frontend_arg = DeclareLaunchArgument(
        "serve_frontend",
        default_value="false",
        description="Also start the web frontend's dev server alongside the orchestrator.",
    )
    serve_frontend = LaunchConfiguration("serve_frontend")

    orchestrator_process = ExecuteProcess(
        cmd=["node", "packages/orchestrator/dist/index.js"],
        cwd=_INTEGRATION_ROOT,
        output="screen",
    )

    frontend_process = ExecuteProcess(
        cmd=["npm", "run", "dev", "--workspace", "@pennair/web"],
        cwd=_INTEGRATION_ROOT,
        output="screen",
        condition=IfCondition(serve_frontend),
    )

    return LaunchDescription(
        [
            serve_frontend_arg,
            orchestrator_process,
            frontend_process,
        ]
    )
