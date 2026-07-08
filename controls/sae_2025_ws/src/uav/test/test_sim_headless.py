#!/usr/bin/env python3
"""Headless sim CI gate: launches the basic UAV mission via main.launch.py and
asserts, over MAVLink, that the vehicle ARMs, goes AIRBORNE, and LANDS again.

Written as a launch_testing test (rather than a bash script driving a bare
python process) so process teardown -- Gazebo, PX4 SITL, the MicroXRCEAgent
bridge, and whatever ROS nodes main.launch.py starts -- is handled by the
LaunchService itself. It tears down every process it started, transitively,
with no per-process-name pkill list to maintain.

Uses launch_params_basic.yaml rather than the default launch_params.yaml so
CI stays decoupled from mission-config edits made for real flight tuning.

Guarded by the pytest.importorskip below: pymavlink is only installed in the
sim CI image (see .github/ci/ros-jazzy-sim-ci/Dockerfile), so this file is
skipped -- not collected as an error -- when the plain (non-sim) CI runs
`colcon test` over this same test/ directory.
"""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from headless_ground_station import HeadlessGroundStation
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

# PX4 SITL streams MAVLink to UDP 14550 (the port a ground station such as
# QGroundControl listens on). We bind there and behave as that ground station.
MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

# Whole-mission time budget. Overridable for slower/faster environments.
TIMEOUT_S = float(os.environ.get("SIM_SMOKE_TIMEOUT", "180"))


@pytest.mark.launch_test
def generate_test_description():
    px4_path = os.environ["PX4_PATH"]
    params_file = os.path.join(
        get_package_share_directory("uav"), "launch", "ci", "launch_params_basic.yaml"
    )
    main_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory("uav"), "launch", "main.launch.py")
        ),
        launch_arguments={
            "px4_path": px4_path,
            "params_file": params_file,
        }.items(),
    )
    return launch.LaunchDescription(
        [
            main_launch,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestBasicMission(unittest.TestCase):
    @pytest.mark.live  # requires a running sim peer stack; excluded from CI by `-m "not live"`
    def test_arms_takes_off_and_lands(self):
        with HeadlessGroundStation(
            MAVLINK_ENDPOINT, mavutil_module=mavutil
        ) as ground_station:
            self.assertIsNotNone(
                ground_station.connect(timeout_s=60), "no PX4 heartbeat within 60s"
            )
            ground_station.start_heartbeats()
            ground_station.request_extended_sys_state()
            mission_deadline = ground_station.deadline(TIMEOUT_S)
            try:
                ground_station.wait_airborne(deadline=mission_deadline)
                ground_station.wait_landed(deadline=mission_deadline)
            except TimeoutError as exc:
                self.fail(str(exc))
