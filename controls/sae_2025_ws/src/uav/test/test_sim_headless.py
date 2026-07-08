#!/usr/bin/env python3
"""Launch the basic sim mission and verify arm, takeoff, and landing."""

import os
import unittest

from ament_index_python.packages import get_package_share_directory
from headless_ground_station import HeadlessGroundStation
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest

# Live marker prevents colcon test from running this test in non-sim CI
pytestmark = pytest.mark.live

MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

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
    def test_arms_takes_off_and_lands(self):
        from pymavlink import mavutil

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
                ground_station.wait_armed(deadline=mission_deadline)
                ground_station.wait_airborne(deadline=mission_deadline)
                ground_station.wait_landed(deadline=mission_deadline)
            except TimeoutError as exc:
                self.fail(str(exc))
