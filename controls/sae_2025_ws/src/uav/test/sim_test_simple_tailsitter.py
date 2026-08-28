#!/usr/bin/env python3
"""Launch the tailsitter sim mission and verify the VTOL transition sequence."""

import os
import unittest

from headless_ground_station import HeadlessGroundStation
import launch
import launch_testing.actions
import pytest

from vehicle_common.launch_utils import include_launch

# Live marker prevents colcon test from running this test in non-sim CI
pytestmark = pytest.mark.live

MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

TIMEOUT_S = float(os.environ.get("SIM_SMOKE_TIMEOUT", "180"))


@pytest.mark.launch_test
@pytest.mark.sim_test
def generate_test_description():
    uav_sitl = include_launch(
        pkg="uav",
        launch_file="uav_sitl.launch.py",
        launch_arguments={"airframe": "quadtailsitter", "mission": "simple_tailsitter"},
    )
    return launch.LaunchDescription(
        [
            uav_sitl,
            launch_testing.actions.ReadyToTest(),
        ]
    )


class TestTailsitterMission(unittest.TestCase):
    def test_arms_transitions_and_lands(self):
        from pymavlink import mavutil

        expected_vtol_states = [
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC,
            mavutil.mavlink.MAV_VTOL_STATE_MC,
        ]

        with HeadlessGroundStation(MAVLINK_ENDPOINT, mavutil_module=mavutil) as ground_station:
            self.assertIsNotNone(
                ground_station.connect(timeout_s=60),
                "no PX4 heartbeat within 60s",
            )
            ground_station.start_heartbeats()
            ground_station.request_extended_sys_state()
            ground_station.expect_vtol_states(expected_vtol_states)
            mission_deadline = ground_station.deadline(TIMEOUT_S)
            try:
                ground_station.wait_armed(deadline=mission_deadline)
                ground_station.wait_airborne(deadline=mission_deadline)
                ground_station.wait_landed(deadline=mission_deadline)
            except TimeoutError as exc:
                self.fail(str(exc))
            pending_vtol_states = ground_station.pending_vtol_states()
            self.assertEqual(
                pending_vtol_states,
                [],
                f"landed before all VTOL transitions completed, still pending: {pending_vtol_states}",
            )
