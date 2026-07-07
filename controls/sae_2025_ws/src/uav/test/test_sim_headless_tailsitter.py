#!/usr/bin/env python3
"""Headless sim CI gate: launches the simple_tailsitter UAV mission via
main.launch.py and asserts, over MAVLink, that the vehicle ARMs, transitions
MC -> FW -> MC, goes AIRBORNE, and LANDS again.

See test_sim_headless.py for the launch_testing / teardown rationale and the
pytest.importorskip cross-CI skip behavior -- this file follows the same
pattern, just for the standard_vtol airframe and simple_tailsitter mission.
"""

import os
import threading
import time
import unittest

from ament_index_python.packages import get_package_share_directory
import launch
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
import launch_testing.actions
import pytest

mavutil = pytest.importorskip("pymavlink.mavutil")

MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

TIMEOUT_S = float(os.environ.get("SIM_SMOKE_TIMEOUT", "180"))


@pytest.mark.launch_test
def generate_test_description():
    px4_path = os.environ["PX4_PATH"]
    params_file = os.path.join(
        get_package_share_directory("uav"),
        "launch",
        "ci",
        "launch_params_tailsitter.yaml",
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


class TestTailsitterMission(unittest.TestCase):
    @pytest.mark.live  # requires a running sim peer stack; excluded from CI by `-m "not live"`
    def test_arms_transitions_and_lands(self):
        master = mavutil.mavlink_connection(MAVLINK_ENDPOINT)

        print("waiting for PX4 heartbeat ...", flush=True)
        self.assertIsNotNone(
            master.wait_heartbeat(timeout=60), "no PX4 heartbeat within 60s"
        )
        print(
            f"connected: sys {master.target_system} comp {master.target_component}",
            flush=True,
        )

        # Send GCS heartbeats continuously for the whole flight
        stop = threading.Event()

        def heartbeat_loop():
            while not stop.is_set():
                master.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0,
                    0,
                    0,
                )
                time.sleep(0.5)

        threading.Thread(target=heartbeat_loop, daemon=True).start()

        # Ask PX4 to stream EXTENDED_SYS_STATE so we can read landed_state and vtol_state
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL,
            0,
            mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE,
            200000,
            0,
            0,
            0,
            0,
            0,
        )

        armed = False
        in_air = False
        expected_vtol_states = [
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_FW,
            mavutil.mavlink.MAV_VTOL_STATE_FW,
            mavutil.mavlink.MAV_VTOL_STATE_TRANSITION_TO_MC,
            mavutil.mavlink.MAV_VTOL_STATE_MC,
        ]
        start = time.time()

        try:
            while time.time() - start < TIMEOUT_S:
                msg = master.recv_match(
                    type=["HEARTBEAT", "EXTENDED_SYS_STATE"],
                    blocking=True,
                    timeout=1,
                )
                if msg is None:
                    continue
                elapsed = time.time() - start

                if (
                    msg.get_type() == "HEARTBEAT"
                    and msg.type != mavutil.mavlink.MAV_TYPE_GCS
                ):
                    is_armed = bool(
                        msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED
                    )
                    if is_armed and not armed:
                        armed = True
                        print(f"[{elapsed:6.1f}s] ARMED", flush=True)

                elif msg.get_type() == "EXTENDED_SYS_STATE":
                    if (
                        expected_vtol_states
                        and msg.vtol_state == expected_vtol_states[0]
                    ):
                        reached = expected_vtol_states.pop(0)
                        print(f"[{elapsed:6.1f}s] VTOL_STATE -> {reached}", flush=True)

                    state = msg.landed_state
                    if state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR and not in_air:
                        in_air = True
                        print(f"[{elapsed:6.1f}s] AIRBORNE (takeoff)", flush=True)
                    elif state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND and in_air:
                        self.assertEqual(
                            expected_vtol_states,
                            [],
                            f"landed before all VTOL transitions completed, "
                            f"still pending: {expected_vtol_states}",
                        )
                        print(
                            f"[{elapsed:6.1f}s] LANDED -- mission complete",
                            flush=True,
                        )
                        return

            self.fail(
                f"TIMEOUT after {TIMEOUT_S:.0f}s (armed={armed}, airborne={in_air}, "
                f"pending_vtol_states={expected_vtol_states})"
            )
        finally:
            stop.set()
