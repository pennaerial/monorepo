#!/usr/bin/env python3
"""Headless sim smoke test.

Acts as a minimal MAVLink ground station for a running PX4 SITL instance and
asserts that the UAV completes a basic flight: it ARMS, goes AIRBORNE (takeoff),
and LANDS again. Exits 0 on success and non-zero otherwise, so it can be used
directly as a CI gate.

It talks to PX4 purely over MAVLink (no ROS / px4_msgs dependency) so it stays
decoupled from the workspace build and can run anywhere PX4 SITL is reachable.
"""

import os
import sys
import threading
import time

from pymavlink import mavutil

# PX4 SITL streams MAVLink to UDP 14550 (the port a ground station such as
# QGroundControl listens on). We bind there and behave as that ground station.
MAVLINK_ENDPOINT = "udpin:0.0.0.0:14550"

# Whole-mission time budget. Overridable for slower/faster environments.
TIMEOUT_S = float(os.environ.get("SIM_SMOKE_TIMEOUT", "180"))


def main() -> int:
    master = mavutil.mavlink_connection(MAVLINK_ENDPOINT)

    print("waiting for PX4 heartbeat ...", flush=True)
    if master.wait_heartbeat(timeout=60) is None:
        print("ERROR: no PX4 heartbeat within 60s", file=sys.stderr)
        return 1
    print(
        f"connected: sys {master.target_system} comp {master.target_component}",
        flush=True,
    )

    # PX4 fails preflight (and refuses to arm) without a connected GCS. Send GCS
    # heartbeats continuously for the WHOLE flight -- dropping them mid-mission
    # trips PX4's GCS-loss failsafe.
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

    # Ask PX4 to stream EXTENDED_SYS_STATE so we can read landed_state
    # (on-ground / in-air) to detect takeoff and landing.
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
                state = msg.landed_state
                if state == mavutil.mavlink.MAV_LANDED_STATE_IN_AIR and not in_air:
                    in_air = True
                    print(f"[{elapsed:6.1f}s] AIRBORNE (takeoff)", flush=True)
                elif state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND and in_air:
                    print(f"[{elapsed:6.1f}s] LANDED -- mission complete", flush=True)
                    return 0

        print(
            f"TIMEOUT after {TIMEOUT_S:.0f}s (armed={armed}, airborne={in_air})",
            file=sys.stderr,
        )
        return 1
    finally:
        stop.set()


if __name__ == "__main__":
    sys.exit(main())
