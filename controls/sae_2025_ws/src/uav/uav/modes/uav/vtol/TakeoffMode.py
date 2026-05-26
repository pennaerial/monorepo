import time
from typing import Literal

import numpy as np
from px4_msgs.msg import VehicleStatus, VtolVehicleStatus
from rclpy.node import Node

from uav.vehicles.UAV import get_nav_state_str
from uav.vehicles.VTOL import VTOL

from ...Mode import Mode


class TakeoffMode(Mode[VTOL]):
    """
    A VTOL takeoff mode that supports both vertical and runway-style launches.
    """

    transition_labels = ("complete",)

    def __init__(
        self,
        node: Node,
        vehicle: VTOL,
        takeoff_type: Literal["vertical", "horizontal"] = "vertical",
        fw_tko_pitch: float = float("nan"),
        yaw: float = float("nan"),
        latitude: float = float("nan"),
        longitude: float = float("nan"),
        altitude: float = float("nan"),
    ):
        super().__init__(node, vehicle)
        self.takeoff_type = takeoff_type.lower()
        self.takeoff_commanded = False
        self.takeoff_elapsed_time = (
            0.0  # PX4-Autopilot on ARM has a race condition when changing vehicle state
        )
        self.fw_takeoff_phase = 0
        self.fw_tko_pitch = fw_tko_pitch
        self.yaw = yaw
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def on_update(self, time_delta: float) -> None:
        self.takeoff_elapsed_time += time_delta

        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            self.log("Waiting for position data...")
            return

        if self.takeoff_commanded:
            self.log(f"{self.takeoff_type.capitalize()} takeoff in progress.")
        else:
            self.takeoff_elapsed_time = 0.0
            if self.takeoff_type == "horizontal":
                self._run_horizontal_takeoff()
            else:
                self.log("Attempting vertical VTOL takeoff")
                self.vehicle.takeoff()
                self.takeoff_commanded = True

        if (
            self.takeoff_elapsed_time >= 1.0
            and self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            self.log("Takeoff complete. Engaging offboard mode.")
            self.vehicle.engage_offboard_mode()

    def _run_horizontal_takeoff(self) -> None:
        if self.vehicle.vtol_vehicle_status is None:
            self.node.get_logger().info("FW takeoff: Vehicle status not available yet.")
            return

        if (
            self.vehicle.vtol_vehicle_status.vehicle_vtol_state
            == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
        ):
            self.vehicle.vtol_transition_to("FW", immediate=False)
            self.node.get_logger().info(
                "FW takeoff Step 1: requested VTOL transition to FW."
            )
            return

        if (
            self.vehicle.vtol_vehicle_status.vehicle_vtol_state
            == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW
        ):
            self.node.get_logger().info(
                "FW takeoff Step 1a: transition to FW in progress."
            )
            return

        if (
            self.vehicle.vtol_vehicle_status.vehicle_vtol_state
            == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW
        ):
            self._finish_runway_takeoff()
            return

        if (
            self.vehicle.vtol_vehicle_status.vehicle_vtol_state
            == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC
        ):
            self.node.get_logger().error(
                "FW takeoff: Transition to MC in progress during horizontal takeoff."
            )
            return

        self.node.get_logger().warn("FW takeoff Step 0: unknown vehicle state.")

    def _finish_runway_takeoff(self) -> None:
        if self.fw_takeoff_phase == 0:
            self.node.get_logger().info(
                "FW takeoff Step 2: transition complete. Starting disarm->takeoff->arm sequence."
            )

            global_position = self.vehicle.global_position
            if global_position is None:
                self.node.get_logger().info(
                    "FW takeoff: Waiting for global position before runway takeoff."
                )
                return

            lat = global_position.lat
            lon = global_position.lon
            alt = global_position.alt
            self.node.get_logger().info(f"Current GPS: {lat}, {lon}, {alt}")

            if (
                np.isnan(self.latitude)
                or np.isnan(self.longitude)
                or np.isnan(self.altitude)
            ):
                self.node.get_logger().info("Takeoff Destination GPS: Auto Calculated")
            else:
                self.node.get_logger().info(
                    f"Takeoff Destination GPS: {self.latitude}, {self.longitude}, {self.altitude}"
                )

            self.vehicle.disarm(force=True)
            self.node.get_logger().info(
                "FW takeoff Step 2a: force-disarmed to reset land detector."
            )
            time.sleep(0.5)

            self.vehicle.fixed_wing_takeoff(
                self.fw_tko_pitch,
                self.yaw,
                self.latitude,
                self.longitude,
                self.altitude,
            )
            self.node.get_logger().info(
                "FW takeoff Step 2b: takeoff command sent while disarmed."
            )
            time.sleep(0.1)

            self.vehicle.arm()
            self.node.get_logger().info("FW takeoff Step 2c: re-arm command sent.")
            self.fw_takeoff_phase = 1
            time.sleep(1)

        if self.fw_takeoff_phase != 1:
            return

        if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
            self.node.get_logger().info(
                "FW takeoff Step 3: In AUTO_TAKEOFF. Runway takeoff running."
            )
            self.takeoff_commanded = True
            return

        self.node.get_logger().info(
            "FW takeoff: Waiting for AUTO_TAKEOFF nav state "
            f"(current: {get_nav_state_str(self.vehicle.nav_state)})."
        )

    def check_status(self) -> str:
        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            return "continue"

        if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log("Takeoff complete, in offboard mode.")
            return "complete"
        return "continue"
