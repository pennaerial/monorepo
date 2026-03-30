import time

import numpy as np
from px4_msgs.msg import VehicleStatus, VtolVehicleStatus
from rclpy.node import Node

from uav.vehicles.UAV import UAV, get_nav_state_str
from uav.vehicles.VTOL import VTOL

from ..Mode import Mode


class TakeoffMode(Mode):
    """
    A mode for takeoff (vertical or horizontal).
    Works for both multicopters and VTOLs.
    - Multicopters: always vertical takeoff.
    - VTOLs: vertical or horizontal (fixed-wing style) based on takeoff_type param.
    """

    mission_target = "uav"

    def __init__(
        self,
        node: Node,
        vehicle: UAV,
        takeoff_type: str = "vertical",
        fw_tko_pitch: float = float("nan"),
        yaw: float = float("nan"),
        latitude: float = float("nan"),
        longitude: float = float("nan"),
        altitude: float = float("nan"),
    ):
        """
        Initialize the TakeoffMode.

        Args:
            node (Node): The ROS 2 node.
            vehicle (UAV): The UAV instance.
            takeoff_type (str): 'vertical' or 'horizontal'. Default 'vertical'.
                Horizontal is only valid for VTOLs.
        """
        super().__init__(node, vehicle)
        self.takeoff_type = takeoff_type.lower()
        self.takeoff_commanded = False  # For vertical: only call takeoff() once
        self.takeoff_elapsed_time = (
            0.0  # PX4-Autopilot on ARM has a race condition when changing vehicle state
        )
        self.fw_takeoff_phase = 0  # state machine phase for FW takeoff
        self.fw_tko_pitch = fw_tko_pitch
        self.yaw = yaw
        self.latitude = latitude
        self.longitude = longitude
        self.altitude = altitude

    def on_update(self, time_delta: float) -> None:
        """
        Update the mode
        """

        # Increment elapsed time by the time delta
        self.takeoff_elapsed_time += time_delta

        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            self.log("Waiting for position data...")
            return

        # Takeoff command already sent
        if self.takeoff_commanded:
            self.log(f"{self.takeoff_type.capitalize()} takeoff in progress.")

        # Need to send takeoff command
        else:
            self.takeoff_elapsed_time = 0.0  # time reset as a result of time_delta counting from beginning of launch
            if self.takeoff_type == "horizontal":
                if not self.vehicle.is_vtol or not isinstance(self.vehicle, VTOL):
                    self.node.get_logger().error(
                        "Horizontal takeoff only valid for VTOL - cannot proceed."
                    )
                    return

                if self.vehicle.vtol_vehicle_status is None:
                    self.node.get_logger().info(
                        "FW takeoff: Vehicle status not available yet."
                    )
                    return

                elif (
                    self.vehicle.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
                ):
                    self.vehicle.vtol_transition_to("FW", immediate=False)
                    self.node.get_logger().info(
                        "FW takeoff Step 1: requested VTOL transition to FW."
                    )
                    return

                elif (
                    self.vehicle.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW
                ):
                    self.node.get_logger().info(
                        "FW takeoff Step 1a: transition to FW in progress."
                    )
                    return

                elif (
                    self.vehicle.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW
                ):
                    # FIXED WING TAKEOFF SEQUENCE
                    # After the ground transition to FW, PX4's land detector falsely reports
                    # "not landed." If we send NAV_TAKEOFF in this state, Navigator downgrades
                    # SETPOINT_TYPE_TAKEOFF to SETPOINT_TYPE_POSITION, causing the vehicle to
                    # orbit instead of performing a runway takeoff. The workaround is to briefly
                    # disarm (which resets the land detector to landed=true), send NAV_TAKEOFF
                    # while disarmed, then re-arm. This mirrors PX4's own `commander takeoff`
                    # which sends NAV_TAKEOFF before arming.
                    #
                    # This must be done atomically (single call with sleeps) because ModeManager
                    # auto-arms the vehicle on every spin cycle when it detects disarmed state.
                    if self.fw_takeoff_phase == 0:
                        self.node.get_logger().info(
                            "FW takeoff Step 2: transition complete. Starting disarm->takeoff->arm sequence."
                        )

                        lat = self.vehicle.global_position.lat
                        lon = self.vehicle.global_position.lon
                        alt = self.vehicle.global_position.alt
                        self.node.get_logger().info(f"Current GPS: {lat}, {lon}, {alt}")

                        if (
                            np.isnan(self.latitude)
                            or np.isnan(self.longitude)
                            or np.isnan(self.altitude)
                        ):
                            self.node.get_logger().info(
                                "Takeoff Destination GPS: Auto Calculated"
                            )
                        else:
                            self.node.get_logger().info(
                                f"Takeoff Destination GPS: {self.latitude}, {self.longitude}, {self.altitude}"
                            )

                        # disarm command (needs to be force disarm to reset land detector)
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
                        self.node.get_logger().info(
                            "FW takeoff Step 2c: re-arm command sent."
                        )
                        self.fw_takeoff_phase = 1
                        time.sleep(1)  # ESSENTIAL to finish arming before taking off

                    if self.fw_takeoff_phase == 1:
                        if (
                            self.vehicle.nav_state
                            == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF
                        ):
                            self.node.get_logger().info(
                                "FW takeoff Step 3: In AUTO_TAKEOFF. Runway takeoff running."
                            )
                            self.takeoff_commanded = True
                        else:
                            self.node.get_logger().info(
                                f"FW takeoff: Waiting for AUTO_TAKEOFF nav state (current: {get_nav_state_str(self.vehicle.nav_state)})."
                            )
                elif (
                    self.vehicle.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC
                ):
                    self.node.get_logger().error(
                        "FW takeoff: Transition to MC in progress during horizontal takeoff."
                    )
                else:
                    self.node.get_logger().warn(
                        "FW takeoff Step 0: unknown vehicle state."
                    )

            else:
                # Vertical takeoff (multicopter or VTOL)
                self.log("Attempting vertical takeoff")
                self.vehicle.takeoff()  # TODO: change to multicopter_takeoff()
                self.takeoff_commanded = True
            # TODO: change takeoff_type to enum

        # When in AUTO_LOITER, engage offboard mode after 1 second of elapsed time
        if (
            self.takeoff_elapsed_time >= 1.0
            and self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            self.log("Takeoff complete. Engaging offboard mode.")
            self.vehicle.engage_offboard_mode()

    def check_status(self) -> str:
        """
        Check if takeoff is complete.

        Returns:
            str: "continue" while takeoff in progress, "complete" when takeoff is finished.
        """
        if self.vehicle.local_position is None or self.vehicle.global_position is None:
            return "continue"

        # Invalid config: horizontal takeoff on non-VTOL
        if self.takeoff_type == "horizontal" and not self.vehicle.is_vtol:
            return "error"

        if self.vehicle.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log("Takeoff complete, in offboard mode.")
            return "complete"
        return "continue"
