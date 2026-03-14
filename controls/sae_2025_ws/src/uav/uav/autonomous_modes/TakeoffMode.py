from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV, VTOL
from px4_msgs.msg import VehicleStatus, VtolVehicleStatus, VehicleCommand
import numpy as np
import time


class TakeoffMode(Mode):
    """
    A mode for takeoff (vertical or horizontal).
    Works for both multicopters and VTOLs.
    - Multicopters: always vertical takeoff.
    - VTOLs: vertical or horizontal (fixed-wing style) based on takeoff_type param.
    """

    def __init__(self, node: Node, uav: UAV, takeoff_type: str = "vertical", 
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
            uav (UAV): The UAV instance.
            takeoff_type (str): 'vertical' or 'horizontal'. Default 'vertical'.
                Horizontal is only valid for VTOLs.
        """
        super().__init__(node, uav)
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

        if self.uav.local_position is None or self.uav.global_position is None:
            self.log("Waiting for position data...")
            return

        # Takeoff command already sent
        if self.takeoff_commanded:
            self.log(f"{self.takeoff_type.capitalize()} takeoff in progress.")

        # Need to send takeoff command
        else:
            self.node.get_logger().info(f"BRUHHHH. takeoff {self.fw_takeoff_phase}")
            self.takeoff_elapsed_time = 0.0  # time reset as a result of time_delta counting from beginning of launch
            if self.takeoff_type == "horizontal":
                if not self.uav.is_vtol or not isinstance(self.uav, VTOL):
                    self.node.get_logger().error(
                        "Horizontal takeoff only valid for VTOL - cannot proceed."
                    )
                    return
                
                # fixed wing takeoff sequence
                if self.uav.vtol_vehicle_status is None:
                    self.node.get_logger().info("FW takeoff: Vehicle status not available yet.")
                    return

                elif (
                    self.uav.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC
                ):
                    self.uav.vtol_transition_to("FW", immediate=False)
                    self.node.get_logger().info(
                        "FW takeoff Step 1: requested VTOL transition to FW."
                    )
                    return

                elif (
                    self.uav.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW
                ):
                    self.node.get_logger().info(
                        "FW takeoff Step 1a: transition to FW in progress."
                    )
                    return

                elif (
                    self.uav.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW
                ):
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
                    self.node.get_logger().info(f"HEREEEEEE. takeoff {self.fw_takeoff_phase}")

                    if self.fw_takeoff_phase == 0:
                        self.node.get_logger().info(
                            "FW takeoff Step 2: transition complete. Starting disarm->takeoff->arm sequence."
                        )

                        lat = self.uav.global_position.lat
                        lon = self.uav.global_position.lon
                        alt = self.uav.global_position.alt
                        self.node.get_logger().info(f"Current GPS: {lat}, {lon}, {alt}")

                        if np.isnan(self.latitude) or np.isnan(self.longitude) or np.isnan(self.altitude):
                            self.node.get_logger().info(
                                "Takeoff Destination GPS: Auto Calculated"
                            )
                        else:
                            self.node.get_logger().info(
                                f"Takeoff Destination GPS: {self.latitude}, {self.longitude}, {self.altitude}"
                            )

                        # disarm command (needs to be force disarm to reset land detector)
                        self.uav.disarm(force=True)
                        self.node.get_logger().info(
                            "FW takeoff Step 2a: force-disarmed to reset land detector."
                        )
                        time.sleep(0.5)

                        self.uav.fixed_wing_takeoff(self.fw_tko_pitch, self.yaw, self.latitude, self.longitude, self.altitude)

                        self.node.get_logger().info(
                            "FW takeoff Step 2b: takeoff command sent while disarmed."
                        )
                        time.sleep(0.1) 

                        self.uav.arm()
                        self.node.get_logger().info("FW takeoff Step 2c: re-arm command sent.")
                        self.fw_takeoff_phase = 1
                        time.sleep(1)

                        # # Wait until the vehicle is armed to start the takeoff sequence
                        # arming_wait_start_time = time.time()
                        # arming_timeout = 10  # seconds
                        # while self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED and (time.time() - arming_wait_start_time) < arming_timeout:
                        #     self.node.get_logger().info("Waiting for UAV to finish arming...")
                        #     time.sleep(0.1)
                        # if self.uav.arm_state == VehicleStatus.ARMING_STATE_ARMED:
                        #     self.node.get_logger().info("UAV is armed. Attempting takeoff...")
                        # else:
                        #     self.node.get_logger().warn(f"Timeout of {arming_timeout} seconds hit. Unable to arm UAV after takeoff command.")

                    if self.fw_takeoff_phase == 1:
                        self.node.get_logger().info(f"FW takeoff Step 3: nav state: {self.uav.nav_state}")
                        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_TAKEOFF:
                            self.node.get_logger().info(
                                "FW takeoff Step 3: In AUTO_TAKEOFF. Runway takeoff running."
                            )
                            self.takeoff_commanded = True
                            # return True
                        else:
                            self.node.get_logger().info(
                                f"FW takeoff: Waiting for AUTO_TAKEOFF nav state (current: {self.uav.nav_state})."
                            )
                            # return False
                elif (
                    self.uav.vtol_vehicle_status.vehicle_vtol_state
                    == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC
                ):
                    self.node.get_logger().error(
                        "FW takeoff: Transition to MC in progress during horizontal takeoff."
                    )
                    # return False
                else:
                    self.node.get_logger().warn("FW takeoff Step 0: unknown vehicle state.")
                    # return False


            else:
                # Vertical takeoff (multicopter or VTOL)
                self.log("Attempting vertical takeoff")
                self.uav.takeoff()  # TODO: change to multicopter_takeoff()
                self.takeoff_commanded = True
            # TODO: change takeoff_type to enum

        # When in AUTO_LOITER, engage offboard mode after 1 second of elapsed time
        if (
            self.takeoff_elapsed_time >= 1.0
            and self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER
        ):
            self.log("Takeoff complete. Engaging offboard mode.")
            self.uav.engage_offboard_mode()

    def check_status(self) -> str:
        """
        Check if takeoff is complete.

        Returns:
            str: "continue" while takeoff in progress, "complete" when takeoff is finished.
        """
        if self.uav.local_position is None or self.uav.global_position is None:
            return "continue"

        # Invalid config: horizontal takeoff on non-VTOL
        if self.takeoff_type == "horizontal" and not self.uav.is_vtol:
            return "error"

        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
            self.log("Takeoff complete, in offboard mode.")
            return "complete"
        return "continue"
