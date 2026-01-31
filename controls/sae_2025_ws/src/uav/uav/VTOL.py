from rclpy.node import Node
from px4_msgs.msg import VtolVehicleStatus, VehicleCommand
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import numpy as np
from uav.UAV import UAV


class VTOL(UAV):
    """
    VTOL UAV implementation with MC/FW mode switching.
    Handles both multicopter and fixed-wing flight modes.
    """

    def __init__(self, node: Node, takeoff_amount=5.0, DEBUG=False, camera_offsets=[0, 0, 0]):
        # Initialize VTOL-specific attributes before calling super().__init__
        self.vehicle_type = None  # 'MC' or 'FW' from VtolVehicleStatus
        self.vtol_vehicle_status = None

        super().__init__(node, takeoff_amount, DEBUG, camera_offsets)

    @property
    def is_vtol(self) -> bool:
        """VTOL aircraft can transition between MC and FW modes."""
        return True

    def fixed_wing_takeoff(self):
        """
        Simple horizontal VTOL takeoff:
          1) request transition to FW
          2) wait until vehicle_type == 'FW'
          3) send NAV_TAKEOFF to target altitude

        Call this repeatedly from ModeManager until it returns True.
        """
        # Create simple state the first time this function is used
        if not hasattr(self, "_fw_takeoff_stage"):
            self._fw_takeoff_stage = "idle"   # idle -> transitioning -> takeoff_sent
            self._fw_takeoff_t0 = 0.0

        # Step 0: start sequence + request FW transition
        if self._fw_takeoff_stage == "idle":
            
            self._fw_takeoff_stage = "transitioning"
            self.vtol_transition_to("FW", immediate=False)
            self.node.get_logger().info("FW takeoff Step 1: requested VTOL transition to FW.")
            return False

        # Step 1: wait for FW confirmation
        if self._fw_takeoff_stage == "transitioning":
            if self.vehicle_type == "FW":
                self.node.get_logger().info("FW takeoff Step 2: finished transition to FW mode.")
                self._fw_takeoff_stage = "takeoff_sent"
                return False
            else:
                return False

        # Step 2: send NAV_TAKEOFF once
        if self._fw_takeoff_stage == "takeoff_sent":
            if not self.attempted_takeoff:
                self.attempted_takeoff = True

            lat = self.global_position.lat
            lon = self.global_position.lon
            alt = self.global_position.alt
            self.node.get_logger().info(f"Current GPS: {lat}, {lon}, {alt}")
            
            # Right now is hard coded to take off east but should be modified based on yaw
            takeoff_gps = (lat, lon + 0.004, alt + self.takeoff_amount)

            self.node.get_logger().info(f"Takeoff Destination GPS: {takeoff_gps[0]}, {takeoff_gps[1]}, {takeoff_gps[2]}")

            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                # params={
                #     "param1": float('nan'), # fw_tko_pitch min (minimum pitch during takeoff)
                #     "param2": float('nan'), # Unused
                #     "param3": float('nan'), # Unused
                #     "param4": float('nan'), # Yaw angle
                #     "param5": takeoff_gps[0], #Latitude (in GPS coords)
                #     "param6": takeoff_gps[1], #Longitude (in GPS coords)
                #     "param7": takeoff_gps[2], #Altitude (in meters)
                # },
            )
            self.node.get_logger().info("FW takeoff Step 3: NAV_TAKEOFF sent.")
            # Optional: leave stage as-is so we don't resend
            self._fw_takeoff_stage = "done"
            return True

        return True
    
    def vtol_transition_to(self, vtol_state, immediate=False):
        """
        Command a VTOL transition.
        Following https://mavlink.io/en/messages/common.html#MAV_CMD_DO_VTOL_TRANSITION
        Args:
            vtol_state (str): The desired VTOL state ('MC' or 'FW').
            immediate (bool): If True, the transition should be immediate.
        """
        assert vtol_state in ['MC', 'FW'], "VTOL state must be 'MC' or 'FW'."
        state = VtolVehicleStatus.VEHICLE_VTOL_STATE_MC if vtol_state == 'MC' else VtolVehicleStatus.VEHICLE_VTOL_STATE_FW
        immediate = 1 if immediate else 0
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_VTOL_TRANSITION, params={'param1': float(state), 'param2': float(immediate)})
        self.node.get_logger().info(f"VTOL transition command sent: {state}. Transitioning to {vtol_state} mode.")

    def _calculate_velocity(self, target_pos: tuple, lock_yaw: bool) -> list:
        """
        Calculate velocity switching between FW (forward velocity) and MC (proportional) modes.
        Args:
            target_pos: (x, y, z) target position in local frame
            lock_yaw: Whether yaw is locked (hovering)
        Returns:
            [vx, vy, vz] velocity list in m/s
        """
        is_fw_mode = (self.vehicle_type == 'FW')

        if is_fw_mode:
            # Fixed-wing mode: always maintain forward velocity for lift
            return self._get_fw_forward_velocity()
        else:
            # Multicopter mode: use proportional control (same as Multicopter class)
            if self.local_position is None:
                # Fallback during startup
                direction_est = np.array(target_pos)
                dist_est = np.linalg.norm(direction_est)
                direction = direction_est / dist_est
                return self._calculate_proportional_velocity(direction, dist_est)
            else:
                # Normal case: calculate from current position
                dx = target_pos[0] - self.local_position.x
                dy = target_pos[1] - self.local_position.y
                dz = target_pos[2] - self.local_position.z
                dist = np.sqrt(dx**2 + dy**2 + dz**2)

                direction = np.array([dx, dy, dz]) / dist
                return self._calculate_proportional_velocity(direction, dist)

    def _get_fw_forward_velocity(self):
        """
        Get forward velocity vector for FW mode to maintain lift.
        Returns:
            list: [vx, vy, vz] velocity vector in m/s
        """
        if self.yaw is not None:
            return [float(np.cos(self.yaw) * self.default_velocity),
                    float(np.sin(self.yaw) * self.default_velocity),
                    0.0]
        else:
            # Last resort: forward in X direction
            self.node.get_logger().warn("FW mode: Using fallback velocity (X-axis forward) - yaw not available")
            return [float(self.default_velocity), 0.0, 0.0]

    def _vtol_vehicle_status_callback(self, msg: VtolVehicleStatus):
        """
        Callback for VTOL vehicle status updates.
        Updates vehicle_type based on the actual VTOL state.
        """
        self.vtol_vehicle_status = msg
        if msg.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_MC:
            self.vehicle_type = 'MC'
        elif msg.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_FW:
            self.vehicle_type = 'FW'
        # During transitions, maintain the current state (don't change vehicle_type)
        # VEHICLE_VTOL_STATE_TRANSITION_TO_FW = 1 (still in MC mode)
        # VEHICLE_VTOL_STATE_TRANSITION_TO_MC = 2 (still in FW mode)
        elif msg.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_FW:
            # Transitioning to FW, but still in MC mode
            if self.vehicle_type is None:
                self.vehicle_type = 'MC'
        elif msg.vehicle_vtol_state == VtolVehicleStatus.VEHICLE_VTOL_STATE_TRANSITION_TO_MC:
            # Transitioning to MC, but still in FW mode
            if self.vehicle_type is None:
                self.vehicle_type = 'FW'

    def _initialize_publishers_and_subscribers(self):
        """
        Initialize ROS 2 publishers and subscribers.
        Extends parent to add VTOL-specific subscriber.
        """
        # Call parent to initialize all shared publishers/subscribers
        super()._initialize_publishers_and_subscribers()

        # Add VTOL-specific subscriber for VtolVehicleStatus
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.vtol_vehicle_status_sub = self.node.create_subscription(
            VtolVehicleStatus,
            '/fmu/out/vtol_vehicle_status',
            self._vtol_vehicle_status_callback,
            qos_profile
        )
