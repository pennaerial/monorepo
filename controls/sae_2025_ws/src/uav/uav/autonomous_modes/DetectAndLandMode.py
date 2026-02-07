import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HelipadTracking
from uav.vision_nodes import HelipadTrackingNode
from px4_msgs.msg import VehicleStatus


class DetectAndLandMode(Mode):
    """
    A mode for detecting a helipad (H marker) and landing on it.

    The UAV will:
    1. Search for the helipad using the camera
    2. Center itself over the helipad
    3. Descend while maintaining alignment
    4. Land when close enough
    """

    # State machine states
    STATE_SEARCHING = 0
    STATE_CENTERING = 1
    STATE_DESCENDING = 2
    STATE_LANDING = 3
    STATE_DONE = 4

    def __init__(self, node: Node, uav: UAV,
                 search_altitude: float = 10.0,
                 descent_rate: float = 0.5,
                 center_threshold: float = 0.1,
                 land_altitude: float = 1.0):
        """
        Initialize DetectAndLandMode.

        Args:
            node: ROS 2 node managing the UAV.
            uav: The UAV instance to control.
            search_altitude: Altitude to maintain while searching (meters, positive).
            descent_rate: Rate of descent when aligned (meters per update).
            center_threshold: How centered the helipad must be (0-1, fraction of frame).
            land_altitude: Altitude at which to initiate landing (meters).
        """
        super().__init__(node, uav)
        self.search_altitude = search_altitude
        self.descent_rate = descent_rate
        self.center_threshold = center_threshold
        self.land_altitude = land_altitude

        self.state = self.STATE_SEARCHING
        self.no_detection_count = 0
        self.max_no_detection = 30  # Frames before giving up on current position

    def on_enter(self) -> None:
        """Called when mode is activated."""
        self.state = self.STATE_SEARCHING
        self.no_detection_count = 0
        self.log(f"Starting helipad detection at altitude {self.search_altitude}m")

    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for detecting and landing on helipad.
        """
        # If UAV is unstable, wait for stabilization
        if abs(self.uav.roll) > 0.15 or abs(self.uav.pitch) > 0.15:
            self.log("Waiting for stabilization...")
            return

        # Handle landing state separately
        if self.state == self.STATE_LANDING:
            # Landing is handled by PX4, just wait
            if self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED:
                self.state = self.STATE_DONE
                self.log("Landing complete!")
            return

        # Get current position
        current_pos = self.uav.get_local_position()
        if current_pos is None:
            return

        current_altitude = -current_pos[2]  # NED: negative z is up

        # Request helipad detection
        request = HelipadTracking.Request()
        request.altitude = current_altitude
        request.yaw = float(self.uav.yaw) if self.uav.yaw else 0.0

        response = self.send_request(HelipadTrackingNode, request)

        if response is None:
            # Still waiting for response
            self.uav.publish_position_setpoint(current_pos)
            return

        if response.detected:
            self.no_detection_count = 0
            self._handle_detection(response, current_pos, current_altitude)
        else:
            self.no_detection_count += 1
            self._handle_no_detection(current_pos, current_altitude)

    def _handle_detection(self, response, current_pos, current_altitude):
        """Handle case when helipad is detected."""
        direction = list(response.direction)

        # Transform direction from camera frame to local frame
        # direction[0] = forward, direction[1] = right, direction[2] = down
        move_x = -direction[1]  # Camera Y -> Local X (forward)
        move_y = direction[0]   # Camera X -> Local Y (right)

        # Add camera offset compensation
        camera_offsets = self.uav.camera_offsets
        if current_altitude > 1:
            camera_offsets = tuple(x / current_altitude for x in camera_offsets)
        local_offsets = self.uav.uav_to_local(camera_offsets)
        move_x += local_offsets[0]
        move_y += local_offsets[1]

        # Check if centered
        is_centered = (abs(move_x) < self.center_threshold and
                      abs(move_y) < self.center_threshold)

        if self.state == self.STATE_SEARCHING:
            self.state = self.STATE_CENTERING
            self.log("Helipad detected! Centering...")

        if is_centered:
            if current_altitude <= self.land_altitude:
                # Close enough - initiate landing
                self.state = self.STATE_LANDING
                self.log("Centered and low enough - initiating landing!")
                self.uav.land()
                return
            else:
                # Descend while staying centered
                self.state = self.STATE_DESCENDING
                move_z = self.descent_rate
                self.log(f"Centered - descending. Alt: {current_altitude:.1f}m")
        else:
            # Not centered - move toward helipad, maintain altitude
            self.state = self.STATE_CENTERING
            move_z = 0
            self.log(f"Centering: dx={move_x:.2f}, dy={move_y:.2f}")

        # Publish relative position setpoint
        self.uav.publish_position_setpoint(
            (move_x, move_y, move_z),
            relative=True
        )

    def _handle_no_detection(self, current_pos, current_altitude):
        """Handle case when helipad is not detected."""
        if self.state == self.STATE_DESCENDING:
            # Lost detection while descending - go back up
            self.log("Lost helipad detection - ascending to search altitude")
            target_z = -self.search_altitude
            self.uav.publish_position_setpoint(
                (current_pos[0], current_pos[1], target_z)
            )
            self.state = self.STATE_SEARCHING
        else:
            # Maintain current position while searching
            self.log(f"Searching for helipad... ({self.no_detection_count})")
            self.uav.publish_position_setpoint(current_pos)

    def check_status(self) -> str:
        """Check if mode should complete."""
        if self.state == self.STATE_DONE:
            return 'complete'
        return 'continue'
