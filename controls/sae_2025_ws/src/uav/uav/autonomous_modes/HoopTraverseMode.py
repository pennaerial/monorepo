import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from px4_msgs.msg import VehicleStatus

class HoopTraverseMode(Mode):
    """
    Mode to center on a red hoop, fly through it, and end.
    No constructor params by design.
    """

    # --- Tunables ---
    CENTER_HOLD_SEC = 0.4         # how long the center condition must hold before charging through
    FORWARD_SPEED_MPS = 0.6       # approximate forward progress (meters/second) using relative setpoints
    GO_THROUGH_MAX_SEC = 2.5      # safety cap for forward push
    LOST_HOOP_GRACE_SEC = 0.5     # if hoop is lost this long during go-through, consider it passed

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.mode = 0                  # 0=center, 1=go-through
        self.done = False

        # timers/accumulators
        self.center_hold = 0.0
        self.go_through_elapsed = 0.0
        self.hoop_lost_elapsed = 0.0

        # convenience
        self.camera_offsets = self.uav.camera_offsets  # (x,y,z) camera wrt UAV body (meters)

    def on_update(self, time_delta: float) -> None:
        # Safety: wait for attitude to settle
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # Prepare service request to PayloadTrackingNode (red hoop tracker)
        request = PayloadTracking.Request()
        # altitude: NED z is down; convert to positive meters above ground
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'red'

        response = self.send_request(PayloadTrackingNode, request)
        if response is None:
            # No fresh data — if we were charging through, keep pushing a bit; else hold
            if self.mode == 1:
                self._push_forward(time_delta)
            return

        # Map camera-frame unit ray -> NED direction components (like your template)
        # camera ray: [rx, ry, rz]; you used [-ry, rx, rz] mapping
        ray = response.direction
        direction = [-ray[1], ray[0], ray[2]]

        # Apply camera and manual offsets (scaled by altitude when close; keep it simple here)
        altitude = max(1e-3, request.altitude)
        cam_off_ned = self.uav.uav_to_local(self.camera_offsets)
        # nudge center target by camera offsets in a scale-invariant way when high
        if altitude > 1.0:
            direction = [d + (co / altitude) for d, co in zip(direction, cam_off_ned)]
        else:
            direction = [d + co for d, co in zip(direction, cam_off_ned)]

        # Compute adaptive centering threshold (same style as your dropoff mode)
        center_thresh = altitude / 25.0  # tighter when close

        if self.mode == 0:
            # --- Centering logic ---
            # Zero vertical motion while aligning
            direction[2] = 0.0

            # Are we centered laterally?
            centered = (abs(direction[0]) < center_thresh and
                        abs(direction[1]) < center_thresh)

            # Small forward trickle to approach while centering
            forward_step = 0.15 * time_delta  # small creep forward
            self.uav.publish_position_setpoint([forward_step, 0.0, 0.0], relative=True)

            if centered and not response.dlz_empty:
                self.center_hold += time_delta
                if self.center_hold >= self.CENTER_HOLD_SEC:
                    self.log("Centered on hoop. Initiating go-through.")
                    self.mode = 1
                    # reset timers
                    self.go_through_elapsed = 0.0
                    self.hoop_lost_elapsed = 0.0
            else:
                self.center_hold = 0.0

            # Lateral correction toward center
            lateral_step = [direction[0], direction[1], 0.0]
            self.uav.publish_position_setpoint(lateral_step, relative=True)
            self.log(f"[Centering] dir={direction}, centered={centered}, hold={self.center_hold:.2f}s")

        elif self.mode == 1:
            # --- Charge through the hoop ---
            self.go_through_elapsed += time_delta

            # If we still see a trusted detection, reset "lost" timer; else accumulate
            if not response.dlz_empty:
                self.hoop_lost_elapsed = 0.0
            else:
                self.hoop_lost_elapsed += time_delta

            # Push forward; ignore lateral during this window to commit through the hoop
            self._push_forward(time_delta)

            # End conditions:
            #   - hoop lost for a bit (likely passed)
            #   - or we’ve been pushing forward long enough
            if (self.hoop_lost_elapsed >= self.LOST_HOOP_GRACE_SEC or
                    self.go_through_elapsed >= self.GO_THROUGH_MAX_SEC):
                self.done = True
                self.log("Completed hoop traversal.")
                return

            self.log(f"[GoThrough] t={self.go_through_elapsed:.2f}s, lost={self.hoop_lost_elapsed:.2f}s")

    def _push_forward(self, dt: float):
        """Advance forward approximately FORWARD_SPEED_MPS using relative position nudges."""
        step = max(0.0, self.FORWARD_SPEED_MPS * dt)
        # NED forward is +X
        self.uav.publish_position_setpoint([step, 0.0, 0.0], relative=True)

    def check_status(self) -> str:
        return 'complete' if self.done else 'continue'
