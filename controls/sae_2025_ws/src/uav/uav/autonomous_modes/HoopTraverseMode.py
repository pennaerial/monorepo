import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import PayloadTracking
from uav.vision_nodes import PayloadTrackingNode
from px4_msgs.msg import VehicleStatus


class HoopTraverseMode(Mode):
    """
    Debug mode to sit still and log what the hoop-tracking (PayloadTracking) service reports.
    - Does NOT move the UAV at all.
    - Continuously calls the PayloadTrackingNode service and logs:
        * whether a hoop was detected (dlz_empty)
        * reported center pixel (x, y)
        * direction ray (3D)
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.done = False
        self._time_since_last_log = 0.0

    def on_update(self, time_delta: float) -> None:
        self._time_since_last_log += time_delta

        # Build request to hoop center / payload tracking service
        request = PayloadTracking.Request()

        # altitude: NED z is down; convert to positive meters above ground
        local_pos = self.uav.get_local_position()
        altitude = -local_pos[2]
        request.altitude = float(altitude)
        request.yaw = float(self.uav.yaw)
        request.payload_color = "red"

        response = self.send_request(PayloadTrackingNode, request)

        if response is None:
            msg = "[HoopDebug] No response from PayloadTracking service."
            self._log_all(msg)
            return

        # Unpack response
        x = response.x
        y = response.y
        direction = response.direction
        dlz_empty = response.dlz_empty

        # Build a detailed status string
        status = (
            f"[HoopDebug] alt={altitude:.2f} m | "
            f"dlz_empty={dlz_empty} | "
            f"center=(x={x:.1f}, y={y:.1f}) | "
            f"direction=[{direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}]"
        )

        # Log every call (you can throttle if it's too spammy)
        self._log_all(status)

    def _log_all(self, msg: str):
        """
        Try all forms of logging:
          - print to stdout
          - Mode's self.log()
          - Node's rclpy logger (if available)
        """
        # Plain print
        print(msg)

        # Mode-level logger (likely already used in your framework)
        try:
            self.log(msg)
        except Exception:
            pass

        # Underlying rclpy node logger if accessible
        node_obj = getattr(self, "node", None)
        if node_obj is not None:
            try:
                node_obj.get_logger().info(msg)
            except Exception:
                pass

    def check_status(self) -> str:
        # This mode is meant to run indefinitely until externally stopped.
        return "continue"
