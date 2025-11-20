import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav.vision_nodes import HoopCenterNode


class HoopTraverseMode(Mode):
    """
    Debug mode to sit still and log what the hoop-tracking service reports.
    """

    def __init__(self, node: Node, uav: UAV):
        super().__init__(node, uav)
        self.done = False
        self._time_since_last_log = 0.0

    def on_update(self, time_delta: float) -> None:
        self._time_since_last_log += time_delta

        request = HoopCenterNode.Request()

        local_pos = self.uav.get_local_position()
        altitude = -local_pos[2]
        request.altitude = float(altitude)
        request.yaw = float(self.uav.yaw)
        request.payload_color = "red"

        response = self.send_request(HoopCenterNode, request)

        if response is None:
            print("[HoopDebug] No response from HoopCenterNode.")
            return

        x = response.x
        y = response.y
        direction = response.direction
        dlz_empty = response.dlz_empty

        status = (
            f"[HoopDebug] alt={altitude:.2f} m | "
            f"dlz_empty={dlz_empty} | "
            f"center=(x={x:.1f}, y={y:.1f}) | "
            f"direction=[{direction[0]:.3f}, {direction[1]:.3f}, {direction[2]:.3f}]"
        )

        print(status)
        try:
            self.log(status)
        except Exception:
            pass

    def check_status(self) -> str:
        return "continue"