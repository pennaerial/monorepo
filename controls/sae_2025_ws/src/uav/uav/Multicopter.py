from rclpy.node import Node
import numpy as np
from uav.UAV import UAV


class Multicopter(UAV):
    """
    Multicopter UAV implementation with proportional velocity control.
    """

    def __init__(
        self, node: Node, takeoff_amount=5.0, DEBUG=False, camera_offsets=[0, 0, 0]
    ):
        super().__init__(node, takeoff_amount, DEBUG, camera_offsets)

    @property
    def is_vtol(self) -> bool:
        """Multicopters are not VTOL."""
        return False

    @property
    def vehicle_type(self) -> None:
        """Multicopters don't have vehicle_type (not VTOL)."""
        return None

    def vtol_transition_to(self, vtol_state, immediate=False):
        """Not available on multicopters."""
        self.node.get_logger().warn(
            "vtol_transition_to called on non-VTOL vehicle. Ignoring."
        )

    def _calculate_velocity(self, target_pos: tuple, lock_yaw: bool) -> list:
        """
        Calculate velocity using proportional control to prevent oscillation.
        Args:
            target_pos: (x, y, z) target position in local frame
            lock_yaw: Whether yaw is locked (hovering)
        Returns:
            [vx, vy, vz] velocity list in m/s
        """
        if self.local_position is None:
            # Fallback during startup: use absolute target position as direction estimate
            direction_est = np.array(target_pos)
            dist_est = np.linalg.norm(direction_est)
            if dist_est > 0.01:
                direction = direction_est / dist_est
                return self._calculate_proportional_velocity(direction, dist_est)
            else:
                return [0.0, 0.0, 0.0]

        # Normal case: calculate from current position
        dx = target_pos[0] - self.local_position.x
        dy = target_pos[1] - self.local_position.y
        dz = target_pos[2] - self.local_position.z
        dist = np.sqrt(dx**2 + dy**2 + dz**2)

        if dist > 0.01:
            direction = np.array([dx, dy, dz]) / dist
            return self._calculate_proportional_velocity(direction, dist)
        else:
            # At target: hover
            return [0.0, 0.0, 0.0]
