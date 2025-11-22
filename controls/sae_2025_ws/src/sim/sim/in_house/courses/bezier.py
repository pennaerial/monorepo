from .course_style import Pose, CourseStyle
import math
import random as r
from typing import List, Tuple
from .course_style import Pose, CourseStyle

class BezierCourse(CourseStyle):
    """
    Smooth cubic Bezier curve between UAV start and DLZ.
    P0 = UAV, P3 = DLZ
    P1/P2 are along the line with lateral offsets to create an arc.
    """

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 2.0,
                 lateral_offset: float = 4.0):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.height = height
        self.lateral_offset = lateral_offset

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction UAV -> DLZ in XY
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist_xy == 0 or dist_3d == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        dir_x = dx / dist_xy
        dir_y = dy / dist_xy

        # Perpendicular vector in XY plane
        perp_x = -dir_y
        perp_y = dir_x

        # Bezier control points in XY
        # P0 and P3 are UAV and DLZ
        P0x, P0y = x_uav, y_uav
        P3x, P3y = x_dlz, y_dlz

        # Place P1 and P2 along the line with opposite lateral offsets
        one_third = dist_xy / 3.0
        two_third = 2.0 * dist_xy / 3.0

        P1x = P0x + dir_x * one_third  + perp_x * self.lateral_offset
        P1y = P0y + dir_y * one_third  + perp_y * self.lateral_offset

        P2x = P0x + dir_x * two_third  - perp_x * self.lateral_offset
        P2y = P0y + dir_y * two_third  - perp_y * self.lateral_offset

        # Helper: cubic Bezier in 2D
        def bezier_xy(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            mt2 = mt * mt
            t2 = t * t
            x = (mt2 * mt) * P0x + 3 * (mt2) * t * P1x + 3 * mt * t2 * P2x + (t2 * t) * P3x
            y = (mt2 * mt) * P0y + 3 * (mt2) * t * P1y + 3 * mt * t2 * P2y + (t2 * t) * P3y
            return x, y

        # Helper: derivative for tangent direction
        def bezier_xy_deriv(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            x = 3 * mt * mt * (P1x - P0x) + 6 * mt * t * (P2x - P1x) + 3 * t * t * (P3x - P2x)
            y = 3 * mt * mt * (P1y - P0y) + 6 * mt * t * (P2y - P1y) + 3 * t * t * (P3y - P2y)
            return x, y

        # Place hoops along t in (0, 1)
        for i in range(self.num_hoops):
            # keep hoops off the exact endpoints
            t = (i + 1) / (self.num_hoops + 1)

            x, y = bezier_xy(t)

            # Smooth altitude: interpolate between UAV z and DLZ z
            z_linear = z_uav + t * (z_dlz - z_uav)
            # small noise
            z_noise = r.uniform(-0.2, 0.2)
            z = max(0.5, z_linear + z_noise)  # keep above ground a bit

            # Orientation from tangent of curve
            dx_dt, dy_dt = bezier_xy_deriv(t)
            if dx_dt == 0 and dy_dt == 0:
                yaw_deg = 0.0
            else:
                yaw = math.atan2(dy_dt, dx_dt)
                yaw_deg = math.degrees(yaw)

            hoops.append((x, y, z + 1, 0.0, yaw_deg, 0.0))

        return hoops