import random as r
import math
from typing import List, Tuple
from .course_style import Pose, CourseStyle


class DescentCourse(CourseStyle):
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 start_height: int
                 ):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.start_height = start_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction vector UAV → DLZ
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        # Total distance
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        # Normalize to get unit vector
        dir_x = dx / dist
        dir_y = dy / dist
        dir_z = dz / dist

        # Distance between hoops
        segment_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            # Progress along line (slightly randomized)
            along = segment_len * (i + r.uniform(0.2, 0.8))

            # Height transition from start_height down to DLZ
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)  # dz negative if descending
            z_noise = r.uniform(-0.3, 0.3) * abs(z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

            # Orientation along course
            yaw_deg = math.degrees(math.atan2(dy, dx))
            hoops.append((new_x, new_y, new_z, 0, yaw_deg, 0))

        return hoops