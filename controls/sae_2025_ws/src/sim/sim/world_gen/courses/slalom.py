import random as r
from typing import List, Tuple
import math
from .course_style import Pose, CourseStyle


class SlalomCourse(CourseStyle):
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 width: int,
                 height: int
                 ):
        super().__init__(dlz, uav, num_hoops, max_dist)
        self.width = width
        self.height = height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        # Direction vector from UAV → DLZ
        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        # Distance between UAV and DLZ
        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be at the same position.")

        # Normalize to get unit direction vector
        dir_x = dx / dist
        dir_y = dy / dist
        dir_z = dz / dist

        # Get perpendicular vector in XY plane for slalom offset
        # (-dy, dx) is perpendicular to (dx, dy)
        perp_x = -dir_y
        perp_y = dir_x

        zone_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            alt = 1 if i % 2 == 0 else -1

            # Move forward along direction vector
            along_dist = zone_len * (i + r.uniform(0.2, 0.8))  # slight randomness
            offset = alt * self.width * 0.5 * r.uniform(0.5, 1.0)

            # Compute new hoop position
            new_x = x_uav + dir_x * along_dist + perp_x * offset
            new_y = y_uav + dir_y * along_dist + perp_y * offset
            new_z = r.uniform(self.height * 0.3, self.height * 0.7)

            hoops.append((new_x, new_y, new_z, 0, 90, 0))

        return hoops