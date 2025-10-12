import random as r
from typing import List, Tuple
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
        # Call parent initializer
        super().__init__(dlz, uav, num_hoops, max_dist)
        
        # How wide/tall slalom course is
        self.width = width
        self.height = height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        zone_len = abs(x_dlz - x_uav) / self.num_hoops
        zone_width = self.width
        zone_height = self.height

        for i in range(self.num_hoops):

            # alternate to create slalom pattern
            alt = 1 if i % 2 == 0 else -1

            # regenerate step functions
            x_step = r.uniform(0, zone_len)
            y_step = r.uniform(0, zone_width) * alt
            z_step = r.uniform(zone_height / 2, zone_height)

            # update x, y, z values
            new_x = x_uav + (zone_len * i) + x_step
            new_y = y_uav + (zone_width * i) + y_step
            new_z = z_step
            hoops.append((new_x, new_y, new_z, 0, 90, 0))

        return hoops