import random as r
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
        # Call parent initializer
        super().__init__(dlz, uav, num_hoops, max_dist)
        
        # Height of first hoop
        self.start_height = start_height
        
    def generate_course(self) -> List[Pose]:
        hoops = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        zone_len = abs(x_dlz - x_uav) / self.num_hoops
        zone_width = (y_dlz - y_uav) / self.num_hoops
        zone_height = abs(self.start_height - z_dlz) / self.num_hoops

        x_step = r.uniform(0, zone_len)
        y_step = r.uniform(0, zone_width)
        
        for i in range(self.num_hoops):

            if i == 0:
                start: Pose = (x_uav + x_step, y_uav + y_step, self.start_height, 0, 90, 0)
                hoops.append(start)
                continue
            
            # regenerate step functions
            x_step = r.uniform(0, zone_len)
            y_step = r.uniform(0, abs(zone_width)) * (1 if zone_width >= 0 else -1)
            z_step = r.uniform(0, zone_height)

            # update x, y, z values
            new_x = x_uav + (zone_len * i) + x_step
            new_y = y_uav + (zone_width *i ) + y_step
            new_z = self.start_height - (zone_height * i) - z_step
            hoops.append((new_x, new_y, new_z, 0, 90, 0))

        return hoops