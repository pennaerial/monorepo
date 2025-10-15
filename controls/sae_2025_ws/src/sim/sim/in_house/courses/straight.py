import random as r
from typing import List, Tuple
from .course_style import Pose, CourseStyle

class StraightCourse(CourseStyle):
    """
    Simple straight course with hoops in a straight line.
    Perfect for testing basic navigation and scoring.
    """
    
    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 2.0,
                 spacing: float = 2.0
                ):
        # Call parent initializer
        super().__init__(dlz, uav, num_hoops, max_dist)
        
        # Height of all hoops (same for straight course)
        self.height = height
        
        # Distance between hoops
        self.spacing = spacing
        
    def generate_course(self) -> List[Pose]:
        """
        Generate a straight line of hoops.
        
        Returns:
            List of (x, y, z, roll, pitch, yaw) positions for each hoop
        """
        hoops = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav
        
        # Calculate total course length
        total_length = (self.num_hoops - 1) * self.spacing
        
        # Start position (slightly offset from UAV start)
        start_x = x_uav + 1.0  # 1 meter forward from UAV
        start_y = y_uav
        
        # Generate hoops in a straight line
        for i in range(self.num_hoops):
            # Calculate position
            hoop_x = start_x + (i * self.spacing)
            hoop_y = start_y
            hoop_z = self.height
            
            # Add small random variation (±0.2m) for realism
            variation_x = r.uniform(-0.2, 0.2)
            variation_y = r.uniform(-0.2, 0.2)
            variation_z = r.uniform(-0.1, 0.1)
            
            # Final position with variation
            final_x = hoop_x + variation_x
            final_y = hoop_y + variation_y
            final_z = hoop_z + variation_z
            
            # All hoops face forward (yaw = 0)
            hoop_pose = (final_x, final_y, final_z, 0, 90, 0)
            hoops.append(hoop_pose)
            
        return hoops