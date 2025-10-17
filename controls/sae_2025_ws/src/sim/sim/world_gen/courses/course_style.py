from abc import ABC, abstractmethod
from typing import List, Tuple

Pose = Tuple[float, float, float, float, float, float]

class CourseStyle(ABC):
    """Abstract base class for all course styles."""

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int
                ):
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops

        # max_dist represents max distance we want next step for hoop in our course
        self.max_dist = max_dist

    @abstractmethod
    def generate_course(self) -> List[Pose]:
        """Generate a list of (x, y, z, roll, pitch, yaw) for hoop placement."""
        pass