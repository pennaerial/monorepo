import random as r
import xml.etree.ElementTree as ET
from sim.world_gen import WorldNode
from xml.dom import minidom
from pathlib import Path
from abc import ABC, abstractmethod
from typing import List, Tuple
from sim_interfaces.srv import HoopList

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

class AscentCourse(CourseStyle):
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

            # Height transition from UAV’s start_height to DLZ altitude
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)
            z_noise = r.uniform(-0.3, 0.3) * (z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

            # Each hoop faces along the course direction
            yaw_deg = math.degrees(math.atan2(dy, dx))
            hoops.append((new_x, new_y, new_z, 0, yaw_deg, 0))

        return hoops

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
            variation_x = 0
            variation_y = 0
            variation_z = 0
            
            # Final position with variation
            final_x = hoop_x + variation_x
            final_y = hoop_y + variation_y
            final_z = hoop_z + variation_z
            
            # All hoops face forward (yaw = 0)
            hoop_pose = (final_x, final_y, final_z, 0, 90, 0)
            hoops.append(hoop_pose)
            
        return hoops


class HoopCourseNode(WorldNode):
    """
    ROS node for generating hoop course for in house comp.
    """

    def __init__(self, course, dlz, uav, num_hoops, max_dist, world_name, output_file):
        super().__init__('in_house_world_gen')
        self.course = course
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops
        self.max_dist = max_dist
        self.world_name = world_name
        self.generate_world()
        self.srv = self.create_service(HoopList, "list_hoops", self.hoop_list_req)
    
    def hoop_list_req(self, request, response):
        response.hoop_positions = self.hoop_positions
        return response


    def add_hoops(self, input_file, output_file, hoop_positions):
        # Expand ~, make absolute, and validate paths
        in_path = Path(input_file).expanduser().resolve()
        out_path = Path(output_file).expanduser().resolve()
        if not in_path.exists():
            raise FileNotFoundError(
                f"Input SDF not found: {in_path}\nCWD: {Path.cwd().resolve()}"
            )
        out_path.parent.mkdir(parents=True, exist_ok=True)

        # Parse
        tree = ET.parse(str(in_path))  # str() for safety on older libs
        root = tree.getroot()

        # handle namespace if present (root.tag may be like "{...}sdf")
        ns = ""
        if root.tag.startswith("{"):
            ns = root.tag.split("}")[0] + "}"

        # try to find <world> with/without namespace
        world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
        if world is None:
            raise RuntimeError("No <world> tag found in the SDF!")

        # Add new hoops with given positions
        for i, pos in enumerate(hoop_positions, start=1):
            x, y, z, roll, pitch, yaw = pos
            inc = ET.Element("include")
            ET.SubElement(inc, "uri").text = "model://hoop"
            ET.SubElement(inc, "pose").text = f"{x} {y} {z} {roll} {pitch} {yaw}"
            ET.SubElement(inc, "name").text = f"hoop_{i}"
            world.append(inc)

        # --- remove whitespace-only text/tail nodes to avoid minidom producing extra blank lines ---
        def strip_whitespace(self, elem):
            if elem.text is not None and elem.text.strip() == "":
                elem.text = None
            for child in list(elem):
                strip_whitespace(child)
                if child.tail is not None and child.tail.strip() == "":
                    child.tail = None
        strip_whitespace(root)

        # Pretty print and write
        rough_bytes = ET.tostring(root, encoding="utf-8")
        pretty = minidom.parseString(rough_bytes.decode("utf-8")).toprettyxml(indent="  ")
        lines = [ln for ln in pretty.splitlines() if ln.strip() != ""]
        out_path.write_text("\n".join(lines) + "\n", encoding="utf-8")

    def generate_world(self):
        courses = ['ascent', 'descent', 'slalom']

        if gen_style.lower() == 'random':
            course_id = r.uniform(0, len(courses) - 1)
            gen_style = courses[course_id]
            
        if gen_style.lower() == "previous":
            return

        elif gen_style.lower() == "ascent":
            course = AscentCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist, 
                                start_height=2)
            hoop_poses = course.generate_course()
        elif gen_style.lower() == "descent":
            course = DescentCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist, 
                                start_height=4)
            hoop_poses = course.generate_course()
        elif gen_style.lower() == "slalom":
            course = SlalomCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist,
                                width=4,
                                height=2)
            hoop_poses = course.generate_course()
        elif gen_style.lower() == "straight":
            course = StraightCourse(dlz=self.dlz, 
                                uav=self.uav, 
                                num_hoops=self.num_hoops, 
                                max_dist=self.max_dist,
                                height=2,
                                spacing=2)
            hoop_poses = course.generate_course()
        self.hoop_positions = hoop_poses
        add_hoops(input_file=self.world_name, output_file=course_params.op_file, hoop_positions=hoop_poses)


