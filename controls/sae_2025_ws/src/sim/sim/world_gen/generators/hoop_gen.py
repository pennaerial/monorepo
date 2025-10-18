import random as r
import xml.etree.ElementTree as ET
from pathlib import Path
from xml.dom import minidom
from .world_gen import WorldGenerator
from .courses.ascent import AscentCourse
from .courses.descent import DescentCourse
from .courses.slalom import SlalomCourse
from .courses.straight import StraightCourse


class HoopWorldGenerator(WorldGenerator):
    """
    World generator for the in-house hoop competition.
    Implements specific logic for generating worlds with hoop courses.
    """
    
    AVAILABLE_COURSES = ['ascent', 'descent', 'slalom', 'straight']

    def _generate_course(self, style: str, params: dict) -> list:
        """
        Generate hoop positions based on the specified course style.
        
        Args:
            style (str): Course style ('ascent', 'descent', 'slalom', 'straight', 'random')
            params (dict): Course parameters
            
        Returns:
            list: List of hoop positions (x, y, z, roll, pitch, yaw)
        """
        if style.lower() == 'random':
            style = r.choice(self.AVAILABLE_COURSES)
            
        if style.lower() == "previous":
            return []

        course_generators = {
            "ascent": lambda: AscentCourse(
                dlz=params.dlz,
                uav=params.uav,
                num_hoops=params.num_hoops,
                max_dist=params.max_dist,
                start_height=params.asc_start_height
            ),
            "descent": lambda: DescentCourse(
                dlz=params.dlz,
                uav=params.uav,
                num_hoops=params.num_hoops,
                max_dist=params.max_dist,
                start_height=params.des_start_height
            ),
            "slalom": lambda: SlalomCourse(
                dlz=params.dlz,
                uav=params.uav,
                num_hoops=params.num_hoops,
                max_dist=params.max_dist,
                width=params.width,
                height=params.height
            ),
            "straight": lambda: StraightCourse(
                dlz=params.dlz,
                uav=params.uav,
                num_hoops=params.num_hoops,
                max_dist=params.max_dist,
                height=params.height,
                spacing=params.spacing
            )
        }
        
        if style.lower() not in course_generators:
            raise ValueError(f"Unknown course style: {style}")
            
        course = course_generators[style.lower()]()
        return course.generate_course()

    def _add_to_world(self, hoop_positions: list) -> None:
        """
        Add hoops to the world SDF file.
        
        Args:
            hoop_positions (list): List of hoop positions (x, y, z, roll, pitch, yaw)
        """
        # Parse input SDF
        tree = ET.parse(str(self.input_file))
        root = tree.getroot()

        # Handle namespace if present
        ns = ""
        if root.tag.startswith("{"):
            ns = root.tag.split("}")[0] + "}"

        # Find world element
        world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
        if world is None:
            raise RuntimeError("No <world> tag found in the SDF!")

        # Add hoops
        for i, pos in enumerate(hoop_positions, start=1):
            x, y, z, roll, pitch, yaw = pos
            inc = ET.Element("include")
            ET.SubElement(inc, "uri").text = "model://hoop"
            ET.SubElement(inc, "pose").text = f"{x} {y} {z} {roll} {pitch} {yaw}"
            ET.SubElement(inc, "name").text = f"hoop_{i}"
            world.append(inc)

        # Format and save
        formatted_sdf = self._format_sdf(root)
        Path(self.output_file).write_text(formatted_sdf, encoding="utf-8")
    # Expand ~, make absolute, and validate paths
def generate_world(gen_style: str, course_params) -> None:
    """
    Generate a world with hoops based on the specified course style.
    
    Args:
        gen_style (str): Course generation style ('ascent', 'descent', 'slalom', 'straight', 'random')
        course_params: Object containing course parameters
    """
    generator = HoopWorldGenerator(
        input_file=course_params.ip_file,
        output_file=course_params.op_file
    )
    generator.generate_world(gen_style, course_params)