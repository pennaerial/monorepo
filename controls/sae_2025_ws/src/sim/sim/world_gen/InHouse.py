import json
import logging
import math
import random as r
import sys
import xml.etree.ElementTree as ET
from abc import ABC, abstractmethod
from pathlib import Path
from typing import List, Tuple
from xml.dom import minidom

import rclpy
from sim_interfaces.msg import HoopPose
from sim_interfaces.srv import HoopList
from sim.utils import find_package_resource
from sim.world_gen.WorldGenerator import WorldGenerator
from sim.world_gen.WorldNode import WorldNode

Pose = Tuple[float, float, float, float, float, float]


class CourseStyle(ABC):
    """Abstract base class for all course styles."""

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: int
                ):
        self.dlz = dlz
        self.uav = uav
        self.num_hoops = num_hoops
        self.height = height
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
        super().__init__(dlz, uav, num_hoops, max_dist, start_height)
        self.start_height = start_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        dir_x = dx / dist
        dir_y = dy / dist

        segment_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            along = segment_len * (i + r.uniform(0.2, 0.8))
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)
            z_noise = r.uniform(-0.3, 0.3) * (z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

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
        super().__init__(dlz, uav, num_hoops, max_dist, start_height)
        self.start_height = start_height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        dx = x_dlz - x_uav
        dy = y_dlz - y_uav
        dz = z_dlz - z_uav

        dist = math.sqrt(dx**2 + dy**2 + dz**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        dir_x = dx / dist
        dir_y = dy / dist

        segment_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            along = segment_len * (i + r.uniform(0.2, 0.8))
            frac = (i + 1) / self.num_hoops
            z_interp = self.start_height + frac * (z_dlz - self.start_height)
            z_noise = r.uniform(-0.3, 0.3) * abs(z_dlz - self.start_height) / self.num_hoops

            new_x = x_uav + dir_x * along
            new_y = y_uav + dir_y * along
            new_z = z_interp + z_noise

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
        super().__init__(dlz, uav, num_hoops, max_dist, height)
        self.width = width
        self.height = height

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        dx = x_dlz - x_uav
        dy = y_dlz - y_uav

        dist = math.sqrt(dx**2 + dy**2)
        if dist == 0:
            raise ValueError("DLZ and UAV cannot be at the same position.")

        dir_x = dx / dist
        dir_y = dy / dist

        perp_x = -dir_y
        perp_y = dir_x

        zone_len = dist / self.num_hoops

        for i in range(self.num_hoops):
            alt = 1 if i % 2 == 0 else -1
            along_dist = zone_len * (i + r.uniform(0.2, 0.8))
            offset = alt * self.width * 0.5 * r.uniform(0.5, 1.0)

            new_x = x_uav + dir_x * along_dist + perp_x * offset
            new_y = y_uav + dir_y * along_dist + perp_y * offset
            new_z = r.uniform(self.height * 0.3, self.height * 0.7)

            hoops.append((new_x, new_y, new_z, 0, 90, 0))

        return hoops


class StraightCourse(CourseStyle):
    """Simple straight course with hoops in a straight line."""

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 2.0,
                 spacing: float = 2.0
                ):
        super().__init__(dlz, uav, num_hoops, max_dist, height)
        self.height = height
        self.spacing = spacing

    def generate_course(self) -> List[Pose]:
        hoops = []
        x_uav, y_uav, z_uav = self.uav

        start_x = x_uav + 1.0
        start_y = y_uav

        for i in range(self.num_hoops):
            hoop_x = start_x + (i * self.spacing)
            hoop_y = start_y
            hoop_z = self.height

            hoop_pose = (hoop_x, hoop_y, hoop_z, 0, 90, 0)
            hoops.append(hoop_pose)

        return hoops


class BezierCourse(CourseStyle):
    """Smooth cubic Bezier curve between UAV start and DLZ."""

    def __init__(self,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: float = 2.0,
                 lateral_offset: float = 4.0):
        super().__init__(dlz, uav, num_hoops, max_dist, height)
        self.height = height
        self.lateral_offset = lateral_offset

    def generate_course(self) -> List[Pose]:
        hoops: List[Pose] = []
        x_dlz, y_dlz, z_dlz = self.dlz
        x_uav, y_uav, z_uav = self.uav

        dx = x_dlz - x_uav
        dy = y_dlz - y_uav

        dist_xy = math.hypot(dx, dy)
        dist_3d = math.sqrt(dx**2 + dy**2 + (z_dlz - z_uav)**2)
        if dist_xy == 0 or dist_3d == 0:
            raise ValueError("DLZ and UAV cannot be the same point.")

        dir_x = dx / dist_xy
        dir_y = dy / dist_xy

        perp_x = -dir_y
        perp_y = dir_x

        P0x, P0y = x_uav, y_uav
        P3x, P3y = x_dlz, y_dlz

        one_third = dist_xy / 3.0
        two_third = 2.0 * dist_xy / 3.0

        P1x = P0x + dir_x * one_third + perp_x * self.lateral_offset
        P1y = P0y + dir_y * one_third + perp_y * self.lateral_offset

        P2x = P0x + dir_x * two_third - perp_x * self.lateral_offset
        P2y = P0y + dir_y * two_third - perp_y * self.lateral_offset

        def bezier_xy(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            mt2 = mt * mt
            t2 = t * t
            x = (mt2 * mt) * P0x + 3 * (mt2) * t * P1x + 3 * mt * t2 * P2x + (t2 * t) * P3x
            y = (mt2 * mt) * P0y + 3 * (mt2) * t * P1y + 3 * mt * t2 * P2y + (t2 * t) * P3y
            return x, y

        def bezier_xy_deriv(t: float) -> Tuple[float, float]:
            mt = 1.0 - t
            x = 3 * mt * mt * (P1x - P0x) + 6 * mt * t * (P2x - P1x) + 3 * t * t * (P3x - P2x)
            y = 3 * mt * mt * (P1y - P0y) + 6 * mt * t * (P2y - P1y) + 3 * t * t * (P3y - P2y)
            return x, y

        for i in range(self.num_hoops):
            t = (i + 1) / (self.num_hoops + 1)
            x, y = bezier_xy(t)

            z_linear = z_uav + t * (z_dlz - z_uav)
            z_noise = r.uniform(-0.2, 0.2)
            z = max(0.5, z_linear + z_noise)

            dx_dt, dy_dt = bezier_xy_deriv(t)
            if dx_dt == 0 and dy_dt == 0:
                yaw_deg = 0.0
            else:
                yaw_deg = math.degrees(math.atan2(dy_dt, dx_dt))

            hoops.append((x, y, z, 0.0, yaw_deg, 0.0))

        return hoops


class InHouseGenerator(WorldGenerator):
    """
    World generator for in_house competition.

    Generates a hoop course with various course styles and writes the world file.
    After generation, hoop_positions contains the generated positions which can
    be passed to InHouseNode via create_node().
    """

    COURSES = ['ascent', 'descent', 'slalom', 'bezier', 'straight']

    def __init__(self,
                 course: str,
                 dlz: Tuple[float, float, float],
                 uav: Tuple[float, float, float],
                 num_hoops: int,
                 max_dist: int,
                 height: int):
        self.course = course
        self.dlz = tuple(dlz) if isinstance(dlz, list) else dlz
        self.uav = tuple(uav) if isinstance(uav, list) else uav
        self.num_hoops = num_hoops
        self.max_dist = max_dist
        self.height = height
        self.hoop_positions: List[Pose] = []
        self._logger = logging.getLogger(self.__class__.__name__)

        self.world_name = "world_gen/worlds/in_house.sdf"
        self._validate_parameters()

    def create_node(self) -> 'InHouseNode':
        """Create and return the InHouseNode with hoop positions from generation."""
        return InHouseNode(hoop_positions=self.hoop_positions)

    def get_logger(self):
        return self._logger

    def _validate_parameters(self):
        if self.course not in self.COURSES and self.course != 'random':
            raise ValueError(f"Invalid course type: '{self.course}'. Must be one of {self.COURSES}")
        if self.num_hoops < 1:
            raise ValueError(f"num_hoops must be >= 1, got {self.num_hoops}")
        if self.height < 0:
            raise ValueError(f"height must be >= 0, got {self.height}")
        if self.max_dist < 0:
            raise ValueError(f"max_dist must be >= 0, got {self.max_dist}")
        if not (isinstance(self.dlz, (list, tuple)) and len(self.dlz) == 3):
            raise ValueError(f"dlz must be a list/tuple of 3 numbers, got {self.dlz}")
        if not (isinstance(self.uav, (list, tuple)) and len(self.uav) == 3):
            raise ValueError(f"uav must be a list/tuple of 3 numbers, got {self.uav}")
        if self.course not in ['previous', 'straight']:
            if tuple(self.uav) == tuple(self.dlz):
                raise ValueError("UAV and DLZ coordinates cannot be identical for this course type")

    def generate_world(self, output_dir: Path) -> None:
        """Generate the world file with hoops and DLZs."""
        output_path = output_dir / "in_house.sdf"

        course = self.course
        if course == 'random':
            course = r.choice(self.COURSES)
            self.course = course

        base_params = {
            "dlz": self.dlz,
            "uav": self.uav,
            "num_hoops": self.num_hoops,
            "max_dist": self.max_dist
        }

        if course == "ascent":
            generator = AscentCourse(**{**base_params, "start_height": self.height})
        elif course == "descent":
            generator = DescentCourse(**{**base_params, "start_height": self.height})
        elif course == "slalom":
            generator = SlalomCourse(**{**base_params, "width": 4, "height": self.height})
        elif course == "straight":
            generator = StraightCourse(**{**base_params, "height": self.height, "spacing": 2})
        elif course == "bezier":
            generator = BezierCourse(**{**base_params, "height": self.height, "lateral_offset": 4.0})
        else:
            raise ValueError(f"Invalid course: {course}")

        self.hoop_positions = generator.generate_course()
        self._logger.info(f"Generated {len(self.hoop_positions)} hoops for {course} course")

        self._write_world_file(output_path)

    def _write_world_file(self, output_path: Path) -> None:
        """Write the world SDF file with hoops and DLZs."""
        in_path = find_package_resource(
            relative_path=self.world_name,
            package_name='sim',
            resource_type='file',
            logger=self._logger,
            base_file=Path(__file__)
        )

        output_path.parent.mkdir(parents=True, exist_ok=True)

        tree = ET.parse(str(in_path))
        root = tree.getroot()

        ns = ""
        if root.tag.startswith("{"):
            ns = root.tag.split("}")[0] + "}"

        world = root.find(f"{ns}world") or root.find("world") or root.find(".//world")
        if world is None:
            raise RuntimeError("No <world> tag found in the SDF!")

        # Add hoops
        for i, pos in enumerate(self.hoop_positions, start=1):
            inc = ET.Element("include")
            x, y, z, roll, pitch, yaw = pos
            z = 1 if z < 1 else z
            ET.SubElement(inc, "uri").text = "model://hoop"
            ET.SubElement(inc, "pose").text = f"{x} {y} {z} {roll} {pitch} {yaw}"
            ET.SubElement(inc, "name").text = f"hoop_{i}"
            world.append(inc)

        # Add DLZs
        if len(self.hoop_positions) >= 2:
            (prev_x, prev_y, *_), (end_x, end_y, *_) = self.hoop_positions[-2], self.hoop_positions[-1]
        else:
            start_x, start_y, _ = self.uav
            end_x, end_y, *_ = self.hoop_positions[-1]
            prev_x, prev_y = start_x, start_y

        dir_x = end_x - prev_x
        dir_y = end_y - prev_y
        length = math.hypot(dir_x, dir_y)
        if length == 0:
            dir_x, dir_y = 1.0, 0.0
        else:
            dir_x /= length
            dir_y /= length

        perp_x = -dir_y
        perp_y = dir_x
        spacing = 3.0
        yaw = math.atan2(dir_y, dir_x)

        colors = [("red", "1 0 0 1"), ("green", "0 1 0 1"), ("blue", "0 0 1 1")]

        for i in range(3):
            offset = (i - 1) * spacing
            dlz_x = end_x + offset * perp_x
            dlz_y = end_y + offset * perp_y

            color_name, rgba = colors[i]

            dlz_inc = ET.Element("include")
            ET.SubElement(dlz_inc, "uri").text = "model://dlz_" + color_name
            ET.SubElement(dlz_inc, "name").text = f"dlz_{i+1}"
            ET.SubElement(dlz_inc, "pose").text = f"{dlz_x} {dlz_y} 0 0 0 {yaw}"

            model_override = ET.SubElement(dlz_inc, "model")
            link_override = ET.SubElement(model_override, "link", {"name": "link"})
            visual_override = ET.SubElement(link_override, "visual", {"name": "visual"})
            material = ET.SubElement(visual_override, "material")
            diffuse = ET.SubElement(material, "diffuse")
            diffuse.text = rgba

            world.append(dlz_inc)

        def strip_whitespace(elem):
            if elem.text is not None and elem.text.strip() == "":
                elem.text = None
            for child in list(elem):
                strip_whitespace(child)
                if child.tail is not None and child.tail.strip() == "":
                    child.tail = None
        strip_whitespace(root)

        rough_bytes = ET.tostring(root, encoding="utf-8")
        pretty = minidom.parseString(rough_bytes.decode("utf-8")).toprettyxml(indent="  ")
        lines = [ln for ln in pretty.splitlines() if ln.strip() != ""]
        output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
        self._logger.info(f"Generated world file: {output_path}")


class InHouseNode(WorldNode):
    """
    ROS node for in_house competition.

    Created by InHouseGenerator.create_node() with hoop positions from generation.
    Provides the list_hoops service for querying hoop positions.
    """

    def __init__(self, hoop_positions: List[Pose]):
        super().__init__()

        self.hoop_positions = hoop_positions

        # Set up ROS services
        self.srv = self.create_service(HoopList, "list_hoops", self._hoop_list_callback)
        self.get_logger().info("World node ready")  # Standard marker for launch detection
        self.get_logger().info(f"InHouseNode ready with {len(self.hoop_positions)} hoops")

    def _hoop_list_callback(self, request, response):
        """Handle list_hoops service request."""
        response.hoop_positions = []
        for (x, y, z, roll, pitch, yaw) in self.hoop_positions:
            hp = HoopPose()
            hp.x = float(x)
            hp.y = float(y)
            hp.z = float(z)
            hp.roll = float(roll)
            hp.pitch = float(pitch)
            hp.yaw = float(yaw)
            response.hoop_positions.append(hp)
        return response


def main(args=None):
    """Entry point for running InHouseNode."""
    rclpy.init(args=args)

    params = json.loads(sys.argv[1])
    # hoop_positions is passed as a list of lists from JSON
    hoop_positions = [tuple(p) for p in params['hoop_positions']]

    node = InHouseNode(hoop_positions=hoop_positions)

    try:
        rclpy.spin(node)
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        rclpy.shutdown()
