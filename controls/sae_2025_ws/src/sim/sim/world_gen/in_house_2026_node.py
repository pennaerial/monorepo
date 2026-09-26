import math
import os
import re
import time
import xml.etree.ElementTree as ET

import rclpy
from gz.math7 import Angle, SphericalCoordinates, Vector3d
from pydantic import BaseModel, Field
from rclpy.executors import ExternalShutdownException
from sim_interfaces.msg import ObjectState, SearchLocation
from sim_interfaces.srv import GetSearchLocations, ObjectStateList

from sim.entity import Entity
from sim.world_gen.world_node import WorldNode

RGBA = tuple[float, float, float, float]
XY = tuple[float, float]

SHAPE_MODELS = {
    "circle": "InHouse2027Circle",
    "square": "InHouse2027Square",
    "triangle": "InHouse2027Triangle",
    "star": "InHouse2027Star",
}

# Give up on a shape after this many rejected samples (area too crowded / keep_out too big)
MAX_PLACEMENT_ATTEMPTS = 200

TAG_FAMILY = "tag36h11"
TAG_MODEL = "AprilTag36h11"  # gz-models dir
TAG_CELLS = 8
TAG_QUIET_CELLS = 1
TAG_Z = 0.012
OBJECT_STATE_SERVICE = "publish_object_state"
NO_TAG_ID = -1


class Material(BaseModel):
    """SDF <material> colours, each RGBA in 0-1."""

    ambient: RGBA
    diffuse: RGBA
    specular: RGBA = (0.01, 0.01, 0.01, 1.0)
    emissive: RGBA = (0.0, 0.0, 0.0, 1.0)


DEFAULT_PALETTE = {
    "red": Material(ambient=(1, 0, 0, 1), diffuse=(1, 0, 0, 1)),
    "green": Material(ambient=(0, 1, 0, 1), diffuse=(0, 1, 0, 1)),
    "blue": Material(ambient=(0, 0, 1, 1), diffuse=(0, 0, 1, 1)),
}


BORDER_MODEL = "InHouse2026Border"

DEFAULT_BORDER_MATERIAL = Material(
    ambient=(0.6, 0.0, 0.0, 1.0),
    diffuse=(1.0, 0.0, 0.0, 1.0),
    emissive=(0.25, 0.0, 0.0, 1.0),
)


class BorderConfig(BaseModel):
    """Red outline drawn around the shape spawn `area`."""

    enabled: bool = True
    margin: float = 0.5
    thickness: float = 0.15
    height: float = 0.02
    material: Material = Field(default_factory=lambda: DEFAULT_BORDER_MATERIAL.model_copy())


class TagConfig(BaseModel):
    """AprilTag decal stamped on each spawned shape."""

    enabled: bool = True
    size: float = 0.0254  # Size defaults to 2.54 cm = 1 in
    id_range: tuple[int, int] = (10, 586)  # Range of potential ids


class InHouse2026Config(BaseModel):
    """Schema for `world.config` in simulations/in_house_2026/*.yaml"""

    seed: int | None = None  # set for a reproducible layout
    num_patches: int = Field(default=1, ge=1)
    area: tuple[XY, XY] = ((-10.0, -10.0), (10.0, 10.0))  # xy min / xy max
    counts: dict[str, int] = Field(
        default_factory=lambda: {"circle": 3, "square": 3, "triangle": 3, "star": 3}
    )
    min_spacing: float = 2.0  # centre-to-centre, metres
    keep_out: list[tuple[float, float, float]] = Field(default_factory=list)  # (x, y, radius)
    random_yaw: bool = True
    palette: dict[str, Material] = Field(default_factory=lambda: dict(DEFAULT_PALETTE))
    border: BorderConfig = Field(default_factory=BorderConfig)
    tags: TagConfig = Field(default_factory=TagConfig)


class InHouse2026WorldNode(WorldNode):
    """
    World generation node for the In House 2026 challenge.
    Litters the ground with randomly placed, randomly coloured InHouse2027 shapes.
    """

    def __init__(self):
        super().__init__("in_house_2026_node")
        self.config = InHouse2026Config.model_validate(self.sim_params.world.config)
        self.spherical_coordinates = self.load_world_coordinates()
        self.entities = self.sim_params.world.entities
        self.controllables = self.sim_params.world.controllables
        self.shapes: list[Entity] = []
        self.object_states: list[dict] = []
        self.generated_at = ""
        self.object_state_srv = self.create_service(
            ObjectStateList, OBJECT_STATE_SERVICE, self.object_state_req
        )
        self.cached_templates: dict[str, str] = {}  # model.sdf text keyed by path
        # (x, y, radius) zones shapes must avoid
        self.keep_out: list[tuple[float, float, float]] = list(self.config.keep_out)

        # shape search location query variables
        self.search_locations: list[SearchLocation] = []
        self.target_tag_id: int = -1
        self.query_search_locations_service = self.create_service(
            GetSearchLocations, "get_search_patches", self.get_patches_callback
        )
        self.patches_ready = False
        self.generation_started = False
        self.pending_spawns = 0
        self.spawn_failed = False

        if self.config.seed is not None:
            self.rng.seed(self.config.seed)

    def get_patches_callback(
        self, request: GetSearchLocations.Request, response: GetSearchLocations.Response
    ) -> GetSearchLocations.Response:
        response.ready = self.patches_ready
        response.target_tag_id = self.target_tag_id
        response.search_locations = self.search_locations
        return response

    def sample_position(self, placed: list[XY]) -> XY | None:
        """Rejection-sample an xy inside `area` honouring keep_out and min_spacing."""
        (x_min, y_min), (x_max, y_max) = self.config.area
        min_sq = self.config.min_spacing**2

        for _ in range(MAX_PLACEMENT_ATTEMPTS):
            x = self.rng.uniform(x_min, x_max)
            y = self.rng.uniform(y_min, y_max)
            if any((x - kx) ** 2 + (y - ky) ** 2 < kr**2 for kx, ky, kr in self.keep_out):
                continue
            if any((x - px) ** 2 + (y - py) ** 2 < min_sq for px, py in placed):
                continue
            return (x, y)
        return None

    def random_shapes(self) -> list[Entity]:
        """Generate the shape entities described by the config."""
        entities: list[Entity] = []
        placed: list[XY] = []
        color_names = list(self.config.palette)

        tag_ids: list[int] = []
        if self.config.tags.enabled:
            total = sum(self.config.counts.values())
            available = self.available_tag_ids()
            if len(available) < total:
                raise ValueError(
                    f"{total} shapes but only {len(available)} tag PNGs in "
                    f"{TAG_MODEL}; add more with gz-models/tools/generate_apriltags.py"
                )
            tag_ids = self.rng.sample(available, total)
        self.object_states = []

        for shape, count in self.config.counts.items():
            model_name = SHAPE_MODELS.get(shape)
            if not model_name:
                self.get_logger().error(f"Unknown shape '{shape}' in counts; skipping.")
                continue

            for i in range(count):
                xy = self.sample_position(placed)
                if xy is None:
                    self.get_logger().warning(
                        f"Could not place {shape}_{i} after {MAX_PLACEMENT_ATTEMPTS} attempts; "
                        "skipping (area too crowded?)"
                    )
                    continue
                placed.append(xy)

                yaw = self.rng.uniform(0.0, 2 * math.pi) if self.config.random_yaw else 0.0
                entity = Entity(
                    name=f"{shape}_{i}",
                    model=model_name,
                    position=(xy[0], xy[1], 0.0),
                    rpy=(0.0, 0.0, yaw),
                    world=self.world,
                )
                # Entity resolved the on-disk model.sdf; recolour a copy of it for this instance
                color = self.rng.choice(color_names)
                entity.sdf = self.generate_sdf_string(
                    entity.path_to_model, self.config.palette[color]
                )
                tag_id = tag_ids.pop() if tag_ids else NO_TAG_ID
                if tag_id != NO_TAG_ID:
                    entity.sdf = self.with_apriltag(entity.sdf, tag_id)
                self.object_states.append(
                    {
                        "name": entity.name,
                        "shape": shape,
                        "model": model_name,
                        "color": color,
                        "tag_id": tag_id,
                        "x": round(xy[0], 4),
                        "y": round(xy[1], 4),
                        "yaw": round(yaw, 4),
                    }
                )
                entities.append(entity)

        return entities

    def build_border_sdf(self) -> str:
        """Return an SDF model outlining `area` in red."""
        border = self.config.border
        (x_min, y_min), (x_max, y_max) = self.config.area
        x_min, y_min = x_min - border.margin, y_min - border.margin
        x_max, y_max = x_max + border.margin, y_max + border.margin

        t = border.thickness
        z = border.height / 2
        # strips are centred on the boundary lines; overlap by `t` so the corners join
        span_x = (x_max - x_min) + t
        span_y = (y_max - y_min) + t
        mid_x = (x_min + x_max) / 2
        mid_y = (y_min + y_max) / 2

        strips = {
            "border_y_min": ((mid_x, y_min), (span_x, t)),
            "border_y_max": ((mid_x, y_max), (span_x, t)),
            "border_x_min": ((x_min, mid_y), (t, span_y)),
            "border_x_max": ((x_max, mid_y), (t, span_y)),
        }

        material = "".join(
            f"<{tag}>{' '.join(f'{c:g}' for c in rgba)}</{tag}>"
            for tag, rgba in border.material.model_dump().items()
        )

        # no collision
        visuals = "".join(
            f'<visual name="{name}">'
            f"<pose>{cx:g} {cy:g} {z:g} 0 0 0</pose>"
            f"<geometry><box><size>{sx:g} {sy:g} {border.height:g}</size></box></geometry>"
            f"<material>{material}</material>"
            f"</visual>"
            for name, ((cx, cy), (sx, sy)) in strips.items()
        )

        return (
            '<?xml version="1.0"?>'
            '<sdf version="1.9">'
            f'<model name="{BORDER_MODEL}">'
            "<static>true</static>"
            f'<link name="link">{visuals}</link>'
            "</model>"
            "</sdf>"
        )

    def border_entity(self) -> Entity:
        """Border entity whose on-disk model.sdf is replaced by geometry matching `area`."""
        entity = Entity(
            name="spawn_border",
            model=BORDER_MODEL,
            position=(0.0, 0.0, 0.0),
            rpy=(0.0, 0.0, 0.0),
            world=self.world,
        )
        entity.sdf = self.build_border_sdf()
        return entity

    def generate_sdf_string(self, template_path: str, material: Material) -> str:
        """Return the model.sdf at `template_path` with its <material> colours replaced."""
        sdf = self.cached_templates.get(template_path)
        if sdf is None:
            with open(template_path) as f:
                sdf = f.read()
            self.cached_templates[template_path] = sdf

        for tag, rgba in material.model_dump().items():
            value = " ".join(f"{c:g}" for c in rgba)
            sdf = re.sub(rf"<{tag}>[^<]*</{tag}>", f"<{tag}>{value}</{tag}>", sdf)
        return sdf

    def available_tag_ids(self) -> list[int]:
        """Tag ids committed under $PENNAIR_GZ_MODELS_PATH/models/AprilTag36h11."""
        tag_dir = f"{os.environ['PENNAIR_GZ_MODELS_PATH']}/models/{TAG_MODEL}"
        prefix = f"{TAG_FAMILY}_"
        return sorted(
            int(name[len(prefix) : -len(".png")])
            for name in os.listdir(tag_dir)
            if name.startswith(prefix) and name.endswith(".png")
        )

    def with_apriltag(self, sdf: str, tag_id: int) -> str:
        """Return `sdf` with a tag plane appended to the shape's link.

        Must run AFTER generate_sdf_string, whose recolour regex is global and would
        otherwise repaint the tag's <ambient>/<diffuse> the shape's colour.
        """
        root = ET.fromstring(sdf)
        link = root.find("./model/link")
        if link is None:
            self.get_logger().warning(f"No <model>/<link> found; skipping tag {tag_id}")
            return sdf

        # the link <pose> recentres asymmetric meshes (star, triangle); undo it so the
        # tag sits on the shape's visual centre rather than the link origin
        pose = link.find("pose")
        x, y = (float(v) for v in pose.text.split()[:2]) if pose is not None else (0.0, 0.0)
        tag_x, tag_y = -x or 0.0, -y or 0.0  # avoid rendering "-0"
        # the PNG carries a quiet zone, so the plane is wider than the black square
        size = self.config.tags.size * (TAG_CELLS + 2 * TAG_QUIET_CELLS) / TAG_CELLS
        texture = f"model://{TAG_MODEL}/{TAG_FAMILY}_{tag_id:05d}.png"

        link.append(
            ET.fromstring(
                '<visual name="apriltag_visual">'
                f"<pose>{tag_x:g} {tag_y:g} {TAG_Z:g} 0 0 0</pose>"
                "<cast_shadows>false</cast_shadows>"
                f"<geometry><plane><normal>0 0 1</normal>"
                f"<size>{size:g} {size:g}</size></plane></geometry>"
                "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
                "<specular>0.1 0.1 0.1 1</specular>"
                f"<pbr><metal><albedo_map>{texture}</albedo_map>"
                "<metalness>0</metalness><roughness>0.9</roughness></metal></pbr>"
                "</material></visual>"
            )
        )
        return ET.tostring(root, encoding="unicode")

    def object_state_req(self, request, response):
        """Serve the tag, colour and pose of every spawned shape so runs can be graded."""
        response.world = self.world
        response.stage = self.stage
        response.generated_at = self.generated_at
        response.tag_family = TAG_FAMILY
        response.tag_size_m = float(self.config.tags.size)
        response.objects = []
        for row in self.object_states:
            obj = ObjectState()
            obj.name = row["name"]
            obj.shape = row["shape"]
            obj.model = row["model"]
            obj.color = row["color"]
            obj.tag_id = int(row["tag_id"])
            obj.x = float(row["x"])
            obj.y = float(row["y"])
            obj.yaw = float(row["yaw"])
            response.objects.append(obj)
        return response

    def load_world_coordinates(self) -> SphericalCoordinates:
        """Read the world's geographic reference once when the node starts."""
        world_path = os.path.join(
            os.environ["PENNAIR_GZ_MODELS_PATH"], "worlds", f"{self.world}.sdf"
        )
        coordinates = ET.parse(world_path).find("./world/spherical_coordinates")
        if coordinates is None:
            raise ValueError(f"No geographic origin found in {world_path}")
        if (
            coordinates.findtext("surface_model", "EARTH_WGS84") != "EARTH_WGS84"
            or coordinates.findtext("world_frame_orientation", "ENU") != "ENU"
        ):
            raise ValueError("Patch GPS conversion requires an EARTH_WGS84, ENU world")

        return SphericalCoordinates(
            SphericalCoordinates.EARTH_WGS84,
            Angle(math.radians(float(coordinates.findtext("latitude_deg", "0")))),
            Angle(math.radians(float(coordinates.findtext("longitude_deg", "0")))),
            float(coordinates.findtext("elevation", "0")),
            Angle(math.radians(float(coordinates.findtext("heading_deg", "0")))),
        )

    def world_xy_to_gps(self, center_xy: XY) -> tuple[float, float]:
        """Convert a ground point (world z=0) to latitude/longitude in degrees."""
        # Match Gazebo NavSat: LOCAL2 handles world heading; SPHERICAL uses radians.
        geographic = self.spherical_coordinates.position_transform(
            Vector3d(center_xy[0], center_xy[1], 0.0),
            SphericalCoordinates.LOCAL2,
            SphericalCoordinates.SPHERICAL,
        )
        return math.degrees(geographic.x()), math.degrees(geographic.y())

    def sample_patch_center(self, radius_m: float, placed: list[XY]) -> XY:
        """Samples patch locations and rejecting overlaps"""
        (x_min, y_min), (x_max, y_max) = self.config.area
        if x_max - x_min < 2 * radius_m or y_max - y_min < 2 * radius_m:
            raise ValueError("Area is too small for the patch radius")
        separation = 2 * radius_m + self.config.min_spacing
        for _ in range(MAX_PLACEMENT_ATTEMPTS):
            x = self.rng.uniform(x_min + radius_m, x_max - radius_m)
            y = self.rng.uniform(y_min + radius_m, y_max - radius_m)
            if any(math.hypot(x - px, y - py) < separation for px, py in placed):
                continue
            if any(math.hypot(x - kx, y - ky) < radius_m + kr for kx, ky, kr in self.keep_out):
                continue
            return x, y
        raise ValueError("Could not place all patches; enlarge area or reduce num_patches")

    def sample_patch_position(self, center_xy: XY, radius_m: float, placed: list[XY]) -> XY:
        """Sample a shape center inside the circle, respecting spacing and keep-out zones."""
        cx, cy = center_xy
        for _ in range(MAX_PLACEMENT_ATTEMPTS):
            x = self.rng.uniform(cx - radius_m, cx + radius_m)
            y = self.rng.uniform(cy - radius_m, cy + radius_m)
            if (x - cx) ** 2 + (y - cy) ** 2 > radius_m**2:
                continue
            if any((x - kx) ** 2 + (y - ky) ** 2 < kr**2 for kx, ky, kr in self.keep_out):
                continue
            if any((x - px) ** 2 + (y - py) ** 2 < self.config.min_spacing**2 for px, py in placed):
                continue
            return x, y
        raise ValueError("Could not fit all patch shapes; increase radius or reduce spacing")

    def generate_patch(
        self,
        center_xy: XY,
        radius_m: float,
        contains_target: bool,
        target_tag_id: int,
        name_prefix: str = "patch_0",
    ) -> list[Entity]:
        """Generate four gold stars and twelve random shapes; reserve the target tag."""
        if radius_m <= 0 or not self.config.tags.enabled:
            raise ValueError("Patches require a positive radius and enabled AprilTags")
        available_tags = self.available_tag_ids()
        decoy_tags = [tag for tag in available_tags if tag != target_tag_id]
        if target_tag_id not in available_tags or not decoy_tags:
            raise ValueError("Patches require an available target tag and at least one decoy tag")
        background_colors = ["gold"] + [color for color in self.config.palette if color != "gold"]

        gold = Material(ambient=(1.0, 0.84, 0.0, 1.0), diffuse=(1.0, 0.84, 0.0, 1.0))
        entities: list[Entity] = []
        object_states: list[dict] = []
        placed: list[XY] = []
        for i in range(16):
            # The target is the first of four gold stars, never an extra shape.
            shape = "star" if i < 4 else self.rng.choice(list(SHAPE_MODELS))
            color = "gold" if i < 4 else self.rng.choice(background_colors)
            tag_id = target_tag_id if contains_target and i == 0 else self.rng.choice(decoy_tags)
            x, y = self.sample_patch_position(center_xy, radius_m, placed)
            placed.append((x, y))
            yaw = self.rng.uniform(0.0, 2 * math.pi) if self.config.random_yaw else 0.0
            entity = Entity(
                name=f"{name_prefix}_{shape}_{i}",
                model=SHAPE_MODELS[shape],
                position=(x, y, 0.0),
                rpy=(0.0, 0.0, yaw),
                world=self.world,
            )
            material = gold if color == "gold" else self.config.palette[color]
            entity.sdf = self.generate_sdf_string(entity.path_to_model, material)
            entity.sdf = self.with_apriltag(entity.sdf, tag_id)
            entities.append(entity)
            object_states.append(
                {
                    "name": entity.name,
                    "shape": shape,
                    "model": entity.model,
                    "color": color,
                    "tag_id": tag_id,
                    "x": x,
                    "y": y,
                    "yaw": yaw,
                }
            )

        self.object_states.extend(object_states)
        return entities

    def spawn_next_entity(self) -> bool:
        """Send one request at a time so the bridge is not flooded with spawns."""
        entity = next(self.spawn_queue)
        if not self.spawn_entity(entity):
            self.spawn_failed = True
            return False
        return True

    def _log_spawn_result(self, name: str, future) -> None:
        """Keep the existing spawn logging and mark ready only after all replies succeed."""
        super()._log_spawn_result(name, future)
        try:
            if not future.result().success:
                self.spawn_failed = True
        except Exception:
            self.spawn_failed = True
        self.pending_spawns -= 1
        if self.pending_spawns > 0:
            self.spawn_next_entity()
        elif not self.spawn_failed:
            self.patches_ready = True
            self.get_logger().info("Patches ready; call /get_search_patches for the challenge")

    def generate_world(self):
        if self.generation_started:
            self.get_logger().error("Generation already attempted; restart the world and node")
            return False
        self.generation_started = True
        self.shapes = []
        self.object_states = []
        self.search_locations = []
        self.patches_ready = False
        self.keep_out = list(self.config.keep_out)

        try:
            available_tags = self.available_tag_ids()
            if not self.config.tags.enabled or len(available_tags) < 2:
                raise ValueError("Enable AprilTags and provide at least two tag images")
            target_patch = self.rng.randrange(self.config.num_patches)
            self.target_tag_id = self.rng.choice(available_tags)

            radius_m = 4.0
            centers: list[XY] = []
            for i in range(self.config.num_patches):
                center_xy = self.sample_patch_center(radius_m, centers)
                centers.append(center_xy)
                patch_entities = self.generate_patch(
                    center_xy=center_xy,
                    radius_m=radius_m,
                    contains_target=(i == target_patch),
                    target_tag_id=self.target_tag_id,
                    name_prefix=f"patch_{i}",
                )
                self.shapes.extend(patch_entities)

                latitude, longitude = self.world_xy_to_gps(center_xy)
                location = SearchLocation()
                location.latitude_deg = latitude
                location.longitude_deg = longitude
                location.radius_m = radius_m
                self.search_locations.append(location)
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"Patch generation failed: {exc}")
            return False

        self.generated_at = time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime())
        entities = self.shapes + self.entities + self.controllables
        self.pending_spawns = len(entities)
        self.spawn_queue = iter(entities)
        return self.spawn_next_entity()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = InHouse2026WorldNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    except Exception as e:
        print(e)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
