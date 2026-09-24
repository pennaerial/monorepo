import math
import re
import json
import os
import time

import rclpy
from pydantic import BaseModel, Field
from rclpy.executors import ExternalShutdownException
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from std_msgs.msg import String

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
TAG_MODEL = "AprilTag36h11" # gz-models dir
TAG_CELLS = 8
TAG_QUIET_CELLS = 1
TAG_Z = 0.012
ANSWER_KEY_TOPIC = "answer_key"


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
    size: float = 0.0254 # Size defaults to 2.54 cm = 1 in
    id_range: tuple[int, int] = (10, 586) # Range of potential ids


class InHouse2026Config(BaseModel):
    """Schema for `world.config` in simulations/in_house_2026/*.yaml"""

    seed: int | None = None  # set for a reproducible layout
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
        self.entities = self.sim_params.world.entities
        self.controllables = self.sim_params.world.controllables
        self.shapes: list[Entity] = []
        self.answer_key: list[dict] = []
        self.answer_key_pub = self.create_publisher(
            String,
            f"/{self.world}/{ANSWER_KEY_TOPIC}",
            QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        )
        self.cached_templates: dict[str, str] = {}  # model.sdf text keyed by path
        # (x, y, radius) zones shapes must avoid
        self.keep_out: list[tuple[float, float, float]] = list(self.config.keep_out)

        if self.config.seed is not None:
            self.rng.seed(self.config.seed)

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
        self.answer_key = []

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
                tag_id = tag_ids.pop() if tag_ids else None
                if tag_id is not None:
                    entity.sdf = self.with_apriltag(entity.sdf, tag_id)
                self.answer_key.append(
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
        """Return `sdf` with a tag plane laid on top of the shape.
        
        Must run AFTER generate_sdf_string, whose recolor regex is global and would repaint the tag.
        """
        # the link <pose> recentres asymmetric meshes (star, triangle); undo it so the
        # tag sits on the shape's visual centre rather than the link origin
        match = re.search(r"<link\b[^>]*>\s*<pose>([^<]*)</pose>", sdf)
        x, y = (float(v) for v in match.group(1).split()[:2]) if match else (0.0, 0.0)
        # the PNG carries a quiet zone, so the plane is wider than the black square
        size = self.config.tags.size * (TAG_CELLS + 2 * TAG_QUIET_CELLS) / TAG_CELLS
        texture = f"model://{TAG_MODEL}/{TAG_FAMILY}_{tag_id:05d}.png"
        visual = (
            '<visual name="apriltag_visual">'
            f"<pose>{-x:g} {-y:g} {TAG_Z:g} 0 0 0</pose>"
            "<cast_shadows>false</cast_shadows>"
            f"<geometry><plane><normal>0 0 1</normal>"
            f"<size>{size:g} {size:g}</size></plane></geometry>"
            "<material><ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>"
            "<specular>0.1 0.1 0.1 1</specular>"
            f"<pbr><metal><albedo_map>{texture}</albedo_map>"
            "<metalness>0</metalness><roughness>0.9</roughness></metal></pbr>"
            "</material></visual>"
        )
        return sdf.replace("</link>", visual + "</link>", 1)

    def publish_answer_key(self) -> None:
        """Publish which tag and color landed on which shape so runs can be graded later."""
        payload = {
            "world": self.world,
            "stage": self.stage,
            "seed": self.config.seed,
            "generated_at": time.strftime("%Y-%m-%dT%H:%M:%SZ", time.gmtime()),
            "tag_family": TAG_FAMILY,
            "tag_size_m": self.config.tags.size,
            "shapes": self.answer_key,
        }
        self.answer_key_pub.publish(String(data=json.dumps(payload)))
        self.get_logger().info(
            f"Published answer key for {len(self.answer_key)} shapes on "
            f"/{self.world}/{ANSWER_KEY_TOPIC}"
        )

    def generate_world(self):
        # reset so re-triggering generate_world doesn't stack keep-out zones
        self.keep_out = list(self.config.keep_out)
        self.shapes = self.random_shapes()
        self.publish_answer_key()
        success = True
        if self.config.border.enabled:
            success = self.spawn_entity(self.border_entity()) and success
        success = self.spawn_entities(self.shapes) and success
        success = self.spawn_entities(self.entities) and success
        success = self.spawn_entities(self.controllables) and success
        return success


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
