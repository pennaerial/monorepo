import math
import re

import rclpy
from rclpy.executors import ExternalShutdownException
from pydantic import BaseModel, Field

from sim.world_gen.world_node import WorldNode
from sim.entity import Entity

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
        self.cached_templates: dict[str, str] = {}  # model.sdf text keyed by path

        if self.config.seed is not None:
            self.rng.seed(self.config.seed)

    def sample_position(self, placed: list[XY]) -> XY | None:
        """Rejection-sample an xy inside `area` honouring keep_out and min_spacing."""
        (x_min, y_min), (x_max, y_max) = self.config.area
        min_sq = self.config.min_spacing**2

        for _ in range(MAX_PLACEMENT_ATTEMPTS):
            x = self.rng.uniform(x_min, x_max)
            y = self.rng.uniform(y_min, y_max)
            if any((x - kx) ** 2 + (y - ky) ** 2 < kr**2 for kx, ky, kr in self.config.keep_out):
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
                entities.append(entity)

        return entities

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

    def generate_world(self):
        self.shapes = self.random_shapes()
        success = True
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
