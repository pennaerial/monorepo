import time
from pathlib import Path

import rclpy
from ament_index_python import get_package_share_path
from pydantic import BaseModel, model_validator
from pydantic import Field as PydanticField
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String

from sim.entity import Entity
from sim.minefield import (
    Field,
    FieldStyle,
    Layout,
    RandomConfig,
    dump_layout,
    field_model_sdf,
    generate_random,
    load_layout,
    write_csv,
)
from sim.minefield.sdf import FIELD_MODEL
from sim.world_gen.world_node import WorldNode

MINE_MODEL = "PFM1"  # gz-models dir; its model.sdf is replaced by the merged field model
READY_TOPIC = "/iarc_field/ready"  # std_msgs/String: path of this run's ground-truth YAML


class IarcFieldConfig(BaseModel):
    """Schema for `world.config` in simulations/iarcField/*.yaml. Give `layout` or `random`."""

    # Layout YAML; a relative path is looked up in the installed simulations/<world>/ dir
    layout: str | None = None
    random: RandomConfig | None = None
    field: Field = PydanticField(default_factory=Field)  # only used by `random`
    style: FieldStyle = PydanticField(default_factory=FieldStyle)
    # The resolved layout (= ground truth) and a CSV of it are written here every run
    output_dir: str = "~/.ros/iarcField"

    @model_validator(mode="after")
    def one_source(self) -> "IarcFieldConfig":
        if (self.layout is None) == (self.random is None):
            raise ValueError("Set exactly one of config.layout or config.random")
        return self


class IarcFieldWorldNode(WorldNode):
    """
    World generation node for the IARC mine field (80 x 300 ft, 2 ft cells).
    Spawns the whole field, markings plus every PFM-1, as one static model.
    """

    def __init__(self):
        super().__init__("iarc_field_node")
        self.config = IarcFieldConfig.model_validate(self.sim_params.world.config)
        self.entities = self.sim_params.world.entities
        self.controllables = self.sim_params.world.controllables
        self.layout: Layout | None = None
        self.layout_path = ""
        self.generation_started = False
        self.pending_spawns = 0
        self.spawn_failed = False
        # Latched: subscribers that join late still get it. Carries the ground-truth path.
        self.ready_pub = self.create_publisher(
            String,
            READY_TOPIC,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL),
        )

    def resolve_layout_path(self, path: str) -> Path:
        layout_path = Path(path).expanduser()
        if not layout_path.is_absolute():
            layout_path = get_package_share_path("sim") / "simulations" / self.world / layout_path
        return layout_path

    def build_layout(self) -> Layout:
        if self.config.layout is not None:
            return load_layout(self.resolve_layout_path(self.config.layout))
        return generate_random(self.config.field, self.config.random)

    def save_ground_truth(self, layout: Layout) -> Path:
        stamp = time.strftime("%Y%m%dT%H%M%SZ", time.gmtime())
        out = Path(self.config.output_dir).expanduser() / f"{self.stage}_{stamp}_seed{layout.seed}"
        dump_layout(layout, out.with_suffix(".yaml"))
        write_csv(layout, out.with_suffix(".csv"))
        return out.with_suffix(".yaml")

    def _log_spawn_result(self, name: str, future) -> None:
        """Keep the base logging and announce the field once every spawn has succeeded."""
        super()._log_spawn_result(name, future)
        try:
            if not future.result().success:
                self.spawn_failed = True
        except Exception:
            self.spawn_failed = True
        self.pending_spawns -= 1
        if self.pending_spawns == 0 and not self.spawn_failed:
            self.ready_pub.publish(String(data=self.layout_path))
            self.get_logger().info(f"iarcField ready; published {READY_TOPIC}")

    def generate_world(self) -> bool:
        if self.generation_started:
            self.get_logger().error("Generation already attempted; restart the world and node")
            return False
        self.generation_started = True
        try:
            self.layout = self.build_layout()
            self.layout_path = str(self.save_ground_truth(self.layout))
            field = Entity(
                name=FIELD_MODEL,
                model=MINE_MODEL,
                position=(0.0, 0.0, 0.0),
                rpy=(0.0, 0.0, 0.0),
                world=self.world,
            )
            field.sdf = field_model_sdf(self.layout, self.config.style)
        except (ValueError, OSError) as exc:
            self.get_logger().error(f"iarcField generation failed: {exc}")
            return False

        self.get_logger().info(
            f"{len(self.layout.mines)} mines (seed={self.layout.seed}); "
            f"ground truth written to {self.layout_path}"
        )
        entities = [field] + self.entities + self.controllables
        self.pending_spawns = len(entities)
        return self.spawn_entities(entities)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = IarcFieldWorldNode()
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
