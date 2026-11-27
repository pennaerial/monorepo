"""GPS fix types and the RTK monitor that fires the StopTrigger."""

from __future__ import annotations

from typing import Any

from px4_msgs.msg import SensorGps
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy

from iarc_flight.runtime import per_node
from iarc_flight.runtime.stop import StopTrigger

RTK_FIXED = 6
FIX_TYPES = {  # SensorGps.fix_type by name
    "no_fix": 1,
    "2d": 2,
    "3d": 3,
    "dgps": 4,
    "rtk_float": 5,
    "rtk_fixed": RTK_FIXED,
    "extrapolated": 8,
}

# PX4's DDS bridge QoS, publisher must be TRANSIENT_LOCAL too
PX4_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10,
)


def fix_name(fix_type: int) -> str:
    names = {v: k for k, v in FIX_TYPES.items()} | {0: "no_fix"}
    return names.get(fix_type, str(fix_type))


class RtkMonitor:
    """While `watching` (its TrackMode is active), fires the StopTrigger's rtk_loss source when
    GPS isn't RTK fixed"""

    @classmethod
    def get(cls, node: Any, vehicle: Any, topic: str | None, timeout: float) -> RtkMonitor:
        topic = topic or vehicle.px4_transport_path("out/vehicle_gps_position")
        return per_node(node, f"rtk_monitor:{topic}:{timeout}", lambda: cls(node, topic, timeout))

    def __init__(self, node: Any, topic: str, timeout: float) -> None:
        self.node = node
        self.topic = topic
        self.timeout = timeout
        self.trigger = StopTrigger.get(node)
        self.fix: int | None = None
        self.stamp = 0.0  # s, node clock at the last message
        self.watching = False
        self.sub = node.create_subscription(SensorGps, topic, self.on_gps, PX4_QOS)
        self.timer = node.create_timer(0.1, self.check)

    def now(self) -> float:
        return self.node.get_clock().now().nanoseconds * 1e-9

    def watch(self, on: bool) -> None:
        self.watching = on
        if on:
            self.check()  # not RTK fixed on entry fires right away

    def on_gps(self, msg: SensorGps) -> None:
        self.fix, self.stamp = int(msg.fix_type), self.now()
        if self.watching and self.fix != RTK_FIXED:
            self.trigger.fire("rtk_loss", f"RTK fixed lost ({fix_name(self.fix)})")

    def check(self) -> None:
        if not self.watching:
            return
        if self.fix is None or self.now() - self.stamp > self.timeout:
            self.trigger.fire("rtk_loss", f"no GPS fix on {self.topic} for {self.timeout:.1f} s")
        elif self.fix != RTK_FIXED:
            self.trigger.fire("rtk_loss", f"not RTK fixed ({fix_name(self.fix)})")
