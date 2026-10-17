from __future__ import annotations

import math
from typing import override

from px4_msgs.msg import TrajectorySetpoint
from rclpy.node import Node
from uav.vehicles.UAV import UAV
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from iarc.planning import circle_state


class CircleParams(ParamsBase):
    """
    radius: Circle radius in meters.
    initial_speed: Speed when the circle starts, m/s.
    tangential_accel: Rate the speed changes along the circle, m/s^2.
    max_speed: Speed cap in m/s, or null for no cap.
    altitude: Height above the origin in meters (positive up), or null to keep the current height.
    center: (x, y) center in local NED, or null to place the circle so it starts at the drone.
    laps: Number of times around before the mode completes.
    face_velocity: If true, yaw along the direction of travel; otherwise hold the starting yaw.
    """

    radius: float = 5.0
    initial_speed: float = 1.0
    tangential_accel: float = 0.2
    max_speed: float | None = 5.0
    altitude: float | None = None
    center: tuple[float, float] | None = None
    laps: float = 1.0
    face_velocity: bool = True


@register_mode(
    id="iarc.CircleMode",
    params_cls=CircleParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class CircleMode(Mode[UAV, CircleParams]):
    """Flies a circle by streaming position, velocity and acceleration setpoints over time."""

    @override
    def initialize(self, node: Node, vehicle: UAV, params: CircleParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params
        self.t = 0.0
        self.circle = None  # (cx, cy, altitude, start_angle, yaw), fixed on the first update
        self.finished = False

    @override
    def on_update(self, time_delta: float) -> None:
        if self.vehicle.local_position is None:
            return
        if self.circle is None:
            self.circle = self._place_circle()
            self.log(f"Starting circle: center ({self.circle[0]:.2f}, {self.circle[1]:.2f})")
        cx, cy, altitude, start_angle, start_yaw = self.circle

        if not self.finished:
            self.t += time_delta

        state = circle_state(
            self.t,
            radius=self.p.radius,
            initial_speed=self.p.initial_speed,
            tangential_accel=self.p.tangential_accel,
            altitude=altitude,
            cx=cx,
            cy=cy,
            start_angle=start_angle,
            max_speed=self.p.max_speed,
        )

        if state.distance >= self.p.laps * 2 * math.pi * self.p.radius:
            if not self.finished:
                self.log(f"Completed {self.p.laps} laps")
            self.finished = True

        msg = TrajectorySetpoint()
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        msg.position = [float(x) for x in state.position]
        if self.finished:
            # Hold the last point on the circle until the next mode takes over
            msg.velocity = [0.0, 0.0, 0.0]
            msg.acceleration = [0.0, 0.0, 0.0]
        else:
            msg.velocity = [float(x) for x in state.velocity]
            msg.acceleration = [float(x) for x in state.acceleration]
        msg.yaw = float(state.yaw if self.p.face_velocity else start_yaw)
        self.vehicle.trajectory_publisher.publish(msg)

    def _place_circle(self) -> tuple[float, float, float, float, float]:
        """
        Fixes the circle's center, altitude and start angle from the drone's current state.

        Without a configured center, the circle is placed so the drone is already on it at angle 0.
        Otherwise the circle starts at the point nearest the drone.
        """
        pos = self.vehicle.local_position
        altitude = self.p.altitude if self.p.altitude is not None else -pos.z
        start_yaw = self.vehicle.yaw if self.vehicle.yaw is not None else pos.heading

        if self.p.center is None:
            return pos.x - self.p.radius, pos.y, altitude, 0.0, start_yaw

        cx, cy = self.p.center
        return cx, cy, altitude, math.atan2(pos.y - cy, pos.x - cx), start_yaw

    @override
    def check_status(self) -> str:
        return "complete" if self.finished else "continue"
