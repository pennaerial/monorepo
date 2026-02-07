from __future__ import annotations

import math
from time import time

from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleStatus

from uav import UAV
from uav.autonomous_modes import Mode


def _topics(node: Node) -> tuple[str, str]:
    vid = getattr(node, 'vehicle_id', 0)
    return (
        f'/safe_test/vehicle_{vid}/command',
        f'/safe_test/vehicle_{vid}/status',
    )


def _home_with_vehicle_offset(node: Node, x0: float, y0: float, z0: float, xy_spacing_m: float) -> tuple[float, float, float]:
    vid = int(getattr(node, 'vehicle_id', 0))
    # NED local frame: x=north, y=east, z=down. Offset in y to separate vehicles.
    return (float(x0), float(y0) + float(xy_spacing_m) * vid, float(z0))


def _ensure_safe_test_io(node: Node) -> None:
    cmd_topic, status_topic = _topics(node)
    if not hasattr(node, '_safe_test_cmd'):
        node._safe_test_cmd = None
        node._safe_test_cmd_time = 0.0

    if not hasattr(node, '_safe_test_status_pub'):
        node._safe_test_status_pub = node.create_publisher(String, status_topic, 10)

    if not hasattr(node, '_safe_test_cmd_sub'):
        def _cb(msg: String, _node: Node = node) -> None:
            cmd = (msg.data or '').strip().lower()
            if not cmd:
                return
            _node._safe_test_cmd = cmd
            _node._safe_test_cmd_time = time()

        node._safe_test_cmd_sub = node.create_subscription(String, cmd_topic, _cb, 10)


def _publish_status(node: Node, text: str) -> None:
    _ensure_safe_test_io(node)
    msg = String()
    msg.data = text
    node._safe_test_status_pub.publish(msg)


def _consume_cmd(node: Node) -> str | None:
    _ensure_safe_test_io(node)
    cmd = node._safe_test_cmd
    node._safe_test_cmd = None
    return cmd


class SafeTestWaitMode(Mode):
    """Wait for operator command: launch|abort."""

    def __init__(
        self,
        node: Node,
        uav: UAV,
        hover_alt_m=6.0,
        home_xy_spacing_m=2.0,
    ):
        super().__init__(node, uav)
        self.hover_alt_m = float(hover_alt_m)
        self.home_xy_spacing_m = float(home_xy_spacing_m)
        self._next = 'continue'
        self._last_status = 0.0

    def on_enter(self) -> None:
        _ensure_safe_test_io(self.node)
        _publish_status(self.node, 'safe_test:WAIT (publish launch|abort to /safe_test/vehicle_<id>/command)')

    def on_update(self, time_delta: float) -> None:
        cmd = _consume_cmd(self.node)
        if cmd in ('abort', 'failsafe'):
            self._next = 'error'
            return
        if cmd in ('launch', 'takeoff', 'start'):
            # stash parameters on node for later modes
            self.node._safe_test_hover_alt_m = self.hover_alt_m
            self.node._safe_test_home_xy_spacing_m = self.home_xy_spacing_m
            self._next = 'launch'
            _publish_status(self.node, 'safe_test:LAUNCH')
            return

        now = time()
        if now - self._last_status > 1.0:
            self._last_status = now
            _publish_status(self.node, 'safe_test:WAIT')

    def check_status(self) -> str:
        return self._next


class SafeTestArmTakeoffMode(Mode):
    """Arm + PX4 auto-takeoff, then transition once OFFBOARD is reached."""

    def __init__(
        self,
        node: Node,
        uav: UAV,
        hover_alt_m=6.0,
        home_xy_spacing_m=2.0,
    ):
        super().__init__(node, uav)
        self.hover_alt_m = float(hover_alt_m)
        self.home_xy_spacing_m = float(home_xy_spacing_m)
        self._next = 'continue'
        self._start_time = 0.0
        self._last_status = 0.0

    def on_enter(self) -> None:
        _ensure_safe_test_io(self.node)
        self._next = 'continue'
        self._start_time = time()
        # Reset takeoff so this mode is re-runnable.
        self.uav.attempted_takeoff = False
        self.uav.takeoff_amount = self.hover_alt_m
        _publish_status(self.node, 'safe_test:ARM_TAKEOFF')

    def on_update(self, time_delta: float) -> None:
        cmd = _consume_cmd(self.node)
        if cmd in ('abort', 'failsafe'):
            self._next = 'error'
            return
        if cmd in ('land', 'stop'):
            self._next = 'land'
            return

        if self.uav.local_position is None or self.uav.global_position is None:
            return

        if self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            self.uav.arm()
            self._start_time = time()

        if not self.uav.attempted_takeoff:
            self.uav.takeoff()
            self._start_time = time()
            return

        # Heartbeat helps OFFBOARD engagement once LOITER.
        self.uav.publish_offboard_control_heartbeat_signal()

        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_AUTO_LOITER:
            self.uav.engage_offboard_mode()
            return

        # Wait a short time window after starting heartbeat.
        if time() - self._start_time < 1.0:
            return

        if self.uav.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD and self.uav.flight_check:
            # record home setpoint (NED: z down)
            x0 = float(self.uav.local_position.x)
            y0 = float(self.uav.local_position.y)
            z0 = -abs(self.hover_alt_m)
            spacing = float(getattr(self.node, '_safe_test_home_xy_spacing_m', self.home_xy_spacing_m))
            self.node._safe_test_home = _home_with_vehicle_offset(self.node, x0, y0, z0, spacing)
            self._next = 'hover'
            _publish_status(self.node, 'safe_test:OFFBOARD_READY')
            return

        now = time()
        if now - self._last_status > 1.0:
            self._last_status = now
            _publish_status(self.node, f'safe_test:ARM_TAKEOFF nav_state={self.uav.nav_state}')

    def check_status(self) -> str:
        return self._next


class SafeHoverMode(Mode):
    """Hover at home until orbit|land command."""

    def __init__(
        self,
        node: Node,
        uav: UAV,
        hover_alt_m=6.0,
        home_xy_spacing_m=2.0,
    ):
        super().__init__(node, uav)
        self.hover_alt_m = float(hover_alt_m)
        self.home_xy_spacing_m = float(home_xy_spacing_m)
        self._next = 'continue'
        self._last_status = 0.0

    def on_enter(self) -> None:
        _ensure_safe_test_io(self.node)
        self._next = 'continue'
        if not hasattr(self.node, '_safe_test_home') or self.node._safe_test_home is None:
            if self.uav.local_position is not None:
                x0 = float(self.uav.local_position.x)
                y0 = float(self.uav.local_position.y)
                z0 = -abs(self.hover_alt_m)
                spacing = float(getattr(self.node, '_safe_test_home_xy_spacing_m', self.home_xy_spacing_m))
                self.node._safe_test_home = _home_with_vehicle_offset(self.node, x0, y0, z0, spacing)
        _publish_status(self.node, 'safe_test:HOVER')

    def on_update(self, time_delta: float) -> None:
        cmd = _consume_cmd(self.node)
        if cmd in ('abort', 'failsafe'):
            self._next = 'error'
            return
        if cmd in ('orbit', 'circle'):
            self._next = 'orbit'
            _publish_status(self.node, 'safe_test:ORBIT')
            return
        if cmd in ('land', 'stop'):
            self._next = 'land'
            _publish_status(self.node, 'safe_test:LAND')
            return

        if self.uav.local_position is None:
            return

        self.uav.publish_offboard_control_heartbeat_signal()

        home = getattr(self.node, '_safe_test_home', None)
        if home is None:
            return
        self.uav.publish_position_setpoint(home, calculate_yaw=False)

        now = time()
        if now - self._last_status > 2.0:
            self._last_status = now
            _publish_status(self.node, 'safe_test:HOVER')

    def check_status(self) -> str:
        return self._next


class SafeOrbitMode(Mode):
    """Orbit around home for N revolutions."""

    def __init__(
        self,
        node: Node,
        uav: UAV,
        radius_m=5.0,
        omega_rad_s=0.6,
        target_revs=10,
    ):
        super().__init__(node, uav)
        self.radius_m = float(radius_m)
        self.omega_rad_s = float(omega_rad_s)
        self.target_revs = int(target_revs)
        self._angle = 0.0
        self._next = 'continue'
        self._last_status = 0.0

    def on_enter(self) -> None:
        _ensure_safe_test_io(self.node)
        self._angle = 0.0
        self._next = 'continue'
        _publish_status(self.node, 'safe_test:ORBIT')

    def on_update(self, time_delta: float) -> None:
        cmd = _consume_cmd(self.node)
        if cmd in ('abort', 'failsafe'):
            self._next = 'error'
            return
        if cmd in ('land', 'stop'):
            self._next = 'land'
            _publish_status(self.node, 'safe_test:LAND')
            return

        if self.uav.local_position is None:
            return

        home = getattr(self.node, '_safe_test_home', None)
        if home is None:
            return

        self.uav.publish_offboard_control_heartbeat_signal()

        self._angle += self.omega_rad_s * max(0.0, float(time_delta))
        cx, cy, cz = home
        x = cx + self.radius_m * math.cos(self._angle)
        y = cy + self.radius_m * math.sin(self._angle)
        z = cz
        self.uav.publish_position_setpoint((x, y, z), calculate_yaw=True)

        completed_revs = int(self._angle / (2.0 * math.pi))
        now = time()
        if now - self._last_status > 1.0:
            self._last_status = now
            _publish_status(self.node, f'safe_test:ORBIT rev={completed_revs}/{self.target_revs}')

        if completed_revs >= self.target_revs:
            self._next = 'land'
            _publish_status(self.node, 'safe_test:LAND')

    def check_status(self) -> str:
        return self._next


class SafeLandMode(Mode):
    """Return near home then command PX4 land."""

    def __init__(
        self,
        node: Node,
        uav: UAV,
        return_tolerance_m=1.0,
    ):
        super().__init__(node, uav)
        self.return_tolerance_m = float(return_tolerance_m)
        self._land_started = False

    def on_enter(self) -> None:
        _ensure_safe_test_io(self.node)
        self._land_started = False
        _publish_status(self.node, 'safe_test:LAND')

    def on_update(self, time_delta: float) -> None:
        cmd = _consume_cmd(self.node)
        if cmd in ('abort', 'failsafe'):
            # still try to land, but allow ModeManager failsafe to take over
            self.uav.failsafe = True
            return

        if self.uav.local_position is None:
            return

        self.uav.publish_offboard_control_heartbeat_signal()

        home = getattr(self.node, '_safe_test_home', None)
        if home is not None and not self._land_started:
            hx, hy, hz = home
            dx = float(self.uav.local_position.x) - hx
            dy = float(self.uav.local_position.y) - hy
            dist_xy = math.hypot(dx, dy)
            if dist_xy > self.return_tolerance_m:
                self.uav.publish_position_setpoint((hx, hy, hz), calculate_yaw=True)
                return

        self._land_started = True
        self.uav.land()

    def check_status(self) -> str:
        # Consider done once disarmed.
        if self.uav.arm_state != VehicleStatus.ARMING_STATE_ARMED:
            _publish_status(self.node, 'safe_test:DONE')
            return 'terminate'
        return 'continue'
