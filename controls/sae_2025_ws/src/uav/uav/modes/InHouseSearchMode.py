from __future__ import annotations

import math
import time
from enum import Enum, auto
from typing import override

import cv2
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile
from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from sim_interfaces.srv import GetSearchLocations
from vehicle_common.mode import Mode
from vehicle_common.mode_loader import ParamsBase, register_mode

from uav.vehicles.UAV import UAV

# mono_cam horizontal FOV (PX4 gz model), used until camera_info arrives
DEFAULT_HFOV = 1.74


class InHouseSearchParams(ParamsBase):
    """
    scan_altitude: minimum height (m) for spotting gold stars over a whole patch.
        Raised automatically so the camera footprint covers the patch radius.
    max_scan_altitude: cap on the automatic scan height.
    inspect_altitude: height for reading a star's AprilTag (tags are ~2.5 cm).
    retry_altitude: lower height tried once if no tag is read at inspect_altitude.
    margin: distance (m) at which a setpoint counts as reached.
    settle_time: seconds to hover before trusting camera frames after moving.
    scan_frames: frames combined when spotting stars from scan altitude.
    center_tolerance: horizontal star offset (m) accepted as "above the star".
    max_center_moves: centering corrections before reading the tag anyway.
    read_timeout: seconds of reading at one height before giving up on a star.
    gold_hsv_low / gold_hsv_high: OpenCV HSV bounds for the gold colour.
    min_star_area_m2: smallest gold blob (ground area) treated as a star.
    """

    scan_altitude: float = 6.0
    max_scan_altitude: float = 20.0
    inspect_altitude: float = 0.8
    retry_altitude: float = 0.6
    margin: float = 0.25
    settle_time: float = 1.5
    scan_frames: int = 3
    center_tolerance: float = 0.2
    max_center_moves: int = 3
    read_timeout: float = 6.0
    gold_hsv_low: tuple[int, int, int] = (15, 80, 80)
    gold_hsv_high: tuple[int, int, int] = (40, 255, 255)
    min_star_area_m2: float = 0.15


class State(Enum):
    WAIT_PATCHES = auto()
    GOTO_PATCH = auto()
    SCAN = auto()
    GOTO_STAR = auto()
    CENTER = auto()
    READ = auto()
    DONE = auto()


@register_mode(
    id="uav.InHouseSearchMode",
    params_cls=InHouseSearchParams,
    targets=[UAV],
    transition_labels=["complete"],
)
class InHouseSearchMode(Mode[UAV, InHouseSearchParams]):
    """
    Finds the gold star carrying the target AprilTag and reports its GPS position.

    For each search patch: hover high enough to see the whole patch, spot gold
    blobs (gold is the only yellow in the world), then descend over each one,
    centre on it and read its tag. The drone's position when centred over the
    target tag is reported as the answer.
    """

    @override
    def initialize(self, node: Node, vehicle: UAV, params: InHouseSearchParams) -> None:
        self.node = node
        self.vehicle = vehicle
        self.p = params

        self.state = State.WAIT_PATCHES
        self.target_tag_id: int | None = None
        self.patches: list[tuple[float, float, float]] = []  # (north, east, radius) local
        self.patch_index = -1
        self.stars: list[tuple[float, float]] = []  # local (north, east) still to inspect
        self.star: tuple[float, float] | None = None
        self.setpoint: tuple[float, float, float] | None = None
        self.arrived_at: float | None = None
        self.state_started = time.monotonic()
        self.scan_hits: list[tuple[float, float]] = []
        self.scan_frame_count = 0
        self.read_altitude = self.p.inspect_altitude
        self.center_moves = 0
        self.result: tuple[float, float, float] | None = None

        # latest camera frame (BGR) and the time it arrived
        self.frame: np.ndarray | None = None
        self.frame_time = 0.0
        self.last_processed = 0.0
        self.intrinsics: tuple[float, float, float, float] | None = None  # fx, fy, cx, cy

        self.detector = self._make_detector()
        self.patch_client = node.create_client(GetSearchLocations, "/get_search_patches")
        self.patch_future = None
        image_topic = vehicle.namespaced_path(vehicle.image_topic, namespace=vehicle.camera_namespace)
        info_topic = vehicle.namespaced_path(
            vehicle.camera_info_topic, namespace=vehicle.camera_namespace
        )
        self.image_sub = node.create_subscription(Image, image_topic, self._image_cb, 1)
        self.info_sub = node.create_subscription(CameraInfo, info_topic, self._info_cb, 1)
        latched = QoSProfile(depth=1, durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.result_pub = node.create_publisher(
            NavSatFix, f"/{vehicle.name}/target_star_gps", latched
        )

    # ------------------------------------------------------------------ callbacks

    def _image_cb(self, msg: Image) -> None:
        channels = 3 if msg.encoding in ("rgb8", "bgr8") else 1
        data = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
        img = data[:, : msg.width * channels].reshape(msg.height, msg.width, channels)
        if msg.encoding == "rgb8":
            img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
        elif channels == 1:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        self.frame = img
        self.frame_time = time.monotonic()
        if self.intrinsics is None:
            # pinhole from FOV until camera_info shows up
            fx = (msg.width / 2) / math.tan(DEFAULT_HFOV / 2)
            self.intrinsics = (fx, fx, msg.width / 2, msg.height / 2)

    def _info_cb(self, msg: CameraInfo) -> None:
        if msg.k[0] > 0:
            self.intrinsics = (msg.k[0], msg.k[4], msg.k[2], msg.k[5])

    # ------------------------------------------------------------------ main loop

    @override
    def on_update(self, time_delta: float) -> None:
        if self.vehicle.local_position is None:
            return

        handler = {
            State.WAIT_PATCHES: self._wait_patches,
            State.GOTO_PATCH: self._goto_patch,
            State.SCAN: self._scan,
            State.GOTO_STAR: self._goto_star,
            State.CENTER: self._center,
            State.READ: self._read,
        }.get(self.state)
        if handler:
            handler()

        # PX4 offboard needs a continuous setpoint stream
        if self.setpoint is not None:
            self.vehicle.publish_position_setpoint(self.setpoint, lock_yaw=self._reached())
        else:
            lp = self.vehicle.local_position
            self.vehicle.publish_position_setpoint((lp.x, lp.y, lp.z), lock_yaw=True)

    @override
    def check_status(self) -> str:
        return "complete" if self.state == State.DONE else "continue"

    # ------------------------------------------------------------------ states

    def _wait_patches(self) -> None:
        if self.patch_future is None:
            if not self.patch_client.service_is_ready():
                self._log_throttled("Waiting for /get_search_patches service...")
                return
            self.patch_future = self.patch_client.call_async(GetSearchLocations.Request())
            return
        if not self.patch_future.done():
            return

        res = self.patch_future.result()
        self.patch_future = None
        if res is None or not res.ready or self.vehicle.gps_origin is None:
            self._log_throttled("Patches not ready yet, retrying...")
            return

        self.target_tag_id = res.target_tag_id
        ref_alt = self.vehicle.gps_origin[2]
        for loc in res.search_locations:
            n, e, _ = self.vehicle.gps_to_local((loc.latitude_deg, loc.longitude_deg, ref_alt))
            self.patches.append((n, e, loc.radius_m))
        self.log(f"Target tag {self.target_tag_id}; {len(self.patches)} patch(es): {self.patches}")
        self._next_patch()

    def _goto_patch(self) -> None:
        if self._settled():
            self.scan_hits = []
            self.scan_frame_count = 0
            self._enter(State.SCAN)

    def _scan(self) -> None:
        frame = self._fresh_frame()
        if frame is None:
            return
        for blob in self._gold_blobs(frame):
            self.scan_hits.append(self._pixel_to_local(*blob[:2]))
        self.scan_frame_count += 1
        if self.scan_frame_count < self.p.scan_frames:
            return

        self.stars = self._cluster(self.scan_hits, radius=0.6, min_hits=2)
        self.log(f"Patch {self.patch_index}: spotted {len(self.stars)} gold star(s) at {self._fmt(self.stars)}")
        self._next_star()

    def _goto_star(self) -> None:
        if self._settled():
            self._enter(State.CENTER)

    def _center(self) -> None:
        frame = self._fresh_frame()
        if frame is None:
            if self._timed_out():
                self._give_up_star("no frames while centering")
            return

        blobs = self._gold_blobs(frame)
        if not blobs:
            if self._timed_out():
                self._give_up_star("star not visible")
            return

        h, w = frame.shape[:2]
        u, v, _ = min(blobs, key=lambda b: (b[0] - w / 2) ** 2 + (b[1] - h / 2) ** 2)
        n, e = self._pixel_to_local(u, v)
        lp = self.vehicle.local_position
        offset = math.hypot(n - lp.x, e - lp.y)
        self.star = (n, e)
        if offset < self.p.center_tolerance or self.center_moves >= self.p.max_center_moves:
            self._enter(State.READ)
        else:
            self.center_moves += 1
            self._set(n, e, self.read_altitude)  # re-settles before the next frame

    def _read(self) -> None:
        frame = self._fresh_frame()
        if frame is None:
            if self._timed_out():
                self._retry_or_skip()
            return

        tags = self._detect_tags(frame)
        if not tags:
            if self._timed_out():
                self._retry_or_skip()
            return

        h, w = frame.shape[:2]
        tag_id, (u, v) = min(tags, key=lambda t: (t[1][0] - w / 2) ** 2 + (t[1][1] - h / 2) ** 2)
        if tag_id != self.target_tag_id:
            self.log(f"Star at {self._fmt([self.star])} has tag {tag_id} (decoy)")
            self._next_star()
            return

        n, e = self._pixel_to_local(u, v)
        lat, lon, _ = self.vehicle.local_to_gps((n, e, 0.0))
        self.result = (lat, lon, self.vehicle.gps_origin[2])
        self.log(
            f"FOUND target tag {tag_id} | local N={n:.2f} E={e:.2f} | "
            f"GPS lat={lat:.8f} lon={lon:.8f}"
        )
        fix = NavSatFix()
        fix.header.stamp = self.node.get_clock().now().to_msg()
        fix.latitude, fix.longitude, fix.altitude = self.result
        self.result_pub.publish(fix)
        self._enter(State.DONE)

    # ------------------------------------------------------------------ transitions

    def _next_patch(self) -> None:
        self.patch_index += 1
        if self.patch_index >= len(self.patches):
            self.log(f"Searched every patch; target tag {self.target_tag_id} NOT found")
            self._enter(State.DONE)
            return
        n, e, radius = self.patches[self.patch_index]
        self._set(n, e, self._scan_altitude(radius))
        self.log(f"Flying to patch {self.patch_index} at N={n:.1f} E={e:.1f} (alt {-self.setpoint[2]:.1f} m)")
        self._enter(State.GOTO_PATCH)

    def _next_star(self) -> None:
        if not self.stars:
            self._next_patch()
            return
        lp = self.vehicle.local_position
        self.stars.sort(key=lambda s: math.hypot(s[0] - lp.x, s[1] - lp.y))
        self.star = self.stars.pop(0)
        self.read_altitude = self.p.inspect_altitude
        self.center_moves = 0
        self._set(self.star[0], self.star[1], self.read_altitude)
        self._enter(State.GOTO_STAR)

    def _retry_or_skip(self) -> None:
        if self.read_altitude > self.p.retry_altitude:
            self.read_altitude = self.p.retry_altitude
            self.log(f"No tag read; retrying lower at {self.read_altitude} m")
            self.center_moves = 0
            self._set(self.star[0], self.star[1], self.read_altitude)
            self._enter(State.GOTO_STAR)
        else:
            self._give_up_star("no tag read")

    def _give_up_star(self, reason: str) -> None:
        self.log(f"Skipping star at {self._fmt([self.star])}: {reason}")
        self._next_star()

    def _enter(self, state: State) -> None:
        self.state = state
        self.state_started = time.monotonic()

    def _set(self, n: float, e: float, altitude: float) -> None:
        self.setpoint = (n, e, -altitude)
        self.arrived_at = None

    # ------------------------------------------------------------------ helpers

    def _reached(self) -> bool:
        if self.setpoint is None:
            return False
        lp = self.vehicle.local_position
        return math.dist((lp.x, lp.y, lp.z), self.setpoint) < self.p.margin

    def _settled(self) -> bool:
        """True once the setpoint has been held for settle_time."""
        if not self._reached():
            self.arrived_at = None
            return False
        if self.arrived_at is None:
            self.arrived_at = time.monotonic()
        return time.monotonic() - self.arrived_at >= self.p.settle_time

    def _fresh_frame(self) -> np.ndarray | None:
        """Newest frame taken after the drone settled, each frame used once."""
        if not self._settled() or self.frame is None or self.intrinsics is None:
            return None
        if self.frame_time <= self.arrived_at + self.p.settle_time or self.frame_time <= self.last_processed:
            return None
        self.last_processed = self.frame_time
        return self.frame

    def _timed_out(self) -> bool:
        return time.monotonic() - self.state_started > self.p.read_timeout + self.p.settle_time + 5.0

    def _height(self) -> float:
        origin_z = self.vehicle.local_origin[2] if self.vehicle.local_origin else 0.0
        return max(origin_z - self.vehicle.local_position.z, 0.1)

    def _scan_altitude(self, radius: float) -> float:
        # the short (vertical) side of the image has to cover the patch plus 1 m slack
        fx, fy, cx, cy = self.intrinsics or (
            640 / math.tan(DEFAULT_HFOV / 2),
            640 / math.tan(DEFAULT_HFOV / 2),
            640,
            480,
        )
        half_vfov = math.atan(cy / fy)
        needed = (radius + 1.0) / math.tan(half_vfov)
        return min(max(self.p.scan_altitude, needed), self.p.max_scan_altitude)

    def _pixel_to_local(self, u: float, v: float) -> tuple[float, float]:
        """Project an image pixel onto flat ground (camera points straight down).

        Image up is the drone's nose, image right is its right side.
        """
        fx, fy, cx, cy = self.intrinsics
        h = self._height()
        forward = -(v - cy) * h / fy
        right = (u - cx) * h / fx
        yaw = self.vehicle.yaw or 0.0
        lp = self.vehicle.local_position
        n = lp.x + forward * math.cos(yaw) - right * math.sin(yaw)
        e = lp.y + forward * math.sin(yaw) + right * math.cos(yaw)
        return n, e

    def _gold_blobs(self, frame: np.ndarray) -> list[tuple[float, float, float]]:
        """(u, v, area_px) of every gold region big enough to be a star."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        mask = cv2.inRange(hsv, np.array(self.p.gold_hsv_low), np.array(self.p.gold_hsv_high))
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        fx, fy, _, _ = self.intrinsics
        h = self._height()
        min_area_px = self.p.min_star_area_m2 * (fx / h) * (fy / h)
        blobs = []
        for c in contours:
            area = cv2.contourArea(c)
            if area < min_area_px:
                continue
            m = cv2.moments(c)
            blobs.append((m["m10"] / m["m00"], m["m01"] / m["m00"], area))
        return blobs

    def _make_detector(self):
        try:
            import pupil_apriltags

            det = pupil_apriltags.Detector(families="tag36h11", quad_decimate=1.0)
            return lambda gray: [(d.tag_id, tuple(d.center)) for d in det.detect(gray)]
        except ImportError:
            pass
        try:
            import apriltag

            det = apriltag.Detector(apriltag.DetectorOptions(families="tag36h11"))
            return lambda gray: [(d.tag_id, tuple(d.center)) for d in det.detect(gray)]
        except ImportError:
            self.node.get_logger().error(
                "No AprilTag library found: pip install pupil-apriltags"
            )
            return lambda gray: []

    def _detect_tags(self, frame: np.ndarray) -> list[tuple[int, tuple[float, float]]]:
        """Tags as (id, (u, v)). Upscales 2x since tags are only ~15-25 px wide."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        big = cv2.resize(gray, None, fx=2, fy=2, interpolation=cv2.INTER_CUBIC)
        return [(tid, (u / 2, v / 2)) for tid, (u, v) in self.detector(big)]

    @staticmethod
    def _cluster(points, radius: float, min_hits: int) -> list[tuple[float, float]]:
        """Merge detections from several frames into one position per star."""
        clusters: list[list[tuple[float, float]]] = []
        for p in points:
            for c in clusters:
                cn, ce = np.mean(c, axis=0)
                if math.hypot(p[0] - cn, p[1] - ce) < radius:
                    c.append(p)
                    break
            else:
                clusters.append([p])
        return [tuple(np.mean(c, axis=0)) for c in clusters if len(c) >= min_hits]

    @staticmethod
    def _fmt(points) -> str:
        return ", ".join(f"({p[0]:.1f}, {p[1]:.1f})" for p in points if p is not None)

    def _log_throttled(self, msg: str) -> None:
        self.node.get_logger().info(f"[InHouseSearchMode] {msg}", throttle_duration_sec=2.0)
