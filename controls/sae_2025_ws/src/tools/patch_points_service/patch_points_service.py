import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from px4_msgs.msg import VehicleLocalPosition
from rclpy.node import Node
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from sensor_msgs.msg import CameraInfo, Image
from sim_interfaces.srv import GetPatchPoints

# finding star centers from image

GOLD_LOW = np.array([15, 120, 120])  # (H, S, V) lower bound
GOLD_HIGH = np.array([35, 255, 255])  # (H, S, V) upper bound

MIN_AREA_PX = 40  # ignore blobs smaller than this (noise)
MAX_SOLIDITY = 0.85  # area / convex-hull area. Circles/squares/triangles ~0.95, stars ~0.77
MIN_NOTCHES, MAX_NOTCHES = 4, 6  # a star has 5 deep "notches" between its arms
NOTCH_DEPTH_FRACTION = 0.08  # a notch counts if deeper than this fraction of the shape's size

EARTH_RADIUS_M = 6378137.0


def gold_mask(frame_bgr: np.ndarray) -> np.ndarray:
    # Black/white image: white where the pixel is gold
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GOLD_LOW, GOLD_HIGH)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def count_notches(contour: np.ndarray) -> int:
    # Count deep inward notches (convexity defects). Star = 5, convex shapes = 0
    hull_idx = cv2.convexHull(contour, returnPoints=False)
    if hull_idx is None or len(hull_idx) < 3:
        return 0
    defects = cv2.convexityDefects(contour, hull_idx)
    if defects is None:
        return 0
    # scale the threshold with the shape's size so it works at any altitude
    _, _, width, height = cv2.boundingRect(contour)
    min_depth = NOTCH_DEPTH_FRACTION * max(width, height)
    # each defect row is (start, end, farthest_point, depth * 256)
    depths = defects[:, 0, 3] / 256.0
    return int(np.sum(depths > min_depth))


def is_star(contour: np.ndarray) -> bool:
    # Shape test on one blob outline
    area = cv2.contourArea(contour)
    hull_area = cv2.contourArea(cv2.convexHull(contour))
    if hull_area == 0:
        return False
    solidity = area / hull_area
    return solidity < MAX_SOLIDITY and MIN_NOTCHES <= count_notches(contour) <= MAX_NOTCHES


def find_star_centers(frame_bgr: np.ndarray) -> list[tuple[float, float]]:
    # Return the (u, v) pixel center of every gold star in the frame
    contours, _ = cv2.findContours(gold_mask(frame_bgr), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
    centers = []
    for contour in contours:
        if cv2.contourArea(contour) < MIN_AREA_PX or not is_star(contour):
            continue
        # centroid
        moments = cv2.moments(contour)
        if moments["m00"]:
            centers.append((moments["m10"] / moments["m00"], moments["m01"] / moments["m00"]))
    return centers


# Copy of the frame with a magenta ring + dot on each detected star
def draw_detections(frame_bgr: np.ndarray, centers: list[tuple[float, float]]) -> np.ndarray:
    # Full-frame magenta image-coordinate crosshair: +u points right, +v points down.
    out = frame_bgr.copy()
    height, width = out.shape[:2]
    center_u, center_v = width // 2, height // 2
    magenta = (255, 0, 255)
    cv2.line(out, (0, center_v), (width - 1, center_v), magenta, 2)
    cv2.line(out, (center_u, 0), (center_u, height - 1), magenta, 2)
    cv2.arrowedLine(out, (center_u, center_v), (width - 1, center_v), magenta, 2, tipLength=0.02)
    cv2.arrowedLine(out, (center_u, center_v), (center_u, height - 1), magenta, 2, tipLength=0.04)
    cv2.putText(out, "u", (width - 30, center_v - 10), cv2.FONT_HERSHEY_PLAIN, 3, magenta, 2)
    cv2.putText(out, "v", (center_u + 30, height - 15), cv2.FONT_HERSHEY_PLAIN, 3, magenta, 2)
    for u, v in centers:
        cv2.circle(out, (int(u), int(v)), 20, magenta, 2)
        cv2.circle(out, (int(u), int(v)), 3, magenta, -1)
    return out


# finding global coordinates
def pixel_to_local_ned(
    u: float,
    v: float,
    camera_matrix: np.ndarray,
    drone_north: float,
    drone_east: float,
    drone_down: float,
    heading_rad: float,
) -> tuple[float, float]:
    # Pixel -> (north, east) on the ground. Assumes flat ground and a level drone.
    fx, fy = camera_matrix[0, 0], camera_matrix[1, 1]  # focal lengths in pixels
    cx, cy = camera_matrix[0, 2], camera_matrix[1, 2]  # image center in pixels
    height = -drone_down  # NED z is down

    # Pinhole ray projection: image slope multiplied by height gives metres.
    right = (u - cx) / fx * height  # right of image = drone's right
    forward = -(v - cy) / fy * height  # up in image = drone's forward

    # Rotate from body forward/right into North/East using vehicle heading.
    north = forward * math.cos(heading_rad) - right * math.sin(heading_rad)
    east = forward * math.sin(heading_rad) + right * math.cos(heading_rad)
    return drone_north + north, drone_east + east


def local_ned_to_gps(
    north: float, east: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    # Local NED -> latitude/longitude using PX4's reference point.
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


class PatchPointsService(Node):
    """Detect gold stars and serve their projected global coordinates."""

    def __init__(self) -> None:
        super().__init__("patch_points_service")
        self.declare_parameter("camera_topic", "/uav_0/camera")
        self.declare_parameter("camera_info_topic", "/uav_0/camera_info")
        self.declare_parameter("position_topic", "/uav_0/fmu/out/vehicle_local_position_v1")
        self.declare_parameter("debug_topic", "/uav_0/stars/debug")

        self.bridge = CvBridge()
        self.camera_matrix = None
        self.position = None
        self.latest_points: list[tuple[float, float]] = []
        self.debug_publisher = self.create_publisher(
            Image, str(self.get_parameter("debug_topic").value), 10
        )

        px4_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter("camera_info_topic").value),
            self._camera_info_callback,
            10,
        )
        self.create_subscription(
            VehicleLocalPosition,
            str(self.get_parameter("position_topic").value),
            self._position_callback,
            px4_qos,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter("camera_topic").value),
            self._image_callback,
            10,
        )
        self.create_service(GetPatchPoints, "/get_patch_points", self._request_callback)
        self.get_logger().info("Gold-star point service is ready")

    def _camera_info_callback(self, message: CameraInfo) -> None:
        # CameraInfo.k is the 3x3 matrix flattened row by row.
        if self.camera_matrix is None:
            self.camera_matrix = np.array(message.k, dtype=float).reshape(3, 3)

    def _position_callback(self, message: VehicleLocalPosition) -> None:
        self.position = message

    def _image_callback(self, message: Image) -> None:
        # Find stars in the current frame and always publish a debug image.
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
        except Exception as exc:
            self.get_logger().error(f"Could not convert camera image: {exc}")
            return

        centers = find_star_centers(frame)
        debug = draw_detections(frame, centers)
        debug_message = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
        debug_message.header = message.header
        self.debug_publisher.publish(debug_message)

        # Cannot place stars on the map without camera calibration and valid PX4 position.
        position = self.position
        if self.camera_matrix is None or position is None:
            return
        if not (position.xy_valid and position.z_valid and position.xy_global):
            return
        if -position.z < 2.0:
            return

        # Pixel -> ground position for every detected star.
        stars_ned = [
            pixel_to_local_ned(
                u,
                v,
                self.camera_matrix,
                position.x,
                position.y,
                position.z,
                position.heading,
            )
            for u, v in centers
        ]
        # Convert the local detections to global GPS coordinates for the service response.
        self.latest_points = [
            local_ned_to_gps(north, east, position.ref_lat, position.ref_lon)
            for north, east in stars_ned
        ]

    def _request_callback(self, request, response):
        # Return the latest frame's gold-star GPS coordinates as parallel arrays.
        del request
        response.latitude_deg = [point[0] for point in self.latest_points]
        response.longitude_deg = [point[1] for point in self.latest_points]
        if not self.latest_points:
            self.get_logger().warning("No projected gold stars available yet")
        else:
            self.get_logger().info(f"Returning {len(self.latest_points)} gold-star points")
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = PatchPointsService()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
