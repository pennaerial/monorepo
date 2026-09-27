import math

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from px4_msgs.msg import VehicleLocalPosition
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image

# finding star centers from image

GOLD_LOW = np.array([15, 120, 120])  # (H, S, V) lower bound
GOLD_HIGH = np.array([35, 255, 255])  # (H, S, V) upper bound

MIN_AREA_PX = 40  # ignore blobs smaller than this (noise)
MAX_SOLIDITY = 0.85  # area / convex-hull area. Circles/squares/triangles ~0.95, stars ~0.77
MIN_NOTCHES, MAX_NOTCHES = 4, 6  # a star has 5 deep "notches" between its arms
NOTCH_DEPTH_FRACTION = 0.08  # a notch counts if deeper than this fraction of the shape's size


def gold_mask(frame_bgr: np.ndarray) -> np.ndarray:
    # Black/white image: white where the pixel is gold
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, GOLD_LOW, GOLD_HIGH)

    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    return mask


def count_notches(contour: np.ndarray) -> int:
    # Count deep inward notches (convexity defects). Star = 5, convex shapes = 0
    hull_idx = cv2.convexHull(contour, returnPoints=False)
    if hull_idx is None or len(hull_idx) < 3:
        return 0
    defects = cv2.convexityDefects(contour, hull_idx)
    if defects is None:
        return 0

    # scale the threshold with the shape's size so it works at any altitude
    _, _, w, h = cv2.boundingRect(contour)
    min_depth = NOTCH_DEPTH_FRACTION * max(w, h)

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
    mask = gold_mask(frame_bgr)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)

    centers = []
    for contour in contours:
        if cv2.contourArea(contour) < MIN_AREA_PX or not is_star(contour):
            continue
        # centroid
        m = cv2.moments(contour)
        centers.append((m["m10"] / m["m00"], m["m01"] / m["m00"]))
    return centers


def draw_detections(frame_bgr: np.ndarray, centers: list[tuple[float, float]]) -> np.ndarray:
    # Copy of the frame with a magenta ring + dot on each detected star
    out = frame_bgr.copy()
    for u, v in centers:
        cv2.circle(out, (int(u), int(v)), 20, (255, 0, 255), 2)
        cv2.circle(out, (int(u), int(v)), 3, (255, 0, 255), -1)
    return out


# finding global coordinates

EARTH_RADIUS_M = 6378137.0  # same flat-earth approximation as UAV.local_to_gps()


def pixel_to_local_ned(
    u: float,
    v: float,
    K: np.ndarray,
    drone_north: float,
    drone_east: float,
    drone_down: float,
    heading_rad: float,
) -> tuple[float, float]:
    # Pixel -> (north, east) on the ground. Assumes flat ground at z = 0 and a level drone
    fx, fy = K[0][0], K[1][1]  # focal lengths in pixels
    cx, cy = K[0][2], K[1][2]  # image center in pixels
    height = -drone_down  # NED z is down

    # Pinhole model: slope of the ray through the pixel x height = metres on the ground
    right = (u - cx) / fx * height  # right of image = drone's right
    forward = -(v - cy) / fy * height  # up in image (smaller v) = drone's forward

    # Rotate (forward, right) by heading (0 = North, +pi/2 = East), then add drone position.
    # Same rotation as UAV.uav_to_local().
    north = forward * math.cos(heading_rad) - right * math.sin(heading_rad)
    east = forward * math.sin(heading_rad) + right * math.cos(heading_rad)
    return drone_north + north, drone_east + east


def local_ned_to_gps(
    north: float, east: float, ref_lat: float, ref_lon: float
) -> tuple[float, float]:
    # Local NED -> (lat, lon) in degrees, from PX4's reference point (ref_lat, ref_lon)
    lat = ref_lat + math.degrees(north / EARTH_RADIUS_M)
    lon = ref_lon + math.degrees(east / (EARTH_RADIUS_M * math.cos(math.radians(ref_lat))))
    return lat, lon


# gives list

HAVE_ROS = True


def make_pose_array(points: list[tuple[float, float]], stamp, frame_id: str):
    # List of (x, y) -> PoseArray, the standard ROS 'list of positions' message
    msg = PoseArray()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    for x, y in points:
        pose = Pose()
        pose.position.x = float(x)
        pose.position.y = float(y)
        pose.orientation.w = 1.0
        msg.poses.append(pose)
    return msg


if HAVE_ROS:

    class FindStar(Node):
        def __init__(self):
            super().__init__("findstar")

            # topics are parameters so they can be changed without editing code
            self.declare_parameter("camera_topic", "/uav_0/camera")
            self.declare_parameter("camera_info_topic", "/uav_0/camera_info")
            self.declare_parameter("position_topic", "/uav_0/fmu/out/vehicle_local_position_v1")
            self.declare_parameter("min_height_m", 2.0)  # ignore frames while near the ground
            camera_topic = self.get_parameter("camera_topic").value
            self.min_height_m = float(self.get_parameter("min_height_m").value)

            self.bridge = CvBridge()  # ROS Image <-> OpenCV array
            self.K = None  # 3x3 camera matrix, from camera_info
            self.pose = None  # latest drone position, from PX4

            px4_qos = QoSProfile(
                reliability=QoSReliabilityPolicy.BEST_EFFORT,
                durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=10,
            )
            self.create_subscription(
                CameraInfo,
                self.get_parameter("camera_info_topic").value,
                self.on_camera_info,
                10,
            )
            self.create_subscription(
                VehicleLocalPosition,
                self.get_parameter("position_topic").value,
                self.on_position,
                px4_qos,
            )
            self.create_subscription(Image, camera_topic, self.on_image, 10)

            self.local_pub = self.create_publisher(PoseArray, "/uav_0/stars/local", 10)
            self.gps_pub = self.create_publisher(PoseArray, "/uav_0/stars/gps", 10)
            self.debug_pub = self.create_publisher(Image, "/uav_0/stars/debug", 10)
            self.get_logger().info(f"findstar listening on {camera_topic}")

        def on_camera_info(self, msg):
            # msg.k is the 3x3 matrix flattened row by row: [fx 0 cx, 0 fy cy, 0 0 1]
            if self.K is None:
                self.K = np.array(msg.k, dtype=float).reshape(3, 3)
                fx, cx = self.K[0, 0], self.K[0, 2]
                self.get_logger().info(f"got camera matrix: fx={fx:.1f} cx={cx:.1f}")

        def on_position(self, msg):
            self.pose = msg

        def on_image(self, msg):
            # find stars in the frame
            frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            centers = find_star_centers(frame)

            # debug image always goes out, so you can check detection in rqt
            debug = self.bridge.cv2_to_imgmsg(draw_detections(frame, centers), encoding="bgr8")
            debug.header = msg.header
            self.debug_pub.publish(debug)

            # can't place stars on the map without the camera matrix and a valid position
            p = self.pose
            if self.K is None or p is None or not (p.xy_valid and p.z_valid):
                self.get_logger().warn(
                    "waiting for camera_info / position", throttle_duration_sec=5.0
                )
                return
            if -p.z < self.min_height_m:
                return

            # pixel -> ground position for every star
            stars_ned = [
                pixel_to_local_ned(u, v, self.K, p.x, p.y, p.z, p.heading) for u, v in centers
            ]
            self.local_pub.publish(make_pose_array(stars_ned, msg.header.stamp, "local_ned"))

            # GPS version, only if PX4 has a GPS reference point
            if p.xy_global:
                stars_gps = [local_ned_to_gps(n, e, p.ref_lat, p.ref_lon) for n, e in stars_ned]
                self.gps_pub.publish(make_pose_array(stars_gps, msg.header.stamp, "wgs84"))

            if stars_ned:
                text = ", ".join(f"(N {n:.2f}, E {e:.2f})" for n, e in stars_ned)
                self.get_logger().info(
                    f"{len(stars_ned)} star(s): {text}", throttle_duration_sec=1.0
                )


def main(args=None):
    # ROS entry point: ros2 run pennair_vision findstar
    if not HAVE_ROS:
        print("ROS isn't available here. To test on a picture: python3 findstar.py <image>")
        return 1
    rclpy.init(args=args)
    node = None
    try:
        node = FindStar()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0
