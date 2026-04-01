from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CompressedImage
import cv2 as cv
import numpy as np
import pupil_apriltags
from cv_bridge import CvBridge


class AprilTagDebugger(Node):
    def __init__(self):
        super().__init__("apriltag_debugger")
        self.declare_parameter("topic", "")
        self.declare_parameter("compressed", False)
        self.declare_parameter("tag_size_m", 0.1)
        self.declare_parameter("camera_fx", 600.0)
        self.declare_parameter("camera_fy", 600.0)
        self.declare_parameter("camera_cx", 320.0)
        self.declare_parameter("camera_cy", 240.0)

        self.topic = self.get_parameter("topic").get_parameter_value().string_value
        self.compressed = (
            self.get_parameter("compressed").get_parameter_value().bool_value
        )
        self.tag_size_m = (
            self.get_parameter("tag_size_m").get_parameter_value().double_value
        )
        fx = self.get_parameter("camera_fx").get_parameter_value().double_value
        fy = self.get_parameter("camera_fy").get_parameter_value().double_value
        cx = self.get_parameter("camera_cx").get_parameter_value().double_value
        cy = self.get_parameter("camera_cy").get_parameter_value().double_value
        self.camera_params = (fx, fy, cx, cy)

        self.bridge = CvBridge()
        self.detector = pupil_apriltags.Detector(families="tag36h11")

        if self.compressed:
            topic = self.topic if self.topic else "/camera/image_raw/compressed"
            self.create_subscription(CompressedImage, topic, self._compressed_cb, 10)
            self.get_logger().info(f"Subscribing to compressed topic: {topic}")
        else:
            topic = self.topic if self.topic else "/camera/image_raw"
            self.create_subscription(Image, topic, self._image_cb, 10)
            self.get_logger().info(f"Subscribing to raw image topic: {topic}")

    def _image_cb(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        self._process_and_show(frame)

    def _compressed_cb(self, msg: CompressedImage):
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        frame = cv.imdecode(buf, cv.IMREAD_COLOR)
        self._process_and_show(frame)

    def _process_and_show(self, frame: np.ndarray):
        gray = cv.cvtColor(frame, cv.COLOR_BGR2GRAY)
        detections = self.detector.detect(
            gray,
            estimate_tag_pose=True,
            camera_params=self.camera_params,
            tag_size=self.tag_size_m,
        )

        vis = frame.copy()
        for det in detections:
            # Draw tag corners and border
            corners = det.corners.astype(int)
            for i in range(4):
                cv.line(
                    vis, tuple(corners[i]), tuple(corners[(i + 1) % 4]), (0, 255, 0), 2
                )

            # Draw center
            cx, cy = int(det.center[0]), int(det.center[1])
            cv.circle(vis, (cx, cy), 5, (0, 0, 255), -1)

            # Tag ID label
            cv.putText(
                vis,
                f"id={det.tag_id}",
                (cx - 20, cy - 10),
                cv.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 0),
                2,
            )

            # Pose info
            if det.pose_t is not None:
                tx, ty, tz = det.pose_t.flatten()
                dist = float(np.linalg.norm([tx, ty, tz]))
                label = f"d={dist:.2f}m  x={tx:.2f} y={ty:.2f} z={tz:.2f}"
                cv.putText(
                    vis,
                    label,
                    (cx - 20, cy + 20),
                    cv.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (255, 200, 0),
                    1,
                )

            # Draw pose axes (10 cm arms)
            if det.pose_R is not None and det.pose_t is not None:
                self._draw_axes(vis, det.pose_R, det.pose_t, self.tag_size_m * 0.5)

            self.get_logger().info(
                f"Tag {det.tag_id}: t={det.pose_t.flatten().tolist() if det.pose_t is not None else 'N/A'}"
            )

        cv.imshow("AprilTag Debug", vis)
        cv.waitKey(1)

    def _draw_axes(self, img, R, t, length):
        fx, fy, cx, cy = self.camera_params
        K = np.array([[fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)

        def project(pt3d):
            p = K @ (R @ pt3d + t.flatten())
            return int(p[0] / p[2]), int(p[1] / p[2])

        origin = project(np.zeros(3))
        x_tip = project(np.array([length, 0, 0]))
        y_tip = project(np.array([0, length, 0]))
        z_tip = project(np.array([0, 0, length]))

        cv.arrowedLine(img, origin, x_tip, (0, 0, 255), 2, tipLength=0.3)  # X = red
        cv.arrowedLine(img, origin, y_tip, (0, 255, 0), 2, tipLength=0.3)  # Y = green
        cv.arrowedLine(img, origin, z_tip, (255, 0, 0), 2, tipLength=0.3)  # Z = blue


def main():
    rclpy.init()
    apriltag_debugger = AprilTagDebugger()
    try:
        rclpy.spin(apriltag_debugger)
    finally:
        cv.destroyAllWindows()
        apriltag_debugger.destroy_node()
        rclpy.shutdown()
