# apriltag_detection_node.py
import cv2
import numpy as np
from uav.cv.apriltag_detection import detect_apriltags, rotate_image
from uav.vision_nodes import VisionNode
from uav_interfaces.msg import AprilTagDetection, AprilTagDetectionArray
from std_msgs.msg import Header
import rclpy


class AprilTagDetectionNode(VisionNode):
    """
    ROS node for AprilTag detection with 6DOF pose estimation.
    Publishes detections continuously as a topic for both payloads to use.
    """

    def __init__(self):
        super().__init__(custom_service=None, display=False, use_service=False)

        # AprilTag parameters
        self.declare_parameter('tag_size', 0.165)  # Tag size in meters (165mm default)
        self.declare_parameter('tag_family', 'tag36h11')  # AprilTag family
        self.declare_parameter('publish_rate', 10.0)  # Hz

        self.tag_size = self.get_parameter('tag_size').value
        self.tag_family = self.get_parameter('tag_family').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f"AprilTag Detection Node initialized")
        self.get_logger().info(f"Tag family: {self.tag_family}, Tag size: {self.tag_size}m")

        # Publisher for AprilTag detections
        self.detection_publisher = self.create_publisher(
            AprilTagDetectionArray,
            '/apriltag_detections',
            10
        )

        # Timer for periodic detection
        self.timer = self.create_timer(1.0 / publish_rate, self.detection_callback)

        self.last_camera_matrix = None

    def detection_callback(self):
        """
        Periodic callback to detect AprilTags and publish results.
        """
        try:
            # Get camera data directly from subscriptions
            if self.image is None or self.camera_info is None:
                self.get_logger().debug("Waiting for camera data...", throttle_duration_sec=5.0)
                return

            # Convert ROS image to OpenCV format
            image = self.convert_image_msg_to_frame(self.image)

            # Get camera matrix from camera_info
            camera_matrix = np.array(self.camera_info.k).reshape(3, 3)
            self.last_camera_matrix = camera_matrix

            # Detect AprilTags
            detections = detect_apriltags(
                image,
                camera_matrix,
                tag_size=self.tag_size,
                tag_family=self.tag_family,
                uuid=self.uuid,
                debug=self.debug,
                save_vision=self.save_vision
            )

            # Create and publish detection message
            detection_array_msg = AprilTagDetectionArray()
            detection_array_msg.header = Header()
            detection_array_msg.header.stamp = self.get_clock().now().to_msg()
            detection_array_msg.header.frame_id = "camera_frame"

            if detections is not None:
                for det in detections:
                    tag_msg = AprilTagDetection()
                    tag_msg.tag_id = int(det['id'])
                    tag_msg.x = float(det['center'][0])
                    tag_msg.y = float(det['center'][1])
                    tag_msg.distance = float(det['distance'])
                    tag_msg.roll = float(det['rotation'][0])
                    tag_msg.pitch = float(det['rotation'][1])
                    tag_msg.yaw = float(det['rotation'][2])
                    tag_msg.position = [float(x) for x in det['tvec']]

                    detection_array_msg.detections.append(tag_msg)

                self.get_logger().debug(
                    f"Published {len(detections)} AprilTag detection(s)",
                    throttle_duration_sec=2.0
                )

            # Publish even if empty (so subscribers know we're still looking)
            self.detection_publisher.publish(detection_array_msg)

        except Exception as e:
            self.get_logger().error(f"Error in detection callback: {e}")
            self.publish_failsafe()


def main():
    rclpy.init()
    node = AprilTagDetectionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"AprilTag Detection Node error: {e}")
        node.publish_failsafe()
    finally:
        node.cleanup()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
