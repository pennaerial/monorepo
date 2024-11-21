import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class CameraDisplayNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Replace with your actual topic name
            self.listener_callback,
            10
        )
        self.bridge = CvBridge()  # Bridge for converting ROS Image to OpenCV
        self.get_logger().info('Camera Display Node has started!')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Display the image using OpenCV
            cv2.imshow("Camera Feed", cv_image)
            
            # Required to process OpenCV events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutting down display.")
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = CameraDisplayNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()

