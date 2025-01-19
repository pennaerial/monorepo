import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from std_msgs.msg import Header

class CameraDisplayNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Replace with your actual topic name
            self.listener_callback,
            10
        )
        self.get_logger().info('Camera Display Node has started!')

    def listener_callback(self, msg):
        try:
            # The image data is in msg.data as a byte array
            # Convert the byte array to a NumPy array
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            
            # The message may be in different encoding (e.g., 'bgr8', 'rgb8', etc.)
            # Ensure the encoding matches your camera's output.
            # In this case, assuming 'bgr8' encoding (3 channels for BGR)
            img_height = msg.height
            img_width = msg.width
            img_channels = 3  # Assuming 3 channels (BGR)

            # Reshape the image data to a 2D array (height, width, channels)
            cv_image = img_data.reshape((img_height, img_width, img_channels))

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
