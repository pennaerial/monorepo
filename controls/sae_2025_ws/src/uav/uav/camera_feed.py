# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import Image
# import cv2
# import numpy as np
# from std_msgs.msg import Header

# class CameraDisplayNode(Node):
#     def __init__(self):
#         super().__init__('camera_display_node')
#         self.subscription = self.create_subscription(
#             Image,
#             '/camera',  # Replace with your actual topic name
#             self.listener_callback,
#             10
#         )
#         self.get_logger().info('Camera Display Node has started!')

#     def listener_callback(self, msg):
#         try:
#             # The image data is in msg.data as a byte array
#             # Convert the byte array to a NumPy array
#             img_data = np.frombuffer(msg.data, dtype=np.uint8)
            
#             # The message may be in different encoding (e.g., 'bgr8', 'rgb8', etc.)
#             # Ensure the encoding matches your camera's output.
#             # In this case, assuming 'bgr8' encoding (3 channels for BGR)
#             img_height = msg.height
#             img_width = msg.width
#             img_channels = 3  # Assuming 3 channels (BGR)

#             # Reshape the image data to a 2D array (height, width, channels)
#             cv_image = img_data.reshape((img_height, img_width, img_channels))

#             # Display the image using OpenCV
#             cv2.imshow("Camera Feed", cv_image)
            
#             # Required to process OpenCV events
#             if cv2.waitKey(1) & 0xFF == ord('q'):
#                 self.get_logger().info("Shutting down display.")
#                 rclpy.shutdown()
#         except Exception as e:
#             self.get_logger().error(f"Failed to process image: {e}")

# def main(args=None):
#     rclpy.init(args=args)
#     node = CameraDisplayNode()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     # Destroy the node explicitly
#     node.destroy_node()
#     rclpy.shutdown()
#     cv2.destroyAllWindows()


# if __name__ == '__main__':
#     main()



import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float32MultiArray
import cv2
import numpy as np



def navigate(image, cx, cy):
    height, width, _ = image.shape

    # Define the center of the image (target position)
    center_x = width // 2
    center_y = height // 2

    # Calculate the relative horizontal and vertical offsets
    dx = cx - center_x
    dy = cy - center_y

    # Normalize the direction vector to a unit vector
    magnitude = np.sqrt(dx**2 + dy**2)
    unit_vector = (dx / magnitude, dy / magnitude)

    return unit_vector

def find_payload(image):
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the HSV range for detecting neon pink (square)
    lower_pink = np.array([290 / 2, 100, 100])
    upper_pink = np.array([360 / 2, 255, 255])
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)
    pink_image = cv2.bitwise_and(image, image, mask=pink_mask)

    # Morphological operations to clean the mask
    kernel = np.ones((5, 5), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel)
    pink_image = cv2.bitwise_and(image, image, mask=pink_mask)

    # cv2.imshow("mask", pink_image)
    # Find contours
    contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    result_image = image.copy()

    if contours:
        # Find the largest pink square
        largest_contour = max(contours, key=cv2.contourArea)
        pink_square_mask = np.zeros_like(pink_mask)
        cv2.drawContours(pink_square_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # Mask the region within the pink square
        masked_image = cv2.bitwise_and(image, image, mask=pink_square_mask)
        hsv_masked = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
        cv2.imshow("masked image", masked_image)

        # Detect neon green (circle) within the pink square
        lower_green = np.array([71 / 2, 50, 50])
        upper_green = np.array([160 / 2, 255, 255])
        green_mask = cv2.inRange(hsv_masked, lower_green, upper_green)
        green_image = cv2.bitwise_and(image, image, mask=green_mask)

        cv2.imshow("green mask", green_image)
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if green_contours:
            # Find the largest green circle
            largest_green_contour = max(green_contours, key=cv2.contourArea)
            cv2.drawContours(result_image, [largest_green_contour], -1, (0, 255, 0), 3)
            x, y, w, h = cv2.boundingRect(largest_green_contour)
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

            M = cv2.moments(largest_green_contour)
            if M["m00"] != 0 or True:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                cv2.circle(result_image, (cx, cy), 10, (120, 255, 120), 5)
                direction_vector = navigate(result_image, cx, cy)
                return result_image, cx, cy, direction_vector

        print("No green circle detected within the pink square.")
    else:
        print("No pink square detected.")

    return result_image, None, None, None


class CameraDisplayNode(Node):
    def __init__(self):
        super().__init__('camera_display_node')

        # Subscription to the camera feed
        self.subscription = self.create_subscription(
            Image,
            '/camera',  # Replace with your actual topic name
            self.listener_callback,
            10
        )
        
        # Publisher for the payload tracking results
        self.payload_publisher = self.create_publisher(
            Float32MultiArray,
            '/payload_tracking',
            10
        )

        self.get_logger().info('Camera Display Node with Payload Tracking has started!')

    def listener_callback(self, msg):
        try:
            # Convert ROS2 Image message to OpenCV format
            img_data = np.frombuffer(msg.data, dtype=np.uint8)
            img_height = msg.height
            img_width = msg.width
            img_channels = 3  # Assuming 3 channels (BGR)
            cv_image = img_data.reshape((img_height, img_width, img_channels))

            # Pass the frame to find_payload for processing
            result_image, cx, cy, direction_vector = find_payload(cv_image)
            if cx is not None:
                self.get_logger().info(f"Payload Center: ({cx}, {cy}), Direction: {direction_vector}")
                tracking_msg = Float32MultiArray()
                tracking_msg.data = [float(cx), float(cy), direction_vector[0], direction_vector[1]]
                self.payload_publisher.publish(tracking_msg)
            else:
                self.get_logger().info("No payload detected.")

            # Display the processed frame (result_image with annotations)
            cv2.imshow("Camera Feed with Payload Tracking", result_image)
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

    # Clean up resources
    node.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
