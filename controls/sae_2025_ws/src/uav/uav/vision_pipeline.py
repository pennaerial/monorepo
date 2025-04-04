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
            # cv2.imshow("Camera Feed", cv_image)
            filtered_image = find_payload(cv_image)
            cv2.imshow("Camera Feed", filtered_image)
            # Required to process OpenCV events
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.get_logger().info("Shutting down display.")
                rclpy.shutdown()
        except Exception as e:
            self.get_logger().error(f"Failed to process image: {e}")

def find_payload(image):
    # Convert the image to HSV color space
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Define the HSV range for detecting neon pink (square)
    lower_pink = np.array([140, 50, 50])  # Lower bound of pink
    upper_pink = np.array([170, 255, 255])  # Upper bound of pink

    # Create a mask for the pink color
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

    # Apply morphological operations to clean up the mask
    kernel = np.ones((5, 5), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel)

    # Find contours in the pink mask
    contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    result_image = image.copy()
    largest_green_contour = None

    if contours:
        # Find the largest contour (assuming it's the pink square)
        largest_contour = max(contours, key=cv2.contourArea)

        # Create a mask for the pink square
        pink_square_mask = np.zeros_like(pink_mask)
        cv2.drawContours(pink_square_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

        # Mask the original image to only search within the pink square
        masked_image = cv2.bitwise_and(image, image, mask=pink_square_mask)

        # Convert the masked image to HSV
        hsv_masked = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)

        # Define the HSV range for detecting neon green (circle)
        lower_green = np.array([40, 50, 50])  # Lower bound of green
        upper_green = np.array([80, 255, 255])  # Upper bound of green

        # Create a mask for the green color within the pink square
        green_mask = cv2.inRange(hsv_masked, lower_green, upper_green)

        # Find contours in the green mask
        green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if green_contours:
            # Find the largest contour (assuming it's the green circle)
            largest_green_contour = max(green_contours, key=cv2.contourArea)

            # Draw the green circle's contour
            cv2.drawContours(result_image, [largest_green_contour], -1, (0, 255, 0), 3)

            # Get bounding box for the largest green contour
            x, y, w, h = cv2.boundingRect(largest_green_contour)
            cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)

            # print(f"Green circle found at: x={x}, y={y}, width={w}, height={h}")
        else:
            pass
            # print("No green circle detected within the pink square.")
    else:
        pass
        # print("No pink square detected.")

    if largest_green_contour is not None:
        # Calculate the centroid of the green cylinder
        M = cv2.moments(largest_green_contour)
        if M["m00"] != 0:
            cx = int(M["m10"] / M["m00"])
            cy = int(M["m01"] / M["m00"])

            # Draw a circle around the cylinder and its center
            cv2.circle(result_image, (cx, cy), 5, (0, 255, 0), -1)
            cv2.putText(result_image, f"Payload Center: ({cx}, {cy})", (cx + 10, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Display the results
    # cv2.imshow("Pink Mask", pink_mask)
    # cv2.imshow("Green Mask", green_mask)
    return pink_mask

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