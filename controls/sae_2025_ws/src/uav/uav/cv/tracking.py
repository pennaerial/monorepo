import cv2
import numpy as np


class DroneTracker:
    def __init__(self):
        self.initial_altitude = None # To store the initial altitude of the drone

    def initialize_tracking(self, alt_drone):
        # Save the initial altitude when the tracking script starts
        if self.initial_altitude is None:
            self.initial_altitude = alt_drone
            print(f"Initial drone altitude set to: {self.initial_altitude} meters")

    def navigate(self, image, cx, cy, alt_drone):
        height, width, _ = image.shape

        # Define the center of the image (target position)
        center_x = width // 2
        center_y = height // 2

        # Calculate the relative horizontal and vertical offsets
        dx = cx - center_x
        dy = cy - center_y

        # Calculate dz (vertical offset) based on initial altitude
        if self.initial_altitude is not None:
            dz = alt_drone - self.initial_altitude  # Difference in altitude
        else:
            dz = 0  # If we haven't initialized, assume no vertical offset

        # Compute magnitude and normalize
        magnitude = np.sqrt(dx ** 2 + dy ** 2 + dz ** 2)
        unit_vector = (dx / magnitude, dy / magnitude, dz / magnitude)

        return unit_vector


def find_payload(image, tracker, alt_drone):
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

            print(f"Green circle found at: x={x}, y={y}, width={w}, height={h}")

            if largest_green_contour is not None:
                # Calculate the centroid of the green cylinder
                M = cv2.moments(largest_green_contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])

                    # Draw a circle around the cylinder and its center
                    cv2.circle(result_image, (cx, cy), 5, (0, 255, 0), -1)
                    cv2.putText(result_image, f"Payload Center: ({cx}, {cy})", (cx + 10, cy - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                    # Calculate direction vector with altitude reference
                    direction_vector = tracker.navigate(frame, cx, cy, alt_drone)

                    cv2.imshow("Result Image", result_image)
                    return cx, cy, direction_vector
        else:
            print("No green circle detected within the pink square.")
    else:
        print("No pink square detected.")

    # Display the results
    cv2.imshow("Result Image", result_image)
    return None, None, None


if __name__ == "__main__":
    # Open the video file
    cap = cv2.VideoCapture("download (1).mp4")

    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()

    # Initialize the DroneTracker class
    tracker = DroneTracker()

    # Assuming you have a way to get the initial drone altitude
    initial_altitude = 100  # Example initial altitude of the drone (can be from GPS)
    tracker.initialize_tracking(initial_altitude)

    while cap.isOpened():
        ret, frame = cap.read()  # Read a frame from the video
        if not ret:
            print("End of video or error reading frame.")
            break

        # Simulate current altitude of the drone from GPS
        current_altitude = 110  # Example current altitude of the drone (can change as the drone moves)

        # Pass the current frame, tracker, and current altitude to find_payload
        cx, cy, direction_vector = find_payload(frame, tracker, current_altitude)
        if cx is not None:
            print(f"Payload Center: ({cx}, {cy}), Direction Vector: {direction_vector}")

        # Quit the loop if 'q' is pressed
        if cv2.waitKey(50) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
