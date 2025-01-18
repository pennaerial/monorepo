import cv2
import numpy as np

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
        else:
            print("No green circle detected within the pink square.")
    else:
        print("No pink square detected.")

    # Display the results
    cv2.imshow("Pink Mask", pink_mask)
    cv2.imshow("Green Mask", green_mask)
    cv2.imshow("Result Image", result_image)

    direction = navigate_using_shapes(frame, largest_contour, largest_green_contour)
    print(direction)

def navigate_using_shapes(image, square_contour, circle_contour):
    # Calculate the centers of the square and circle
    square_moments = cv2.moments(square_contour)
    circle_moments = cv2.moments(circle_contour)

    if square_moments["m00"] == 0 or circle_moments["m00"] == 0:
        print("Error: Contours are degenerate.")
        return None

    C_square = (int(square_moments["m10"] / square_moments["m00"]),
                int(square_moments["m01"] / square_moments["m00"]))
    C_circle = (int(circle_moments["m10"] / circle_moments["m00"]),
                int(circle_moments["m01"] / circle_moments["m00"]))

    # Offset between the centers
    dx = C_circle[0] - C_square[0]
    dy = C_circle[1] - C_square[1]

    # Aspect ratio of the square (for orientation check)
    x, y, w, h = cv2.boundingRect(square_contour)
    square_aspect_ratio = w / h

    # Check circle distortion (optional)
    circle_aspect_ratio = cv2.boundingRect(circle_contour)[2] / cv2.boundingRect(circle_contour)[3]

    # Determine navigation directions
    direction = {
        "horizontal": "center",
        "vertical": "center",
        "height_adjustment": "stable"
    }

    # Adjust horizontal and vertical alignment
    if dx < -10:  # Circle is to the left
        direction["horizontal"] = "left"
    elif dx > 10:  # Circle is to the right
        direction["horizontal"] = "right"

    if dy < -10:  # Circle is above
        direction["vertical"] = "forward"
    elif dy > 10:  # Circle is below
        direction["vertical"] = "backward"

    # Adjust height or tilt based on aspect ratio
    if square_aspect_ratio < 0.9:  # Square looks tall, drone may need to descend
        direction["height_adjustment"] = "descend"
    elif square_aspect_ratio > 1.1:  # Square looks wide, drone may need to ascend
        direction["height_adjustment"] = "ascend"

    # Visualization (optional)
    result_image = image.copy()
    cv2.circle(result_image, C_square, 5, (255, 0, 0), -1)  # Square center (blue)
    cv2.circle(result_image, C_circle, 5, (0, 255, 0), -1)  # Circle center (green)
    cv2.line(result_image, C_square, C_circle, (0, 255, 255), 2)  # Line between centers
    cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 0), 2)  # Square bounding box
    cv2.imshow("Navigation", result_image)
    cv2.waitKey(1)

    return direction

# Example usage
# Assuming `frame` is the video frame, `pink_square_contour` and `green_circle_contour` are detected:
# direction = navigate_using_shapes(frame, pink_square_contour, green_circle_contour)
# print(direction)

if __name__ == "__main__":
    # Open the video file
    cap = cv2.VideoCapture("controls\\sae_2025_ws\\src\\uav\\uav\\cv\\video_files\\download (1).mp4")
    
    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()

    while cap.isOpened():
        ret, frame = cap.read()  # Read a frame from the video
        if not ret:
            print("End of video or error reading frame.")
            break

        # Pass the current frame to find_pink_square
        find_payload(frame)

        # Quit the loop if 'q' is pressed
        if cv2.waitKey(100) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
