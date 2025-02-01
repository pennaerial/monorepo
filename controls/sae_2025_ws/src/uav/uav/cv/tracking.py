import cv2
import numpy as np


def find_payload(image, camera_info, altitude):
    if altitude is None:
        print("No altitude data available.")
        return None
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Detect pink square
    lower_pink = np.array([140, 50, 50])
    upper_pink = np.array([170, 255, 255])
    pink_mask = cv2.inRange(hsv_image, lower_pink, upper_pink)

    kernel = np.ones((5, 5), np.uint8)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_CLOSE, kernel)
    pink_mask = cv2.morphologyEx(pink_mask, cv2.MORPH_OPEN, kernel)

    contours, _ = cv2.findContours(pink_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        print("No pink square detected.")
        return None

    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)

    # Create a bounding mask for detected pink square
    pink_square_mask = np.zeros_like(pink_mask)
    cv2.drawContours(pink_square_mask, [largest_contour], -1, 255, thickness=cv2.FILLED)

    # Convert masked image to HSV
    hsv_masked = cv2.bitwise_and(hsv_image, hsv_image, mask=pink_square_mask)

    # Detect green (payload) within the pink square
    lower_green = np.array([40, 50, 50])
    upper_green = np.array([80, 255, 255])
    green_mask = cv2.inRange(hsv_masked, lower_green, upper_green)

    green_contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not green_contours:
        print("No green cylinder detected within the pink square.")
        return None

    largest_green_contour = max(green_contours, key=cv2.contourArea)
    M = cv2.moments(largest_green_contour)
    if M["m00"] == 0:
        return None

    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    # Debug visualization
    result_image = image.copy()
    cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 255), 2)  # Pink square
    cv2.drawContours(result_image, [largest_green_contour], -1, (0, 255, 0), 3)  # Green cylinder
    cv2.circle(result_image, (cx, cy), 5, (0, 255, 0), -1)  # Green center point
    cv2.imshow("Tracking Result", result_image)

    # Convert to 3D vector
    direction_vector = compute_3d_vector(cx, cy, camera_info, altitude)
    return direction_vector


def compute_3d_vector(cx, cy, camera_info, altitude):
    K = np.array(camera_info)  # Camera intrinsic matrix]  # Drone altitude (z-coordinate)

    # Convert pixel coordinates to normalized camera coordinates
    pixel_coords = np.array([cx, cy, 1.0])
    cam_coords = np.linalg.inv(K) @ pixel_coords  # Apply inverse intrinsic matrix

    # Convert to unit vector
    direction = cam_coords / np.linalg.norm(cam_coords)
    real_world_vector = cam_coords * altitude
    direction = real_world_vector / np.linalg.norm(real_world_vector)
    
    return tuple(direction)
