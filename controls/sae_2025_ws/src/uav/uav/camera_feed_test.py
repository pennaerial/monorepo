
from typing import Tuple
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
    lower_pink = np.array([200 / 2, 50, 50])
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
        lower_green = np.array([71 / 2, 30, 25])
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


def compute_3d_vector(
    x: float, 
    y: float, 
    camera_info: np.ndarray,  # expected to be a 3x3 camera intrinsic matrix
    altitude: float
) -> Tuple[float, float, float]:
    """Convert pixel coordinates to a 3D direction vector."""
    K = np.array(camera_info)
    pixel_coords = np.array([x, y, 1.0])
    cam_coords = np.linalg.inv(K) @ pixel_coords
    
    # Convert to unit vector
    # direction = cam_coords / np.linalg.norm(cam_coords)
    real_world_vector = cam_coords * altitude
    return tuple(real_world_vector / np.linalg.norm(real_world_vector))

def main():
    cap = cv2.VideoCapture(0)
    
    camera_matrix = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # Example intrinsic matrix
    altitude = 10.0  # Example altitude value
    
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        result = find_payload(frame)
        # print(result[0])
        if result:
            result_image, cx, cy, direction_vector = result
            # vector = compute_3d_vector(cx, cy, camera_matrix, altitude)
            # text = f"3D Vector: {vector}"
            # cv2.putText(result_image, text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.imshow("Payload Detection", result_image)
            # print(text)
        else:
            pass
            # cv2.imshow("Payload Detection", frame)
        
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()