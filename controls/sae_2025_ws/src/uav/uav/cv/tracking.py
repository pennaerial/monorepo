import cv2
import numpy as np


def find_payload(image, camera_info, altitude):
    if altitude is None:
        print("No altitude data available.")
        return None
    
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    # Detect pink square
    lower_pink = np.array([140, 90, 50])
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
    #if not green_contours:
        #print("No green cylinder detected within the pink square.")
        #return None

    largest_green_contour = max(contours, key=cv2.contourArea)
    M = cv2.moments(largest_green_contour)
    if M["m00"] == 0:
        return None

    cx, cy = int(M["m10"] / M["m00"]), int(M["m01"] / M["m00"])

    # Debug visualization
    result_image = image.copy()
    cv2.rectangle(result_image, (x, y), (x + w, y + h), (255, 0, 255), 2)  # Pink square
    cv2.drawContours(result_image, [largest_green_contour], -1, (0, 255, 0), 3)  # Green cylinder
    cv2.circle(result_image, (cx, cy), 5, (0, 255, 0), -1)  # Green center point
    #cv2.imshow("Tracking Result", result_image)

    # Convert to 3D vector
    direction_vector = compute_3d_vector(cx, cy, camera_info, altitude)
    return direction_vector, cx, cy, result_image


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

def process_frame(image, camera_info, altitude, kalman):
        # 1. Predict the next state.
        prediction = kalman.predict()  # prediction is a 4x1 vector: [x, y, vx, vy]
        predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]

        # 2. Detect payload in the frame (using your existing code)
        result = find_payload(image, camera_info, altitude)
        
        if result is not None:
            detection, cx, cy, result_image = result
            # Assume find_payload returns the direction vector based on (cx, cy)
            # You may need to extract (cx, cy) directly from the detection process.
            
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            # 3. Correct the state with the new measurement.
            corrected_state = kalman.correct(measurement)
            estimated_x, estimated_y = corrected_state[0, 0], corrected_state[1, 0]
        else:
            # No detection: rely on the predicted state.
            print("Payload not detected in current frame; using prediction.")
            result_image = image.copy()
            estimated_x, estimated_y = predicted_x, predicted_y

        # 4. Now, use (estimated_x, estimated_y) to compute the 3D vector.
        direction_vector = compute_3d_vector(estimated_x, estimated_y, camera_info, altitude)
        
        # Optionally, add visualization using estimated state.
        cv2.circle(result_image, (int(estimated_x), int(estimated_y)), 5, (0, 0, 255), -1)  # Red dot for estimate
        cv2.imshow("Tracking Result", result_image)
        
        return direction_vector, estimated_x, estimated_y

if __name__ == "__main__":
    # Open the video file
    cap = cv2.VideoCapture("controls\\sae_2025_ws\\src\\uav\\uav\\cv\\video_files\\a.mov")

    # Assume dt (delta time) between frames is known (e.g., 1 if frame rate is constant)
    dt = 1.0  

    # Create a Kalman filter with 4 state variables and 2 measurement variables
    kalman = cv2.KalmanFilter(4, 2)

    # Define the state transition matrix A
    kalman.transitionMatrix = np.array([[1, 0, dt, 0],
                                        [0, 1, 0, dt],
                                        [0, 0, 1,  0],
                                        [0, 0, 0,  1]], dtype=np.float32)

    # Define the measurement matrix H
    kalman.measurementMatrix = np.array([[1, 0, 0, 0],
                                        [0, 1, 0, 0]], dtype=np.float32)

    # Process noise covariance matrix Q: accounts for uncertainty in the model (tune as needed)
    kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2

    # Measurement noise covariance matrix R: accounts for detection uncertainty (tune as needed)
    kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1

    # Optional: Set the initial error covariance matrix P
    kalman.errorCovPost = np.eye(4, dtype=np.float32)

    if not cap.isOpened():
        print("Error: Could not open video file.")
        exit()

    while cap.isOpened():
        ret, frame = cap.read()  # Read a frame from the video
        if not ret:
            print("End of video or error reading frame.")
            break

        # Pass the current frame to find_payload
        #direction_vector, cx, cy = find_payload(frame, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 100)
        direction_vector, cx, cy = process_frame(frame, [[1, 0, 0], [0, 1, 0], [0, 0, 1]], 100, kalman)
        if cx is not None:
            print(f"Payload Center: ({cx}, {cy}), Direction Vector: {direction_vector}")

        # Quit the loop if 'q' is pressed
        if cv2.waitKey(50) == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()