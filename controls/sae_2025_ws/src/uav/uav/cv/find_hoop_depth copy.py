import cv2
import numpy as np
from typing import Optional, Tuple
import os
from decimal import Decimal, ROUND_DOWN
import atexit

# Global lists to store history for plotting
history_raw_x = []
history_raw_y = []
history_raw_z = []
history_kalman_x = []
history_kalman_y = []
history_kalman_z = []

# Flag to check if Kalman filter has been initialized with the first measurement
kalman_initialized = False

kalman = cv2.KalmanFilter(6, 3)
kalman.transitionMatrix = np.array([
    [1, 0, 0, 1, 0, 0], 
    [0, 1, 0, 0, 1, 0],
    [0, 0, 1, 0, 0, 1],
    [0, 0, 0, 1, 0, 0],
    [0, 0, 0, 0, 1, 0],
    [0, 0, 0, 0, 0, 1]
], dtype=np.float32)
kalman.measurementMatrix = np.array([
    [1, 0, 0, 0, 0, 0],  # Measure x
    [0, 1, 0, 0, 0, 0],  # Measure y
    [0, 0, 1, 0, 0, 0]   # Measure z
], dtype=np.float32)
# Process noise covariance (Q)
kalman.processNoiseCov = np.eye(6, dtype=np.float32) * 1e-3

# Measurement noise covariance (R)
kalman.measurementNoiseCov = np.eye(3, dtype=np.float32) * 1e-2

# Error covariance matrix (P)
kalman.errorCovPost = np.eye(6, dtype=np.float32)

# Initial state (x, y, z, vx, vy, vz)
kalman.statePost = np.zeros(6, dtype=np.float32)

K_matrix = np.array([
        [0.5984375 * 539.9363327026367, 0.0, 0.5984375 * 640.0],
        [0.0, 0.4145833333 * 539.9363708496094, 0.4145833333 * 480.0],
        [0.0, 0.0, 1.0]
    ])

    # Known real radius (meters)
# real_world_r = 0.33 #actually 0.551
real_world_r = 0.551 #actually 0.551

object_points = np.array([
    [real_world_r, 0, 0],
    [-real_world_r, 0, 0],
    [0, real_world_r, 0],
    [0, -real_world_r, 0]
], dtype=np.float32)

dist_coeffs = np.zeros(5)

def truncateDec(num):
    num = Decimal(str(num))
    return float(num.quantize(Decimal('0.00'), rounding=ROUND_DOWN))

def find_hoop_w_depth(
        image: np.ndarray,
):
    """
    Detect hoop center in image using color thresholding.

    Args:
        image (np.ndarray): Input BGR image.
        debug (bool): If True, return an image with visualizations.

    Returns:
        Optional[Tuple[float, float, float, np.ndarray]]: A tuple (vector_x, vector_y, vector_z, img) if detection is successful;
        otherwise, a tuple (None, None, None, img).
    """
    img = cv2.resize(image, (1532//2, 796//2))
    copy = img.copy()
    copy = cv2.cvtColor(copy, cv2.COLOR_RGB2GRAY, None)
    hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV, None)

    lower_bound = np.array([0, 50, 50])
    upper_bound = np.array([10, 255, 255])

    mask1 = cv2.inRange(hsv, lower_bound, upper_bound)
    mask2 = cv2.inRange(hsv, (0, 0, 0), (180, 80, 80))
    combined_mask = cv2.bitwise_or(mask1, mask2)

    blurred = cv2.blur(combined_mask, (7, 7), None)

    retval, thresh1 = cv2.threshold(blurred, 10, 255, cv2.THRESH_BINARY_INV)

    h, w = img.shape[:2]
    mask = np.zeros((h + 2, w + 2), np.uint8)
    seed_points = [(0, 0), (w-1, 0), (0, h-1), (w-1, h-1)]
    for point in seed_points:
        if thresh1[point[1]][point[0]] != 0:
            cv2.floodFill(thresh1, mask, point, 0)


    contours, hierarchy = cv2.findContours(thresh1, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    if len(contours) > 0:
        selected_contours = contours[0]
    else: selected_contours = []
    maxArea = 0
    maxScore = 0
    for contour in contours:
        score = 0
        area = cv2.contourArea(contour)
        if area > maxArea:
            score += area
            maxArea = area

        moments = cv2.moments(contour) #returns a dictionary of moments
        if moments['m00'] != 0:
            Cx = int(moments['m10'] / moments['m00'])
            Cy = int(moments['m01'] / moments['m00'])
        else:
            Cx = w//2

        score -= abs(Cx - w//2)
        if score > maxScore:
            selected_contours = [contour]

    for contour in selected_contours:
        moments = cv2.moments(contour) #returns a dictionary of moments
        if moments['m00'] != 0:
            Cx = int(moments['m10'] / moments['m00'])
            Cy = int(moments['m01'] / moments['m00'])
            cv2.circle(img, (Cx, Cy), 4, (255, 255, 255), -1)
            cv2.arrowedLine(img, (w//2, h//2), (Cx, Cy ), (255, 0, 0), 2, cv2.LINE_AA, 0, 0.1)
            (xc, yc), (major, minor), angle_deg = cv2.fitEllipse(contour)

            a = major / 2
            b = minor / 2
            v1_unrotated = np.array([xc + a, yc])
            v2_unrotated = np.array([xc - a, yc])
            v3_unrotated = np.array([xc, yc + b])
            v4_unrotated = np.array([xc, yc - b])
            M = cv2.getRotationMatrix2D((xc, yc), angle_deg, 1)

            if xc - a > Cx or xc + a < Cx or yc + a < Cy or yc - a > Cy:
                break

            cv2.ellipse(img, (int(xc), int(yc)), (int(major/2), int(minor/2)), angle_deg, 0, 360, (255, 0, 0), 2)

            # Apply rotation to the unrotated vertices
            vertices = []
            for v_unrotated in [v1_unrotated, v2_unrotated, v3_unrotated, v4_unrotated]:
                rotated_x = M[0, 0] * v_unrotated[0] + M[0, 1] * v_unrotated[1] + M[0, 2]
                rotated_y = M[1, 0] * v_unrotated[0] + M[1, 1] * v_unrotated[1] + M[1, 2]
                vertices.append((int(rotated_x), int(rotated_y)))
            
            for vertex in vertices:
                cv2.circle(img, (vertex[0], vertex[1]), 4, (255, 255, 255), -1)

            image_points = np.array([
                vertices[0], 
                vertices[1],
                vertices[2], 
                vertices[3]
            ], dtype=np.float32)

            success, rvec, tvec, inliers = cv2.solvePnPRansac(
                object_points, image_points, K_matrix, dist_coeffs
            )
            x = truncateDec(tvec[0][0])
            y = -truncateDec(tvec[1][0])
            z = truncateDec(tvec[2][0])

            # Store raw measurements for plotting
            history_raw_x.append(x)
            history_raw_y.append(y)
            history_raw_z.append(z)

            # Kalman filter update
            measurement = np.array([[np.float32(x)], [np.float32(y)], [np.float32(z)]])

            # global kalman_initialized
            # if not kalman_initialized:
            #     # Initialize state with the first measurement
            #     # State: [x, y, z, vx, vy, vz]
            #     kalman.statePost = np.array([x, y, z, 0, 0, 0], dtype=np.float32)
            #     kalman.errorCovPost = np.eye(6, dtype=np.float32) * 0.1
            #     kalman_initialized = True
                
            #     # For the first frame, use the measurement directly as the prediction
            #     prediction = kalman.statePost
            # else:
            #     kalman.correct(measurement)  # Correct with the measurement
            #     prediction = kalman.predict()  # Predict the next state
            kalman.correct(measurement)  # Correct with the measurement
            prediction = kalman.predict()  # Predict the next state
            # Use the predicted values
            x, y, z = prediction[0][0], prediction[1][0], prediction[2][0]

            # Store Kalman estimates for plotting
            history_kalman_x.append(x)
            history_kalman_y.append(y)
            history_kalman_z.append(z)

            # Project Kalman filtered point back to image plane for visualization
            # Reconstruct 3D point in camera frame (reversing the y negation used earlier)
            point_3d = np.array([[[x, -y, z]]], dtype=np.float32)
            point_2d, _ = cv2.projectPoints(point_3d, np.zeros(3), np.zeros(3), K_matrix, dist_coeffs)
            px, py = int(point_2d[0][0][0]), int(point_2d[0][0][1])

            # Draw Kalman filtered center (Green point)
            cv2.circle(img, (px, py), 6, (0, 255, 0), -1)
            cv2.putText(img, "Kalman", (px + 10, py), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.putText(img, f"Center@({x:.2f}, {y:.2f}, {z:.2f})", (Cx-20, Cy-10), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1, cv2.LINE_AA)
            return x, y, z, img
        return None, None, None, img
    return None, None, None, img

def save_kalman_data(filename="kalman_data.npz"):
    """
    Save the history of raw measurements and Kalman filter estimates to a file.
    """
    if not history_raw_x:
        print("No data to save.")
        return

    np.savez(filename,
             raw_x=history_raw_x,
             raw_y=history_raw_y,
             raw_z=history_raw_z,
             kalman_x=history_kalman_x,
             kalman_y=history_kalman_y,
             kalman_z=history_kalman_z)
    print(f"Kalman data saved to {filename}")

# Register save_kalman_data to run on program exit
atexit.register(save_kalman_data)
