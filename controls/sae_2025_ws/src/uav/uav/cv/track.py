import numpy as np
import cv2
from calibrate import calibrate
from recalibrate import recalibrate
from threshold import threshold
from confidence import confidence
import matplotlib.pyplot as plt

range = ((0, 0, 180), (225, 225, 255))

points = []

#returns the range of the threshold
def process_first_frame(first_frame):
    return calibrate(first_frame)

def process_video(input_path: str):
    cap = cv2.VideoCapture(input_path)

    opened, first_frame = cap.read()

    prev_frame = first_frame
    prev_center = (0, 0)


    threshold_range = calibrate(first_frame)

    while cap.isOpened():
        _, frame = cap.read()

        if recalibrate(frame, prev_frame, range):
            threshold_range = calibrate(frame)

        points, center = threshold(threshold_range, prev_center, frame)

        new_points = [conversion(p, 100) for p in points]
        print(new_points)

        conf = confidence(points, -1, *frame[:2])
        print(f"{conf:.2f}, {center}")

        prev_frame = frame
        prev_center = center

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


def conversion(pixel, altitude):
    K = np.array([[1, 0, 1],
              [0, 1, 1],
              [0,  0,  10]])
    u, v = pixel
    Z = altitude
    X_c = (u - K[0, 2]) * Z / K[0, 0]
    Y_c = (v - K[1, 2]) * Z / K[1, 1]
    Z_c = Z
    # Define extrinsic parameters for a downward-facing camera
    R = np.array([[1,  0,  0],
                [0,  0, -1],
                [0,  1,  0]])  # Rotation matrix
    # Transform to world coordinates
    coords = np.array([X_c, Y_c, Z_c])
    coords = np.dot(R, coords)
    return coords

if __name__ == "__main__":
    input_path = "a.MOV"
    process_video(input_path)
