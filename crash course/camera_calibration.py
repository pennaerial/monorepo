import cv2
import numpy as np


def calibrate_camera():
    # TODO: calibrate camera
    world_points = []  # 3D points in world coordinates
    image_points = []  # 2D points in image coordinates
    # HINT: use cv.findChessboardCorners() to find the corners of the chessboard in image coordinates
    w, h = None, None  # image width and height in pixels
    K, D = None, None
    _, K, D, _, _ = cv2.calibrateCamera(world_points, image_points, (w, h), K, D)
    return K, D


def live_feed(K, R, T, D):
    cap = cv2.VideoCapture(0)
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Undistort the frame
        h, w = frame.shape[:2]
        newcameramtx, roi = cv2.getOptimalNewCameraMatrix(K, D, (w, h), 1, (w, h))
        undistorted = cv2.undistort(frame, K, D, None, newcameramtx)

        # Draw a 1 foot square at 5 feet depth
        square_size = 1
        depth = 5

        # Define 3D points of the square in world coordinates
        object_points = np.array(
            [
                [-square_size / 2, -square_size / 2, depth],
                [square_size / 2, -square_size / 2, depth],
                [square_size / 2, square_size / 2, depth],
                [-square_size / 2, square_size / 2, depth],
            ],
            dtype=np.float32,
        )

        # Project 3D points to image plane
        image_points, _ = cv2.projectPoints(
            object_points, R, T, K, D
        )  # point = K @ cv2.undistort((R @ point + T))
        image_points = image_points.reshape(-1, 2)

        # Draw the square on the undistorted frame
        for i in range(4):
            pt1 = tuple(image_points[i].astype(int))
            pt2 = tuple(image_points[(i + 1) % 4].astype(int))
            cv2.line(undistorted, pt1, pt2, (0, 255, 0), 2)

        # Combine frames side by side
        combined = np.hstack((frame, undistorted))

        cv2.imshow("Live Feed | Undistorted with 1ft Square", combined)
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    K, D = calibrate_camera()
    # assume no rotation and translation from camera coordinate to world coordinate
    R = np.eye(3, dtype=np.float64)
    T = np.zeros((3, 1), dtype=np.float64)
    live_feed(K, R, T, D)
