import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopTracking
from uav.vision_nodes import HoopTrackingNode
from typing import Optional, Tuple
from px4_msgs.msg import VehicleStatus
import cv2

class HoopMode(Mode):
    """
    A mode for flying through a hoop.
    """

    def __init__(self, node: Node, uav: UAV, offsets: Optional[Tuple[float, float, float]] = (0.0, 0.0, 0.0)):
        """
        Initialize the HoopMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
            offsets (Optional[Tuple[float, float, float]]):
                Should denote the position of flight path relative to the center of hoop, in meters
                In NED frame: x is forward, y is right, and z is down.
        """
        super().__init__(node, uav)

        self.response = None
        self.done = False
        self.offsets = offsets
        self.camera_offsets = self.uav.camera_offsets
        self.mode = 0 # 0 for uav centering, 1 for landing, 2 for retracting, 3 for taking off
        # New: 0 for uav centering, 1 for flying to hoop center, 2 for through hoop, 3 for done

        # VZ edits - minimal distance needed to clear hoop
        # Hard set for now, should add 0.5 meters to calcualted distance
        self.min_travel_distance = 1.5 # meters



    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for finding hoop and flying through it.
        """
        self.log(f"=== MODE {self.mode} ===")

        # If UAV is unstable, skip the update
        if self.uav.roll > 0.1 or self.uav.pitch > 0.1:
            self.log("Roll or pitch detected. Waiting for stabilization.")
            return

        # Mode transitions for post-hoop states
        if self.mode == 1:
            # After starting forward motion, advance to clearing phase
            self.mode = 2
            self.forward_start_pos = self.uav.get_local_position()
            self.min_travel_distance = self.calculate_distance()
            return

        if self.mode == 2:
            # Check if we've traveled enough distance forward (e.g., 2 meters)
            current_pos = self.uav.get_local_position()
            distance_traveled = np.linalg.norm(
                np.array(current_pos[:2]) - np.array(self.forward_start_pos[:2])
            )
            self.log(f"Distance traveled: {distance_traveled:.2f}m")
            if distance_traveled > self.min_travel_distance:
                self.mode = 3
                self.done = True
                self.log("DONE! Clearing complete")
            else:
                # Keep moving forward
                self.log("Mode 2: Publishing (0, 1, 0)")
                self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)
            return

        if self.mode == 3:
            return  # Done, waiting for transition
    
        request = HoopTracking.Request()
        request.altitude = -self.uav.get_local_position()[2]
        request.yaw = float(self.uav.yaw)
        request.payload_color = 'red'
        response = self.send_request(HoopTrackingNode, request)
        
        # If no hoop pose is received, exit early
        if response is None:
            return

        # Transform from camera frame to UAV NED frame (forward-facing camera)
        # Camera: X=right, Y=down, Z=forward -> UAV NED: X=forward, Y=right, Z=down
        # Camera Y is down (positive), UAV Z is down (positive), so direct mapping
        # Camera X is right (positive), UAV Y is right (positive), so direct mapping
        # Camera Z is forward (positive), UAV X is forward (positive), so direct mapping
        self.log(f"Pixel coords: x={response.x}, y={response.y}")
        self.log(f"Raw response.direction: {response.direction}")
        direction = [response.direction[0], response.direction[2], response.direction[1]]
        self.log(f"After frame transform: {direction}")

        offsets = tuple(x / request.altitude for x in self.offsets) if request.altitude > 1 else self.offsets
        camera_offsets = tuple(x / request.altitude for x in self.camera_offsets) if request.altitude > 1 else self.camera_offsets
        direction = [x + y + z for x, y, z in zip(direction, offsets, self.uav.uav_to_local(camera_offsets))]

        # Check if centered on hoop (left/right and up/down)
        threshold = 0.05
        if (np.abs(direction[0]) < threshold and
            np.abs(direction[2]) < threshold):
            # Centered! Fly forward through the hoop
            self.log(f"CENTERED! Publishing: (0, 1, 0)")
            self.uav.publish_position_setpoint((0, 0.5, 0), relative=True)
            self.mode = 1
            return


        #edits 

        direction[0] = -direction[0]/3  # Left / Right
        direction[1] = 0  # Forward  / Back
        direction[2] = direction[2]/3 # Up / Down 
        
        # Not centered yet - adjust position without moving forward
        # The direction vector points FROM drone TO hoop, so we use it directly to move toward the hoop
        # direction[0] = 0  # Zero out forward movement until centered
        # direction[1] = direction[1]  
        # direction[2] = direction[2]  


        self.log(f"Centering - Publishing direction: {direction}")
        self.uav.publish_position_setpoint(direction, relative=True)

    def check_status(self) -> str:
        """
        Check the status of the payload lowering.

        Returns:
            str: The status of the payload lowering.
        """
        if self.done:
            return 'complete'
        return 'continue'

#----------- VZ edits - helper functions for PnP to estimate depth ------------------
# -----------------------------
# CONFIG
# -----------------------------
# Physical hoop radius in meters (match your Gazebo model!)
HOOP_RADIUS_M = 0.5

# HSV thresholds for red/orange hoop (tune as needed)
LOWER1 = np.array([0, 120, 80], dtype=np.uint8)
UPPER1 = np.array([10, 255, 255], dtype=np.uint8)
LOWER2 = np.array([170, 120, 80], dtype=np.uint8)
UPPER2 = np.array([180, 255, 255], dtype=np.uint8)

MIN_CONTOUR_AREA = 500  # ignore tiny blobs


# -----------------------------
# PnP helper functions
# -----------------------------
def extract_hoop_keypoints_from_contour(contour: np.ndarray) -> Optional[np.ndarray]:
    """
    Given a hoop contour from OpenCV (shape Nx1x2 or Nx2),
    return 4 keypoints on the rim:
        [rightmost, leftmost, topmost, bottommost]
    as a (4, 2) float32 array of pixel coordinates [u, v].

    Returns:
        keypoints_2d: np.ndarray of shape (4, 2), dtype=float32
                      or None if contour is invalid.
    """
    if contour is None or len(contour) == 0:
        return None

    # Ensure shape (N, 2)
    contour_xy = contour.reshape(-1, 2)

    # Rightmost (max x)
    right = contour_xy[np.argmax(contour_xy[:, 0])]

    # Leftmost (min x)
    left = contour_xy[np.argmin(contour_xy[:, 0])]

    # Topmost (min y)
    top = contour_xy[np.argmin(contour_xy[:, 1])]

    # Bottommost (max y)
    bottom = contour_xy[np.argmax(contour_xy[:, 1])]

    keypoints = np.stack([right, left, top, bottom], axis=0).astype(np.float32)
    return keypoints


def estimate_hoop_pose_pnp(
    keypoints_2d: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius_m: float,
    flags: int = cv2.SOLVEPNP_ITERATIVE
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray]]:
    """
    Estimate hoop pose in camera frame using PnP.

    Args:
        keypoints_2d: (4, 2) array of pixel coords [u, v]
                      must correspond to [right, left, top, bottom] on the hoop rim.
        camera_matrix: 3x3 intrinsic matrix (fx, 0, cx; 0, fy, cy; 0, 0, 1)
        dist_coeffs:   distortion coefficients (1xN), or zeros if ideal pinhole
        hoop_radius_m: physical hoop radius in meters
        flags:         cv2.solvePnP flag

    Returns:
        (rvec, tvec, R):
            rvec: (3, 1) rotation vector (Rodrigues) hoop→camera
            tvec: (3, 1) translation vector hoop→camera
            R:    (3, 3) rotation matrix hoop→camera
        or None if PnP fails.
    """
    if keypoints_2d is None or keypoints_2d.shape != (4, 2):
        return None

    # Known 3D points in hoop frame (XY plane, Z=0)
    # Order must match keypoints_2d: [right, left, top, bottom]
    object_points_3d = np.array([
        [ hoop_radius_m,  0.0,           0.0],  # right
        [-hoop_radius_m,  0.0,           0.0],  # left
        [ 0.0,            hoop_radius_m, 0.0],  # top
        [ 0.0,           -hoop_radius_m, 0.0],  # bottom
    ], dtype=np.float32)

    obj = object_points_3d.reshape(-1, 1, 3)
    img = keypoints_2d.reshape(-1, 1, 2)

    success, rvec, tvec = cv2.solvePnP(
        obj,
        img,
        camera_matrix,
        dist_coeffs,
        flags=flags
    )

    if not success:
        return None

    R, _ = cv2.Rodrigues(rvec)
    return rvec, tvec, R


def estimate_hoop_pose_from_contour(
    contour: np.ndarray,
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    hoop_radius_m: float,
    flags: int = cv2.SOLVEPNP_ITERATIVE
) -> Optional[Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    """
    Convenience wrapper:
      contour → 4 keypoints → PnP → (keypoints_2d, rvec, tvec, R)

    Returns:
        (keypoints_2d, rvec, tvec, R) or None if anything fails.
    """
    keypoints_2d = extract_hoop_keypoints_from_contour(contour)
    if keypoints_2d is None:
        return None

    pose = estimate_hoop_pose_pnp(
        keypoints_2d,
        camera_matrix,
        dist_coeffs,
        hoop_radius_m,
        flags=flags
    )
    if pose is None:
        return None

    rvec, tvec, R = pose
    return keypoints_2d, rvec, tvec, R


# -----------------------------
# Hoop detection (color + contour)
# -----------------------------
def detect_hoop_contour(image_bgr: np.ndarray) -> Optional[np.ndarray]:
    """
    Simple color-based hoop detection:
    - Convert BGR -> HSV
    - Threshold red using two ranges
    - Morphology
    - Find largest contour
    """
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)

    mask1 = cv2.inRange(hsv, LOWER1, UPPER1)
    mask2 = cv2.inRange(hsv, LOWER2, UPPER2)
    mask = cv2.bitwise_or(mask1, mask2)

    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None

    # choose largest blob by area
    largest = max(contours, key=cv2.contourArea)
    if cv2.contourArea(largest) < MIN_CONTOUR_AREA:
        return None

    return largest


# -----------------------------
# Camera intrinsics helper
# -----------------------------
def guess_camera_matrix(width: int, height: int) -> np.ndarray:
    """
    If you don't have a calibration, we guess some reasonable intrinsics
    based on image size. For proper metric Z, use your real calibration.
    """
    fx = 0.8 * width   # rough guess
    fy = 0.8 * height  # rough guess
    cx = width / 2.0
    cy = height / 2.0
    K = np.array([
        [fx,  0.0, cx],
        [0.0, fy,  cy],
        [0.0, 0.0, 1.0]
    ], dtype=np.float32)
    return K

# -----------------------------
# Main video loop
# -----------------------------
def main():
    if len(sys.argv) < 2:
        print("Usage: python hoop_pnp_video.py <video_path_or_index>")
        print("  Example: python hoop_pnp_video.py video.mp4")
        print("           python hoop_pnp_video.py 0   # webcam index 0")
        sys.exit(1)

    src = sys.argv[1]
    # Allow numeric webcam index
    try:
        src_int = int(src)
        cap = cv2.VideoCapture(src_int)
    except ValueError:
        cap = cv2.VideoCapture(src)

    if not cap.isOpened():
        print(f"❌ Could not open video source: {src}")
        sys.exit(1)

    ret, frame = cap.read()
    if not ret or frame is None:
        print("❌ Failed to read first frame.")
        cap.release()
        sys.exit(1)

    H, W = frame.shape[:2]
    camera_matrix = guess_camera_matrix(W, H)
    dist_coeffs = np.zeros((5, 1), dtype=np.float32)  # assume no distortion in sim

    print("Camera matrix (guess, please replace with real intrinsics if you have them):")
    print(camera_matrix)

    paused = False

    while True:
        if not paused:
            ret, frame = cap.read()
            if not ret or frame is None:
                print("End of stream or read error.")
                break

        vis = frame.copy()
        hoop_contour = detect_hoop_contour(frame)

        z_text = "Z: N/A"
        color = (0, 0, 255)

        if hoop_contour is not None:
            # draw contour
            cv2.drawContours(vis, [hoop_contour], -1, (0, 255, 0), 2)

            result = estimate_hoop_pose_from_contour(
                hoop_contour,
                camera_matrix,
                dist_coeffs,
                HOOP_RADIUS_M
            )

            if result is not None:
                keypoints_2d, rvec, tvec, R = result
                t_cam = tvec.reshape(3)  # [X, Y, Z] of hoop origin in camera frame

                Xc, Yc, Zc = float(t_cam[0]), float(t_cam[1]), float(t_cam[2])

                # draw keypoints
                for (u, v) in keypoints_2d:
                    cv2.circle(vis, (int(u), int(v)), 4, (255, 0, 0), -1)

                z_text = f"Z (forward distance) = {Zc:.2f} m"
                color = (0, 255, 0)

                # also print to console occasionally
                print(f"Hoop in camera frame: X={Xc:.2f} Y={Yc:.2f} Z={Zc:.2f}")

        # overlay text
        cv2.putText(vis, z_text, (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        cv2.putText(vis, "SPACE: pause/resume, Q/ESC: quit",
                    (20, H - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)

        cv2.imshow("Hoop PnP Z Distance", vis)

        key = cv2.waitKey(20) & 0xFF
        if key in (27, ord('q')):  # ESC or q
            break
        elif key == 32:  # space toggles pause
            paused = not paused
            print("⏸️  Paused" if paused else "▶️  Resumed")

    cap.release()
    cv2.destroyAllWindows()