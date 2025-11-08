#pose_estimator.py
"""
Pose Estimation Node for Hoop Detection
Estimates 3D position and orientation of hoop relative to drone using PnP/RANSAC-PnP.
"""
import cv2
import numpy as np
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import PoseEstimate
from uav_interfaces.msg import HoopContours
import rclpy
import yaml
import os
from typing import Optional, Tuple
from pathlib import Path

#define ellipse type
Ellipse = Tuple[Tuple[float, float], Tuple[float, float], float]

def preprocess_contour(contour: np.ndarray, epsilon_factor: float = 0.01) -> np.ndarray:
    """
    process contour to filter noise using Douglas-Peucker approximation
    
    params:
        contour: input contour points (N, 1, 2)
        epsilon_factor: approximation accuracy factor
        
    returns:
        preprocessed contour with reduced noise
    """
    if len(contour) < 5:
        return contour
    
    perimeter = cv2.arcLength(contour, True)
    epsilon = epsilon_factor * perimeter
    approx_contour = cv2.approxPolyDP(contour, epsilon, True)

    # make sure we have enough points for ellipse fitting (need at least 5)
    if len(approx_contour) >= 5:
        return approx_contour
    
    return contour


def validate_contour(contour: np.ndarray, min_points: int = 5, min_area: float = 100.0) -> bool:
    """
    validate that contour works for pose estimation (will hopefully help filter out incomplete pictures of hoops)
    
    params:
        contour: contour points to validate
        min_points: minimum number of points required
        min_area: minimum contour area required
        
    returns:
        true if valid contour, false otherwise
    """
    if contour is None or len(contour) < min_points:
        return False
    
    area = cv2.contourArea(contour)
    if area < min_area:
        return False
    
    return True


def hoop_pos_estimation(
    contour: np.ndarray, 
    camera_matrix: np.ndarray, 
    dist_coeffs: np.ndarray, 
    hoop_radius: float = 0.63825, 
    num_samples: int = 8,
    reprojection_error: float = 3.0,
    max_iterations: int = 100,
    confidence: float = 0.99
) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray], Optional[Ellipse]]:
    """
    estimates rotation + translation to reach a circular hoop using contour points
    uses ellipse fitting + PNP RANSAC 
    
    params:
    - contour: Contour points of the hoop in image space (numpy array of shape (N, 1, 2))
    - camera_matrix: camera's intrinsic matrix (3x3 numpy array)
    - dist_coeffs: camera's distortion coefficients (numpy array)
    - hoop_radius: radius of the hoop in meters
    - num_samples: number of points to sample from the fitted ellipse for PnP RANSAC
    - reprojection_error: maximum reprojection error threshold for RANSAC (pixels)
    - max_iterations: maximum iterations for RANSAC
    - confidence: confidence level for RANSAC (0-1)
    
    outputs:
    - success: boolean indicating if pose estimation was successful or not
    - rvec: rotation vector from camera to hoop (3x1 numpy array)
    - tvec: translation vector from camera to hoop (3x1 numpy array)
    - c_obj: camera position in hoop coordinates (3x1 numpy array)
    - ellipse: fitted ellipse parameters ((center), (major, minor), angle)
    """
    
    # validate contour
    if not validate_contour(contour):
        return False, None, None, None, None
    
    # preprocess contour to reduce noise
    preprocessed_contour = preprocess_contour(contour)

    # fit ellipse to contour points (handles noise and outliers)
    try:
        ellipse = cv2.fitEllipse(preprocessed_contour)
        (cx, cy), (major, minor), angle = ellipse
    except cv2.error as e:
        # if ellipse fitting fails, contour may be too incomplete or irregular
        return False, None, None, None, None

    # sample points along the fitted ellipse in image space
    angle_radians = np.deg2rad(angle)
    r_ellipse = np.array([[np.cos(angle_radians), -np.sin(angle_radians)],
                          [np.sin(angle_radians),  np.cos(angle_radians)]])
    a, b = major / 2.0, minor / 2.0

    # get evenly spaced sample points on ellipse
    thetas = np.linspace(0, 2*np.pi, num_samples, endpoint=False)
    ellipse_points = np.stack([a * np.cos(thetas), b * np.sin(thetas)], axis=1)
    image_points = (ellipse_points @ r_ellipse.T) + np.array([cx, cy])
    image_points = image_points.astype(np.float32)

    # get corresponding 3D points on hoop circle 
    object_points = np.stack([
        hoop_radius * np.cos(thetas),
        hoop_radius * np.sin(thetas),
        np.zeros_like(thetas)
    ], axis=1).astype(np.float32)

    #solve PNP ransac
    success, rvec, tvec, inliers = cv2.solvePnPRansac(
        objectPoints=object_points,
        imagePoints=image_points,
        cameraMatrix=camera_matrix,
        distCoeffs=dist_coeffs,
        flags=cv2.SOLVEPNP_EPNP,  # Efficient PnP solver
        reprojectionError=reprojection_error,
        iterationsCount=max_iterations,
        confidence=confidence
    )

    if not success or rvec is None or tvec is None:
        return False, None, None, None, None
    
    # verify we have enough inliers (at least 50% of points)
    if inliers is not None and len(inliers) < num_samples * 0.5:
        return False, None, None, None, None
    
    # calculate camera position in hoop frame
    R, _ = cv2.Rodrigues(rvec)
    c_obj = -R.T @ tvec

    return True, rvec, tvec, c_obj, ellipse


def visualize_pose(
    image: np.ndarray, 
    rvec: np.ndarray, 
    tvec: np.ndarray, 
    camera_matrix: np.ndarray, 
    dist_coeffs: np.ndarray, 
    hoop_radius: float = 0.63825,
    num_points: int = 100
) -> np.ndarray:
    """
    draw reprojected hoop back onto image for visualiazation & debugging 
    draws hoop circle & coordinate axes
    
    params:
        image: input image to draw on
        rvec: rotation vector
        tvec: translation vector
        camera_matrix: camera intrinsic matrix
        dist_coeffs: distortion coefficients
        hoop_radius: radius of hoop in meters
        num_points: number of points to use for circle visualization
        
    outputs:
        image with reprojected hoop and axes drawn
    """
    # get points around the hoop circle
    thetas = np.linspace(0, 2*np.pi, num_points)
    object_points = np.stack([
        hoop_radius * np.cos(thetas),
        hoop_radius * np.sin(thetas),
        np.zeros_like(thetas)
    ], axis=1).astype(np.float32)

    # reproject 3D points to image plane
    reprojected, _ = cv2.projectPoints(object_points, rvec, tvec, camera_matrix, dist_coeffs)
    reprojected = reprojected.reshape(-1, 2).astype(np.int32)

    # draw the hoop circle
    for i in range(len(reprojected) - 1):
        pt1 = tuple(reprojected[i])
        pt2 = tuple(reprojected[i + 1])
        cv2.line(image, pt1, pt2, (0, 255, 255), 2) 
    cv2.line(image, tuple(reprojected[-1]), tuple(reprojected[0]), (0, 255, 255), 2)

    # draw coordinate axes at origin
    axis_length = hoop_radius * 0.3
    axis_points = np.array([
        [0, 0, 0],  # Origin
        [axis_length, 0, 0],  # x-axis
        [0, axis_length, 0],  # y-axis
        [0, 0, -axis_length],  # z-axis (negative for camera pointing)
    ], dtype=np.float32)
    
    axis_reprojected, _ = cv2.projectPoints(axis_points, rvec, tvec, camera_matrix, dist_coeffs)
    axis_reprojected = axis_reprojected.reshape(-1, 2).astype(np.int32)
    
    origin = tuple(axis_reprojected[0])
    cv2.line(image, origin, tuple(axis_reprojected[1]), (0, 0, 255), 3)  # Red is x-axis
    cv2.line(image, origin, tuple(axis_reprojected[2]), (0, 255, 0), 3)  # Green is y-axis
    cv2.line(image, origin, tuple(axis_reprojected[3]), (255, 0, 0), 3)  # Blue is z-axis

    return image


class PoseEstimatorNode(VisionNode):
    """
    ROS node for estimating 3D pose of circular hoops relative to drone.
    
    inputs:
    - HoopContours.msg (from CV node)
    - Camera calibration parameters (from YAML file)
    
    outputs:
    - PoseEstimate.msg (translation vector, rotation vector)
    """
    srv = PoseEstimate
    
    def __init__(self):
        super().__init__('pose_estimator', self.__class__.srv)
        
        # get path to camera calibration file relative to this file
        # pose_estimator.py is in uav/vision_nodes/
        # camera_calibration.yaml is in uav/cv/
        current_file = Path(__file__).resolve()
        default_calib_file = str(current_file.parent.parent / 'cv' / 'camera_calibration.yaml')
        
        self.declare_parameter('hoop_radius', 0.63825)  
        self.declare_parameter('camera_calibration_file', default_calib_file)
        self.declare_parameter('num_samples', 8)  
        self.declare_parameter('reprojection_error', 3.0)  
        self.declare_parameter('max_iterations', 100)  
        self.declare_parameter('confidence', 0.99)  
        self.declare_parameter('min_contour_area', 100.0)  
        
        self.hoop_radius = self.get_parameter('hoop_radius').value
        self.camera_calibration_file = self.get_parameter('camera_calibration_file').value
        self.num_samples = self.get_parameter('num_samples').value
        self.reprojection_error = self.get_parameter('reprojection_error').value
        self.max_iterations = self.get_parameter('max_iterations').value
        self.confidence = self.get_parameter('confidence').value
        self.min_contour_area = self.get_parameter('min_contour_area').value
        
        self.create_service(PoseEstimate, self.service_name(), self.service_callback)
        
        if self.camera_calibration_file and os.path.exists(self.camera_calibration_file):
            self.get_logger().info(f'PoseEstimatorNode initialized with hoop_radius={self.hoop_radius}m, calibration: {self.camera_calibration_file}')
        else:
            self.get_logger().warn(f'PoseEstimatorNode initialized with hoop_radius={self.hoop_radius}m, but calibration file not found at {self.camera_calibration_file}')
    
    def _load_camera_info_from_yaml(self, yaml_file: str) -> Tuple[np.ndarray, np.ndarray]:
        """
        Load camera intrinsic parameters and distortion coefficients from a YAML file.
        
        Params:
            yaml_file: Path to YAML calibration file
            
        Returns:
            camera_matrix: 3x3 numpy array
            dist_coeffs: distortion coefficients array
        """
        with open(yaml_file, 'r') as file:
            cam_data = yaml.safe_load(file)
        
        camera_matrix = np.array(cam_data['camera_matrix']['data']).reshape(3, 3).astype(np.float32)
        dist_coeffs = np.array(cam_data['distortion_coefficients']['data']).astype(np.float32)
        
        return camera_matrix, dist_coeffs
    
    def _hoop_contours_to_numpy(self, contours_msg: HoopContours) -> Optional[np.ndarray]:
        """
        convert HoopContours message to OpenCV contour format
        
        params:
            contours_msg: HoopContours ROS message
            
        outputs:
            Contour in format (N, 1, 2) or None if invalid
        """
        points_x = contours_msg.points_x
        points_y = contours_msg.points_y
        
        if len(points_x) != len(points_y) or len(points_x) < 5:
            self.get_logger().warn(f'Invalid contour data: {len(points_x)} x points, {len(points_y)} y points')
            return None
        
        # convert to numpy array and reshape to opencv contour format (N, 1, 2)
        points = np.array([[x, y] for x, y in zip(points_x, points_y)], dtype=np.float32)
        contour = points.reshape(-1, 1, 2)
        
        return contour
    
    def service_callback(self, request: PoseEstimate.Request, 
                        response: PoseEstimate.Response):
        #get camera image (only needed for visualization/debugging)
        image_msg, _ = self.request_data(cam_image=True, cam_info=False)
        
        if image_msg is None:
            self.get_logger().error('Failed to get camera image')
            response.success = False
            return response
        
        # convert image message to numpy array (for visualization)
        image = self.convert_image_msg_to_frame(image_msg)
        
        # get camera matrix & distortion coefficients from YAML file
        try:
            camera_matrix, dist_coeffs = self._load_camera_info_from_yaml(self.camera_calibration_file)
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration file {self.camera_calibration_file}: {e}')
            response.success = False
            return response
        
        # get contour from HoopContours message
        contour = self._hoop_contours_to_numpy(request.contours)
        if contour is None:
            self.get_logger().warn('Invalid or empty contour data in request')
            response.success = False
            return response
        
        # validate contour area
        contour_area = cv2.contourArea(contour)
        if contour_area < self.min_contour_area:
            self.get_logger().warn(f'Contour area too small: {contour_area} < {self.min_contour_area}')
            response.success = False
            return response
        
        # get hoop radius from request (use node default if 0 or negative)
        hoop_radius = request.hoop_radius if request.hoop_radius > 0 else self.hoop_radius
        
        # do pose estimation with PnP/RANSAC
        success, rvec, tvec, c_obj, ellipse = hoop_pos_estimation(
            contour=contour,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            hoop_radius=hoop_radius,
            num_samples=self.num_samples,
            reprojection_error=self.reprojection_error,
            max_iterations=self.max_iterations,
            confidence=self.confidence
        )
        
        if not success:
            self.get_logger().warn('Pose estimation failed - check contour quality and camera calibration')
            response.success = False
            return response
        
        # populate response
        response.success = True
        response.rotation_vector = rvec.flatten().tolist()
        response.translation_vector = tvec.flatten().tolist()
        response.camera_position = c_obj.flatten().tolist()
        
        self.get_logger().debug(
            f'Pose estimation successful: tvec={tvec.flatten()}, rvec={rvec.flatten()}'
        )
        
        # visualize pose for debugging if enabled
        if self.debug:
            try:
                image_with_pose = visualize_pose(
                    image.copy(), 
                    rvec, 
                    tvec, 
                    camera_matrix, 
                    dist_coeffs, 
                    hoop_radius
                )
                self.display_frame(image_with_pose, self.node_name())
            except Exception as e:
                self.get_logger().warn(f'Failed to visualize pose: {e}')
        
        return response


def main():
    rclpy.init()
    node = PoseEstimatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down PoseEstimatorNode')
    except Exception as e:
        node.get_logger().error(f'Error in PoseEstimatorNode: {e}')
        node.publish_failsafe()
    finally:
        node.cleanup()
        rclpy.shutdown()
