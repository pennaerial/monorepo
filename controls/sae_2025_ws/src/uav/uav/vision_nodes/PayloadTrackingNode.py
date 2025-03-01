# payload_tracking_node.py
import cv2
import numpy as np
from cv.tracking import find_payload, compute_3d_vector, find_dlz
from uav import VisionNode
from uav.srv import PayloadTracking
from rclpy.parameter import Parameter

class PayloadTrackingNode(VisionNode):
    """
    ROS node for payload tracking with Kalman filtering.
    """
    def __init__(self):
        super().__init__('payload_tracking', PayloadTracking)
        
        # Declare parameters
        self.declare_parameter('debug', False)
        self.declare_parameter('lower_pink', [140, 90, 50])
        self.declare_parameter('upper_pink', [170, 255, 255])
        self.declare_parameter('lower_green', [40, 50, 50])
        self.declare_parameter('upper_green', [80, 255, 255])
        
        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self.kalman_dlz = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()
        
        # Get parameters
        self.debug = self.get_parameter('debug').value
        self.lower_pink = np.array(self.get_parameter('lower_pink').value)
        self.upper_pink = np.array(self.get_parameter('upper_pink').value)
        self.lower_green = np.array(self.get_parameter('lower_green').value)
        self.upper_green = np.array(self.get_parameter('upper_green').value)
        
    def _setup_kalman_filter(self):
        """Initialize Kalman filter matrices"""
        dt = 1  
        
        # State transition matrix [x, y, vx, vy]
        self.kalman.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Measurement matrix [x, y]
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Tune these covariances idk
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        self.kalman_dlz.transitionMatrix = np.array([
            [1, 0, dt, 0],
            [0, 1, 0, dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1]
        ], dtype=np.float32)
        
        # Measurement matrix [x, y]
        self.kalman_dlz.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        
        # Tune these covariances idk
        self.kalman_dlz.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman_dlz.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman_dlz.errorCovPost = np.eye(4, dtype=np.float32)
        
    def service_callback(self, request: PayloadTracking.Request, 
                        response: PayloadTracking.Response):
        """Process tracking service request with Kalman filtering"""
        # Predict next state
        prediction = self.kalman.predict()
        predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]
        dlz_prediction = self.kalman_dlz.predict()
        dlz_predicted_x, dlz_predicted_y = dlz_prediction[0, 0], dlz_prediction[1, 0] 
        # Get raw detection
        payload_detection = find_payload(
            self.curr_frame,
            self.lower_pink,
            self.upper_pink,
            self.lower_green,
            self.upper_green,
            self.debug
        )
        
        if payload_detection is not None:
            cx, cy, vis_image = payload_detection
            # Update Kalman filter with measurement
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            corrected_state = self.kalman.correct(measurement)
            x, y = corrected_state[0, 0], corrected_state[1, 0]
        else:
            # Use prediction if no detection
            dlz_detection = find_dlz(self.curr_frame,
            self.lower_pink,
            self.upper_pink,
            self.lower_green,
            self.upper_green,
            self.debug)
            if dlz_detection is not None:
                cx_dlz, cy_dlz, vis_image = dlz_detection
                measurement = np.array([[np.float32(cx_dlz)], [np.float32(cy_dlz)]])
                corrected_state = self.kalman_dlz.correct(measurement)
                x, y = predicted_x, predicted_y
            else:
                x, y = dlz_predicted_x, dlz_predicted_y

            vis_image = self.curr_frame.copy()
            
        # Compute 3D direction vector
        direction = self.compute_3d_vector(x, y, request.camera_info, request.altitude)
        
        # Show debug visualization if enabled
        if self.debug:
            cv2.circle(vis_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.imshow("Tracking Debug", vis_image)
            cv2.waitKey(1)
            
        # Populate response
        response.x = x
        response.y = y
        response.direction = direction
        return response
