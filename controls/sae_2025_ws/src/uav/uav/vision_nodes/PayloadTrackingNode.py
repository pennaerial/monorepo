# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_payload, compute_3d_vector, rotate_image
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import PayloadTracking
import rclpy

class PayloadTrackingNode(VisionNode):
    """
    ROS node for payload tracking with Kalman filtering.
    """
    srv = PayloadTracking
    def __init__(self):
        super().__init__('payload_tracking', self.__class__.srv)
        
        # Declare parameters
        self.declare_parameter('lower_pink', [140, 90, 50])
        self.declare_parameter('upper_pink', [170, 255, 255])
        self.declare_parameter('lower_green', [40, 50, 50])
        self.declare_parameter('upper_green', [80, 255, 255])
        
        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()
        
        # Get parameters
        self.lower_pink = np.array(self.get_parameter('lower_pink').value)
        self.upper_pink = np.array(self.get_parameter('upper_pink').value)
        self.lower_green = np.array(self.get_parameter('lower_green').value)
        self.upper_green = np.array(self.get_parameter('upper_green').value)

        self.create_service(PayloadTracking, self.service_name(), self.service_callback)
        
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
        
    def service_callback(self, request: PayloadTracking.Request, 
                        response: PayloadTracking.Response):
        """Process tracking service request with Kalman filtering"""
        image, camera_info = self.request_data(cam_image=True, cam_info=True)
        image = self.convert_image_msg_to_frame(image)
        image = rotate_image(image, -request.yaw)

        # Predict next state
        prediction = self.kalman.predict()
        predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]
        
        # Get raw detection
        detection = find_payload(
            image,
            self.lower_pink,
            self.upper_pink,
            self.lower_green,   
            self.upper_green,
            self.debug
        )
        
        if detection is not None:
            cx, cy, vis_image = detection
            # Update Kalman filter with measurement
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            corrected_state = self.kalman.correct(measurement)
            x, y = corrected_state[0, 0], corrected_state[1, 0]
            x, y = cx, cy
        else:
            # Use prediction if no detection
            x, y = predicted_x, predicted_y
            vis_image = image.copy()
            
        # Compute 3D direction vector
        direction = compute_3d_vector(x, y, np.array(camera_info.k, ).reshape(3, 3), request.altitude)
        
        # Show debug visualization if enabled
        if self.debug:
            cv2.circle(vis_image, (int(x), int(y)), 5, (0, 0, 255), -1)
            cv2.imshow("Tracking Debug", vis_image)
            cv2.waitKey(1)
            
        # Populate response
        response.x = float(x)
        response.y = float(y)
        response.direction = direction
        return response

def main():
    rclpy.init()
    node = PayloadTrackingNode()
    rclpy.spin(node)
    rclpy.shutdown()