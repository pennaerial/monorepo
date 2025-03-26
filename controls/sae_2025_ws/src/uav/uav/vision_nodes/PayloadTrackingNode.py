# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_payload, compute_3d_vector, rotate_image
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import PayloadTracking
import rclpy
from uav.utils import pink, green, blue

class PayloadTrackingNode(VisionNode):
    """
    ROS node for payload tracking with Kalman filtering.
    """
    srv = PayloadTracking
    def __init__(self):
        super().__init__('payload_tracking', self.__class__.srv)
        
        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

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
        image = rotate_image(image, -np.rad2deg(request.yaw))

        # Predict next state
        prediction = self.kalman.predict()
        predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]
        
        # Get raw detection
        detection = find_payload(
            image,
            *pink,
            *(green if request.payload_color == 'green' else blue if request.payload_color == 'blue' else pink),
            self.debug
        )
        dlz_empty = False
        if detection is not None:
            cx, cy, dlz_empty = detection
            # Update Kalman filter with measurement
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            corrected_state = self.kalman.correct(measurement)
            x, y = corrected_state[0, 0], corrected_state[1, 0]
            x, y = cx, cy
        else:
            # Use prediction if no detection
            x, y = predicted_x, predicted_y
            
        # Compute 3D direction vector
        direction = compute_3d_vector(x, y, np.array(camera_info.k, ).reshape(3, 3), request.altitude)
            
        # Populate response
        response.x = float(x)
        response.y = float(y)
        response.direction = direction
        response.dlz_empty = dlz_empty
        return response

def main():
    rclpy.init()
    node = PayloadTrackingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()