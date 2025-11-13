#!/usr/bin/env python3
# hoop_tracking_node.py
import cv2
import numpy as np
from uav.cv.in_house.hoop import detect_hoop
from uav.cv.tracking import compute_3d_vector, rotate_image
from uav.vision_nodes import VisionNode
from uav_interfaces.msg import HoopTracking, UAVState
import rclpy


class HoopTrackingNode(VisionNode):
    """
    ROS node for hoop tracking with Kalman filtering.
    Publishes tracking data continuously.
    """
    
    def __init__(self):
        super().__init__('hoop_tracking', None)
        
        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

        # Create publisher for hoop tracking data
        self.tracking_publisher = self.create_publisher(
            HoopTracking, 
            '/vision/hoop_tracking', 
            10
        )
        
        # Subscribe to UAV state
        self.uav_state_sub = self.create_subscription(
            UAVState,
            '/uav_state',
            self.uav_state_callback,
            10
        )
        
        self.current_altitude = 0.0
        self.current_yaw = 0.0
        
        # Create timer to publish tracking data at regular intervals
        self.timer = self.create_timer(0.1, self.publish_tracking_data)  # 10 Hz
        
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
        
        # Tune these covariances
        self.kalman.processNoiseCov = np.eye(4, dtype=np.float32) * 1e-2
        self.kalman.measurementNoiseCov = np.eye(2, dtype=np.float32) * 1e-1
        self.kalman.errorCovPost = np.eye(4, dtype=np.float32)
        
    def uav_state_callback(self, msg: UAVState):
        """Update current UAV state from subscription"""
        self.current_altitude = msg.altitude
        self.current_yaw = msg.yaw
        
    def publish_tracking_data(self):
        """Publish hoop tracking data at regular intervals"""
        image, camera_info = self.request_data(cam_image=True, cam_info=True)
        
        # Check if we have valid image and camera info
        if image is None or camera_info is None:
            return
            
        image = self.convert_image_msg_to_frame(image)
        image = rotate_image(image, -np.rad2deg(self.current_yaw))

        # Predict next state
        prediction = self.kalman.predict()
        predicted_x, predicted_y = prediction[0, 0], prediction[1, 0]
        
        # Get raw detection
        detection = detect_hoop(
            image,
            self.uuid,
            self.debug,
            self.save_vision
        )
        
        hoop_detected = False
        if detection is not None:
            cx, cy = detection
            # Update Kalman filter with measurement
            measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
            corrected_state = self.kalman.correct(measurement)
            x, y = corrected_state[0, 0], corrected_state[1, 0]
            hoop_detected = True
        else:
            # Use prediction if no detection
            x, y = predicted_x, predicted_y
            # Fallback to image center if prediction is invalid
            if x <= 0 or y <= 0:
                h, w = image.shape[:2]
                x, y = w / 2, h / 2
        
        # Compute 3D direction vector
        direction = compute_3d_vector(x, y, np.array(camera_info.k).reshape(3, 3), self.current_altitude)
            
        # Create and publish message
        msg = HoopTracking()
        msg.x = float(x)
        msg.y = float(y)
        msg.direction = direction
        msg.hoop_detected = hoop_detected
        
        self.tracking_publisher.publish(msg)


def main():
    rclpy.init()
    node = HoopTrackingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
