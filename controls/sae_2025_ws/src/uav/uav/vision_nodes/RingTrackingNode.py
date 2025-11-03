# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_payload, compute_3d_vector, rotate_image
from uav.vision_nodes import VisionNode
# from uav_interfaces.srv import PayloadTracking  # OLD: not using a custom service anymore
import rclpy
from uav.utils import pink, green, blue, yellow

class RingTrackingNode(VisionNode):
    """
    ROS node for ring tracking
    """
    # srv = PayloadTracking # OLD: not using service anymore, instead using topic
    def __init__(self):
        super().__init__(custom_service=None)  # not using a custom service anymore. Rather, using publisher

        self.color_map = {
            'pink': pink,
            'green': green,
            'blue': blue,
            'yellow': yellow
        }
        
        # Initialize Kalman filter
        self.kalman = cv2.KalmanFilter(4, 2)
        self._setup_kalman_filter()

        from std_msgs.msg import Float64MultiArray  # moved here to avoid global import clutter
        self.publish_msg_type = Float64MultiArray
        self.ring_pub = self.create_publisher(self.publish_msg_type, '/ring_tracking', 10)

        # Publish at ≥20 Hz (0.05 s)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

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
        
    # ---------------------  PUBLISHER CALLBACK  ---------------------
    def timer_callback(self):
        """Runs at 20 Hz; performs detection and publishes results."""
        # Acquire newest data
        image_msg, camera_info_msg = self.request_data(cam_image=True, cam_info=True)
        if image_msg is None:
            # No camera yet – skip until first frame arrives
            return

        image = self.convert_image_msg_to_frame(image_msg)

        # NOTE: Replace the following with actual ring-finding algorithm
        # For now we reuse the existing find_payload helper (update later)
        detection = find_payload(
            image,
            *pink,
            *(self.color_map[request.payload_color]),
            self.uuid,
            self.debug,
            self.save_vision
        )


####### OLD PAYLOADTRACKING NODE CODE ##############

# dlz_empty = False
#         if detection is not None:
#             cx, cy, dlz_empty = detection
#             # Update Kalman filter with measurement
#             # measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
#             # corrected_state = self.kalman.correct(measurement)
#             # x, y = corrected_state[0, 0], corrected_state[1, 0]
#             x, y = cx, cy
#         else:
#             # # Use prediction if no detection
#             # x, y = predicted_x, predicted_y
#             x, y = image.shape[:2]
#             x /= 2
#             y /= 2
            
        
#         # Compute 3D direction vector
#         direction = compute_3d_vector(x, y, np.array(camera_info.k, ).reshape(3, 3), request.altitude)
            
#         # Populate response
#         response.x = float(x)
#         response.y = float(y)
#         response.direction = direction
#         response.dlz_empty = dlz_empty
#         return response
        if detection is None:
            return  # nothing to publish this cycle

        cx, cy, dlz_empty = detection

        direction = compute_3d_vector(
            cx,
            cy,
            np.array(camera_info_msg.k).reshape(3, 3),
            0.0  # altitude not used for direction-only vector; adjust if needed
        )

        # Pack message: [x, y, dir_x, dir_y, dir_z, dlz_empty]
        msg = self.publish_msg_type()
        msg.data = [float(cx), float(cy)] + list(direction) + [1.0 if dlz_empty else 0.0]
        self.ring_pub.publish(msg)

        if self.display:
            self.display_frame(image, self.node_name())

        # ---------------------  LEGACY SERVICE CALLBACK (optional)  ---------------------

def main():
    rclpy.init()
    node = RingTrackingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
