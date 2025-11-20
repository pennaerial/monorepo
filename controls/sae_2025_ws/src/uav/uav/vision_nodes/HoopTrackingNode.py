# hoop_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import compute_3d_vector, rotate_image
from uav.cv.hoop import find_hoop
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import HoopTracking
import rclpy


class HoopTrackingNode(VisionNode):
    """
    ROS node for hoop tracking.
    """
    srv = HoopTracking
    
    def __init__(self):
        super().__init__('hoop_tracking', self.__class__.srv)
        self.create_service(HoopTracking, self.service_name(), self.service_callback)
        
    def service_callback(self, request: HoopTracking.Request, 
                        response: HoopTracking.Response):
        """Process hoop tracking service request"""
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        frame = self.convert_image_msg_to_frame(image_msg)

        # Rotate image to compensate for vehicle yaw for detection.
        image = rotate_image(frame, -np.rad2deg(request.yaw))
        
        # Get raw detection
        detection = find_hoop(image, self.uuid, self.debug, self.save_vision)
        
        if detection is not None:
            cx, cy = detection
            x, y = cx, cy
        else:
            # Use image center if no detection
            x = image.shape[1] / 2
            y = image.shape[0] / 2
        
        # Compute 3D direction vector
        direction = compute_3d_vector(
            x, y, 
            np.array(camera_info.k).reshape(3, 3), 
            request.altitude
        )
            
        # Populate response
        response.x = float(x)
        response.y = float(y)
        response.direction = direction
        response.detected = detection is not None
        return response

def main():
    rclpy.init()
    node = HoopTrackingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
