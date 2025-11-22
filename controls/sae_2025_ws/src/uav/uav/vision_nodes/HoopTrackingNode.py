# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import rotate_image
from uav.vision_nodes import VisionNode
from uav.cv.hoop_detect import detect_contour, estimate_pose_from_contour
import rclpy
from uav_interfaces.srv import HoopTracking

class HoopTrackingNode(VisionNode):

    srv = HoopTracking
    
    def __init__(self):
        super().__init__('hoop_tracking', self.__class__.srv)
        self.create_service(HoopTracking, self.service_name(), self.service_callback)
        
    def service_callback(self, request, response):
        """Main loop: get camera image, detect hoop, estimate pose, and publish."""
        # Request image
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        if image_msg is None:
            return response

        frame = self.convert_image_msg_to_frame(image_msg)
        # frame = rotate_image(frame, -np.rad2deg(request.yaw))

        # Detect the largest contour
        ellipse_contour, ellipse_params = detect_contour(frame)
        
        # Check if detection was successful
        if ellipse_contour is None or ellipse_params is None:
            response.success = False
            response.r_vec = [0.0, 0.0, 0.0]
            response.t_vec = [0.0, 0.0, 0.0]
            return response
        
        # PnP pose estimation
        success, rvec, tvec, c_obj = estimate_pose_from_contour(
            ellipse_contour, 
            ellipse_params,
            camera_matrix = np.array(camera_info.k).reshape(3,3),
            dist_coeffs = np.array(camera_info.d)
        )

        response.success = success
        response.r_vec = rvec.flatten().tolist() if rvec is not None else [0.0,0.0,0.0]
        response.t_vec = tvec.flatten().tolist() if tvec is not None else [0.0,0.0,0.0]
        # response.camera_pos = c_obj.flatten().tolist() if c_obj is not None else [0.0,0.0,0.0]
        # if ellipse is not None:
        #     (cx, cy), (major, minor), angle = ellipse
        #     response.ellipse = [float(cx), float(cy), float(major), float(minor), float(angle)]
        # else:
        #     response.ellipse = [0.0,0.0,0.0,0.0,0.0]
    
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
