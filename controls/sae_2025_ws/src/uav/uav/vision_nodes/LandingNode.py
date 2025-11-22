import cv2
import numpy as np
from uav.vision_nodes import VisionNode
import rclpy
from uav.cv.recogniseRectangle import recognise_rectangle
from uav.cv.pose_estimate import estimate_hoop_pose_from_contour
from uav_interfaces.srv import Landing
from uav.utils import red, green, blue

class LandingNode(VisionNode):
    """
    Detects a colored landing rectangle and publishes its pose.
    """

    srv = Landing

    def __init__(self, display: bool = False):
        super().__init__(self.__class__.srv, display=display)

        self.display = display
        self.color_map = {
            'red': red,
            'green': green,
            'blue': blue
        }

        self.create_service(Landing, self.service_name(), self.service_callback)

    def service_callback(self, request, response):
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        if image_msg is None:
            return response

        frame = self.convert_image_msg_to_frame(image_msg)

        contour = recognise_rectangle(frame, self.color_map[request.landing_color])
        if contour is None:
            return response
        # PnP pose estimation
        success, rvec, tvec, c_obj, ellipse = estimate_hoop_pose_from_contour(
            contour,
            camera_matrix = np.array(camera_info.k).reshape(3,3),
            dist_coeffs = np.array(camera_info.d)
        )

        response.success = success
        response.r_vec = rvec.flatten().tolist() if rvec is not None else [0.0,0.0,0.0]
        response.t_vec = tvec.flatten().tolist() if tvec is not None else [0.0,0.0,0.0]

        if self.display:
            debug_img = frame.copy()
            cv2.drawContours(debug_img, [contour], -1, (0, 255, 0), 2)
            self.display_frame(debug_img, f"Landing: {self.color_map[request.landing_color]}")

        return response


def main():
    rclpy.init()
    node = LandingNode(display=False)
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
