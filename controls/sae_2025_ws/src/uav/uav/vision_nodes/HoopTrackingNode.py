# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import rotate_image
from uav.vision_nodes import VisionNode

from geometry_msgs.msg import PoseStamped
from uav.cv.hoop_detect import find_hoop
import rclpy

class HoopTrackingNode(VisionNode):
    """
    Publishes the detected hoop pose as a ROS topic.
    """

    def __init__(self, display: bool = False):
        super().__init__('hoop_tracking', self.__class__.srv)
        
        self.display = display

        # Publisher for hoop pose
        # self.pose_pub = self.create_publisher(PoseStamped, '/hoop_pose', 10)
        # self.timer = self.create_timer(0.1, self.process_frame)  # 10Hz
        # self.get_logger().info("HoopTrackingNode started (Topic mode).")

        self.create_service(PayloadTracking, self.service_name(), self.service_callback)
        
    def service_callback(self):
        """Main loop: get camera image, detect hoop, estimate pose, and publish."""

        # Request image
        image_msg, camera_info = self.request_data(cam_image=True, cam_info=True)
        if image_msg is None:
            return

        frame = self.convert_image_msg_to_frame(image_msg)
        # frame = rotate_image(frame, -np.rad2deg(request.yaw))

        # Detect the largest contour
        contour = find_hoop(frame)  # (N,1,2)
        # # PnP pose estimation
        # pose = estimate_pose(contour)
        # # Kalman filter
        # filtered_pose = apply_kalman_filter(pose)

        # # Publish the filtered pose
        # self.pose_pub.publish(filtered_pose)

        # Visualization
        if self.display:
            debug_img = frame.copy()
            cv2.drawContours(debug_img, [contour.astype(int)], -1, (0, 255, 0), 2)
            self.display_frame(debug_img, "Hoop Tracking")
    



def main():
    rclpy.init()
    node = HoopTrackingNode(display=True)
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()
