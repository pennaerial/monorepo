# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_payload, compute_3d_vector
from uav.cv.hoopDetectorAlgorithm import find_nearest_hoop_pose
from uav.cv.hoopDetectorAlgorithmFloodfill import detect
from uav.vision_nodes import VisionNode
# from uav_interfaces.srv import PayloadTracking  # OLD: not using a custom service anymore
from std_srvs.srv import Trigger
import rclpy
from uav.utils import pink, green, blue, yellow

class RingTrackingNode(VisionNode):
    """
    ROS node for ring tracking
    """
    srv = Trigger # OLD: not using service anymore, instead using topic

    #fake service to trick modemanager
    @staticmethod
    def service_name() -> str:
        # ModeManager calls vision_class.service_name()
        return "/vision/ring_tracking_node"

    def __init__(self):
        super().__init__(custom_service=None)  # not using a custom service anymore. Rather, using publisher

        self.color_map = {
            'pink': pink,
            'green': green,
            'blue': blue,
            'yellow': yellow
        }
        
        # Initialize Kalman filter
        # self.kalman = cv2.KalmanFilter(4, 2)
        # self._setup_kalman_filter()
        
        #fake to trick modemaanager
        from std_msgs.msg import Float64MultiArray  # moved here to avoid global import clutter

        self.trigger_srv = self.create_service(
            self.__class__.srv,
            self.__class__.service_name(),
            self._handle_trigger,
        )

        self.publish_msg_type = Float64MultiArray
        self.ring_pub = self.create_publisher(self.publish_msg_type, '/ring_tracking', 10)

        # Publish at ≥20 Hz (0.05 s)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
    
    
    #Fake service
    def _handle_trigger(self, request, response):
        # Minimal “I’m alive” response
        response.success = True
        response.message = "RingTrackingNode is running (fake service)."
        return response
        
    # ---------------------  PUBLISHER CALLBACK  ---------------------
    def timer_callback(self):
        """Runs at 20 Hz; performs detection and publishes results."""
        # Ensure we have at least one image and camera_info before proceeding
        if self.image is None:
            print("RingTrackingNode: waiting for image...")
            return
        if self.camera_info is None:
            print("RingTrackingNode: waiting for camera_info...")
            return  # wait until the bridge delivers first data

        # Acquire newest data via helper (now guaranteed to be non-None)
        image_msg, camera_info_msg = self.request_data(cam_image=True, cam_info=True)

        # Sanity checks: ensure buffers are populated
        if image_msg is None or len(image_msg.data) < 10:
            self.get_logger().debug("RingTrackingNode: empty image_msg, skipping.")
            return
        if camera_info_msg is None or len(camera_info_msg.k) != 9:
            self.get_logger().debug("RingTrackingNode: incomplete camera_info, skipping.")
            return

        image = self.convert_image_msg_to_frame(image_msg)
        # image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')

        
        ###### CRAZY PRANAV ALGO #####
        # result = find_nearest_hoop_pose(image, camera_info_msg.k, 0.5)


        # if len(result) == 2:
        #     self.display_frame(image, self.node_name())

        #     result_data, intermediate_frames = result
        #     result_data = None
        #     print("not detecting anyyyything")
        #     dummy = self.publish_msg_type()
        #     dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
        #     self.ring_pub.publish(dummy)
        #     return
        # else:
        #     center_3d, normal_3d, ellipse, used_radius, annotated_frame = result
        #     self.display_frame(annotated_frame, self.node_name())

        #     # result_data = (center_3d, normal_3d, ellipse, used_radius)



        # if center_3d is not None:
        #     dir_x, dir_z, dir_y = center_3d

        #     dir_x = -dir_x

        #     #need to flip direction of the z axis because in gazebo, up is negative z
        #     # dir_z = -dir_z
        #     good = self.publish_msg_type()
        #     good.data = [0.0, 0.0, dir_x, dir_y, dir_z, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
        #     self.ring_pub.publish(good)
        #     return
        # else:
        #     dummy = self.publish_msg_type()
        #     dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
        #     self.ring_pub.publish(dummy)
        #     return

        
        ###### NEW FLOODFILL ALGO ####
        pose, debug_frame = detect(image, camera_info_msg.k)

        if pose is not None:
            # We have a valid detection
            self.display_frame(debug_frame, self.node_name())
            dir_x, dir_z, dir_y = pose['x'], pose['y'], pose['z']
            dir_x = -dir_x
            nx, ny, nz = pose['nx'], pose['ny'], pose['nz']
            good = self.publish_msg_type()
            good.data = [0.0, 0.0, dir_x, dir_y, dir_z, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
            self.ring_pub.publish(good)
        else:
            # No hoop detected this frame
            print("not detecting anyyyything")
            self.display_frame(image, self.node_name())
            dummy = self.publish_msg_type()
            dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
            self.ring_pub.publish(dummy)


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
