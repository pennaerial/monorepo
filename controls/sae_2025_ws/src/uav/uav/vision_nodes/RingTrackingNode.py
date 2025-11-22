# payload_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_payload, compute_3d_vector
from uav.cv.hoopDetectorAlgorithm import find_nearest_hoop_pose
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
        if self.image is None or self.camera_info is None:
            print("RingTrackingNode: waiting for image/camera_info...")
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

        # # TODO: implement proper ring-detection; placeholder uses "pink" threshold
        # # this outputs cx, cy, and dlz_empty
        # detection = find_payload(
        #     image,
        #     *pink,                      # low HSV bound
        #     *self.color_map["pink"],   # high HSV bound
        #     self.uuid,
        #     self.debug,
        #     self.save_vision
        # )

        

        


####### OLD PAYLOADTRACKING NODE CODE ##############

        # dlz_empty = False
        # if detection is not None:
        #     cx, cy, dlz_empty = detection
        #     # Update Kalman filter with measurement
        #     # measurement = np.array([[np.float32(cx)], [np.float32(cy)]])
        #     # corrected_state = self.kalman.correct(measurement)
        #     # x, y = corrected_state[0, 0], corrected_state[1, 0]
        #     x, y = cx, cy
        # else:
        #     # # Use prediction if no detection
        #     # x, y = predicted_x, predicted_y
        #     x, y = image.shape[:2]
        #     x /= 2
        #     y /= 2
            
        

        #we should only need center_3d
        result = find_nearest_hoop_pose(image, camera_info_msg.k, 0.5)



        if len(result) == 2:
            self.display_frame(image, self.node_name())

            result_data, intermediate_frames = result
            result_data = None
            print("not detecting anyyyything")
            dummy = self.publish_msg_type()
            dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
            self.ring_pub.publish(dummy)
            return
        else:
            center_3d, normal_3d, ellipse, used_radius, annotated_frame = result
            self.display_frame(annotated_frame, self.node_name())

            # result_data = (center_3d, normal_3d, ellipse, used_radius)



        if center_3d is not None:
            dir_x, dir_z, dir_y = center_3d

            dir_x = -dir_x

            #need to flip direction of the z axis because in gazebo, up is negative z
            # dir_z = -dir_z
            good = self.publish_msg_type()
            good.data = [0.0, 0.0, dir_x, dir_y, dir_z, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
            self.ring_pub.publish(good)
            return
        else:
            dummy = self.publish_msg_type()
            dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
            self.ring_pub.publish(dummy)
            return

        

        # if detection is None:
        #     # Publish constant dummy vector so downstream nodes see a steady stream
        #     print("not detecting anyyyything")
        #     dummy = self.publish_msg_type()
        #     dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x,y,dir_x,dir_y,dir_z,flag
        #     self.ring_pub.publish(dummy)
        #     return

        # cx, cy, dlz_empty = detection

        # direction = compute_3d_vector(
        #     cx,
        #     cy,
        #     np.array(camera_info_msg.k).reshape(3, 3),
        #     0.0  # altitude not used for direction-only vector; adjust if needed
        # )

        # # Pack message: [x, y, dir_x, dir_y, dir_z, dlz_empty]
        # msg = self.publish_msg_type()
        # msg.data = [float(cx), float(cy)] + list(direction) + [1.0 if dlz_empty else 0.0]
        # self.ring_pub.publish(msg)

        # if self.display:
        #     self.display_frame(image, self.node_name()) 


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
