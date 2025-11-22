# landing_pad_tracking_node.py
import cv2
import numpy as np
from uav.cv.tracking import find_landing_pad, compute_3d_vector
from uav.vision_nodes import VisionNode
from std_srvs.srv import Trigger
import rclpy
from std_msgs.msg import Float64MultiArray

class LandingPadTrackingNode(VisionNode):
    """
    ROS node for landing pad tracking
    """
    srv = Trigger

    @staticmethod
    def service_name() -> str:
        return "/vision/landing_pad_tracking_node"

    def __init__(self):
        super().__init__(custom_service=None)

        # Fake service to trick modemanager
        self.trigger_srv = self.create_service(
            self.__class__.srv,
            self.__class__.service_name(),
            self._handle_trigger,
        )

        self.publish_msg_type = Float64MultiArray
        self.pad_pub = self.create_publisher(self.publish_msg_type, '/landing_pad_tracking', 10)

        # Publish at ≥20 Hz (0.05 s)
        self.timer_period = 0.05  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Color parameter (can be set by mission or via ROS parameter)
        self.declare_parameter('color', 'red')
        self.declare_parameter('landing_pad_color', 'red')  # Alternative parameter name
        # Try to get color from landing_pad_color first, then color
        if self.has_parameter('landing_pad_color'):
            self.color = self.get_parameter('landing_pad_color').value
        else:
            self.color = self.get_parameter('color').value
        
        # Create a parameter callback to update color dynamically
        self.add_on_set_parameters_callback(self._parameter_callback)
    
    def _handle_trigger(self, request, response):
        """Fake service handler"""
        response.success = True
        response.message = f"LandingPadTrackingNode is running (color: {self.color})."
        return response
    
    def _parameter_callback(self, params):
        """Callback for parameter updates"""
        for param in params:
            if param.name == 'landing_pad_color' or param.name == 'color':
                self.color = param.value
                self.get_logger().info(f"Landing pad color updated to: {self.color}")
        return rclpy.node.SetParametersResult(successful=True)
        
    def timer_callback(self):
        """Runs at 20 Hz; performs detection and publishes results."""
        # Ensure we have at least one image and camera_info before proceeding
        if self.image is None:
            print("LandingPadTrackingNode: waiting for image...")
            return
        if self.camera_info is None:
            print("LandingPadTrackingNode: waiting for camera_info...")
            return

        # Acquire newest data
        image_msg, camera_info_msg = self.request_data(cam_image=True, cam_info=True)

        # Sanity checks
        if image_msg is None or len(image_msg.data) < 10:
            self.get_logger().debug("LandingPadTrackingNode: empty image_msg, skipping.")
            return
        if camera_info_msg is None or len(camera_info_msg.k) != 9:
            self.get_logger().debug("LandingPadTrackingNode: incomplete camera_info, skipping.")
            return

        image = self.convert_image_msg_to_frame(image_msg)
        
        # Detect landing pad
        result = find_landing_pad(image, self.color, debug=self.debug)
        
        if result is not None and result[0] is not None:
            cx, cy, area_ratio, debug_frame = result
            
            # Display debug frame if available
            if debug_frame is not None:
                self.display_frame(debug_frame, self.node_name())
            else:
                self.display_frame(image, self.node_name())
            
            # Get current altitude for 3D vector computation
            # We'll use area_ratio to estimate if we're close enough
            # For now, we'll compute direction vector assuming we're at a reasonable altitude
            # Note: altitude will be passed from the autonomous mode
            altitude = 5.0  # Default altitude, will be overridden by mode
            
            # Compute 3D direction vector
            try:
                camera_matrix = np.array(camera_info_msg.k).reshape(3, 3)
                dir_x, dir_y, dir_z = compute_3d_vector(
                    cx, cy, camera_matrix, altitude,
                    offset_x=0, offset_y=0, offset_z=0
                )
                
                # Adjust for camera orientation (similar to ring tracking)
                dir_x = -dir_x
                
                # Publish detection
                msg = self.publish_msg_type()
                msg.data = [float(cx), float(cy), float(dir_x), float(dir_y), float(dir_z), float(area_ratio), 1.0]  # x, y, dir_x, dir_y, dir_z, area_ratio, detected_flag
                self.pad_pub.publish(msg)
            except Exception as e:
                self.get_logger().error(f"Error computing 3D vector: {e}")
                # Publish dummy message
                dummy = self.publish_msg_type()
                dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                self.pad_pub.publish(dummy)
        else:
            # No landing pad detected
            self.display_frame(image, self.node_name())
            dummy = self.publish_msg_type()
            dummy.data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # x, y, dir_x, dir_y, dir_z, area_ratio, detected_flag
            self.pad_pub.publish(dummy)

def main():
    rclpy.init()
    node = LandingPadTrackingNode()
    try:    
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()

    rclpy.shutdown()

