# hoop_detection_node.py
import cv2
import numpy as np
from uav.cv.hoop_detection import detect_hoops
from uav.cv.tracking import compute_3d_vector, rotate_image
from uav.vision_nodes import VisionNode
from uav_interfaces.srv import HoopDetection
from sensor_msgs.msg import Image
import rclpy

class HoopDetectionNode(VisionNode):
    """
    ROS node for detecting hoops (circles) in camera images.
    """
    srv = HoopDetection
    
    def __init__(self):
        super().__init__(self.__class__.srv, display=False, use_service=False)
        self.create_service(HoopDetection, self.service_name(), self.service_callback)
        
        # Publisher for visualization images (detailed visualization)
        self.vis_publisher = self.create_publisher(Image, '/vision/hoop_detection/visualization', 10)
        # Publisher for annotated camera feed (easier to find in rqt)
        self.annotated_camera_publisher = self.create_publisher(Image, '/camera/annotated', 10)
        # CvBridge will be imported conditionally based on sim mode
        self.bridge = None
        self.last_image = None  # Store last published image for republishing
        
        # Cache camera info for faster access
        self.cached_camera_info = None
        
        # Subscribe to camera topic directly to continuously publish annotated images
        # This ensures rqt always has something to display
        # Note: VisionNode already subscribes if use_service=False, but we add our own callback
        self.camera_subscription = self.create_subscription(
            Image,
            '/camera',
            self._camera_image_callback,
            10
        )
        
        # Create a timer to periodically republish the last image to keep rqt feed alive
        self.create_timer(0.1, self._republish_last_image)  # 10 Hz
        
        self.get_logger().info("HoopDetectionNode initialized")
        
    def service_callback(self, request: HoopDetection.Request, 
                        response: HoopDetection.Response):
        """Process hoop detection service request."""
        self.get_logger().info(f"Service callback called: alt={request.altitude:.2f}m, yaw={np.degrees(request.yaw):.1f}°")
        # Get camera image and info
        image, camera_info = self.request_data(cam_image=True, cam_info=True)
        self.get_logger().info(f"Got camera data: image={'yes' if image else 'no'}, camera_info={'yes' if camera_info else 'no'}")
        
        # Cache camera info if available
        if camera_info is not None:
            self.cached_camera_info = camera_info
        if image is None:
            response.hoop_detected = False
            # Try to publish last known image if available
            if hasattr(self, 'last_image') and self.last_image is not None:
                try:
                    self.annotated_camera_publisher.publish(self.last_image)
                except:
                    pass
            return response
        
        # Store image for visualization even if camera_info is missing
        if camera_info is None:
            self.get_logger().warn("No camera info available, but proceeding with image processing")
            # Still try to process image for visualization
            image_frame = self.convert_image_msg_to_frame(image)
            image_frame_rotated = rotate_image(image_frame, -np.rad2deg(request.yaw))
            
            # Publish raw image with "No camera info" message
            try:
                vis_image_rgb = cv2.cvtColor(image_frame_rotated, cv2.COLOR_BGR2RGB)
                cv2.putText(vis_image_rgb, "No camera info available",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                
                if not self.sim:
                    if self.bridge is None:
                        from cv_bridge import CvBridge
                        self.bridge = CvBridge()
                    vis_msg = self.bridge.cv2_to_imgmsg(vis_image_rgb, encoding='rgb8')
                else:
                    vis_msg = Image()
                    vis_msg.height = vis_image_rgb.shape[0]
                    vis_msg.width = vis_image_rgb.shape[1]
                    vis_msg.encoding = 'rgb8'
                    vis_msg.data = vis_image_rgb.tobytes()
                    vis_msg.step = vis_image_rgb.shape[1] * 3
                    vis_msg.header = image.header
                
                self.annotated_camera_publisher.publish(vis_msg)
                self.last_image = vis_msg
            except Exception as e:
                self.get_logger().warn(f"Failed to publish visualization without camera info: {e}")
            
            response.hoop_detected = False
            return response
        
        # Convert image to numpy array
        image_frame = self.convert_image_msg_to_frame(image)
        
        # Keep original for visualization (don't rotate for display)
        original_image = image_frame.copy()
        
        # Rotate image based on UAV yaw to compensate for rotation (only for detection)
        image_frame_rotated = rotate_image(image_frame, -np.rad2deg(request.yaw))
        
        # Detect hoops with visualization (on rotated image)
        self.get_logger().info("Calling detect_hoops()...")
        closest_center, all_centers, vis_image = detect_hoops(
            image_frame_rotated,
            debug=self.debug,
            save_vision=self.save_vision,
            uuid=self.uuid,
            detect_all=request.detect_all_hoops,
            return_visualization=True
        )
        self.get_logger().info(f"detect_hoops() returned: closest={closest_center}, total={len(all_centers)}")
        
        # For visualization, rotate the vis_image back to original orientation
        # OR use original image and convert coordinates
        if vis_image is not None:
            # Rotate vis_image back to original orientation (inverse of detection rotation)
            rotation_angle = -np.rad2deg(request.yaw)  # Original rotation
            inverse_angle = -rotation_angle  # Reverse it
            display_image = rotate_image(vis_image, inverse_angle)
        else:
            display_image = original_image.copy()
        
        # Convert detection coordinates back to original image space for direction vector
        if closest_center is not None:
            # Rotate detection coordinates back to original image coordinates
            # We rotated by -np.rad2deg(request.yaw), so reverse is +np.rad2deg(request.yaw)
            h, w = original_image.shape[:2]
            center = (w / 2, h / 2)
            # Get inverse rotation matrix (rotate by +np.rad2deg(request.yaw) to reverse)
            inverse_angle_deg = np.rad2deg(request.yaw)  # Positive angle to reverse negative rotation
            M_inv = cv2.getRotationMatrix2D(center, inverse_angle_deg, 1.0)
            # Transform point back using rotation matrix
            # cv2.getRotationMatrix2D returns a 2x3 matrix, so we need to handle the translation
            cx_rot, cy_rot = closest_center
            # Translate to origin, rotate, translate back
            x_translated = cx_rot - center[0]
            y_translated = cy_rot - center[1]
            # Apply rotation (inverse of the original rotation)
            cos_angle = np.cos(np.radians(inverse_angle_deg))
            sin_angle = np.sin(np.radians(inverse_angle_deg))
            x_rotated = x_translated * cos_angle - y_translated * sin_angle
            y_rotated = x_translated * sin_angle + y_translated * cos_angle
            # Translate back
            cx_orig = x_rotated + center[0]
            cy_orig = y_rotated + center[1]
            # Clamp to image bounds
            cx_orig = np.clip(cx_orig, 0, w - 1)
            cy_orig = np.clip(cy_orig, 0, h - 1)
            closest_center_orig = (int(cx_orig), int(cy_orig))
        else:
            closest_center_orig = None
        
        try:
            # Convert BGR to RGB for ROS
            vis_image_rgb = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            
            # Draw direction vector if hoop detected (using original coordinates)
            if closest_center_orig is not None:
                # Get camera intrinsic matrix
                K = np.array(camera_info.k).reshape(3, 3)
                
                # Compute 3D direction vector using original (unrotated) coordinates
                direction = compute_3d_vector(
                    closest_center_orig[0],
                    closest_center_orig[1],
                    K,
                    request.altitude
                )
                
                # Draw direction vector on image
                center = closest_center_orig
                h, w = vis_image_rgb.shape[:2]
                
                # Scale vector for visualization (scale by image size)
                scale = min(w, h) * 0.3
                end_point = (
                    int(center[0] + direction[0] * scale),
                    int(center[1] + direction[1] * scale)
                )
                
                # Draw arrow (yellow for direction vector)
                cv2.arrowedLine(vis_image_rgb, center, end_point, (255, 255, 0), 3, tipLength=0.3)
                
                # Add text showing direction
                cv2.putText(vis_image_rgb, f"Dir: [{direction[0]:.2f}, {direction[1]:.2f}, {direction[2]:.2f}]",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
                cv2.putText(vis_image_rgb, f"Alt: {request.altitude:.2f}m",
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            else:
                # No hoops detected - show status
                cv2.putText(vis_image_rgb, "No hoops detected",
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Convert to ROS Image message
            if not self.sim:
                if self.bridge is None:
                    from cv_bridge import CvBridge
                    self.bridge = CvBridge()
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image_rgb, encoding='rgb8')
            else:
                # For simulation, create Image message directly
                vis_msg = Image()
                vis_msg.height = vis_image_rgb.shape[0]
                vis_msg.width = vis_image_rgb.shape[1]
                vis_msg.encoding = 'rgb8'
                vis_msg.data = vis_image_rgb.tobytes()
                vis_msg.step = vis_image_rgb.shape[1] * 3
                vis_msg.header = image.header  # Copy header from original image
            
            if not self.sim:
                vis_msg.header = image.header  # Copy header from original image
            
            # Publish to both topics
            self.vis_publisher.publish(vis_msg)
            self.annotated_camera_publisher.publish(vis_msg)  # Also publish to /camera/annotated for easy rqt viewing
            self.last_image = vis_msg  # Store for potential republishing
        except Exception as e:
            self.get_logger().warn(f"Failed to publish visualization: {e}")
            # Try to publish at least the original image
            try:
                if image is not None:
                    self.annotated_camera_publisher.publish(image)
            except:
                pass
        
        if closest_center is None:
            response.hoop_detected = False
            self.get_logger().info("No hoop detected in image")
            return response
        
        # Convert detection coordinates back to original (unrotated) image coordinates
        # for the response (so coordinates match the visualization)
        # We rotated by -np.rad2deg(request.yaw), so reverse is +np.rad2deg(request.yaw)
        h, w = original_image.shape[:2]
        center = (w / 2, h / 2)
        # Get inverse rotation angle
        inverse_angle_deg = np.rad2deg(request.yaw)  # Positive angle to reverse negative rotation
        # Transform point back manually (more reliable than matrix multiplication)
        cx_rot, cy_rot = closest_center
        self.get_logger().info(f"Rotated image detection: ({cx_rot:.1f}, {cy_rot:.1f}), image_size=({w}, {h}), rotation={inverse_angle_deg:.1f}°")
        
        # Translate to origin, rotate, translate back
        x_translated = cx_rot - center[0]
        y_translated = cy_rot - center[1]
        # Apply rotation (inverse of the original rotation)
        cos_angle = np.cos(np.radians(inverse_angle_deg))
        sin_angle = np.sin(np.radians(inverse_angle_deg))
        x_rotated = x_translated * cos_angle - y_translated * sin_angle
        y_rotated = x_translated * sin_angle + y_translated * cos_angle
        # Translate back
        cx_orig = x_rotated + center[0]
        cy_orig = y_rotated + center[1]
        # Clamp to image bounds (ensure we're within valid pixel range)
        cx_orig_unclamped = cx_orig
        cy_orig_unclamped = cy_orig
        cx_orig = float(np.clip(cx_orig, 0, w - 1))
        cy_orig = float(np.clip(cy_orig, 0, h - 1))
        closest_center_orig = (cx_orig, cy_orig)
        
        # Log if coordinates were clamped (for debugging)
        if abs(cx_orig - cx_orig_unclamped) > 0.1 or abs(cy_orig - cy_orig_unclamped) > 0.1:
            self.get_logger().warn(f"Clamped coordinates: unclamped=({cx_orig_unclamped:.1f}, {cy_orig_unclamped:.1f}), "
                                  f"clamped=({cx_orig:.1f}, {cy_orig:.1f}), image_size=({w}, {h})")
        else:
            self.get_logger().info(f"Transformed coordinates: ({cx_orig:.1f}, {cy_orig:.1f})")
        
        # Get camera intrinsic matrix
        K = np.array(camera_info.k).reshape(3, 3)
        
        # Compute 3D direction vector using original (unrotated) coordinates
        direction = compute_3d_vector(
            closest_center_orig[0],
            closest_center_orig[1],
            K,
            request.altitude
        )
        
        # Populate response with original coordinates
        response.x = float(closest_center_orig[0])
        response.y = float(closest_center_orig[1])
        response.direction = list(direction)
        response.hoop_detected = True
        self.get_logger().info(f"HOOP DETECTED! Pixel: ({response.x:.1f}, {response.y:.1f}), direction={direction}")
        
        # If detect_all_hoops is True, populate all hoop coordinates
        if request.detect_all_hoops and all_centers:
            response.all_hoop_x = [float(center[0]) for center in all_centers]
            response.all_hoop_y = [float(center[1]) for center in all_centers]
        else:
            response.all_hoop_x = []
            response.all_hoop_y = []
        
        return response
    
    def _camera_image_callback(self, msg: Image):
        """Callback for camera images - publishes annotated images continuously."""
        try:
            # Convert and process image
            image_frame = self.convert_image_msg_to_frame(msg)
            
            # Create a simple annotated image (just show the raw feed for now)
            # When service is called, it will update with detections
            vis_image_rgb = cv2.cvtColor(image_frame, cv2.COLOR_BGR2RGB)
            
            # Add text indicating node is running
            cv2.putText(vis_image_rgb, "Hoop Detection Active",
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Convert to ROS Image message
            if not self.sim:
                if self.bridge is None:
                    from cv_bridge import CvBridge
                    self.bridge = CvBridge()
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image_rgb, encoding='rgb8')
            else:
                vis_msg = Image()
                vis_msg.height = vis_image_rgb.shape[0]
                vis_msg.width = vis_image_rgb.shape[1]
                vis_msg.encoding = 'rgb8'
                # Convert numpy array to list of uint8 values (0-255)
                vis_msg.data = vis_image_rgb.flatten().astype(np.uint8).tolist()
                vis_msg.step = vis_image_rgb.shape[1] * 3
                vis_msg.header = msg.header
            
            self.annotated_camera_publisher.publish(vis_msg)
            self.last_image = vis_msg
        except Exception as e:
            self.get_logger().debug(f"Failed to process camera image: {e}")
    
    def _republish_last_image(self):
        """Periodically republish the last image to keep rqt feed alive."""
        if self.last_image is not None:
            try:
                self.annotated_camera_publisher.publish(self.last_image)
            except Exception as e:
                self.get_logger().debug(f"Failed to republish image: {e}")

def main():
    rclpy.init()
    node = HoopDetectionNode()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(e)
        node.publish_failsafe()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

