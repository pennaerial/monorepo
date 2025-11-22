import numpy as np
from uav import UAV
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav_interfaces.srv import HoopDetection
from uav.vision_nodes import HoopDetectionNode
from typing import List, Tuple
import time
from sensor_msgs.msg import Image
import cv2

def pixel_to_local_coords(
    pixel_x: float,
    pixel_y: float,
    camera_matrix: np.ndarray,
    altitude: float,
    current_local_pos: Tuple[float, float, float],
    yaw: float,
    logger=None  # Optional logger for debug output
) -> Tuple[float, float, float]:
    """
    Convert pixel coordinates to 3D local NED coordinates for a FRONT-FACING camera.
    
    Args:
        pixel_x: Pixel x coordinate (horizontal, right is positive)
        pixel_y: Pixel y coordinate (vertical, down is positive)
        camera_matrix: 3x3 camera intrinsic matrix K
        altitude: Current altitude above ground (positive up)
        current_local_pos: Current UAV local position (x, y, z) in NED frame
        yaw: Current yaw angle in radians (0 = north)
    
    Returns:
        Tuple[float, float, float]: Target position in local NED coordinates
    """
    # Convert pixel to normalized camera coordinates (ray direction)
    pixel_coords = np.array([pixel_x, pixel_y, 1.0])
    cam_ray = np.linalg.inv(camera_matrix) @ pixel_coords
    cam_ray = cam_ray / np.linalg.norm(cam_ray)  # Normalize
    
    # For a front-facing camera, we need to estimate the distance to the hoop
    # The ray direction tells us where to look, but we need to intersect with a plane
    # Since we know altitude, we can estimate horizontal distance
    # If the ray is pointing forward (cam_ray[2] > 0), the hoop is ahead
    
    # Estimate distance to hoop based on pixel position and camera geometry
    # For a front-facing camera, if the hoop is centered, it's directly ahead
    # The pixel offset from center tells us the angle, and we can estimate distance
    
    # Get camera parameters from intrinsic matrix K
    # Camera matrix K format:
    # [fx  0  cx]   where fx, fy = focal lengths (in pixels)
    # [0  fy  cy]              cx, cy = principal point (image center)
    # [0   0   1]
    cx = camera_matrix[0, 2]  # Principal point x (image center x)
    cy = camera_matrix[1, 2]  # Principal point y (image center y)
    fx = camera_matrix[0, 0]  # Focal length x (horizontal) in pixels
    fy = camera_matrix[1, 1]  # Focal length y (vertical) in pixels
    
    # Average focal length (usually fx ≈ fy for most cameras)
    focal_length = (fx + fy) / 2.0
    
    # Calculate pixel offset from center
    dx = pixel_x - cx  # Horizontal offset (positive = right)
    dy = pixel_y - cy  # Vertical offset (positive = down)
    
    # Calculate angles
    angle_horizontal = np.arctan2(dx, fx)  # Angle from center horizontally
    angle_vertical = np.arctan2(dy, fy)    # Angle from center vertically
    
    # IMPROVED: Estimate distance using hoop size (if available)
    # Formula: distance = (real_hoop_diameter * focal_length) / (pixel_diameter)
    # For now, we'll use a combination of methods:
    
    # Method 1: Use pixel position from center (heuristic)
    pixel_distance_from_center = np.sqrt(dx**2 + dy**2)
    max_pixel_distance = np.sqrt(cx**2 + cy**2)  # Approximate max distance from center
    normalized_distance = min(pixel_distance_from_center / max_pixel_distance, 1.0)
    estimated_distance_heuristic = 2.0 + (normalized_distance * 3.0)  # 2-5 meters
    
    # Method 2: Use vertical angle (if significant)
    estimated_distance_angle = None
    if abs(angle_vertical) > 0.1:  # If looking significantly up/down
        # Distance based on vertical angle and altitude
        estimated_distance_angle = altitude / abs(np.tan(angle_vertical))
        estimated_distance_angle = np.clip(estimated_distance_angle, 1.0, 10.0)
    
    # Method 3: Use hoop size (if we had it from detection)
    # TODO: Get hoop radius from detection response
    # For now, assume typical hoop diameter is ~0.5m (50cm)
    # If we had pixel_diameter, we could calculate:
    # estimated_distance_size = (0.5 * focal_length) / pixel_diameter
    
    # Use the best available estimate
    if estimated_distance_angle is not None:
        # Prefer angle-based if available (more accurate)
        estimated_distance = estimated_distance_angle
    else:
        # Fall back to heuristic
        estimated_distance = estimated_distance_heuristic
    
    # Log focal length for debugging (first time only)
    if not hasattr(pixel_to_local_coords, '_focal_length_logged'):
        print(f"[DEBUG] Camera focal length: fx={fx:.1f}, fy={fy:.1f}, avg={focal_length:.1f} pixels")
        pixel_to_local_coords._focal_length_logged = True
    
    # Scale the ray by estimated distance to get 3D point in camera frame
    # Camera frame: x-right, y-down, z-forward
    point_camera = cam_ray * estimated_distance
    
    # Transform from camera frame to body frame (NED)
    # Camera frame (front-facing): x-right, y-down, z-forward
    # Body frame (NED): x-forward, y-right, z-down
    # 
    # DEBUGGING: If going to wrong side or wrong altitude, try these sign corrections:
    # - If going LEFT when hoop is on RIGHT: flip sign of camera x (change [1,0,0] to [-1,0,0])
    # - If going HIGH when hoop is at same altitude: flip sign of camera y (change [0,1,0] to [0,-1,0])
    R_cam_to_body = np.array([
        [0, 0, 1],   # camera z (forward) -> body x (forward)
        [1, 0, 0],   # camera x (right) -> body y (right) - FLIP TO [-1,0,0] if going wrong side
        [0, 1, 0]    # camera y (down) -> body z (down) - FLIP TO [0,-1,0] if going too high
    ])
    
    point_body = R_cam_to_body @ point_camera
    
    # DEBUG: Log intermediate values for debugging transformation issues
    if not hasattr(pixel_to_local_coords, '_debug_logged'):
        debug_msg = (f"[TRANSFORM DEBUG] cam_ray: {cam_ray}, "
                    f"point_camera: {point_camera}, point_body: {point_body}")
        if logger:
            logger.info(debug_msg)
        else:
            print(debug_msg)  # Fallback to print if no logger
        pixel_to_local_coords._debug_logged = True
    
    # Rotate by yaw to get to local NED frame
    cos_yaw = np.cos(yaw)
    sin_yaw = np.sin(yaw)
    R_yaw = np.array([
        [cos_yaw, -sin_yaw, 0],
        [sin_yaw, cos_yaw, 0],
        [0, 0, 1]
    ])
    
    point_local = R_yaw @ point_body
    
    # DEBUG: Log after yaw rotation
    if not hasattr(pixel_to_local_coords, '_debug_logged2'):
        debug_msg = (f"[TRANSFORM DEBUG] point_local (after yaw): {point_local}, "
                    f"yaw: {np.degrees(yaw):.1f}°")
        if logger:
            logger.info(debug_msg)
        else:
            print(debug_msg)  # Fallback to print if no logger
        pixel_to_local_coords._debug_logged2 = True
    
    # Add to current position
    # IMPORTANT: Keep the same altitude (z coordinate in NED is negative for up)
    # Hoops are at roughly the same altitude as the drone, so don't add z offset
    target_pos = (
        current_local_pos[0] + point_local[0],
        current_local_pos[1] + point_local[1],
        current_local_pos[2]  # Keep same altitude - hoops are at same height
    )
    
    # DEBUG: Log final target
    if not hasattr(pixel_to_local_coords, '_debug_logged3'):
        offset = (target_pos[0]-current_local_pos[0], 
                 target_pos[1]-current_local_pos[1], 
                 target_pos[2]-current_local_pos[2])
        debug_msg = (f"[TRANSFORM DEBUG] current_pos: {current_local_pos}, "
                    f"target_pos: {target_pos}, offset: ({offset[0]:.2f}, {offset[1]:.2f}, {offset[2]:.2f})")
        if logger:
            logger.info(debug_msg)
        else:
            print(debug_msg)  # Fallback to print if no logger
        pixel_to_local_coords._debug_logged3 = True
    
    return target_pos


class HoopWaypointCollectionMode(Mode):
    """
    A mode that detects hoops using computer vision and collects waypoints dynamically.
    Once waypoints are collected, transitions to NavGPSMode to navigate through them.
    """
    
    def __init__(self, node: Node, uav: UAV, 
                 waypoint_tolerance: float = 0.5,
                 min_hoops: int = 1,
                 max_hoops: int = 10,
                 collection_timeout: float = 60.0,
                 forward_speed: float = 1.0,
                 initial_altitude: float = 2.0):
        """
        Initialize the HoopWaypointCollectionMode.
        
        Args:
            node: ROS 2 node managing the UAV.
            uav: The UAV instance to control.
            waypoint_tolerance: Distance tolerance for considering a waypoint collected (meters).
            min_hoops: Minimum number of hoops to detect before transitioning.
            max_hoops: Maximum number of hoops to collect.
            collection_timeout: Maximum time to spend collecting waypoints (seconds).
            forward_speed: Speed to move forward while collecting waypoints (m/s).
            initial_altitude: Starting altitude before vision-based adjustment (meters, positive up).
        """
        super().__init__(node, uav)
        
        self.initial_altitude = initial_altitude
        self.detected_target_altitude = None  # Will be set by vision
        self.waypoint_tolerance = waypoint_tolerance
        self.min_hoops = min_hoops
        self.max_hoops = max_hoops
        self.collection_timeout = collection_timeout
        self.forward_speed = forward_speed
        
        # State variables
        self.waypoints = []  # List of waypoints: [(x, y, z), wait_time, 'LOCAL']
        self.collection_start_time = None
        self.last_hoop_detection_time = None
        self.done_collecting = False
        self.current_target = None
        self.hoops_detected = set()  # Track detected hoops to avoid duplicates
        
        # State machine for better waypoint calculation
        self.state = 'SEARCHING'  # SEARCHING, BACKING_UP, ANALYZING, READY
        self.backup_start_pos = None
        self.backup_distance = 1.5  # Move 1.5m backward (reduced from 2.5m - was too far)
        self.waypoint_offset = 0.3  # Place waypoint 0.3m past hoop center
        self.detected_hoop_pixel = None  # Store pixel coordinates when hoop detected
        self.detected_hoop_response = None  # Store full response
        
        # Publisher for waypoint visualization
        self.waypoint_vis_publisher = self.node.create_publisher(Image, '/vision/waypoint_collection/visualization', 10)
        self.bridge = None  # Will be initialized if needed
        
    def on_enter(self) -> None:
        """Called when mode is activated."""
        self.log("Starting hoop waypoint collection mode")
        self.collection_start_time = time.time()
        self.waypoints = []
        self.done_collecting = False
        self.hoops_detected = set()
        self.state = 'SEARCHING'
        self.backup_start_pos = None
        self.detected_hoop_pixel = None
        self.detected_hoop_response = None
        self.detected_target_altitude = None  # Reset - will be set by vision when hoop detected
        self.detection_request_time = None  # Track when we sent detection request
        self.detection_timeout = 1.0  # Wait up to 1 second for detection response
        self.sent_request = False  # Reset request tracking
        
    def on_update(self, time_delta: float) -> None:
        """Periodic logic for collecting waypoints from hoop detection."""
        current_time = time.time()
        
        # Check timeout
        if self.collection_start_time and \
           (current_time - self.collection_start_time) > self.collection_timeout:
            self.log(f"Collection timeout reached. Collected {len(self.waypoints)}/{self.min_hoops} waypoints.")
            self.done_collecting = True
            # Don't return here - let check_status handle whether to error or continue
            return
        
        # Get current position and altitude
        current_pos = self.uav.get_local_position()
        if current_pos is None:
            return
        
        altitude = -current_pos[2]  # Convert NED z (down) to altitude (up)
        yaw = self.uav.yaw
        
        # Maintain altitude (either initial or vision-detected)
        target_altitude = self.detected_target_altitude if self.detected_target_altitude is not None else self.initial_altitude
        
        if abs(altitude - target_altitude) > 0.2:
            target_alt = -target_altitude  # NED z is negative for up
            self.uav.publish_position_setpoint((current_pos[0], current_pos[1], target_alt))
            return
        
        # State machine for better waypoint calculation
        if self.state == 'SEARCHING':
            # Call vision service to detect hoops
            request = HoopDetection.Request()
            request.altitude = altitude
            request.yaw = float(yaw)
            request.detect_all_hoops = False  # Just get closest for now
            
            # Log when sending request (throttled)
            if not hasattr(self, '_last_request_log') or (current_time - self._last_request_log) > 2.0:
                self.log(f"Sending detection request (alt={altitude:.2f}m, yaw={np.degrees(yaw):.1f}°)")
                self._last_request_log = current_time
            
            response = self.send_request(HoopDetectionNode, request)
            
            # Track when we sent the request (first time only)
            if response is None and self.detection_request_time is None:
                self.detection_request_time = current_time
                self.log("Detection request sent, waiting for response...")
            
            # If request is in progress, wait for it (but with timeout)
            if response is None:
                # Check if we've waited too long
                if self.detection_request_time is not None:
                    wait_time = current_time - self.detection_request_time
                    if wait_time > self.detection_timeout:
                        # Timeout - reset and try again next cycle
                        self.log(f"Detection timeout after {wait_time:.2f}s, retrying...")
                        self.sent_request = False  # Reset to allow new request
                        self.detection_request_time = None
                        # Move forward slightly while retrying
                        forward_distance = self.forward_speed * time_delta * 0.5  # Slower while searching
                        target = (
                            current_pos[0] + forward_distance * np.cos(yaw),
                            current_pos[1] + forward_distance * np.sin(yaw),
                            current_pos[2]
                        )
                        self.uav.publish_position_setpoint(target)
                return
            
            # Got a response - reset timeout tracking
            self.detection_request_time = None
            self.log(f"Received detection response: hoop_detected={response.hoop_detected}")
            
            if not response.hoop_detected:
                # No hoop detected, move forward slowly
                if not hasattr(self, '_last_no_hoop_log') or (current_time - self._last_no_hoop_log) > 2.0:
                    self.log("No hoop detected, moving forward...")
                    self._last_no_hoop_log = current_time
                # No hoop detected, move forward slowly
                if self.current_target is None:
                    # Set forward target
                    forward_distance = self.forward_speed * time_delta
                    self.current_target = (
                        current_pos[0] + forward_distance * np.cos(yaw),
                        current_pos[1] + forward_distance * np.sin(yaw),
                        current_pos[2]
                    )
                else:
                    # Continue moving toward current target
                    dist = self.uav.distance_to_waypoint('LOCAL', self.current_target)
                    if dist > self.waypoint_tolerance:
                        self.uav.publish_position_setpoint(self.current_target)
                    else:
                        self.current_target = None  # Reset for next forward movement
                return
            
            # Hoop detected! Calculate target altitude from vision
            # Use vertical position of hoop in image to determine altitude adjustment
            self._calculate_target_altitude_from_vision(response, altitude, yaw)
            
            # Calculate waypoint immediately - no need to back up if detection is working
            self.log(f"Hoop detected! Target altitude: {self.detected_target_altitude:.2f}m (from vision)")
            self.log(f"Hoop at pixel: ({response.x:.1f}, {response.y:.1f})")
            
            # Calculate waypoint directly from detection
            pixel_x, pixel_y = response.x, response.y
            
            # Validate pixel coordinates (should be within reasonable image bounds)
            # Typical camera images are 640x480 or similar
            if pixel_x < 0 or pixel_x > 2000 or pixel_y < 0 or pixel_y > 2000:
                self.log(f"WARNING: Invalid pixel coordinates: ({pixel_x:.1f}, {pixel_y:.1f}), using fallback")
                # Use fallback calculation
                pixel_x = 320  # Default to image center
                pixel_y = 240
            
            # Get camera info for coordinate conversion
            # Try to get it from the camera service, but use direction vector as fallback
            from uav_interfaces.srv import CameraData
            if not hasattr(self, '_camera_client') or self._camera_client is None:
                self._camera_client = self.node.create_client(CameraData, '/camera_data')
            
            # Check if we have cached camera info
            camera_matrix = None
            if hasattr(self, '_cached_camera_matrix') and self._cached_camera_matrix is not None:
                camera_matrix = self._cached_camera_matrix
                self.log("Using cached camera info")
            else:
                # Try to get camera info from service (with multiple retries)
                if self._camera_client.wait_for_service(timeout_sec=0.5):
                    camera_data_req = CameraData.Request()
                    camera_data_req.cam_info = True
                    camera_data_req.cam_image = False
                    
                    # Try multiple times with shorter timeouts
                    import rclpy
                    for attempt in range(3):
                        future = self._camera_client.call_async(camera_data_req)
                        rclpy.spin_until_future_complete(self.node, future, timeout_sec=0.5)
                        
                        if future.done():
                            try:
                                camera_data_resp = future.result()
                                if camera_data_resp and hasattr(camera_data_resp, 'camera_info') and camera_data_resp.camera_info is not None:
                                    camera_matrix = np.array(camera_data_resp.camera_info.k).reshape(3, 3)
                                    # Cache it for future use
                                    self._cached_camera_matrix = camera_matrix
                                    self.log(f"Successfully retrieved and cached camera info (attempt {attempt + 1})")
                                    break
                            except Exception as e:
                                self.log(f"Failed to get camera info result (attempt {attempt + 1}): {e}")
                        else:
                            if attempt < 2:
                                self.log(f"Camera info request timed out (attempt {attempt + 1}), retrying...")
                            else:
                                self.log("Camera info request timed out after 3 attempts")
                else:
                    self.log("Camera service not available")
            
            if camera_matrix is not None:
                # Use detected target altitude if available, otherwise use current altitude
                target_altitude_for_calc = self.detected_target_altitude if self.detected_target_altitude is not None else altitude
                
                # Calculate target position using pixel coordinates
                target_pos = pixel_to_local_coords(
                    pixel_x, pixel_y,
                    camera_matrix,
                    target_altitude_for_calc,  # Use vision-calculated altitude
                    current_pos,
                    yaw,
                    logger=self.node.get_logger()  # Pass logger for debug output
                )
                
                # Add waypoint offset past the hoop center
                # Calculate direction to hoop and extend it
                direction_to_hoop = np.array([
                    target_pos[0] - current_pos[0],
                    target_pos[1] - current_pos[1],
                    0  # No vertical component
                ])
                direction_to_hoop_norm = direction_to_hoop / (np.linalg.norm(direction_to_hoop) + 1e-6)
                
                # Use detected target altitude for waypoint (convert to NED: positive altitude = negative z)
                waypoint_altitude = -target_altitude_for_calc if self.detected_target_altitude is not None else target_pos[2]
                
                # Add conditional vertical offset based on hoop position
                # Get hoop pixel y to determine if it's above or below center
                hoop_pixel_y = response.y
                # Get actual image center from camera matrix if available
                cy = camera_matrix[1, 2]  # Image center y from camera matrix
                vertical_offset_pixels = hoop_pixel_y - cy
                
                if vertical_offset_pixels > -50:  # Hoop is at center or below
                    # Add small downward offset to go through center, not hit top edge
                    vertical_offset = 0.1  # meters down (positive in NED = down)
                else:  # Hoop is significantly above center
                    # Don't add downward offset, or add a very small one
                    vertical_offset = 0.02  # Very small downward offset
                
                waypoint_altitude = waypoint_altitude + vertical_offset
                
                # Place waypoint offset past the hoop center
                final_waypoint = (
                    target_pos[0] + direction_to_hoop_norm[0] * self.waypoint_offset,
                    target_pos[1] + direction_to_hoop_norm[1] * self.waypoint_offset,
                    waypoint_altitude  # Use vision-calculated altitude with small downward offset
                )
                
                # Calculate distance to waypoint for logging
                waypoint_distance = np.sqrt(
                    (final_waypoint[0] - current_pos[0])**2 +
                    (final_waypoint[1] - current_pos[1])**2 +
                    (final_waypoint[2] - current_pos[2])**2
                )
                self.log(f"Calculated waypoint: {final_waypoint}, distance={waypoint_distance:.2f}m")
                # Waypoint format: (position_tuple, wait_time, frame)
                waypoint = (final_waypoint, 2.0, 'LOCAL')
                self.waypoints.append(waypoint)
                self.hoops_detected.add((pixel_x, pixel_y))
                self.log(f"Collected {len(self.waypoints)}/{self.min_hoops} waypoint(s)")
                
                # Check if we have enough waypoints
                if len(self.waypoints) >= self.min_hoops:
                    self.log(f"Collected {len(self.waypoints)} waypoint(s). Ready to navigate.")
                    self.done_collecting = True
                else:
                    # Reset to search for more hoops
                    self.state = 'SEARCHING'
                    self.detected_hoop_pixel = None
                    self.detected_hoop_response = None
            else:
                self.log("Failed to get camera info, using pixel-based heuristic")
                # Fall back to simple pixel-based heuristic
                # Use pixel position to estimate waypoint
                # Try to get image dimensions from cached camera info or use default
                # Default: 1280x960 (common camera resolution)
                image_width = 1280  # Default - update if we have camera info
                image_height = 960  # Default - update if we have camera info
                
                # Try to get dimensions from cached camera info if available
                if hasattr(self, '_cached_camera_matrix') and self._cached_camera_matrix is not None:
                    # Can't get width/height from matrix, but we know it's likely 1280x960
                    # The principal point (cx, cy) is usually at image center
                    cx = self._cached_camera_matrix[0, 2]
                    cy = self._cached_camera_matrix[1, 2]
                    # Estimate image size from principal point (usually at center)
                    image_width = int(cx * 2) if cx > 0 else 1280
                    image_height = int(cy * 2) if cy > 0 else 960
                    self.log(f"Estimated image size from camera matrix: {image_width}x{image_height}")
                
                image_center_x = image_width / 2
                image_center_y = image_height / 2
                
                self.log(f"[FALLBACK DEBUG] Using image size: {image_width}x{image_height}, center: ({image_center_x}, {image_center_y})")
                self.log(f"[FALLBACK DEBUG] Hoop pixel: ({pixel_x:.1f}, {pixel_y:.1f})")
                
                # Calculate pixel offset from center
                dx_pixel = pixel_x - image_center_x  # Positive = right
                dy_pixel = pixel_y - image_center_y  # Positive = down
                
                self.log(f"[FALLBACK DEBUG] Pixel offset from center: dx={dx_pixel:.1f}, dy={dy_pixel:.1f}")
                
                # Estimate angles (rough approximation: ~60 deg FOV = ~0.52 rad per half-width pixels)
                fov_rad = np.radians(60)  # Assume 60 degree FOV
                angle_horizontal = (dx_pixel / image_center_x) * (fov_rad / 2)
                angle_vertical = (dy_pixel / image_center_y) * (fov_rad / 2)
                
                self.log(f"[FALLBACK DEBUG] Calculated angles: horizontal={np.degrees(angle_horizontal):.2f}°, vertical={np.degrees(angle_vertical):.2f}°")
                
                # Estimate distance (very conservative: 0.5-1.0m range)
                # If hoop is centered, it's close (~0.5m)
                # If hoop is far from center, it's further away
                pixel_dist_from_center = np.sqrt(dx_pixel**2 + dy_pixel**2)
                max_pixel_dist = np.sqrt(image_center_x**2 + image_center_y**2)
                normalized_dist = min(pixel_dist_from_center / max_pixel_dist, 1.0)
                estimated_distance = 0.5 + normalized_dist * 0.5  # 0.5-1.0m range (very close)
                
                # Calculate target position in body frame (front-facing camera)
                # Camera: x-right, y-down, z-forward
                # For small angles: forward distance ≈ estimated_distance
                forward_dist = estimated_distance * np.cos(angle_horizontal) * np.cos(angle_vertical)
                right_dist = estimated_distance * np.sin(angle_horizontal)
                down_dist = estimated_distance * np.sin(angle_vertical)
                
                # Transform to local NED frame
                cos_yaw = np.cos(yaw)
                sin_yaw = np.sin(yaw)
                # Body frame: x-forward, y-right, z-down
                # Local NED: x-north, y-east, z-down
                local_north = forward_dist * cos_yaw - right_dist * sin_yaw
                local_east = forward_dist * sin_yaw + right_dist * cos_yaw
                
                self.log(f"[FALLBACK DEBUG] Body frame: forward={forward_dist:.3f}m, right={right_dist:.3f}m, down={down_dist:.3f}m")
                self.log(f"[FALLBACK DEBUG] Local NED offset: north={local_north:.3f}m, east={local_east:.3f}m, yaw={np.degrees(yaw):.1f}°")
                
                # Use detected target altitude if available, otherwise use current altitude
                target_altitude_for_calc = self.detected_target_altitude if self.detected_target_altitude is not None else altitude
                waypoint_altitude = -target_altitude_for_calc if self.detected_target_altitude is not None else current_pos[2]
                
                # Add conditional vertical offset based on hoop position
                # Get hoop pixel y to determine if it's above or below center
                hoop_pixel_y = pixel_y
                # Assume typical image height (960) for fallback calculation
                assumed_center_y = 480
                vertical_offset_pixels = hoop_pixel_y - assumed_center_y
                
                if vertical_offset_pixels > -50:  # Hoop is at center or below
                    # Add small downward offset to go through center, not hit top edge
                    vertical_offset = 0.1  # meters down (positive in NED = down)
                else:  # Hoop is significantly above center
                    # Don't add downward offset, or add a very small one
                    vertical_offset = 0.02  # Very small downward offset
                
                waypoint_altitude = waypoint_altitude + vertical_offset
                
                # Place waypoint at estimated position, then add offset past hoop
                target_pos = (
                    current_pos[0] + local_north,
                    current_pos[1] + local_east,
                    waypoint_altitude  # Use vision-calculated altitude with small downward offset
                )
                
                self.log(f"[FALLBACK DEBUG] Target position (before offset): {target_pos}")
                self.log(f"[FALLBACK DEBUG] Current position: {current_pos}")
                
                # Add waypoint offset past the hoop center
                # Calculate direction to hoop and extend it
                direction_to_hoop = np.array([
                    target_pos[0] - current_pos[0],
                    target_pos[1] - current_pos[1],
                    0  # No vertical component
                ])
                direction_to_hoop_norm = direction_to_hoop / (np.linalg.norm(direction_to_hoop) + 1e-6)
                
                # Place waypoint offset past the hoop center
                final_waypoint = (
                    target_pos[0] + direction_to_hoop_norm[0] * self.waypoint_offset,
                    target_pos[1] + direction_to_hoop_norm[1] * self.waypoint_offset,
                    waypoint_altitude  # Use vision-calculated altitude
                )
                
                # Calculate distance to waypoint for logging
                waypoint_distance = np.sqrt(
                    (final_waypoint[0] - current_pos[0])**2 +
                    (final_waypoint[1] - current_pos[1])**2 +
                    (final_waypoint[2] - current_pos[2])**2
                )
                
                offset_from_current = (
                    final_waypoint[0] - current_pos[0],
                    final_waypoint[1] - current_pos[1],
                    final_waypoint[2] - current_pos[2]
                )
                self.log(f"[FALLBACK DEBUG] Final waypoint: {final_waypoint}")
                self.log(f"[FALLBACK DEBUG] Offset from current: north={offset_from_current[0]:.3f}m, east={offset_from_current[1]:.3f}m, down={offset_from_current[2]:.3f}m")
                self.log(f"Calculated waypoint (fallback): {final_waypoint}, distance={waypoint_distance:.2f}m")
                # Waypoint format: (position_tuple, wait_time, frame)
                waypoint = (final_waypoint, 2.0, 'LOCAL')
                self.waypoints.append(waypoint)
                self.log(f"Collected {len(self.waypoints)}/{self.min_hoops} waypoint(s)")
                
                # Check if we have enough waypoints
                if len(self.waypoints) >= self.min_hoops:
                    self.log(f"Collected {len(self.waypoints)} waypoint(s). Ready to navigate.")
                    self.done_collecting = True
                else:
                    # Reset to search for more hoops
                    self.state = 'SEARCHING'
                    self.detected_hoop_pixel = None
                    self.detected_hoop_response = None
            return
        
        elif self.state == 'BACKING_UP':
            # Move backward to get better viewing angle
            # Calculate distance already backed up from start position
            if self.backup_start_pos is None:
                self.backup_start_pos = current_pos
                self.log(f"Starting backup from position {current_pos[:2]}")
            
            # Calculate how far we've moved backward
            dx = current_pos[0] - self.backup_start_pos[0]
            dy = current_pos[1] - self.backup_start_pos[1]
            # Project onto backward direction (opposite of yaw)
            dist_backed_up = -(dx * np.cos(yaw) + dy * np.sin(yaw))
            
            if dist_backed_up < self.backup_distance:
                # Still need to back up more - move backward in small increments
                backup_increment = min(0.3, self.backup_distance - dist_backed_up)  # Max 0.3m per step
                backup_target = (
                    current_pos[0] - backup_increment * np.cos(yaw),
                    current_pos[1] - backup_increment * np.sin(yaw),
                    current_pos[2]  # Keep same altitude
                )
                self.uav.publish_position_setpoint(backup_target)
                return
            else:
                # Reached backup position, now analyze
                self.log(f"Backup complete ({dist_backed_up:.2f}m backed up). Analyzing hoop angle and calculating waypoint...")
                self.state = 'ANALYZING'
                return
        
        elif self.state == 'ANALYZING':
            # Now calculate waypoint with better angle analysis
            # We're further back, so we have better perspective
            pass  # Will continue to waypoint calculation below
        
        # Calculate waypoint (either from new detection or stored detection after backup)
        if self.state == 'ANALYZING':
            # Use stored detection from before backup
            response = self.detected_hoop_response
            pixel_x, pixel_y = self.detected_hoop_pixel
            # Re-detect to get fresh angle from backup position
            request = HoopDetection.Request()
            request.altitude = altitude
            request.yaw = float(yaw)
            request.detect_all_hoops = False
            fresh_response = self.send_request(HoopDetectionNode, request)
            if fresh_response and fresh_response.hoop_detected:
                # Use fresh detection from backup position (better angle)
                response = fresh_response
                pixel_x, pixel_y = response.x, response.y
        else:
            # Should not reach here in ANALYZING state, but just in case
            return
        
        # Convert pixel to local coordinates with improved calculation
        try:
            # Request camera info to get camera matrix for proper conversion
            from uav_interfaces.srv import CameraData
            from rclpy.node import Node as RclpyNode
            
            # Get camera info from the vision service
            camera_data_req = CameraData.Request()
            camera_data_req.cam_info = True
            camera_data_req.cam_image = False
            
            # Try to get camera info from the camera node
            # Create client if it doesn't exist
            if not hasattr(self, '_camera_client') or self._camera_client is None:
                self._camera_client = self.node.create_client(CameraData, '/camera_data')
            
            if self._camera_client.wait_for_service(timeout_sec=0.5):
                future = self._camera_client.call_async(camera_data_req)
                import rclpy
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
                try:
                    camera_data_resp = future.result()
                    if camera_data_resp and hasattr(camera_data_resp, 'camera_info') and camera_data_resp.camera_info is not None:
                        K = np.array(camera_data_resp.camera_info.k).reshape(3, 3)
                        # Use proper pixel-to-3D conversion
                        target_pos = pixel_to_local_coords(
                            pixel_x,  # pixel x
                            pixel_y,  # pixel y
                            K,
                            altitude,
                            current_pos,
                            yaw,
                            logger=self.node.get_logger()  # Pass logger for debug output
                        )
                        
                        # IMPROVEMENT: Place waypoint PAST the hoop, not at center
                        # Calculate direction to hoop and extend it
                        direction_to_hoop = np.array([
                            target_pos[0] - current_pos[0],
                            target_pos[1] - current_pos[1],
                            0  # No vertical component
                        ])
                        direction_to_hoop_norm = direction_to_hoop / (np.linalg.norm(direction_to_hoop) + 1e-6)
                        
                        # Place waypoint 1.5m past the hoop center
                        waypoint_offset = 1.5  # meters past hoop
                        final_waypoint = (
                            target_pos[0] + direction_to_hoop_norm[0] * waypoint_offset,
                            target_pos[1] + direction_to_hoop_norm[1] * waypoint_offset,
                            current_pos[2]  # Same altitude
                        )
                        target_pos = final_waypoint
                    else:
                        raise Exception("No camera info in response")
                except Exception as e:
                    self.log(f"Failed to get camera info: {e}, using direction vector fallback")
                    raise
            else:
                raise Exception("Camera service not available")
                
        except Exception as e:
            # Fallback: use direction vector with proper scaling and transformation
            self.log(f"Using direction vector fallback: {e}")
            direction = np.array(response.direction)
            
            # The direction vector is normalized and in camera frame
            # For a front-facing camera: x-right, y-down, z-forward (in camera frame)
            # Estimate distance: use a reasonable default (2-4 meters ahead)
            # This is a simple fallback - proper conversion needs camera matrix
            estimated_distance = 3.0  # Default to 3 meters ahead
            
            # Scale direction by estimated distance
            target_offset_camera = direction * estimated_distance
            
            # Transform from camera frame to body frame
            # Camera frame (front-facing): x-right, y-down, z-forward
            # Body frame (NED): x-forward, y-right, z-down
            R_cam_to_body = np.array([
                [0, 0, 1],   # camera z (forward) -> body x (forward)
                [1, 0, 0],   # camera x (right) -> body y (right)
                [0, 1, 0]    # camera y (down) -> body z (down)
            ])
            target_offset_body = R_cam_to_body @ target_offset_camera
            
            # Transform from body frame to local NED frame using yaw
            cos_yaw = np.cos(yaw)
            sin_yaw = np.sin(yaw)
            # Body frame: x-forward, y-right, z-down
            # Local NED: x-north, y-east, z-down
            local_offset = np.array([
                target_offset_body[0] * cos_yaw - target_offset_body[1] * sin_yaw,  # north
                target_offset_body[0] * sin_yaw + target_offset_body[1] * cos_yaw,  # east
                0.0  # Keep same altitude - don't add z offset
            ])
            
            target_pos = (
                current_pos[0] + local_offset[0],
                current_pos[1] + local_offset[1],
                current_pos[2]  # Keep same altitude
            )
            
            # Check if this hoop is new (not too close to existing waypoints)
            is_new_hoop = True
            for wp in self.waypoints:
                wp_pos = wp[0]
                dist = np.sqrt(
                    (target_pos[0] - wp_pos[0])**2 +
                    (target_pos[1] - wp_pos[1])**2 +
                    (target_pos[2] - wp_pos[2])**2
                )
                if dist < self.waypoint_tolerance * 2:  # Too close to existing waypoint
                    is_new_hoop = False
                    break
            
            if is_new_hoop:
                # Add new waypoint
                waypoint = (target_pos, 2.0, 'LOCAL')  # 2 second wait at each hoop
                self.waypoints.append(waypoint)
                self.last_hoop_detection_time = current_time
                self.log(f"Calculated waypoint {len(self.waypoints)} at {target_pos} (1.5m past hoop center)")
                
                # Publish waypoint visualization
                self._publish_waypoint_visualization(response, current_pos, target_pos)
                
                # Reset state for next hoop
                self.state = 'SEARCHING'
                self.detected_hoop_pixel = None
                self.detected_hoop_response = None
                
                # Check if we have enough waypoints
                if len(self.waypoints) >= self.min_hoops:
                    if len(self.waypoints) >= self.max_hoops:
                        self.log(f"Collected {self.max_hoops} waypoint(s). Ready to navigate.")
                        self.done_collecting = True
                    # Could also check if no new hoops detected for a while
                    elif self.last_hoop_detection_time and \
                         (current_time - self.last_hoop_detection_time) > 5.0:
                        self.log(f"No new hoops for 5 seconds. Collected {len(self.waypoints)} waypoints.")
                        self.done_collecting = True
            
        except Exception as e:
            self.log(f"Error processing hoop detection: {e}")
    
    def check_status(self) -> str:
        """Check if waypoint collection is complete."""
        if self.done_collecting:
            if len(self.waypoints) >= self.min_hoops:
                return "complete"
            else:
                # Only return error if we've timed out AND don't have enough hoops
                # Otherwise, continue collecting
                current_time = time.time()
                if self.collection_start_time and \
                   (current_time - self.collection_start_time) > self.collection_timeout:
                    self.log(f"Timeout reached with only {len(self.waypoints)}/{self.min_hoops} hoops. Error.")
                    return "error"
                # Still within timeout, keep collecting
                self.done_collecting = False
                return "continue"
        return "continue"
    
    def _calculate_target_altitude_from_vision(self, response, current_altitude, yaw):
        """
        Calculate target altitude based on hoop's vertical position in image.
        
        If hoop center is above image center → hoop is above us → need to go up
        If hoop center is below image center → hoop is below us → need to go down
        If hoop center is at image center → we're at correct altitude
        """
        try:
            # Get camera info to determine image center
            from uav_interfaces.srv import CameraData
            if not hasattr(self, '_camera_client') or self._camera_client is None:
                self._camera_client = self.node.create_client(CameraData, '/camera_data')
            
            if self._camera_client.wait_for_service(timeout_sec=0.5):
                camera_data_req = CameraData.Request()
                camera_data_req.cam_info = True
                camera_data_req.cam_image = False
                
                future = self._camera_client.call_async(camera_data_req)
                import rclpy
                rclpy.spin_until_future_complete(self.node, future, timeout_sec=1.0)
                
                try:
                    camera_data_resp = future.result()
                    if camera_data_resp and hasattr(camera_data_resp, 'camera_info') and camera_data_resp.camera_info is not None:
                        K = np.array(camera_data_resp.camera_info.k).reshape(3, 3)
                        cy = K[1, 2]  # Image center y (vertical center)
                        
                        # Hoop pixel y coordinate
                        hoop_pixel_y = response.y
                        
                        # Calculate vertical offset from center
                        vertical_offset = hoop_pixel_y - cy  # Positive = below center, Negative = above center
                        
                        # Calculate angle to hoop center
                        fy = K[1, 1]  # Vertical focal length
                        vertical_angle = np.arctan2(vertical_offset, fy)
                        
                        # Estimate distance to hoop using a more conservative approach
                        # Use pixel distance from center as a proxy for distance
                        # If hoop is centered, it's close (~1.5m)
                        # If hoop is far from center, it's further away
                        pixel_dist_from_center = abs(vertical_offset)
                        max_pixel_dist = cy  # Maximum distance from center (half image height)
                        normalized_dist = min(pixel_dist_from_center / max_pixel_dist, 1.0)
                        estimated_distance = 1.5 + normalized_dist * 1.5  # 1.5-3.0m range
                        
                        # Calculate altitude adjustment needed
                        # If hoop is below center (positive offset), we need to go down
                        # If hoop is above center (negative offset), we need to go up
                        altitude_adjustment = estimated_distance * np.tan(vertical_angle)
                        
                        # Add small downward bias only if hoop is near center or below
                        # If hoop is significantly above center, don't add downward bias
                        if vertical_offset > -50:  # Hoop is at center or below
                            altitude_adjustment = altitude_adjustment - 0.05  # Small downward bias
                        else:  # Hoop is significantly above center
                            # Don't add downward bias, or add a small upward bias
                            altitude_adjustment = altitude_adjustment + 0.02  # Small upward bias
                        
                        # Set target altitude
                        self.detected_target_altitude = current_altitude + altitude_adjustment
                        
                        # Clamp to reasonable range (0.5m to 5m)
                        self.detected_target_altitude = np.clip(self.detected_target_altitude, 0.5, 5.0)
                        
                        self.log(f"Vision-based altitude: hoop at pixel_y={hoop_pixel_y:.1f}, center_y={cy:.1f}, "
                                f"offset={vertical_offset:.1f}, angle={np.degrees(vertical_angle):.1f}°, "
                                f"distance={estimated_distance:.2f}m, adjustment={altitude_adjustment:.2f}m, target={self.detected_target_altitude:.2f}m")
                        return
                except Exception as e:
                    self.log(f"Failed to get camera info for altitude calculation: {e}")
            
            # Fallback: use pixel position as heuristic when camera info unavailable
            # Assume typical image size (1280x960) and estimate based on pixel position
            hoop_pixel_y = response.y
            assumed_image_height = 960  # Typical image height
            assumed_center_y = assumed_image_height / 2  # 480
            
            # Calculate vertical offset from assumed center
            vertical_offset = hoop_pixel_y - assumed_center_y  # Positive = below center, Negative = above center
            
            # Estimate altitude adjustment using simple heuristic
            # Assume ~60 degree vertical FOV, so each pixel represents ~60/960 = 0.0625 degrees
            # For a distance of ~2m, vertical adjustment ≈ distance * tan(angle)
            estimated_distance = 2.0  # Conservative estimate
            pixels_per_degree = assumed_image_height / 60.0  # ~16 pixels per degree
            vertical_angle_deg = vertical_offset / pixels_per_degree
            vertical_angle_rad = np.radians(vertical_angle_deg)
            altitude_adjustment = estimated_distance * np.tan(vertical_angle_rad)
            
            # Add conditional bias based on hoop position
            if vertical_offset > -50:  # Hoop is at center or below
                altitude_adjustment = altitude_adjustment - 0.05  # Small downward bias
            else:  # Hoop is significantly above center
                altitude_adjustment = altitude_adjustment + 0.02  # Small upward bias
            
            self.detected_target_altitude = current_altitude + altitude_adjustment
            self.detected_target_altitude = np.clip(self.detected_target_altitude, 0.5, 5.0)
            
            self.log(f"Fallback altitude calculation: hoop at pixel_y={hoop_pixel_y:.1f}, assumed_center_y={assumed_center_y:.1f}, "
                    f"offset={vertical_offset:.1f}, adjustment={altitude_adjustment:.2f}m, target={self.detected_target_altitude:.2f}m")
        except Exception as e:
            self.log(f"Error calculating target altitude from vision: {e}")
            self.detected_target_altitude = current_altitude
    
    def _publish_waypoint_visualization(self, response, current_pos, target_pos):
        """Publish visualization showing collected waypoints and current target."""
        try:
            # Get latest camera image
            from sensor_msgs.msg import Image as ImageMsg
            from uav_interfaces.srv import CameraData
            
            # Create a simple visualization image (you could also get actual camera image)
            # For now, create a text-based visualization
            vis_img = np.zeros((480, 640, 3), dtype=np.uint8)
            
            # Draw collected waypoints
            for idx, wp in enumerate(self.waypoints):
                wp_pos = wp[0]
                # Convert 3D position to 2D visualization (simplified)
                x = int(320 + (wp_pos[1] - current_pos[1]) * 50)  # Scale for visualization
                y = int(240 - (wp_pos[0] - current_pos[0]) * 50)
                
                if 0 <= x < 640 and 0 <= y < 480:
                    color = (0, 255, 0) if idx == len(self.waypoints) - 1 else (255, 165, 0)
                    cv2.circle(vis_img, (x, y), 10, color, -1)
                    cv2.putText(vis_img, f"WP{idx+1}", (x+15, y), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
            
            # Draw current position
            cv2.circle(vis_img, (320, 240), 8, (0, 0, 255), -1)
            cv2.putText(vis_img, "UAV", (330, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
            
            # Draw target
            if target_pos:
                tx = int(320 + (target_pos[1] - current_pos[1]) * 50)
                ty = int(240 - (target_pos[0] - current_pos[0]) * 50)
                if 0 <= tx < 640 and 0 <= ty < 480:
                    cv2.circle(vis_img, (tx, ty), 12, (255, 255, 0), 2)
                    cv2.line(vis_img, (320, 240), (tx, ty), (255, 255, 0), 2)
            
            # Add text info
            cv2.putText(vis_img, f"Waypoints: {len(self.waypoints)}", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(vis_img, f"Target: {target_pos}", (10, 60),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Convert to ROS message
            vis_img_rgb = cv2.cvtColor(vis_img, cv2.COLOR_BGR2RGB)
            vis_msg = ImageMsg()
            vis_msg.height = vis_img_rgb.shape[0]
            vis_msg.width = vis_img_rgb.shape[1]
            vis_msg.encoding = 'rgb8'
            vis_msg.data = vis_img_rgb.tobytes()
            vis_msg.step = vis_img_rgb.shape[1] * 3
            from std_msgs.msg import Header
            from rclpy.clock import Clock
            vis_msg.header = Header()
            vis_msg.header.stamp = self.node.get_clock().now().to_msg()
            vis_msg.header.frame_id = 'camera'
            
            self.waypoint_vis_publisher.publish(vis_msg)
        except Exception as e:
            self.log(f"Failed to publish waypoint visualization: {e}")
    
    def get_collected_waypoints(self) -> List[Tuple[Tuple[float, float, float], float, str]]:
        """Get the collected waypoints in NavGPSMode format."""
        return self.waypoints.copy()
    
    def on_exit(self) -> None:
        """Called when mode is deactivated. Pass waypoints to next mode if available."""
        if self.waypoints and hasattr(self.node, 'modes'):
            # Store waypoints in ModeManager for next mode to access
            # This is a workaround - ideally ModeManager would handle this
            self.node.collected_waypoints = self.waypoints.copy()
            self.log(f"Stored {len(self.waypoints)} waypoints for next mode")

