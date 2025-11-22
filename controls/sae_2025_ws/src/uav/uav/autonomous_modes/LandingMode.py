import numpy as np
from uav.autonomous_modes import Mode
from rclpy.node import Node
from uav import UAV
from px4_msgs.msg import VehicleStatus
from sensor_msgs.msg import Image
from rectangle_detect import ColorRectangleDetector


class LandingMode(Mode):
    """
    A mode for precision landing on a colored rectangle target.
    """

    def __init__(self, node: Node, uav: UAV):
        """
        Initialize the LandingMode

        Args:
            node (Node): ROS 2 node managing the UAV.
            uav (UAV): The UAV instance to control.
        """
        super().__init__(node, uav)
        
        # Initialize color detector with colored target
        self.detector = ColorRectangleDetector(
            target_color=(255, 255, 0),  # currently set to yellow
            threshold_value=150,
            min_area=1000,
            max_area=10000,
            blur_iterations=4
        )
        
        # CV Bridge for ROS image conversion
        self.bridge = CvBridge()
        
        # Subscribe to camera topic
        self.image_sub = self.node.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        
        # State variables
        self.current_image = None
        self.target_detected = False
        self.target_center = None
        self.image_center = None
        self.landing_initiated = False
        
        # Control parameters
        self.alignment_threshold = 30  # pixels
        self.descent_velocity = -0.5  # m/s
        self.landing_altitude = 2.0  # meters
        self.position_gain = 0.001  # proportional gain for position control
        self.max_velocity = 1.0  # m/s
        
    def image_callback(self, msg: Image):
        """
        Process incoming camera images.
        
        Args:
            msg (Image): ROS Image message
        """
        try:
            self.current_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.node.get_logger().error(f"Image conversion error: {e}")
    
    def detect_target(self):
        """
        Detect the colored rectangle target in current image.
        
        Returns:
            bool: True if target detected, False otherwise
        """
        if self.current_image is None:
            return False
        
        # Get image dimensions and center
        h, w = self.current_image.shape[:2]
        self.image_center = (w // 2, h // 2)
        
        # Detect target
        detection = self.detector.detect_rectangle(self.current_image)
        
        if detection is None:
            self.target_detected = False
            self.target_center = None
            return False
        
        # Store target information
        cX, cY, area = detection
        self.target_center = (cX, cY)
        self.target_detected = True
        
        return True
    
    def calculate_position_error(self):
        """
        Calculate position error between UAV center and target center.
        
        Returns:
            tuple: (error_x, error_y) in pixels, or None if no target
        """
        if not self.target_detected or self.target_center is None:
            return None
        
        error_x = self.target_center[0] - self.image_center[0]
        error_y = self.target_center[1] - self.image_center[1]
        
        return (error_x, error_y)
    
    def is_aligned(self):
        """
        Check if UAV is aligned with target within threshold.
        
        Returns:
            bool: True if aligned
        """
        error = self.calculate_position_error()
        
        if error is None:
            return False
        
        error_x, error_y = error
        distance = np.sqrt(error_x**2 + error_y**2)
        
        return distance < self.alignment_threshold
    
    def on_update(self, time_delta: float) -> None:
        """
        Periodic logic for precision landing.
        """
        # Detect target in current frame
        if not self.detect_target():
            self.node.get_logger().warn("Target not detected, hovering...")
            # Hover in place if target lost
            return
        
        # Calculate position error
        error = self.calculate_position_error()
        
        if error is None:
            return
        
        error_x, error_y = error
        
        # Log current state
        self.node.get_logger().info(
            f"Target: {self.target_center}, Error: ({error_x:.1f}, {error_y:.1f})"
        )
        
        # Check alignment and altitude
        if self.is_aligned():
            altitude = self.uav.get_altitude()
            
            if altitude <= self.landing_altitude and not self.landing_initiated:
                # Close enough to ground and aligned - initiate landing
                self.node.get_logger().info("Initiating final landing sequence...")
                self.uav.land()
                self.landing_initiated = True
            else:
                # Descend while maintaining position
                self.node.get_logger().info(f"Aligned, descending... Alt: {altitude:.2f}m")
                self.uav.set_velocity(0.0, 0.0, self.descent_velocity, 0.0)
        else:
            # Not aligned - adjust position towards target
            # Convert pixel error to velocity commands
            # Note: Image coordinates don't directly map to body frame
            # This assumes forward-facing downward camera
            vel_x = -self.position_gain * error_y  # Forward/backward
            vel_y = self.position_gain * error_x   # Left/right
            
            # Clip velocities to max
            vel_x = np.clip(vel_x, -self.max_velocity, self.max_velocity)
            vel_y = np.clip(vel_y, -self.max_velocity, self.max_velocity)
            
            # Slow descent while adjusting position
            vel_z = self.descent_velocity * 0.5
            
            self.node.get_logger().info(
                f"Adjusting: vx={vel_x:.2f}, vy={vel_y:.2f}, vz={vel_z:.2f}"
            )
            self.uav.set_velocity(vel_x, vel_y, vel_z, 0.0)
    
    def check_status(self) -> str:
        """
        Check the status of the mode.

        Returns:
            str: The status of the mode ("continue" or "terminate").
        """
        if self.uav.vehicle_status == VehicleStatus.NAVIGATION_STATE_AUTO_LAND:
            # Check if landed
            if self.uav.is_landed():
                self.node.get_logger().info("Landing complete!")
                return "terminate"
            return "continue"
        else:
            return "continue"