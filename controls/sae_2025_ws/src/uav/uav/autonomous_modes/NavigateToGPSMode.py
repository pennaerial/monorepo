import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from uav import Mode
from rclpy.node import Node

class NavigateToGPSMode(Mode):
    """
    A mode for navigating to a GPS coordinate while avoiding obstacles.
    """

    def __init__(self, node: Node, coordinate: tuple[float, float, float], avoidance_radius: float = 2.0):
        """
        Initialize the NavigateToGPSMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            coordinate (tuple[float, float, float]): The coordinate (x, y, z).
            avoidance_radius (float): Minimum safe distance from obstacles.
        """
        super().__init__(node)
        self.coordinate = coordinate
        self.reached = False
        self.avoidance_radius = avoidance_radius
        self.obstacle_detected = False
        self.current_pose = None

        # SAMPLE ROS 2 Subscribers
        self.pose_subscription = self.node.create_subscription(
            PoseStamped, '/uav/pose', self.pose_callback, 10
        )
        self.laser_subscription = self.node.create_subscription(
            LaserScan, '/uav/laser_scan', self.laser_callback, 10
        )

        # SAMPLE ROS 2 Publishers
        self.velocity_publisher = self.node.create_publisher(
            PoseStamped, '/uav/setpoint_position', 10
        )

    def on_enter(self):
        """
        Initialize the navigation mode by preparing to go to the GPS coordinate.
        """
        self.log("Entering GPS Coordinate Navigation Mode.")
        self.obstacle_detected = False
        if not self.uav.arm():
            self.log("Failed to arm the UAV.")
        else:
            self.log("UAV armed successfully. Starting navigation.")

    def on_exit(self):
        """
        Stop navigation and clean up.
        """
        self.log("Exiting GPS Navigation Mode.")
        self.uav.land()

    def on_update(self, time_delta: float):
        """
        Periodic logic for navigating to gps coord and handling obstacles.
        """

        if self.reached:
            self.log("GPS coordinate reached. Landing.")
            self.deactivate()
            return

        if self.obstacle_detected:
            self.log("Obstacle detected. Stopping UAV.")
            self.stop_uav()
        else:
            self.navigate_to_gps()

    def pose_callback(self, msg: PoseStamped):
        """
        Callback for receiving the UAV's current pose.

        Args:
            msg (PoseStamped): Current pose of the UAV.
        """
        self.current_pose = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def laser_callback(self, msg: LaserScan):
        """
        Callback for receiving laser scan data to detect obstacles.

        Args:
            msg (LaserScan): Laser scan data.
        """
        # Check if any detected distance is below the avoidance radius
        if any(distance < self.avoidance_radius for distance in msg.ranges):
            self.obstacle_detected = True
        else:
            self.obstacle_detected = False

    def navigate_to_gps(self):
        """
        Navigate to the gps coordinate
        """
        if self.current_pose is None:
            self.log("Current pose not available yet.")
            return

        # Calculate the distance to the gps coordinate
        distance = math.sqrt(
            (self.coordinate[0] - self.current_pose[0]) ** 2 +
            (self.coordinate[1] - self.current_pose[1]) ** 2 +
            (self.coordinate[2] - self.current_pose[2]) ** 2
        )

        if distance < 0.5:  # Threshold to consider the gps coord reached
            self.log(f"GPS coordinate {self.coordinate} reached.")
            self.reached = True
        else:
            self.publish_target_position(self.coordinate)

    def publish_target_position(self):
        """
        Publish the target position to the UAV.
        """
        target_pose = PoseStamped()
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = self.coordinate[0]
        target_pose.pose.position.y = self.coordinate[1]
        target_pose.pose.position.z = self.coordinate[2]
        target_pose.pose.orientation.w = 1.0  # Assume no rotation for simplicity

        self.velocity_publisher.publish(target_pose)
        self.log(f"Publishing target position: {self.coordinate}")

    def stop_uav(self):
        """
        Stop the UAV by publishing the current position as the target.
        """
        if self.current_pose is not None:
            self.publish_target_position(self.current_pose)