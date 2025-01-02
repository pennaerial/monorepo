import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from uav import Mode
from rclpy.node import Node

class WaypointMode(Mode):
    """
    A mode for navigating through a series of waypoints while avoiding obstacles.
    """

    def __init__(self, node: Node, waypoints: list[tuple[float, float, float]], avoidance_radius: float = 2.0):
        """
        Initialize the WaypointNavigationMode.

        Args:
            node (Node): ROS 2 node managing the UAV.
            waypoints (list[tuple[float, float, float]]): List of waypoints (x, y, z).
            avoidance_radius (float): Minimum safe distance from obstacles.
        """
        super().__init__(node)
        self.waypoints = waypoints
        self.current_waypoint_index = 0
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
        Initialize the navigation mode by preparing for waypoint traversal.
        """
        self.log("Entering Waypoint Navigation Mode.")
        self.current_waypoint_index = 0
        self.obstacle_detected = False
        if not self.uav.arm():
            self.log("Failed to arm the UAV.")
        else:
            self.log("UAV armed successfully. Starting navigation.")

    def on_exit(self):
        """
        Stop navigation and clean up.
        """
        self.log("Exiting Waypoint Navigation Mode.")
        self.uav.land()

    def on_update(self, time_delta: float):
        """
        Periodic logic for navigating to waypoints and handling obstacles.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            self.log("All waypoints reached. Landing.")
            self.deactivate()
            return

        if self.obstacle_detected:
            self.log("Obstacle detected. Stopping UAV.")
            self.stop_uav()
        else:
            waypoint = self.waypoints[self.current_waypoint_index]
            self.navigate_to_waypoint(waypoint)

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

    def navigate_to_waypoint(self, waypoint: tuple[float, float, float]):
        """
        Navigate to the current waypoint.

        Args:
            waypoint (tuple[float, float, float]): Target waypoint (x, y, z).
        """
        if self.current_pose is None:
            self.log("Current pose not available yet.")
            return

        # Calculate the distance to the waypoint
        distance = math.sqrt(
            (waypoint[0] - self.current_pose[0]) ** 2 +
            (waypoint[1] - self.current_pose[1]) ** 2 +
            (waypoint[2] - self.current_pose[2]) ** 2
        )

        if distance < 0.5:  # Threshold to consider the waypoint reached
            self.log(f"Waypoint {self.current_waypoint_index} reached.")
            self.current_waypoint_index += 1
        else:
            self.publish_target_position(waypoint)

    def publish_target_position(self, waypoint: tuple[float, float, float]):
        """
        Publish the target position to the UAV.

        Args:
            waypoint (tuple[float, float, float]): Target waypoint (x, y, z).
        """
        target_pose = PoseStamped()
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = waypoint[0]
        target_pose.pose.position.y = waypoint[1]
        target_pose.pose.position.z = waypoint[2]
        target_pose.pose.orientation.w = 1.0  # Assume no rotation for simplicity

        self.velocity_publisher.publish(target_pose)
        self.log(f"Publishing target position: {waypoint}")

    def stop_uav(self):
        """
        Stop the UAV by publishing the current position as the target.
        """
        if self.current_pose is not None:
            self.publish_target_position(self.current_pose)