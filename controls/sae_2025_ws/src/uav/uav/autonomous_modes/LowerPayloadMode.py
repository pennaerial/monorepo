import math
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import LaserScan
from uav import Mode, UAV
from rclpy.node import Node

class LowerPayloadMode(Mode):
    """
    A mode for navigating through a series of waypoints while avoiding obstacles.
    """

    def __init__(self, node: Node, avoidance_radius: float = 2.0):
        """
        Initialize the LowerPayload.

        Args:
            node (Node): ROS 2 node managing the UAV.
            avoidance_radius (float): Minimum safe distance from obstacles.
        """
        super().__init__(node)
        self.avoidance_radius = avoidance_radius
        self.obstacle_detected = False
        self.current_pose = None
        self.payload_pose = None

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

        self.payload_location_subscription = self.node.create_subscription(
            PoseStamped, '/payload_location', self.payload_location_callback, 10
        )

    def on_enter(self):
        """
        Initialize the navigation mode by preparing for waypoint traversal.
        """
        self.log("Entering Payload Lowering Mode.")

        self.payload_location = None
        self.obstacle_detected = False
        if not self.uav.arm():
            self.log("Failed to arm the UAV.")
        else:
            self.log("UAV armed successfully. Starting navigation.")

    def on_exit(self):
        """
        Stop navigation and clean up.
        """
        self.log("Exiting Payload Lowering Mode.")
        self.uav.land()

    def on_update(self, time_delta: float):
        """
        Periodic logic for lowering payload and handling obstacles.
        """
        if self.current_waypoint_index >= len(self.waypoints):
            self.log("Lowered Payload. Landing.")
            self.deactivate()
            return

        if self.obstacle_detected:
            self.log("Obstacle detected. Stopping UAV.")
            self.stop_uav()
        else:
            self.navigate_to_payload()

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
    
    def payload_location_callback(self, msg: PoseStamped):
        """
        Callback for receiving the payload's current pose.

        Args:
            msg (PoseStamped): Current pose of the payload.
        """
        self.payload_location = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)

    def navigate_to_payload(self):
        """
        Navigate to the payload.
        """
        if self.current_pose is None or self.payload_location is None:
            self.log("Current pose not available yet.")
            return

        # Calculate the distance to the payload
        distance = math.sqrt(
            (self.payload_location[0] - self.current_pose[0]) ** 2 +
            (self.payload_location[1] - self.current_pose[1]) ** 2 +
            (self.payload_location[2] - self.current_pose[2]) ** 2
        )

        if distance < 0.5:  # Threshold to consider the payload reached
            self.log(f"Payload reached.")
        else:
            self.publish_target_position(self.payload_location)

    def publish_target_position(self):
        """
        Publish the target position to the UAV.
        """
        if self.payload_location is None:
            return
        target_pose = PoseStamped()
        target_pose.header.stamp = self.node.get_clock().now().to_msg()
        target_pose.header.frame_id = "map"
        target_pose.pose.position.x = self.payload_location[0]
        target_pose.pose.position.y = self.payload_location[1]
        target_pose.pose.position.z = self.payload_location[2]
        target_pose.pose.orientation.w = 1.0  # Assume no rotation for simplicity

        self.velocity_publisher.publish(target_pose)
        self.log(f"Publishing target position: {self.payload_location}")

    def stop_uav(self):
        """
        Stop the UAV by publishing the current position as the target.
        """
        if self.current_pose is not None:
            self.publish_target_position(self.current_pose)