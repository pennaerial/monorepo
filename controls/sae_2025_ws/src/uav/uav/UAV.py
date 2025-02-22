import rclpy
from rclpy.node import Node
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleCommand,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLocalPosition,
)
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import numpy as np
# from geopy.distance import geodesic

class UAV:
    """
    Skeleton class for UAV control and interfacing with PX4 via ROS 2.
    """

    def __init__(self, node: Node):
        self.node = node
        self.flight_check = False
        self.failsafe = False
        
        # Subscribers/Publishers
        self._initialize_publishers_and_subscribers()

        # set yaw
        self.yaw = 0.0
        self.takeoff_height = -3.0
        self.takeoff_complete = False
        self.hover_time = 0
        self.vehicle_local_position = VehicleLocalPosition()


        # vehicle status data --> VehicleStatus object
        self.vehicle_status = None

        # vehicle attitude data --> VehicleAttitude
        self.vehicle_attitude = None

        # global position --> VehicleGlobalPosition
        self.global_position = None

        self.vehicle_local_position = None
        self.nav_state = None
        self.arm_state = None
        self.failsafe = None
        self.flight_check = None
        self.offboard_setpoint_counter = 0

        # mission checkpoints
        self.current_waypoint_index = 0
        self.waypoint_threshold = 2.0
        self.mission_completed = False
        radius = 1.0
        num_points = 100
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        self.waypoints = [('Local', [0.0, 5.0, self.takeoff_height])]  # Takeoff point
        for angle in angles:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.waypoints.append(('Local', [0.0 + x, 5.0 + y, self.takeoff_height]))
        self.waypoints.append(('Local', [0.0, 5.0, self.takeoff_height]))

        # self.waypoints = [('GPS',[40, 80, 10])]
        
    def _initialize_publishers_and_subscribers(self):
        """
        Initialize ROS 2 publishers and subscribers.
        """

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        # Publishers
        self.offboard_mode_publisher = self.node.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_publisher = self.node.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )
        self.target_position_publisher = self.node.create_publisher(
            VehicleLocalPosition, '/fmu/in/target_position', 10
        )

        # Subscribers
        self.status_sub = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            qos_profile
        )
        
        self.attitude_sub = self.node.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile
        )

        self.global_position_sub = self.node.create_subscription(
            VehicleGlobalPosition,
            'fmu/out/vehicle_global_position',
            self._global_position_callback,
            qos_profile
        )

        self.vehicle_local_position_subscriber = self.node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._vehicle_local_position_callback, qos_profile)

    # -------------------------
    # Public commands
    # -------------------------
    def arm(self):
        """Send an arm command to the UAV."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            params={'param1': 1.0}  # param1=1 => Arm
        )
        self.node.get_logger().info("Sent Arm Command")
    
    def distance_to_waypoint(self, coordinate_system, waypoint) -> float:
        """Calculate the distance to the current waypoint."""
        if coordinate_system == 'GPS':
            pass
        elif coordinate_system == 'Local':
            return np.sqrt(
                (self.vehicle_local_position.x - waypoint[0]) ** 2 +
                (self.vehicle_local_position.y - waypoint[1]) ** 2 +
                (self.vehicle_local_position.z - waypoint[2]) ** 2
            )

    def advance_to_next_waypoint(self):
        """Advance to the next waypoint."""
        if self.current_waypoint_index < len(self.waypoints) - 1:
            coordinate_system, current_waypoint = self.waypoints[self.current_waypoint_index]
            if coordinate_system == 'GPS':
                self.goto_gps_waypoint(current_waypoint)
            elif coordinate_system == 'Local':
                self.publish_position_setpoint(*current_waypoint)
            self.node.get_logger().info(f"Advancing to waypoint {self.current_waypoint_index}")
            if self.distance_to_waypoint(coordinate_system, current_waypoint) < self.waypoint_threshold:
                self.current_waypoint_index += 1
        else:
            # Original code:
            # self.mission_completed = True
            # self.node.get_logger().info("Mission completed. Preparing to land.")
            # Add infinite length task to take the screen shot:
            self.current_waypoint_index = 0

    def disarm(self):
        """Send a disarm command to the UAV."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            params={'param1': 0.0}  # param1=0 => Disarm
        )
        self.node.get_logger().info("Sent Disarm Command")
    
    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self.node.get_logger().info("Switching to offboard mode...")
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 
            params={'param1':1.0, 'param2':6.0}
        )
        # self.get_logger().info("Switching to offboard mode")

    def takeoff(self):
        """
        Command the UAV to take off to the specified altitude.
        This uses a NAV_TAKEOFF command; actual behavior depends on PX4 mode.
        """
        self.publish_position_setpoint(0.0, 0.0, self.takeoff_height)
        self.node.get_logger().info("Takeoff command sent.")


    def check_takeoff_complete(self) -> bool:
        """Check if takeoff is complete."""
        height_reached = abs(self.vehicle_local_position.z - self.takeoff_height) < 0.5
        if height_reached and not self.takeoff_complete:
            self.hover_time += 1
            if self.hover_time >= 20:  # Reduced hover time to 2 seconds (20 * 0.1s)
                self.takeoff_complete = True
                self.node.get_logger().info("Takeoff complete, starting mission")
                return True
        return False

    def land(self):
        """Command the UAV to land."""
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.node.get_logger().info("Landing command sent.")
    
    def publish_position_setpoint(self, x: float, y: float, z: float):
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # Calculate yaw to point towards the waypoint
        msg.yaw = self.calculate_yaw(x, y)
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.trajectory_publisher.publish(msg)
        self.node.get_logger().info(f"Publishing setpoint: pos={[x, y, z]}, yaw={msg.yaw:.2f}")
    
    def calculate_yaw(self, x: float, y: float) -> float:
        """Calculate the yaw angle to point towards the next waypoint."""
        # Calculate relative position
        dx = x - self.vehicle_local_position.x
        dy = y - self.vehicle_local_position.y
        
        # Calculate yaw angle
        yaw = np.arctan2(dy, dx)
        return yaw
    
    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_publisher.publish(msg)

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Set velocity commands for offboard control (local frame).
        Only valid if the vehicle is in OFFBOARD mode.
        """
        trajectory = TrajectorySetpoint()
        trajectory.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory.velocity = [vx, vy, vz]
        trajectory.yawspeed = yaw_rate
        # Mark position and acceleration as NaN if you only want to control velocity
        trajectory.position = [float('nan'), float('nan'), float('nan')] 
        trajectory.acceleration = [float('nan')] * 3  
        trajectory.yaw = float('nan') 

        self.trajectory_publisher.publish(trajectory)
        self.node.get_logger().debug(
            f"Velocity command sent: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}"
        )

    def set_target_position(self, pos: tuple[float , float, float]):
        """
        Set the target local position for offboard control.
        Typically used with local-position offboard setpoints (X,Y,Z).
        """
        x, y, z = pos
        position = VehicleLocalPosition()
        position.timestamp = int(Clock().now().nanoseconds / 1000)
        position.x = x
        position.y = y
        position.z = z
        self.target_position_publisher.publish(position)
        self.node.get_logger().debug(f"Target position set: x={x}, y={y}, z={z}")

    def goto_gps_waypoint(self, coordinate):
        """
        Command the UAV to fly to a specific GPS location using
        a PX4 NAV_WAYPOINT command. The autopilot will handle
        navigation to the waypoint in AUTO mode.

        :param lat:  target latitude (degrees)
        :param lon:  target longitude (degrees)
        :param alt:  target altitude (meters AMSL)
        """
        lat, lon, alt = coordinate
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_WAYPOINT,
            params={
                'param5': lat,   # X latitude
                'param6': lon,   # Y longitude
                'param7': alt,   # Z altitude (AMSL)
            }
        )
        self.node.get_logger().info(f"Sent waypoint command to lat={lat}, lon={lon}, alt={alt}")

    # def reached_position 

    # -------------------------
    # Getters / data access
    # -------------------------
    def get_gps(self):
        if self.global_position:
            return {
                "latitude": self.global_position.lat,
                "longitude": self.global_position.lon,
                "altitude": self.global_position.alt,
            }
        else:
            self.node.get_logger().warn("No GPS data available.")
            return None

    def get_quaternion(self):
        if self.vehicle_attitude:
            return {
                "q": self.vehicle_attitude.q,
                "delta_q": self.vehicle_attitude.delta_q_reset
            }
        else:
            self.node.get_logger().warn("No quaternion data available.")
            return None

    def get_state(self):
        # Example placeholder; if you want real nav_state or arm_state,
        # retrieve them from self.vehicle_status
        if self.vehicle_status:
            return {
                "nav_state": self.vehicle_status.nav_state,
                "arm_state": self.vehicle_status.arm_state
            }
        else:
            self.node.get_logger().warn("No state data available.")
            return None

    # -------------------------
    # Internal helper methods
    # -------------------------
    def _send_vehicle_command(
        self,
        command: int,
        params: dict = {},
        target_system=1,
        target_component=1,
        source_system=1,
        source_component=1,
        from_external=True
    ):
        """
        Publish a VehicleCommand message to instruct PX4 to perform an action.
        """
        msg = VehicleCommand()

        # Fill in parameters
        msg.param1 = params.get('param1', 0.0)
        msg.param2 = params.get('param2', 0.0)
        msg.param3 = params.get('param3', 0.0)
        msg.param4 = params.get('param4', 0.0)
        msg.param5 = params.get('param5', 0.0)  # lat
        msg.param6 = params.get('param6', 0.0)  # lon
        msg.param7 = params.get('param7', 0.0)  # alt

        msg.command = command

        msg.target_system = target_system
        msg.target_component = target_component
        msg.source_system = source_system
        msg.source_component = source_component
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # microseconds

        # Publish
        self.vehicle_command_publisher.publish(msg)

    # -------------------------
    # Subscribers / callbacks
    # -------------------------
    def _vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg

        # Example logs
        if msg.nav_state != self.nav_state:
            self.nav_state = msg.nav_state
            self.node.get_logger().info(f"Navigation State: {self.nav_state}")
        if msg.arming_state != self.arm_state:
            self.arm_state = msg.arming_state
            self.node.get_logger().info(f"Arm State: {self.arm_state}")
        if msg.failsafe and not self.failsafe:
            self.failsafe = True
            self.node.get_logger().warn("Failsafe triggered!")

    def _attitude_callback(self, msg: VehicleAttitude):
        self.vehicle_attitude = msg

        q = msg.q
        # Calculate yaw from quaternion
        self.yaw = np.arctan2(
            2.0 * (q[3] * q[0] + q[1] * q[2]),
            1.0 - 2.0 * (q[0]**2 + q[1]**2)
        )
        self.node.get_logger().debug(f"Attitude - Yaw: {self.yaw}")

    def _global_position_callback(self, msg: VehicleGlobalPosition):
        self.global_position = msg
        self.node.get_logger().debug(
            f"Global Position - Lat: {msg.lat:.5f}, Lon: {msg.lon:.5f}, Alt: {msg.alt:.2f}"
        )
    
    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        self.vehicle_local_position = msg
        self.node.get_logger().debug(
            f"Local Position - x: {msg.x:.5f}, y: {msg.y:.5f}, z: {msg.z:.5f}"
        )
