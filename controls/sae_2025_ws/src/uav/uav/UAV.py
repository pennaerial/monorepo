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

class UAV:
    """
    Skeleton class for UAV control and interfacing with PX4 via ROS 2.
    """

    def __init__(self, node: Node):
        self.node = node
        self.nav_state = VehicleStatus.NAVIGATION_STATE_MAX
        self.arm_state = VehicleStatus.ARMING_STATE_INIT
        self.flight_check = False
        self.failsafe = False
        
        # Subscribers/Publishers
        self._initialize_publishers_and_subscribers()

        # set yaw
        self.yaw = 0.0

        # vehicle status data --> VehicleStatus object
        self.vehicle_status = None

        # vehicle attitude data --> VehicleAttitude
        self.vehicle_attitude = None

        # global position --> VehicleGlobalPosition
        self.global_position = None
        
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

    def disarm(self):
        """Send a disarm command to the UAV."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            params={'param1': 0.0}  # param1=0 => Disarm
        )
        self.node.get_logger().info("Sent Disarm Command")

    def takeoff(self, altitude: float = 5.0):
        """
        Command the UAV to take off to the specified altitude.
        This uses a NAV_TAKEOFF command; actual behavior depends on PX4 mode.
        """
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            params={'param7': altitude}  # param7 = desired altitude
        )
        self.node.get_logger().info("Takeoff command sent.")

    def land(self):
        """Command the UAV to land."""
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.node.get_logger().info("Landing command sent.")

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

    def goto_gps_waypoint(self, lat: float, lon: float, alt: float):
        """
        Command the UAV to fly to a specific GPS location using
        a PX4 NAV_WAYPOINT command. The autopilot will handle
        navigation to the waypoint in AUTO mode.

        :param lat:  target latitude (degrees)
        :param lon:  target longitude (degrees)
        :param alt:  target altitude (meters AMSL)
        """
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_WAYPOINT,
            params={
                'param5': lat,   # X latitude
                'param6': lon,   # Y longitude
                'param7': alt,   # Z altitude (AMSL)
            }
        )
        self.node.get_logger().info(f"Sent waypoint command to lat={lat}, lon={lon}, alt={alt}")

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
        if msg.arm_state != self.arm_state:
            self.arm_state = msg.arm_state
            self.node.get_logger().info(f"Arm State: {self.arm_state}")
        if msg.failsafe and not self.failsafe:
            self.failsafe = True
            self.node.get_logger().warn("Failsafe triggered!")
        if not msg.flight_check and not self.flight_check:
            self.flight_check = False
            self.node.get_logger().warn("Pre-flight checks failed!")

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
