import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitude, VehicleGlobalPosition, VehicleLocalPosition
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy

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
        
        # Subscribers
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


        #publishers
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

        # subscribers
        self.status_sub = self.node.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self._vehicle_status_callback,
            qos_profile)
        
        self.attitude_sub = self.node.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self._attitude_callback,
            qos_profile)

        self.global_position_sub = self.node.create_subscription(
            VehicleGlobalPosition,
            'fmu/out/vehicle_global_position',
            self._global_position_callback,
            qos_profile)
        

    # Public methods
    def arm(self):
        """
        Send an arm command to the UAV.
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, params={'param1': 1.0}) 
        self.node.get_logger().info("Sent Arm Command")

    def disarm(self):
        """
        Send a disarm command to the UAV.
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, params={'param2': 1.0})
        self.node.get_logger().info("Sent Disarm Command")


    def takeoff(self, altitude: float = 5.0):
        """
        Command the UAV to take off to the specified altitude.
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, params={'param1': 1.0, 'param7': altitude}) 
        self.get_logger().info("Takeoff command send")

    def land(self):
        """
        Command the UAV to land.
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.node.get_logger().info("Landing command sent.")

    def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
        """
        Set velocity commands for offboard control.
        """
        trajectory = TrajectorySetpoint()
        trajectory.timestamp = int(Clock().now().nanoseconds / 1000)
        trajectory.velocity = [vx, vy, vz]
        trajectory.yawspeed = yaw_rate
        trajectory.position = [float('nan'), float('nan'), float('nan')] 
        trajectory.acceleration = [float('nan')] * 3  
        trajectory.yaw = float('nan') 

        self.trajectory_publisher.publish(trajectory)
        self.node.get_logger().debug(
            f"Velocity command sent: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}"
        )

    def set_target_position(self, pos: tuple[float , float, float]):
        """
        Set the target position for the UAV.
        """
        x, y, z = pos
        position = VehicleLocalPosition()
        position.timestamp = int(Clock().now().nanoseconds / 1000)
        position.x = x
        position.y = y
        position.z = z
        self.target_position_publisher.publish(position)
        self.node.get_logger().debug(f"Target position set: x={x}, y={y}, z={z}")


    # Getters : VehicleStatus

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
        
    # Getters : VehicleAttitude
    def get_quaternion(self):
        if self.vehicle_attitude:
            return {
                "q": self.vehicle_attitude.q,
                "delta_q": self.vehicle_attitude.delta_q_reset
            }
        else:
            self.node.get_logger().warn("No quaternion data available.")
            return None
        
    # Getters : VehicleStatus
    def get_state(self):
        if self.vehicle_attitude:
            return {
                "nav_state": self.vehicle_attitude.nav_state,
                "arm_state": self.vehicle_attitude.arm_state
            }
        else:
            self.node.get_logger().warn("No state data available.")
            return None
    


    # Internal helper methods
    def _send_vehicle_command(self, command: int, params: dict = {},
                               target_system = 1, target_component = 1, source_system = 1,
                               source_component = 1,
                               from_external = True):
        """
        Publish a VehicleCommand message.
        """
        msg = VehicleCommand()
        msg.param1 = params['param1'] if 'param1' in params else 0.0
        msg.param2 = params['param2'] if 'param2' in params else 0.0
        msg.param3 = params['param3'] if 'param3' in params else 0.0
        msg.param4 = params['param4'] if 'param4' in params else 0.0
        msg.param5 = params['param5'] if 'param5' in params else 0.0
        msg.param6 = params['param6'] if 'param6' in params else 0.0
        msg.param7 = params['param7'] if 'param7' in params else 0.0

        msg.command = command

        msg.target_system = target_system  # system to execute command
        msg.target_component = target_component  # component which should execute the command, 0 for all components
        msg.source_system = source_system  # system sending the command
        msg.source_component = source_component  # component sending the command
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000) # time in microseconds
        self.vehicle_command_publisher_.publish(msg)
        


    # Callbacks
    
    def _vehicle_status_callback(self, msg: VehicleStatus):
        """
        Callback for handling VehicleStatus messages.
        
        Args:
            msg (VehicleStatus): The VehicleStatus message received from the UAV.
        """
        #store vars
        self.vehicle_status = msg

        if msg.nav_state != VehicleStatus.NAVIGATION_STATE_MAX:
            self.node.get_logger().info(f"Navigation State Changed: {self.nav_state}")

        # Log changes in arming state
        if msg.arm_state != VehicleStatus.ARMING_STATE_INIT:
            self.node.get_logger().info(f"Arming State: {self.arm_state}")

        # Log failsafe status
        if msg.failsafe:
            self.node.get_logger().warn("Failsafe triggered! Taking safety measures.")

        # Log pre-flight check status
        if not msg.flight_check:
            self.node.get_logger().warn("Pre-flight checks failed!")
        else:
            self.node.get_logger().info("Pre-flight checks passed.")
        
    def _attitude_callback(self, msg: VehicleAttitude):
        """
        Callback for handling VehicleStatus messages.
    
        Args:
            msg (VehicleStatus): The VehicleStatus message received from the UAV.
        """

        # store vars
        self.vehicle_attitude = msg

        
        q = msg.q
        self.node.get_logger().info(f"Received Attitude - Quaternion: {q}")
        self.node.get_logger().debug(f"Delta Q Reset: {msg.delta_q_reset}, Reset Counter: {msg.quat_reset_counter}")
        self.vehicle_attitude = msg

        # Calculate yaw
        self.yaw = np.arctan2(
            2.0 * (q[3] * q[0] + q[1] * q[2]),
            1.0 - 2.0 * (q[0] ** 2 + q[1] ** 2)
        )
        self.node.get_logger().debug(f"Current Yaw: {self.yaw}")

    def _global_position_callback(self, msg: VehicleGlobalPosition):

        """
        Callback for handling global position updates from the UAV.
        
        Args:
            msg (VehicleGlobalPosition): The global position message received.
        """

        self.global_position = msg

        self.node.get_logger().info(
            f"Global Position - Lat: {msg.lat}, Lon: {msg.lon}, Alt: {msg.alt}"
        )
        


    