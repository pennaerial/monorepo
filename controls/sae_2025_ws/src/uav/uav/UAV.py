import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleStatus, VehicleCommand, VehicleAttitude, VehicleGlobalPosition
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

        # Timers
        self._initialize_timers()

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
        



    def _initialize_timers(self):
        """
        Initialize ROS 2 timers.
        """
        self.node.create_timer(0.02, self._offboard_control_loop)
        

    # Public methods
    def arm(self):
        """
        Send an arm command to the UAV.
        """
        self._send_vehicle_command(400, params={"param1": 1.0}) 
        self.node.get_logger().info("Sent Arm Command")

    def disarm(self):
        """
        Send a disarm command to the UAV.
        """
        self._send_vehicle_command(400, params={"param1": 0.0})
        self.node.get_logger().info("Sent Disarm Command")


    def takeoff(self, altitude: float = 5.0):
        """
        Command the UAV to take off to the specified altitude.
        """
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF, param1 = 1.0, param7=altitude) 
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
    def get_gps(self):
        if self.vehicle_global_position is not None:
            gps_data = {
                "latitude": self.vehicle_global_position.lat,
                "longitude": self.vehicle_global_position.lon,
                "altitude": self.vehicle_global_position.alt
            }
            self.node.get_logger().info(
                f"Retrieved GPS Data: Latitude={gps_data['latitude']}, Longitude={gps_data['longitude']}, Altitude={gps_data['altitude']}"
            )
            return gps_data
        else:
            self.node.get_logger().warn("No GPS data available yet.")
            return None


    # Internal helper methods
    def _send_vehicle_command(self, command: int, params: dict = {}):
        """
        Publish a VehicleCommand message.
        """
        self.vehicle_command_publisher.publish(command, params)
        


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
        


    
    def _global_position_callback(self):
        self.vehicle_global_position = msg
        self.node.get_logger().debug(
            f"Global Position Updated - Latitude: {msg.lat}, Longitude: {msg.lon}, Altitude: {msg.alt}"
        )
