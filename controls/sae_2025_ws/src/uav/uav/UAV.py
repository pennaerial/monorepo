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
import math

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
        self.takeoff_height = -5.0
        self.takeoff_complete = False
        self.hover_time = 0

        self.start_local_position = None
        self.start_GPS_WGS84 = None
        self.start_GPS_ref = None


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
        radius = 5.0
        num_points = 100
        angles = np.linspace(0, 2 * np.pi, num_points, endpoint=False)
        self.waypoints = [('Local', (0.0, 0.0, self.takeoff_height))]  # Takeoff point
        for angle in angles:
            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            self.waypoints.append(('Local', (x, y, self.takeoff_height)))
        self.waypoints.append(('Local', (0.0, 0.0, self.takeoff_height)))

        self.waypoints = [('GPS',(47.397972, 8.546165, 5.0))]
        
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

    def gps_distance_3d(self, lat1, lon1, alt1, lat2, lon2, alt2):
        """
        Calculate the 3D distance in feet between two GPS points, including altitude.

        Parameters:
            lat1 (float): Latitude of the first point in decimal degrees.
            lon1 (float): Longitude of the first point in decimal degrees.
            alt1 (float): Altitude of the first point in feet above sea level.
            lat2 (float): Latitude of the second point in decimal degrees.
            lon2 (float): Longitude of the second point in decimal degrees.
            alt2 (float): Altitude of the second point in feet above sea level.

        Returns:
            float: The 3D distance between the two points in feet.
        """
        # Earth's radius in feet (using an average value)
        R_feet = 6371000 * 3.28084

        # Convert latitudes and longitudes from degrees to radians
        lat1_rad = math.radians(lat1)
        lon1_rad = math.radians(lon1)
        lat2_rad = math.radians(lat2)
        lon2_rad = math.radians(lon2)

        # Convert the first point to Cartesian coordinates
        x1 = (R_feet + alt1) * math.cos(lat1_rad) * math.cos(lon1_rad)
        y1 = (R_feet + alt1) * math.cos(lat1_rad) * math.sin(lon1_rad)
        z1 = (R_feet + alt1) * math.sin(lat1_rad)

        # Convert the second point to Cartesian coordinates
        x2 = (R_feet + alt2) * math.cos(lat2_rad) * math.cos(lon2_rad)
        y2 = (R_feet + alt2) * math.cos(lat2_rad) * math.sin(lon2_rad)
        z2 = (R_feet + alt2) * math.sin(lat2_rad)

        # Compute the Euclidean distance between the two points
        distance_feet = math.sqrt((x2 - x1)**2 + (y2 - y1)**2 + (z2 - z1)**2)
        return distance_feet
    
    def distance_to_waypoint(self, coordinate_system, waypoint) -> float:
        """Calculate the distance to the current waypoint."""
        if coordinate_system == 'GPS':
            curr_gps = self.get_gps()
            # self.node.get_logger().info(f"Currently at {curr_gps['lat']}, {curr_gps['lon']}, {curr_gps['alt']}")
            return self.gps_distance_3d(waypoint[0], waypoint[1], waypoint[2], curr_gps[0], curr_gps[1], curr_gps[2])
        elif coordinate_system == 'Local':
            return np.sqrt(
                (self.vehicle_local_position.x - waypoint[0]) ** 2 +
                (self.vehicle_local_position.y - waypoint[1]) ** 2 +
                (self.vehicle_local_position.z - waypoint[2]) ** 2
            )

    def advance_to_next_waypoint(self):
        """Advance to the next waypoint."""
        if self.current_waypoint_index < len(self.waypoints):
            coordinate_system, current_waypoint = self.waypoints[self.current_waypoint_index]
            if coordinate_system == 'GPS':
                self.goto_gps_position(current_waypoint)
            elif coordinate_system == 'Local':
                rel_waypoint = tuple(x-y for x, y in zip(current_waypoint, self.start_local_position))
                self.publish_position_setpoint(rel_waypoint)
            # self.node.get_logger().info(f"Advancing to waypoint {self.current_waypoint_index}")
            self.node.get_logger().info(f"Current distance: {self.distance_to_waypoint(coordinate_system, current_waypoint)}")
            if self.distance_to_waypoint(coordinate_system, current_waypoint) < self.waypoint_threshold:
                self.current_waypoint_index += 1
        else:
            self.mission_completed = True
            self.node.get_logger().info("Mission completed. Preparing to land.")

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
        self.publish_position_setpoint((0.0, 0.0, self.takeoff_height))
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
    
    def publish_position_setpoint(self, coordinate):
        x, y, z = coordinate
        """Publish the trajectory setpoint."""
        msg = TrajectorySetpoint()
        msg.position = [x, y, z]
        # Calculate yaw to point towards the waypoint
        msg.yaw = self.calculate_yaw(x, y)
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.trajectory_publisher.publish(msg)
        # self.node.get_logger().info(f"Publishing setpoint: pos={[x, y, z]}, yaw={msg.yaw:.2f}")
    
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

    # def set_velocity(self, vx: float, vy: float, vz: float, yaw_rate: float):
    #     """
    #     Set velocity commands for offboard control (local frame).
    #     Only valid if the vehicle is in OFFBOARD mode.
    #     """
    #     trajectory = TrajectorySetpoint()
    #     trajectory.timestamp = int(Clock().now().nanoseconds / 1000)
    #     trajectory.velocity = [vx, vy, vz]
    #     trajectory.yawspeed = yaw_rate
    #     # Mark position and acceleration as NaN if you only want to control velocity
    #     trajectory.position = [float('nan'), float('nan'), float('nan')] 
    #     trajectory.acceleration = [float('nan')] * 3  
    #     trajectory.yaw = float('nan') 

    #     self.trajectory_publisher.publish(trajectory)
    #     self.node.get_logger().debug(
    #         f"Velocity command sent: vx={vx}, vy={vy}, vz={vz}, yaw_rate={yaw_rate}"
    #     )

    # def set_target_position(self, pos: tuple[float , float, float]):
    #     """
    #     Set the target local position for offboard control.
    #     Typically used with local-position offboard setpoints (X,Y,Z).
    #     """
    #     x, y, z = pos
    #     position = VehicleLocalPosition()
    #     position.timestamp = int(Clock().now().nanoseconds / 1000)
    #     position.x = x
    #     position.y = y
    #     position.z = z
    #     self.target_position_publisher.publish(position)
    #     self.node.get_logger().debug(f"Target position set: x={x}, y={y}, z={z}")

    # def goto_gps_waypoint(self, coordinate):
    #     """
    #     Calculate the local NED displacement (in feet) from a current GPS coordinate
    #     to a target GPS coordinate. The output is the necessary motion in the local
    #     North, East, and Down directions (in feet) required to reach the target.

    #     Parameters:
    #         current_lat (float): Current latitude in decimal degrees.
    #         current_lon (float): Current longitude in decimal degrees.
    #         current_alt (float): Current altitude in feet.
    #         target_lat (float): Target latitude in decimal degrees.
    #         target_lon (float): Target longitude in decimal degrees.
    #         target_alt (float): Target altitude in feet.

    #     Returns:
    #         tuple: (delta_north, delta_east, delta_down) in feet.
    #             In the NED frame:
    #                 - delta_north: Positive means move north.
    #                 - delta_east:  Positive means move east.
    #                 - delta_down:  Positive means move downward.
    #                         (i.e. current_alt - target_alt)
        
    #     Note:
    #         This function uses a simple local tangent plane approximation which is
    #         valid for relatively short distances.
    #     """
    #     current_lat, current_lon, current_alt = self.get_gps()
    #     target_lat, target_lon, target_alt = coordinate
    #     # Earth's radius in feet (using average Earth radius in meters * conversion factor)
    #     R_feet = 6371000 * 3.28084  # ~20,902,264 feet

    #     # Convert degrees to radians for computation
    #     current_lat_rad = math.radians(current_lat)
    #     target_lat_rad = math.radians(target_lat)

    #     # Differences in latitude and longitude (in degrees)
    #     delta_lat_deg = target_lat - current_lat
    #     delta_lon_deg = target_lon - current_lon

    #     # Convert angular differences to radians
    #     delta_lat_rad = math.radians(delta_lat_deg)
    #     delta_lon_rad = math.radians(delta_lon_deg)

    #     # Calculate North and East offsets in feet (small angle approximation)
    #     delta_north = delta_lat_rad * R_feet
    #     delta_east = delta_lon_rad * R_feet * math.cos(current_lat_rad)

    #     # For the Down component in the NED frame:
    #     # Down is positive if the target is lower than the current altitude.
    #     delta_down = current_alt - target_alt

    #     self.publish_position_setpoint((delta_north, delta_east, delta_down))

    # def goto_gps_waypoint(self, coordinate):
    #     """
    #     Command the UAV to fly to a specific GPS location using
    #     a PX4 NAV_WAYPOINT command. The autopilot will handle
    #     navigation to the waypoint in AUTO mode.

    #     :param lat:  target latitude (degrees)
    #     :param lon:  target longitude (degrees)
    #     :param alt:  target altitude (meters AMSL)
    #     """
    #     lat, lon, alt = coordinate
    #     self._send_vehicle_command(
    #         VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
    #         params={
    #             'param5': lat,   # X latitude
    #             'param6': lon,   # Y longitude
    #             'param7': alt + self.start_GPS_WGS84[2] - self.start_GPS_AMSL[2],   # Z altitude (WGS84)
    #         }
    #     )
    #     self.node.get_logger().info(f"Sent waypoint command to lat={lat}, lon={lon}, alt={alt}")

    import math

    def goto_gps_position(self, target):
        """
        Convert target GPS coordinates to local NED coordinates.

        Args:
            target (tuple): (target_lat, target_lon, target_alt)
            ref (tuple): (ref_lat, ref_lon, ref_alt) from the local position message

        Returns:
            tuple: (x, y, z) in the local frame where:
                x is North (meters),
                y is East (meters),
                z is Down (meters)
        """
        target_lat, target_lon, target_alt = target
        ref_lat, ref_lon, ref_alt = self.start_GPS_ref

        # Convert differences in latitude and longitude from degrees to radians
        d_lat = math.radians(target_lat - ref_lat)
        d_lon = math.radians(target_lon - ref_lon)

        # Earth's radius in meters (WGS84)
        R_earth = 6378137.0

        # Compute local displacements
        x = d_lat * R_earth  # North displacement
        y = d_lon * R_earth * math.cos(math.radians(ref_lat))  # East displacement
        z = -(target_alt - ref_alt)  # Down displacement

        self.publish_position_setpoint((x, y, z))

    # def reached_position 

    # -------------------------
    # Getters / data access
    # -------------------------
    def get_gps(self):
        if self.global_position:
            return (
                self.global_position.lat,
                self.global_position.lon,
                self.global_position.alt
            )
        else:
            self.node.get_logger().warn("No GPS data available.")
            return None

    # def get_quaternion(self):
    #     if self.vehicle_attitude:
    #         return {
    #             "q": self.vehicle_attitude.q,
    #             "delta_q": self.vehicle_attitude.delta_q_reset
    #         }
    #     else:
    #         self.node.get_logger().warn("No quaternion data available.")
    #         return None

    # def get_state(self):
    #     # Example placeholder; if you want real nav_state or arm_state,
    #     # retrieve them from self.vehicle_status
    #     if self.vehicle_status:
    #         return {
    #             "nav_state": self.vehicle_status.nav_state,
    #             "arm_state": self.vehicle_status.arm_state
    #         }
    #     else:
    #         self.node.get_logger().warn("No state data available.")
    #         return None

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
        if not self.start_GPS_WGS84:
            self.start_GPS_WGS84 = (msg.lat, msg.lon, msg.alt)
            self.node.get_logger().info(f"Local start position: {self.start_GPS_WGS84}")
        self.global_position = msg
        self.node.get_logger().info(
            f"Global Position - Lat: {msg.lat:.10f}, Lon: {msg.lon:.10f}, Alt: {msg.alt:.10f}"
        )
    
    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        if not self.start_local_position:
            self.start_local_position = (msg.x, msg.y, msg.z)
            self.start_GPS_ref = (msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.node.get_logger().info(f"Local start position: {self.start_local_position}")
        self.vehicle_local_position = msg
        # self.node.get_logger().info(
        #     f"Local Position - x: {msg.x:.5f}, y: {msg.y:.5f}, z: {msg.z:.5f}"
        # )
