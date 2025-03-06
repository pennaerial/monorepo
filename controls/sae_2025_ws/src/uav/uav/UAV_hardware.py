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
    SensorGps
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
from collections import deque

class UAV:
    """
    Skeleton class for UAV control and interfacing with PX4 via ROS 2.
    """

    def __init__(self, node: Node):

        self.takeoff_gps = None

        self.node = node

        # Initialize necessary parameters to handle PX4 flight failures
        self.flight_check = False
        self.failsafe = False
        self.vehicle_status = None
        self.vehicle_attitude = None
        self.nav_state = None
        self.arm_state = None
        self.offboard_setpoint_counter = 0
        
        # Set up Subscribers/Publishers to communicate with aircraft
        self._initialize_publishers_and_subscribers()

        # set takeoff parameters
        self.yaw = 0.0
        self.takeoff_height = None
        self.takeoff_complete = False
        self.hover_time = 0

        # Initialize drone position
        self.start_local_position = None
        self.start_GPS_ref = None

        # Store current drone position
        self.global_position = None
        self.vehicle_local_position = None
        self.sensor_gps = None


        # self.current_waypoint_index = 0
        self.waypoint_threshold = 0.5
        self.mission_completed = False

        # Start with an empty list of waypoints (will store GPS waypoints)

        self.waypoints = deque()
        self.reached_waypoint = True
        self.curr_waypoint = None
        self.coordinate_system = None
        
        self.initiated_landing = False
        self.safe_landing_hover_time = 30
    

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
        # if self.current_waypoint_index < len(self.waypoints):
        
        # Not landing, reached last waypoint, and there is another waypoint to go to
        # self.node.get_logger().info(f"Waypoints remaining: {len(self.waypoints)}")
        if len(self.waypoints) == 0:
            self.hover()
        if len(self.waypoints) != 0 and self.reached_waypoint and not self.initiated_landing:
            # coordinate_system, current_waypoint = self.waypoints[self.current_waypoint_index]
            self.reached_waypoint = False
            coordinate_system, current_waypoint = self.waypoints.popleft()
            self.curr_waypoint = current_waypoint
            self.coordinate_system = coordinate_system
            if coordinate_system == 'GPS':
                local_target = self.gps_to_local(current_waypoint)
                self.publish_position_setpoint(local_target)
            elif coordinate_system == 'Local':
                rel_waypoint = tuple(x-y for x, y in zip(current_waypoint, self.start_local_position))
                self.publish_position_setpoint(rel_waypoint)
            elif coordinate_system == 'END':
                self.node.get_logger().info("Mission completed. Preparing to land.")
                self.curr_waypoint = self.start_local_position
                self.coordinate_system = 'Local'
                self.initiated_landing = True
            # self.node.get_logger().info(f"Advancing to waypoint {self.current_waypoint_index}")
            # self.node.get_logger().info(f"Current distance: {self.distance_to_waypoint(coordinate_system, current_waypoint)}")
        
        # Not landing, having reached waypoint yet
        elif not self.initiated_landing and not self.reached_waypoint:
            self.node.get_logger().info(f"Current heading to {self.gps_to_local(self.curr_waypoint)}")
            if self.distance_to_waypoint(self.coordinate_system, self.curr_waypoint) < self.waypoint_threshold:
                # self.current_waypoint_index += 1
                self.reached_waypoint = True

        # Not landing, reached waypoint but no more waypoints
        # elif not self.initiated_landing:
        #     self.hover()

        # Landing
        elif self.initiated_landing:
            if self.safe_landing_hover_time > 0:
                self.node.get_logger().info(f"Landing in T - {self.safe_landing_hover_time} seconds.")
                self.safe_landing_hover_time -= 1
                self.hover()
            else:
                self.land()
                self.mission_completed = True

    def hover(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            params={'param1':1.0, 'param2':4.0, 'param3': 3.0})

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
        if not self.takeoff_gps:
            lat = self.sensor_gps.latitude_deg
            lon = self.sensor_gps.longitude_deg
            alt = self.sensor_gps.altitude_msl_m
            self.takeoff_gps = (lat, lon, alt + 5.0)
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                                   params={'param1':10.0, 'param2':0.0, 'param3': 0.0, 'param4': 0.0, 'param5':self.takeoff_gps[0], 'param6': self.takeoff_gps[1], 'param7': self.takeoff_gps[2]})

    def add_waypoint(self, waypoint, coordinate_system):
        self.waypoints.append((coordinate_system, waypoint))
    
    def enter_mission(self):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, params={'param1': 1.0, 'param2': 6.0})
    
    def set_origin(self):
        lat = self.sensor_gps.latitude_deg
        lon = self.sensor_gps.longitude_deg
        alt = self.sensor_gps.altitude_msl_m
        self.node.get_logger().info(f"Lat {lat}, Lon {lon}, Alt {alt}, type: {type(lat)}")
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
                                   params={'param1':0.0, 'param2':0.0, 'param3': 0.0, 'param4': 0.0, 'param5':lat, 'param6': lon, 'param7': alt})

    def test(self, waypoint):
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_REPOSITION,
                                   params={'param1':5.0, 'param2':0.0, 'param3': 0.0, 'param4': 0.0, 'param5':waypoint[0], 'param6': waypoint[1], 'param7': waypoint[2]})
        
    # def test2(self):
    #     self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
    #                                params={'param1': 0.0})

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
        # msg.yaw = self.calculate_yaw(x, y)
        msg.yaw = 0.0
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
        curr_x, curr_y, curr_z = self.gps_to_local((lat1, lon1, alt1))
        tar_x, tar_y, tar_z = self.gps_to_local((lat2, lon2, alt2))
        return np.sqrt(
                (curr_x - tar_x) ** 2 +
                (curr_y - tar_y) ** 2 +
                (curr_z - tar_z) ** 2
            )

    def gps_to_local(self, target):
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

        return (x, y, z)
    
    def local_to_gps(self, local_pos):
        """
        Convert a local NED coordinate to a GPS coordinate.
        
        Args:
            local_pos (tuple): (x, y, z) in meters, where:
                x: North displacement,
                y: East displacement,
                z: Down displacement.
            ref_gps (tuple): (lat, lon, alt) of the reference point (takeoff) in degrees and meters.
        
        Returns:
            tuple: (lat, lon, alt) GPS coordinate corresponding to local_pos.
        """
        x, y, z = local_pos
        lat0, lon0, alt0 = self.start_GPS_ref
        R_earth = 6378137.0  # Earth's radius in meters

        # Convert displacements from meters to degrees
        dlat = (x / R_earth) * (180.0 / math.pi)
        dlon = (y / (R_earth * math.cos(math.radians(lat0)))) * (180.0 / math.pi)
        
        lat = lat0 + dlat
        lon = lon0 + dlon
        alt = alt0 - z  # because z is down in NED
        return (lat, lon, alt)

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
        if msg.nav_state != self.nav_state:
            self.nav_state = msg.nav_state
            self.node.get_logger().info(f"Navigation State: {self.nav_state}")
            if self.nav_state == 4:
                self.takeoff_complete = True
                self.node.get_logger().info("Takeoff Complete")
        if msg.arming_state != self.arm_state:
            self.arm_state = msg.arming_state
            self.node.get_logger().info(f"Arm State: {self.arm_state}")
        if msg.failsafe and not self.failsafe:
            self.failsafe = True
            self.node.get_logger().warn("Failsafe triggered!")

    def _attitude_callback(self, msg: VehicleAttitude):
        self.vehicle_attitude = msg

        q = msg.q
        self.yaw = np.arctan2(
            2.0 * (q[3] * q[0] + q[1] * q[2]),
            1.0 - 2.0 * (q[0]**2 + q[1]**2)
        )
        self.node.get_logger().debug(f"Attitude - Yaw: {self.yaw}")

    def _global_position_callback(self, msg: VehicleGlobalPosition):
        self.global_position = msg
        # self.node.get_logger().info(
            # f"Global Position - Lat: {msg.lat:.7f}, Lon: {msg.lon:.7f}, Alt: {msg.alt:.7f}"
        # )
    
    def _vehicle_gps_callback(self, msg: SensorGps):
        self.sensor_gps = msg
    
    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        if not self.start_local_position:
            self.start_local_position = (msg.x, msg.y, msg.z)
            self.start_GPS_ref = (msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.node.get_logger().info(f"Local start position: {self.start_local_position}")
            self.node.get_logger().info(f"GPS start position: {self.start_GPS_ref}")
        self.vehicle_local_position = msg
        
        # self.node.get_logger().info(
        #     f"Local Position - x: {msg.x:.5f}, y: {msg.y:.5f}, z: {msg.z:.5f}"
        # )

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

        self.vehicle_gps_sub = self.node.create_subscription(
            SensorGps,
            '/fmu/out/vehicle_gps_position',
            self._vehicle_gps_callback,
            qos_profile
        )

        self.vehicle_local_position_subscriber = self.node.create_subscription(
            VehicleLocalPosition, '/fmu/out/vehicle_local_position', self._vehicle_local_position_callback, qos_profile)


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
