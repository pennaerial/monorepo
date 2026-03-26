from abc import abstractmethod
from rclpy.node import Node
from uav.Vehicle import Vehicle
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleStatus,
    VehicleCommand,
    VehicleAttitude,
    VehicleGlobalPosition,
    VehicleLocalPosition,
    SensorGps,
)
from rclpy.clock import Clock
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)
import numpy as np
from uav.px4_modes import PX4CustomMainMode, PX4CustomSubModeAuto

# Map nav_state value -> name for readable logging
_NAV_STATE_NAMES = {
    getattr(VehicleStatus, a): a
    for a in dir(VehicleStatus)
    if a.startswith("NAVIGATION_STATE_")
    and isinstance(getattr(VehicleStatus, a), (int, float))
}


def get_nav_state_str(val):
    return _NAV_STATE_NAMES.get(val, str(val))


class UAV(Vehicle):
    """
    Abstract base class for UAV control and interfacing with PX4 via ROS 2.
    Subclasses: VTOL, Multicopter
    """

    def __init__(
        self, node: Node, takeoff_amount=5.0, DEBUG=False, camera_offsets=[0, 0, 0]
    ):
        super().__init__(node, DEBUG)
        self.node.get_logger().info(f"Initializing UAV with DEBUG={DEBUG}")

        # PX4/flight-specific state
        self.flight_check = False
        self.emergency_landing = False
        self.failsafe_px4 = False
        self.vehicle_status = None
        self.vehicle_attitude = None
        self.nav_state = None

        self.system_id = 1
        self.component_id = 1

        self.max_acceleration = 0.01
        self.default_velocity = 5.0

        self.camera_offsets = camera_offsets

        # Set up Subscribers/Publishers to communicate with aircraft
        self._initialize_publishers_and_subscribers()

        # set takeoff parameters
        self.origin_set = False
        self.roll = None
        self.pitch = None
        self.takeoff_amount = takeoff_amount
        self.attempted_takeoff = False

        # Initialize drone position
        self.local_origin = None


    # -------------------------
    # Public commands
    # -------------------------
    def arm(self):
        """Send an arm command to the UAV."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            params={"param1": 1.0},  # param1=1 => Arm
        )
        self.node.get_logger().info("Sent Arm Command")

    def set_origin(self):
        if self.global_position and self.yaw:
            lat = self.global_position.lat
            lon = self.global_position.lon
            alt = self.global_position.alt
            self.node.get_logger().info(f"Setting origin to {lat}, {lon}, {alt}")
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_SET_GPS_GLOBAL_ORIGIN,
                params={
                    "param1": 0.0,
                    "param2": 0.0,
                    "param3": 0.0,
                    "param4": 0.0,
                    "param5": lat,
                    "param6": lon,
                    "param7": alt,
                },
            )
            self.camera_offsets = self.uav_to_local(self.camera_offsets)
            self.origin_set = True

    def hover(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            params={
                "param1": 1.0,
                "param2": PX4CustomMainMode.AUTO.value,
                "param3": PX4CustomSubModeAuto.LOITER.value,
            },
        )

    def disarm(self, force=False):
        """Send a disarm command to the UAV."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            params={"param1": 0.0, **({"param2": 21196.0} if force else {})},
        )

        self.node.get_logger().info("Sent Disarm Command")

    def engage_offboard_mode(self):
        """Switch to offboard mode."""
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            params={"param1": 1.0, "param2": PX4CustomMainMode.OFFBOARD.value},
        )
        self.node.get_logger().info("Switching to offboard mode")

    def takeoff(self):
        """
        Command the UAV to take off to the specified altitude.
        This uses a NAV_TAKEOFF command; actual behavior depends on PX4 mode.
        """
        if not self.attempted_takeoff:
            self.attempted_takeoff = True
            lat = self.global_position.lat
            lon = self.global_position.lon
            alt = self.global_position.alt
            takeoff_gps = (lat, lon, alt + self.takeoff_amount)
            self._send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
                params={
                    "param1": 10.0,
                    "param2": 0.0,
                    "param3": 0.0,
                    "param4": 0.0,
                    "param5": takeoff_gps[0],
                    "param6": takeoff_gps[1],
                    "param7": takeoff_gps[2],
                },
            )
            self.node.get_logger().info("Takeoff command sent.")

    def land(self):
        """Command the UAV to land."""
        self._send_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.node.get_logger().info("Landing command sent.")

    def drop_payload(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, params={"param1": -1.0}
        )

    def pickup_payload(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, params={"param1": 1.0}
        )

    def disable_servo(self):
        self._send_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR, params={"param1": 0.0}
        )

    def publish_position_setpoint(self, coordinate, relative=False, lock_yaw=False):
        """Publish the trajectory setpoint.

        Args:
            coordinate (tuple): (x, y, z) in the local frame.
            relative (bool): If True, the position is relative to the current local position.
            lock_yaw (bool): If True, maintain current yaw instead of recalculating (prevents spinning when hovering).
        """
        x, y, z = coordinate
        if relative:
            x += self.local_position.x
            y += self.local_position.y
            z += self.local_position.z

        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)

        # Lock yaw when hovering to prevent spinning from position noise
        if lock_yaw and self.yaw is not None:
            msg.yaw = float(self.yaw)
        else:
            msg.yaw = float(self.calculate_yaw(x, y))

        # Delegate velocity calculation to subclass (VTOL or Multicopter)
        # msg.velocity = self._calculate_velocity((x, y, z), lock_yaw) #TODO: Tune this later but for now let px4 handle

        self.trajectory_publisher.publish(msg)


    @abstractmethod
    def _calculate_velocity(self, target_pos: tuple, lock_yaw: bool) -> list:
        """
        Calculate velocity vector for trajectory setpoint.
        Must be implemented by subclasses (VTOL, Multicopter).

        Args:
            target_pos: (x, y, z) target position in local frame
            lock_yaw: Whether yaw is locked (hovering)

        Returns:
            [vx, vy, vz] velocity list in m/s
        """
        pass

    def publish_offboard_control_heartbeat_signal(self):
        """Publish the offboard control mode."""
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = True  # Enable velocity control for trajectory setpoints
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = int(self.node.get_clock().now().nanoseconds / 1000)
        self.offboard_mode_publisher.publish(msg)

    def uav_to_local(self, point, relative=False):
        """Alias for vehicle_to_local — kept for backwards compatibility."""
        return self.vehicle_to_local(point, relative)

    def _calculate_proportional_velocity(
        self, direction: np.ndarray, distance: float
    ) -> list:
        """
        Calculate velocity using proportional control to prevent oscillation.
        Velocity smoothly decreases as distance to target decreases.

        Args:
            direction (np.ndarray): Unit direction vector [dx, dy, dz]
            distance (float): Distance to target in meters

        Returns:
            list: [vx, vy, vz] velocity vector in m/s
        """
        # Proportional control with lenient thresholds to reduce oscillation
        # Full speed above 10m, proportional between 10m-2m, slow below 2m
        if distance > 10.0:
            target_speed = self.default_velocity
        elif distance > 2.0:
            # Smooth deceleration from 10m to 2m
            # At 10m: 5 m/s, at 2m: 1 m/s
            target_speed = max(1.0, self.default_velocity * (distance / 10.0))
        elif distance > 0.1:
            # Close to target: gentle approach to prevent overshoot
            target_speed = 0.8
        else:
            # Very close: hover
            return [0.0, 0.0, 0.0]

        return [
            float(direction[0] * target_speed),
            float(direction[1] * target_speed),
            float(direction[2] * target_speed),
        ]

    # -------------------------
    # Internal helper methods
    # -------------------------
    def _send_vehicle_command(
        self,
        command: int,
        params: dict = {},
        target_system=None,
        target_component=None,
        source_system=None,
        source_component=None,
        from_external=True,
    ):
        """
        Publish a VehicleCommand message to instruct PX4 to perform an action.
        """
        msg = VehicleCommand()

        # Fill in parameters
        msg.param1 = params.get("param1", float("nan"))
        msg.param2 = params.get("param2", float("nan"))
        msg.param3 = params.get("param3", float("nan"))
        msg.param4 = params.get("param4", float("nan"))
        msg.param5 = params.get("param5", float("nan"))
        msg.param6 = params.get("param6", float("nan"))
        msg.param7 = params.get("param7", float("nan"))

        msg.command = command

        msg.target_system = target_system if target_system else self.system_id
        msg.target_component = (
            target_component if target_component else self.component_id
        )
        msg.source_system = source_system if source_system else self.system_id
        msg.source_component = (
            source_component if source_component else self.component_id
        )
        msg.from_external = from_external
        msg.timestamp = int(Clock().now().nanoseconds / 1000)  # microseconds

        # Publish
        self.vehicle_command_publisher.publish(msg)

    # -------------------------
    # Subscribers / callbacks
    # -------------------------
    def _vehicle_status_callback(self, msg: VehicleStatus):
        self.vehicle_status = msg
        if self.nav_state != msg.nav_state:
            self.node.get_logger().info(
                f"Nav state changed from {get_nav_state_str(self.nav_state)} to {get_nav_state_str(msg.nav_state)}"
            )
        self.nav_state = msg.nav_state
        self.arm_state = msg.arming_state
        self.failsafe_px4 = msg.failsafe
        self.flight_check = msg.pre_flight_checks_pass
        self.system_id = msg.system_id
        self.component_id = msg.component_id
        self.failsafe = self.failsafe_px4 or self.failsafe_trigger
        if self.DEBUG:
            self.node.get_logger().info(
                f"Nav State: {self.nav_state}, Arm State: {self.arm_state}, Failsafe: {self.failsafe_px4}, Flight Check: {self.flight_check}"
            )

    def _attitude_callback(self, msg: VehicleAttitude):
        self.vehicle_attitude = msg

        q = msg.q
        w, x, y, z = q[0], q[1], q[2], q[3]

        self.roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x**2 + y**2))
        self.pitch = np.arcsin(2.0 * (w * y - z * x))
        self.yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y**2 + z**2))

    def _global_position_callback(self, msg: VehicleGlobalPosition):
        self.node.destroy_subscription(
            self.vehicle_gps_sub
        )  # vehicle_gps_sub is available faster but more coarse
        self.global_position = msg

    def _vehicle_gps_callback(self, msg: SensorGps):
        self.global_position = VehicleGlobalPosition()
        self.global_position.lat = msg.latitude_deg
        self.global_position.lon = msg.longitude_deg
        self.global_position.alt = msg.altitude_msl_m

    def _vehicle_local_position_callback(self, msg: VehicleLocalPosition):
        if not self.local_origin:
            self.local_origin = (msg.x, msg.y, msg.z)
            self.gps_origin = (msg.ref_lat, msg.ref_lon, msg.ref_alt)
            self.node.get_logger().info(f"Local start position: {self.local_origin}")
            self.node.get_logger().info(f"GPS start position: {self.gps_origin}")
        self.local_position = msg

    def _initialize_publishers_and_subscribers(self):
        """
        Initialize ROS 2 publishers and subscribers.
        """

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Publishers
        self.offboard_mode_publisher = self.node.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_publisher = self.node.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_command_publisher = self.node.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", 10
        )
        self.target_position_publisher = self.node.create_publisher(
            VehicleLocalPosition, "/fmu/in/target_position", 10
        )

        # Subscribers
        self.status_sub = self.node.create_subscription(
            VehicleStatus,
            "/fmu/out/vehicle_status_v1",
            self._vehicle_status_callback,
            qos_profile,
        )

        self.attitude_sub = self.node.create_subscription(
            VehicleAttitude,
            "/fmu/out/vehicle_attitude",
            self._attitude_callback,
            qos_profile,
        )

        self.global_position_sub = self.node.create_subscription(
            VehicleGlobalPosition,
            "fmu/out/vehicle_global_position",
            self._global_position_callback,
            qos_profile,
        )

        self.vehicle_gps_sub = self.node.create_subscription(
            SensorGps,
            "/fmu/out/vehicle_gps_position",
            self._vehicle_gps_callback,
            qos_profile,
        )

        self.vehicle_local_position_subscriber = self.node.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self._vehicle_local_position_callback,
            qos_profile,
        )
