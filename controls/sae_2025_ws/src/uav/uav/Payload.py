from rclpy.node import Node
from payload_interfaces.msg import DriveCommand, MotorState
from payload_interfaces.srv import TimedDrive, ComputePidZieglerNichols
from uav.Vehicle import Vehicle


class Payload(Vehicle):
    """
    Python ROS 2 client wrapper for the C++ payload node (ground car).
    Communicates with the payload node via topics and services defined in payload_interfaces.
    """

    def __init__(self, node: Node, payload_name: str = "payload", DEBUG: bool = False):
        super().__init__(node, DEBUG)
        self.payload_name = payload_name
        self.motor_state: MotorState | None = None

        self._drive_pub = node.create_publisher(
            DriveCommand, f"/{payload_name}/cmd_drive", 10
        )
        self._timed_drive_client = node.create_client(
            TimedDrive, f"/{payload_name}/timed_drive"
        )
        self._compute_pid_client = node.create_client(
            ComputePidZieglerNichols, f"/{payload_name}/compute_pid"
        )
        self._motor_state_sub = node.create_subscription(
            MotorState,
            f"/{payload_name}/motor_state",
            self._motor_state_callback,
            10,
        )

    @property
    def vehicle_type(self) -> str:
        return "PAYLOAD"

    def drive(self, linear: float, angular: float):
        """Publish a continuous drive command. linear: m/s, angular: rad/s (+left, -right)."""
        msg = DriveCommand()
        msg.linear = linear
        msg.angular = angular
        self._drive_pub.publish(msg)

    def timed_drive(self, linear: float, angular: float, duration_sec: float):
        """Call the timed_drive service — drives for duration_sec then auto-stops. Returns a Future."""
        req = TimedDrive.Request()
        req.linear = linear
        req.angular = angular
        req.duration_sec = duration_sec
        return self._timed_drive_client.call_async(req)

    def stop(self):
        """Stop the ground vehicle."""
        self.drive(0.0, 0.0)

    def compute_pid(self, ku: float, pu: float):
        """Call the PID tuning service (Ziegler-Nichols). Returns a Future."""
        req = ComputePidZieglerNichols.Request()
        req.ku = ku
        req.pu = pu
        return self._compute_pid_client.call_async(req)

    def _motor_state_callback(self, msg: MotorState):
        self.motor_state = msg
