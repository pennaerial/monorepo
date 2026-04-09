from rclpy.node import Node

from payload_interfaces.msg import DriveCommand, ServoCommand
from payload_interfaces.srv import TimedDrive

from .Vehicle import Vehicle


class Payload(Vehicle):
    """Mission-side adapter for one payload node namespace."""

    def __init__(self, node: Node, vehicle_name: str):
        namespace = f"/{vehicle_name}"
        super().__init__(
            node,
            vehicle_name,
            has_camera=True,
            camera_namespace=namespace,
            image_topic=f"{namespace}/camera",
            camera_info_topic=f"{namespace}/camera_info",
            camera_service_name=f"{namespace}/camera_data",
        )

        self.drive_publisher = self.node.create_publisher(
            DriveCommand, self.namespaced_path("cmd_drive", namespace=namespace), 10
        )
        self.servo_publisher = self.node.create_publisher(
            ServoCommand, self.namespaced_path("servo", namespace=namespace), 10
        )
        self.timed_drive_client = self.node.create_client(
            TimedDrive, self.namespaced_path("timed_drive", namespace=namespace)
        )

    def drive(self, linear: float, angular: float) -> None:
        self.drive_publisher.publish(
            DriveCommand(linear=float(linear), angular=float(angular))
        )

    def stop(self) -> None:
        self.drive(0.0, 0.0)

    def set_servo(self, degree: float) -> None:
        self.servo_publisher.publish(ServoCommand(degree=float(degree)))

    def timed_drive(self, linear: float, angular: float, duration_sec: float):
        request = TimedDrive.Request()
        request.linear = float(linear)
        request.angular = float(angular)
        request.duration_sec = float(duration_sec)
        return self.timed_drive_client.call_async(request)
