import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand
from rclpy.qos import (
    QoSProfile,
    QoSReliabilityPolicy,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
)


class OscillatoryServoCommandNode(Node):
    def __init__(self):
        super().__init__("oscillatory_servo_command_node")
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos_profile
        )
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.timer1 = 0
        self.lower_time = 0.5
        self.lowering = True
        self.last_time = self.get_clock().now().nanoseconds / 1e9  # in seconds

    def timer_callback(self):
        msg = VehicleCommand()
        current_time = self.get_clock().now().nanoseconds / 1e9  # seconds
        time_delta = current_time - self.last_time
        self.last_time = current_time
        if self.timer1 < self.lower_time and self.lowering:
            self.get_logger().info(f"Dropping: time passed {self.timer1}")
            msg.param1 = -1.0
            self.timer1 += time_delta
        elif self.timer1 >= 0:
            self.get_logger().info(f"Dropping: time passed {self.timer1}")
            self.lowering = False
            msg.param1 = 1.0
            self.timer1 -= time_delta
        elif self.timer1 < 0:
            self.get_logger().info("Done")
            msg.param1 = 0.0
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        self.vehicle_command_pub.publish(msg)
        # self.get_logger().info(f"Current time_delta: {time_delta:.5f}")


def main(args=None):
    rclpy.init(args=args)
    node = OscillatoryServoCommandNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
