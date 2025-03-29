import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, ActuatorServos
from rclpy.clock import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
import math

class OscillatoryServoCommandNode(Node):
    def __init__(self):
        super().__init__('oscillatory_servo_command_node')
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos_profile)
        # Timer to update and send servo command at a fixed rate (e.g., every 0.1 seconds)
        self.timer_period = 0.1  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)
        
        # Oscillation parameters
        self.start_time = self.get_clock().now().nanoseconds / 1e9  # in seconds
        self.frequency = 0.25            # oscillation frequency in Hz (0.5 Hz gives a 2-second period)
        self.amplitude = 0.5            # PWM amplitude (µs); oscillates from 1500-500=1000 to 1500+500=2000 µs
        self.center = 0.0            # center PWM value in µs

    def timer_callback(self):
        current_time = self.get_clock().now().nanoseconds / 1e9  # seconds
        elapsed = current_time - self.start_time
        
        # Calculate oscillatory PWM value using sine wave.
        pwm_value = self.center + self.amplitude + math.sin(2 * math.pi * self.frequency * elapsed)
        
        # Build the servo command message.
        msg = VehicleCommand()
        msg.command = VehicleCommand.VEHICLE_CMD_DO_SET_ACTUATOR
        msg.param1 = float(pwm_value)
        msg.param2 = 0.0
        msg.param3 = 0.0
        msg.param4 = 0.0
        msg.param5 = 0.0
        msg.param6 = 0.0
        msg.param7 = 0.0
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        
        self.vehicle_command_pub.publish(msg)
        # msg = ActuatorServos()
        self.get_logger().info(f"Published servo PWM: {pwm_value:.1f} µs")

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

if __name__ == '__main__':
    main()
