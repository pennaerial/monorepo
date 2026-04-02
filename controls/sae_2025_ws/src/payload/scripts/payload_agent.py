#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node

from payload_interfaces.msg import AgentComm, DriveCommand, PayloadMode
from payload_interfaces.srv import PayloadCommand

MODE_NAMES = {
    PayloadMode.IDLE: 'IDLE',
    PayloadMode.DRIVING_TO_PLANE: 'DRIVING_TO_PLANE',
    PayloadMode.NEAR_PLANE: 'NEAR_PLANE',
    PayloadMode.READY_FOR_PICKUP: 'READY_FOR_PICKUP',
    PayloadMode.HOOKED: 'HOOKED',
    PayloadMode.UNHOOKING: 'UNHOOKING',
    PayloadMode.DRIVING_OFF: 'DRIVING_OFF',
    PayloadMode.CLEAR: 'CLEAR',
}


class PayloadAgent(Node):
    def __init__(self):
        super().__init__('payload_agent')

        self.declare_parameter('payload_name', 'payload_0')
        self.declare_parameter('role', 'incoming')
        self.declare_parameter('approach_speed', 0.15)
        self.declare_parameter('drive_off_speed', 0.15)
        self.declare_parameter('drive_off_duration', 3.0)

        self.payload_name = self.get_parameter('payload_name').value
        self.role = self.get_parameter('role').value
        self.approach_speed = self.get_parameter('approach_speed').value
        self.drive_off_speed = self.get_parameter('drive_off_speed').value
        self.drive_off_duration = self.get_parameter('drive_off_duration').value

        if self.role == 'incoming':
            self.mode = PayloadMode.IDLE
        elif self.role == 'hooked':
            self.mode = PayloadMode.HOOKED
        else:
            raise ValueError(f"Unknown role '{self.role}'. Use 'incoming' or 'hooked'.")

        self.drive_pub = self.create_publisher(
            DriveCommand, f'/{self.payload_name}/cmd_drive', 10)
        self.mode_pub = self.create_publisher(PayloadMode, '/payload/mode', 10)
        self.comm_pub = self.create_publisher(AgentComm, '/payload/agent_comm', 10)

        self.comm_sub = self.create_subscription(
            AgentComm, '/payload/agent_comm', self._on_agent_comm, 10)

        if self.role == 'incoming':
            self.command_srv = self.create_service(
                PayloadCommand,
                f'/{self.payload_name}/agent/command',
                self._on_command)

        self.mode_timer = self.create_timer(0.5, self._publish_mode)
        self.drive_off_timer = None

        self.get_logger().info(
            f'[{self.payload_name}] Agent ready  role={self.role}  mode={self._mode_name()}')

    # -- shared helpers ---------------------------------------------------

    def _mode_name(self):
        return MODE_NAMES.get(self.mode, f'UNKNOWN({self.mode})')

    def _publish_mode(self):
        msg = PayloadMode()
        msg.agent_name = self.payload_name
        msg.mode = self.mode
        self.mode_pub.publish(msg)

    def _send_drive(self, linear, angular):
        cmd = DriveCommand()
        cmd.linear = float(linear)
        cmd.angular = float(angular)
        self.drive_pub.publish(cmd)

    def _send_comm(self, message):
        msg = AgentComm()
        msg.sender_name = self.payload_name
        msg.message = message
        msg.timestamp = time.time()
        self.comm_pub.publish(msg)
        self.get_logger().info(f'AgentComm >> "{message}"')

    # -- agent_comm listener (both roles) ---------------------------------

    def _on_agent_comm(self, msg: AgentComm):
        if msg.sender_name == self.payload_name:
            return

        self.get_logger().info(f'AgentComm << [{msg.sender_name}] "{msg.message}"')

        if self.role == 'incoming':
            if msg.message == 'CLEAR' and self.mode == PayloadMode.NEAR_PLANE:
                self.mode = PayloadMode.READY_FOR_PICKUP
                self.get_logger().info(f'[{self.payload_name}] -> READY_FOR_PICKUP')

        elif self.role == 'hooked':
            if msg.message == 'UNHOOK' and self.mode == PayloadMode.HOOKED:
                self._begin_unhook()

    # -- incoming: operator command service --------------------------------

    def _on_command(self, request, response):
        cmd = request.command.strip().lower()
        self.get_logger().info(f'Command: "{cmd}"')

        if cmd == 'start':
            return self._cmd_start(response)
        elif cmd == 'near_plane':
            return self._cmd_near_plane(response)
        else:
            response.success = False
            response.message = f"Unknown command '{cmd}'. Valid: start, near_plane"
            return response

    def _cmd_start(self, response):
        if self.mode != PayloadMode.IDLE:
            response.success = False
            response.message = f'Must be IDLE, currently {self._mode_name()}'
            return response

        self.mode = PayloadMode.DRIVING_TO_PLANE
        self._send_drive(self.approach_speed, 0.0)
        self.get_logger().info(f'[{self.payload_name}] -> DRIVING_TO_PLANE')

        response.success = True
        response.message = 'DRIVING_TO_PLANE'
        return response

    def _cmd_near_plane(self, response):
        if self.mode != PayloadMode.DRIVING_TO_PLANE:
            response.success = False
            response.message = f'Must be DRIVING_TO_PLANE, currently {self._mode_name()}'
            return response

        self._send_drive(0.0, 0.0)
        self.mode = PayloadMode.NEAR_PLANE
        self._send_comm('UNHOOK')
        self.get_logger().info(f'[{self.payload_name}] -> NEAR_PLANE, sent UNHOOK')

        response.success = True
        response.message = 'NEAR_PLANE, UNHOOK sent to hooked payload'
        return response

    # -- hooked: auto unhook and drive off --------------------------------

    def _begin_unhook(self):
        self.mode = PayloadMode.UNHOOKING
        self.get_logger().info(f'[{self.payload_name}] -> UNHOOKING')

        self.mode = PayloadMode.DRIVING_OFF
        self._send_drive(self.drive_off_speed, 0.0)
        self.get_logger().info(f'[{self.payload_name}] -> DRIVING_OFF')

        self.drive_off_timer = self.create_timer(
            self.drive_off_duration, self._finish_drive_off)

    def _finish_drive_off(self):
        if self.drive_off_timer:
            self.drive_off_timer.cancel()
            self.drive_off_timer = None

        self._send_drive(0.0, 0.0)
        self.mode = PayloadMode.CLEAR
        self._send_comm('CLEAR')
        self.get_logger().info(f'[{self.payload_name}] -> CLEAR, sent CLEAR')


def main(args=None):
    rclpy.init(args=args)
    node = PayloadAgent()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
