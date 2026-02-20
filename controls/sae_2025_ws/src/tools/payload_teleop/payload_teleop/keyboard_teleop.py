import threading
import sys
from select import select

import tty
import termios

import rclpy
from rclpy.node import Node
from payload_interfaces.msg import DriveCommand


def get_key(settings, timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select([sys.stdin], [], [], timeout)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key


def save_terminal_settings():
    return termios.tcgetattr(sys.stdin)


def restore_terminal_settings(settings):
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)


class PublishThread(threading.Thread):
    def __init__(self, node, rate):
        super().__init__()
        self.node = node
        self.linear = 0.0
        self.angular = 0.0
        self.condition = threading.Condition()
        self.done = False
        self.timeout = 1.0 / rate if rate != 0.0 else None
        self.daemon = True
        self.start()

    def update(self, linear, angular):
        with self.condition:
            self.linear = linear
            self.angular = angular
            self.condition.notify()

    def stop(self):
        self.done = True
        self.update(0.0, 0.0)
        self.join()

    def run(self):
        while not self.done:
            with self.condition:
                self.condition.wait(self.timeout)
                linear = self.linear
                angular = self.angular

            msg = DriveCommand()
            msg.linear = linear
            msg.angular = angular
            self.node.pub.publish(msg)

        msg = DriveCommand()
        msg.linear = 0.0
        msg.angular = 0.0
        self.node.pub.publish(msg)


class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__("keyboard_teleop")
        self.declare_parameter('topic', '')
        self.topic = self.get_parameter('topic').get_parameter_value().string_value
        if self.topic == '':
            raise RuntimeError(
                "topic parameter must not be an empty string!\n"
                "set with this flag: --ros-args -p topic:=/my/topic/name"
            )

        self.linear = 0.5
        self.angular = 1.0
        self.pub = self.create_publisher(DriveCommand, self.topic, 10)

    def _print_status(self, key, linear, angular):
        G = '\033[92m'
        R = '\033[0m'

        def k(letter):
            return f"{G}{letter}{R}" if letter == key else letter

        sys.stdout.write('\033[2J\033[H')
        sys.stdout.write("\n".join([
            "┌─────────────────────────────────────────────┐",
            "│           Keyboard Teleop  (DriveCommand)    │",
            "├─────────────────────────────────────────────┤",
            f"│  topic : {self.topic:<36s}│",
            f"│  cmd   : lin={linear:+6.2f}  ang={angular:+6.2f}              │",
            "├──────────────┬──────────────────────────────┤",
            "│  Movement    │  Speed Adjust                │",
            f"│  {k('w')}  forward  │  {k('m')} / {k('M')}  linear  +/- 0.1  {self.linear:.2f}│",
            f"│  {k('s')}  reverse  │  {k('n')} / {k('N')}  angular +/- 0.1  {self.angular:.2f}│",
            f"│  {k('a')}  left     │                              │",
            f"│  {k('d')}  right    │  Ctrl-C to quit              │",
            "└──────────────┴──────────────────────────────┘",
        ]) + "\n")
        sys.stdout.flush()


def main():
    settings = save_terminal_settings()
    rclpy.init()
    node = KeyboardTeleop()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    pub_thread = PublishThread(node, rate=10.0)

    linear = 0.0
    angular = 0.0

    try:
        pub_thread.update(linear, angular)

        while True:
            key = get_key(settings, timeout=0.20)

            if key == '\x03':
                break

            if key == 'w':
                linear = node.linear
                angular = 0.0
            elif key == 's':
                linear = -node.linear
                angular = 0.0
            elif key == 'a':
                linear = 0.0
                angular = node.angular
            elif key == 'd':
                linear = 0.0
                angular = -node.angular
            elif key == 'm':
                node.linear = round(node.linear + 0.1, 2)
            elif key == 'M':
                node.linear = round(node.linear - 0.1, 2)
            elif key == 'n':
                node.angular = round(node.angular + 0.1, 2)
            elif key == 'N':
                node.angular = round(node.angular - 0.1, 2)
            else:
                if key == '' and linear == 0.0 and angular == 0.0:
                    node._print_status(key, linear, angular)
                    continue
                linear = 0.0
                angular = 0.0

            pub_thread.update(linear, angular)
            node._print_status(key, linear, angular)

    except Exception as e:
        print(e)

    finally:
        pub_thread.stop()
        restore_terminal_settings(settings)
        rclpy.shutdown()
