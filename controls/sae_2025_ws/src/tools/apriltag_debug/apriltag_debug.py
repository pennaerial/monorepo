from rclpy.node import Node
import rclpy
from sensor_msgs.msg import Image, CompressedImage
import cv2 as cv
import pupil_apriltags

class AprilTagDebugger(Node):
    def __init__(self):
        super().__init__("apriltag_debugger")
        self.declare_parameter("topic", "")
        self.topic = self.get_parameter("topic").get_parameter_value().string_value

    def timer_cb(self):
        pass


def main():
    rclpy.init()
    apriltag_debugger = AprilTagDebugger()
    rclpy.spin(apriltag_debugger)
    rclpy.shutdown

