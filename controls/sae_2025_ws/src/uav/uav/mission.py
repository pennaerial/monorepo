#!/usr/bin/env python3
import rclpy
import sys
import os
from uav.utils import Vehicle
from uav.ModeManager import ModeManager


def main():
    if len(sys.argv) > 3:
        debug = sys.argv[1]
        yaml_file = sys.argv[2]
        servo_only = sys.argv[3]
        try:
            camera_offsets = [
                float(offset.strip()) for offset in sys.argv[4].split(",")
            ]
        except Exception as e:
            print(f"Error parsing camera_offsets: {e}")
            camera_offsets = [0, 0, 0]
        vehicle_class = Vehicle[sys.argv[5].upper()]
        vision_nodes = sys.argv[6]
    else:
        cwd = os.getcwd()  # default
        yaml_file = f"{cwd}/src/uav/uav/missions/basic.yaml"
        vision_nodes = ""
        debug = "false"
        camera_offsets = [0, 0, 0]
        servo_only = "false"
        vehicle_class = Vehicle.MULTICOPTER
    servo_only = servo_only.lower() == "true"
    DEBUG = debug.lower() == "true"
    rclpy.init()

    mission_node = ModeManager(
        yaml_file,
        vision_nodes,
        camera_offsets,
        DEBUG=DEBUG,
        servo_only=servo_only,
        vehicle_class=vehicle_class,
    )
    rclpy.spin(mission_node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
