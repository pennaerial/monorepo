#!/usr/bin/env python3
import rclpy
import sys
import os
from uav.ModeManager import ModeManager

def main():
    if len(sys.argv) > 2:
        yaml_file = sys.argv[1]
        vision_nodes = sys.argv[2]
    else:
        cwd = os.getcwd() # default
        yaml_file = f'{cwd}/src/uav/uav/missions/basic_payload_landing.yaml'
        vision_nodes = ''
    rclpy.init()
    mission_node = ModeManager(yaml_file, vision_nodes, DEBUG=False)
    rclpy.spin(mission_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
