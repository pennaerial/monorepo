#!/usr/bin/env python3
import rclpy
import sys
import os
from uav.ModeManager import ModeManager

def main():
    if len(sys.argv) > 3:
        debug = sys.argv[1]
        yaml_file = sys.argv[2]
        vision_nodes = sys.argv[3]
    else:
        cwd = os.getcwd() # default
        yaml_file = f'{cwd}/src/uav/uav/missions/basic.yaml'
        vision_nodes = ''
        debug = 'false'
    rclpy.init()
    mission_node = ModeManager(yaml_file, vision_nodes, DEBUG=debug.lower() == 'true')
    rclpy.spin(mission_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
