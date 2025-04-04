#!/usr/bin/env python3
import rclpy
import sys
import os
from uav.ModeManager import ModeManager

def main():
    if len(sys.argv) > 3:
        debug = sys.argv[1]
        yaml_file = sys.argv[2]
        #camera_offsets = sys.argv[3]
        try:
            camera_offsets = [float(offset.strip()) for offset in sys.argv[3].split(',')]
        except Exception as e:
            print(f"Error parsing camera_offsets: {e}")
            camera_offsets = [0, 0, 0]
        vision_nodes = sys.argv[4]
    else:
        cwd = os.getcwd() # default
        yaml_file = f'{cwd}/src/uav/uav/missions/basic.yaml'
        vision_nodes = ''
        debug = 'false'
        camera_offsets = [0, 0, 0]
    rclpy.init()
    mission_node = ModeManager(yaml_file, vision_nodes, camera_offsets, DEBUG=debug.lower() == 'true')
    rclpy.spin(mission_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
