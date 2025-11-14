#!/usr/bin/env python3
import rclpy
import sys
import os
from sim.SimOrchestrator import SimOrchestrator

def main():
    if len(sys.argv) > 3:
        debug = sys.argv[1]
        yaml_file = sys.argv[2]
        servo_only = sys.argv[3]
        try:
            camera_offsets = [float(offset.strip()) for offset in sys.argv[4].split(',')]
        except Exception as e:
            print(f"Error parsing camera_offsets: {e}")
            camera_offsets = [0, 0, 0]
        vision_nodes = sys.argv[5]
    else:
        cwd = os.getcwd() # default
        yaml_file = f'{cwd}/src/uav/uav/missions/basic.yaml'
        vision_nodes = ''
        debug = 'false'
        camera_offsets = [0, 0, 0]
        servo_only = 'false'
    servo_only = servo_only.lower() == 'true'
    DEBUG = debug.lower() == 'true'
    rclpy.init()
    sim_node = SimOrchestrator(yaml_file)
    rclpy.spin(sim_node)
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
