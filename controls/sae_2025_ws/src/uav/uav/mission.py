#!/usr/bin/env python3
import rclpy
import sys
import os
# from uav.ModeManager import ModeManager
from uav.iarc_ModeManager import ModeManager
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
        safe_test = sys.argv[6] if len(sys.argv) > 6 else 'false'
        vehicle_id = sys.argv[7] if len(sys.argv) > 7 else '0'
    else:
        cwd = os.getcwd() # default
        yaml_file = f'{cwd}/src/uav/uav/missions/basic.yaml'
        vision_nodes = ''
        debug = 'false'
        camera_offsets = [0, 0, 0]
        servo_only = 'false'
        safe_test = 'false'
        vehicle_id = '0'
    servo_only = servo_only.lower() == 'true'
    safe_test = safe_test.lower() == 'true'
    try:
        vehicle_id_int = int(vehicle_id)
    except Exception:
        vehicle_id_int = 0
    DEBUG = debug.lower() == 'true'
    rclpy.init()
    mission_node = ModeManager(yaml_file, vision_nodes, camera_offsets, DEBUG=DEBUG, servo_only=servo_only, safe_test=safe_test, vehicle_id=vehicle_id_int)
    rclpy.spin(mission_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
