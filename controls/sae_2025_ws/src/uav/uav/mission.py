#!/usr/bin/env python3
import rclpy
from uav.ModeManager import ModeManager
import os

def main():
    rclpy.init()
    # Instantiate ModeManager with the sim_test.yaml configuration file.
    cwd = os.getcwd()
    mission_node = ModeManager(f'{cwd}/src/uav/uav/missions/sim_test.yaml')
    mission_node.spin()
if __name__ == '__main__':
    main()
