#!/usr/bin/env python3
import rclpy
import sys
import os
from sim.SimOrchestrator import SimOrchestrator

def main():
    debug = sys.argv[1]
    yaml_file = sys.argv[2]
    competition = sys.argv[3]
    import shutil
    from pathlib import Path

    src_models_dir = os.path.join(os.getcwd(), "src", "sim", "sim", "world_gen", "models")
    dst_models_dir = os.path.expanduser("~/.simulation-gazebo")

    # Only copy if source exists and it's a directory
    if os.path.exists(src_models_dir) and os.path.isdir(src_models_dir):
        # Create destination directory if it doesn't exist
        os.makedirs(dst_models_dir, exist_ok=True)
        # Copy all content from src_models_dir to dst_models_dir, merging contents
        for item in os.listdir(src_models_dir):
            s = os.path.join(src_models_dir, item)
            d = os.path.join(dst_models_dir, item)
            if os.path.isdir(s):
                # Copytree with dirs_exist_ok=True to allow merging/updating
                shutil.copytree(s, d, dirs_exist_ok=True)
            else:
                shutil.copy2(s, d)

    DEBUG = debug.lower() == 'true'
    rclpy.init()
    sim_node = SimOrchestrator(yaml_file)
    sim_node.spin()
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
