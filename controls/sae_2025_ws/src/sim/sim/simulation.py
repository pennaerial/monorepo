#!/usr/bin/env python3
import rclpy
import sys
import os
from sim.SimOrchestrator import SimOrchestrator
import shutil
from pathlib import Path

def main():
    competition = sys.argv[1]
    competition_course = sys.argv[2]
    use_scoring = sys.argv[3]

    src_models_dir = os.path.join(os.getcwd(), "src", "sim", "sim", "world_gen", "models")
    # Ensure output directory exists
    os.makedirs(os.path.dirname(os.path.expanduser(f"~/.simulation-gazebo/worlds/{competition}.sdf")), exist_ok=True)

    dst_models_dir = os.path.expanduser("~/.simulation-gazebo/models")

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

    rclpy.init()
    sim_node = SimOrchestrator(competition, competition_course, use_scoring)
    sim_node.spin()
    sim_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
