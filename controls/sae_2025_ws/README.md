recursively clone submodules
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
if building ros_gz_bridge for the first time, also set export GZ_VERSION=harmonic

for OPENCV:
sudo apt-get update
sudo apt install ros-humble-cv-bridge
sudo apt-get install python3-opencv
pip install opencv-python

colcon build

might get issues: apt install vision-msgs, clone random git repo

source install/setup.bash

ros2 launch uav launch.py

