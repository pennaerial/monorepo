# SAE 2025 ROS 2 Setup Guide

This guide will walk you through the setup process for the SAE 2025 project using ROS 2. The setup includes cloning necessary repositories, building dependencies, and launching various components to display the drone's camera feed and control the drone via QGroundControl.

## Table of Contents
- [Prerequisites](#prerequisites)
- [Cloning Repositories](#cloning-repositories)
- [Setting Up ROS 2 Environment](#setting-up-ros-2-environment)
- [Building the Workspace](#building-the-workspace)
- [Solving Common Issues](#solving-common-issues)
- [Launching Components](#launching-components)
- [QGroundControl Setup](#qgroundcontrol-setup)

---

## Prerequisites

Before you begin, make sure you have the following installed:
- ROS 2 Humble (follow the installation guide for your OS)
- Gazebo Harmonic
- PX4 Autopilot
- QGroundControl

Make sure to update your system and install necessary dependencies:

```bash
sudo apt-get update
sudo apt-get upgrade
```

---

## Cloning Repositories

1. Ensure you are up to date on the monorepo

    ```bash
    cd ~/{path_to_monorepo}
    git pull
    ```

2. Clone the necessary repositories and submodules into sae_2025_ws:

    ```bash
    cd ./controls/sae_2025_ws/src/ros_gz 
    git clone --recursive https://github.com/gazebosim/ros_gz.git
    ```


## Setting Up ROS 2 Environment

1. Add the ROS 2 Humble setup script to your `~/.bashrc` to automatically source it. YOU ONLY EVER NEED TO DO THIS ONCE:

    ```bash
    echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
    ```

2. If you're building `ros_gz_bridge` for the first time, set the Gazebo version to `harmonic`:

    ```bash
    export GZ_VERSION=harmonic
    ```

---

## Building the Workspace

1. Build the workspace using `colcon`:

    ```bash
    cd ~/{path_to_monorepo}/controls/sae_2025_ws
    colcon build
    ```

2. If you're encountering issues with OpenCV or NumPy version conflicts, follow these steps:

    ```bash
    sudo apt-get update
    sudo apt install ros-humble-cv-bridge
    sudo apt-get install python3-opencv
    pip install opencv-python
    ```
    You may also need to uninstall your previous OpenCV.
---

## Solving Common Issues

You might run into the following issues during the build process. Here are solutions:

1. **Missing `libgflags-dev`**:
   
    ```bash
    sudo apt-get install libgflags-dev
    ```

2. **Missing `gps_msgs`**:
   
    ```bash
    sudo apt-get install ros-humble-gps-msgs
    ```

3. **Missing `vision_msgs`**:

    ```bash
    sudo apt-get install vision-msgs
    ```

4. **Missing `actuator_msgs`**: If you encounter issues with `Findactuator_msgs.cmake`, clone the repository into `src`:

    ```bash
    cd ~/{path_to_monorepo}/controls/sae_2025_ws/src
    git clone git@github.com:rudislabs/actuator_msgs.git
    ```

---

## Launching Components

### Launch Order (All in Separate Terminals)

1. **From PX4-Autopilot**: 

    Launch PX4 in standalone mode:

    ```bash
    bash standalone_px4_cmd.sh
    ```

2. **From PX4-Autopilot**: 

    Launch Gazebo:

    ```bash
    bash standalone_gazebo_cmd.sh
    ```

3. **From PX4-Autopilot**:

    Start Micro XRCE Agent for communication:

    ```bash
    MicroXRCEAgent udp4 -p 8888
    ```

4. **From Anywhere**: 

    Check ROS 2 topics to ensure that you see a long list of topics, not just the default ones:

    ```bash
    ros2 topic list
    ```

5. **From Where You Have QGroundControl Installed**:

    Launch QGroundControl:

    ```bash
    ./QGroundControl.AppImage
    ```

6. **From `sae_2025_ws`**:

    Source the workspace setup script:

    ```bash
    source install/setup.bash
    ```

    Start the ROS 2 parameter bridge:

    ```bash
    ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image@gz.msgs.Image
    ```

7. **From Anywhere**: 

    Ensure the `/camera` topic is available:

    ```bash
    ros2 topic list
    ```

    Ensure the `/camera` topic is publishing correctly:

    ```bash
    ros2 topic echo /camera
    ```

    You should see a bunch of numbers between 0 and 255, representing the image data.

8. **From `sae_2025_ws`**:

    Launch the UAV node:

    ```bash
    ros2 launch uav launch.py
    ```

---

## QGroundControl Setup

1. Now, you should have a tab displaying the drone camera feed. Open QGroundControl.

2. If you haven't set up the joysticks, click on the **Q** in the top left of QGroundControl, go to **Application Settings**, and enable **Virtual Joysticks** under the **General** tab.

3. You can now control the aircraft using virtual joysticks and see the camera feed as the drone flies around!

---

## Troubleshooting

- **Make Sure ROS 2 is Sourced**: If something isn't working, ensure that you have sourced the ROS 2 workspace:

    ```bash
    source install/setup.bash
    ```

- **Dependencies**: Double-check that all necessary dependencies are installed and that the workspace has been successfully built with `colcon`.

---

That's it! You should now have your drone camera feed up and running, and be able to control the drone through QGroundControl.
