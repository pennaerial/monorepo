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
- PX4 Autopilot — used to be external install, but now vendored as a git submodule, so there is no separate install; it is pulled by the submodule step below
- QGroundControl.

> [!NOTE]
> **Updating from Ubuntu 22.04**
> If you're still on Ubuntu 22.04, we've now updated our stack to 24.04 and updated to ros `jazzy`. Follow the instructions in [ubuntu-update.md](/docs/ubuntu-update.md) for updating to 24.04.

You can refer to https://freedcamp.com/view/3502859/tasks/panel/task/61666972 for this process.

Next, make sure to update your system and install necessary dependencies:

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

2. Clone the necessary submodules (this pulls `PX4-Autopilot` into the repo root):

    ```bash
    git submodule update --init --recursive
    ```


## Setting Up ROS 2 Environment

1. Add the ROS 2 Humble setup script to your `~/.bashrc` to automatically source it. YOU ONLY EVER NEED TO DO THIS ONCE:

    ```bash
    echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
    ```
    For context, `>>` pipes the output of the preceding command into the succeeding file. Running the above command multiple times will just paste in `source /opt/...` multiple times into `~/.bashrc`. This setup script sets up your shell instance to recognize ROS2.

2. If you're building `ros_gz`, set the Gazebo version to `harmonic`:

    ```bash
    export GZ_VERSION=harmonic
    ```
    This workspace targets Gazebo Harmonic. If `GZ_VERSION` is unset, Gazebo-related packages will warn and continue with `harmonic`. If it is set to a conflicting value, the build will fail fast.

3. Make sure you have all of your ROS dependencies installed:
   ```bash
   # From the workspace (this) directory
   rosdep install -r --from-paths src -i -y --rosdistro jazzy
   ```
   `rosdep` on Ubuntu 22.04/Jammy does not provide a Pydantic v2 package, so install that separately in the Python environment you use for `uav`:
   ```bash
   python3 -m pip install "pydantic>=2,<3"
   ```

---

## Building the Workspace

1. Build the workspace using `colcon`:

    ```bash
    cd ~/{path_to_monorepo}/controls/sae_2025_ws
    export GZ_VERSION=harmonic
    colcon build
    ```

2. Install the runtime Python dependencies used by the UAV stack:

    ```bash
    sudo apt-get update
    sudo apt install ros-jazzy-cv-bridge python3-opencv python3-pip build-essential cmake
    python3 -m pip install "pydantic>=2,<3"
    ```
    `uav` mission loading requires `pydantic>=2,<3`. Use the distro `python3-opencv` package for `cv2`.

3. We now install `ros-gz` separately. To install:
    ```bash
    sudo sh -c 'echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2-latest.list'
    curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -
    sudo apt-get update
    sudo apt install ros-jazzy-ros-gz
    ```

    Original instructions from [gazebosim/ros_gz](https://github.com/gazebosim/ros_gz/tree/jazzy).

4. If you are running payload hardware on a Raspberry Pi, also complete the one-time `pigpio` / `pigpiod` setup in [src/payload/README.md](src/payload/README.md). The payload GPIO controller will not start unless `pigpiod` is running.
---

## Solving Common Issues

You might run into the following issues during the build process. Here are solutions:

1. **Missing `libgflags-dev`**:
   
    ```bash
    sudo apt-get install libgflags-dev
    ```

2. **Missing `gps_msgs`**:
   
    ```bash
    sudo apt-get install ros-jazzy-gps-msgs
    ```

3. **Missing `vision_msgs`**:

    ```bash
    sudo apt-get install vision-msgs
    ```

4. **Mac Specific: Gazebo Crashing**: If Gazebo is crashing updating from `PX4-Autopilot` to `1.17`

    Navigate to the PX4 installation and add `--render-engine=ogre` to `simulation-gazebo` (should be line 97). The result should be:
    ```python
    cmd += f"gz sim --render-engine=ogre -r {args.model_store}/worlds/{args.world}.sdf"
    ```

---

## QGroundControl Setup

1. Open QGroundControl.

2. If you haven't set up the joysticks, click on the **Q** in the top left of QGroundControl, go to **Application Settings**, and enable **Virtual Joysticks** under the **General** tab.

3. You can now control the aircraft using virtual joysticks!

---

## Troubleshooting

- **Make Sure ROS 2 is Sourced**: If something isn't working, ensure that you have sourced the ROS 2 workspace:

    ```bash
    source install/setup.bash
    ```

- **Dependencies**: Double-check that all necessary dependencies are installed and that the workspace has been successfully built with `colcon`.

---
