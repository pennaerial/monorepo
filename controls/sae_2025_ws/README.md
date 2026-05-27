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

2. Clone the necessary submodules:

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

2. Install the runtime Python dependencies used by the UAV / vision stack:

    ```bash
    sudo apt-get update
    sudo apt install ros-jazzy-cv-bridge python3-opencv python3-pip build-essential cmake
    python3 -m pip install "pydantic>=2,<3" apriltag
    ```
    `uav` mission and fleet loading now require `pydantic>=2,<3`, and AprilTag missions still require the Python `apriltag` package. Use the distro `python3-opencv` package for `cv2`; do not install `opencv-python` just to get AprilTag support.

3. If you are running payload hardware on a Raspberry Pi, also complete the one-time `pigpio` / `pigpiod` setup in [src/payload/README.md](src/payload/README.md). The payload GPIO controller will not start unless `pigpiod` is running.
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

4. **Missing `actuator_msgs`**: If you encounter issues with `Findactuator_msgs.cmake`, clone the repository into `src`:

    ```bash
    cd ~/{path_to_monorepo}/controls/sae_2025_ws/src
    git clone git@github.com:rudislabs/actuator_msgs.git
    ```

5. **Mac Specific: Gazebo Crashing**: If Gazebo is crashing updating from `PX4-Autopilot` to `1.17`

    Navigate to the PX4 installation and add `--render-engine=ogre` to `simulation-gazebo` (should be line 97). The result should be:
    ```python
    cmd += f'gz sim --render-engine=ogre -r {args.model_store}/worlds/{args.world}.sdf'
    ```

---

## Launching Components

### Recommended Unified Launch
The current entry point is `ros2 launch uav main.launch.py`. This launch file brings up the selected mission, Gazebo, PX4 SITL, the camera bridge, and the relevant vision / payload nodes.

```bash
source /opt/ros/jazzy/setup.bash
export GZ_VERSION=harmonic
cd ~/{path_to_monorepo}/controls/sae_2025_ws
colcon build --packages-select payload sim uav --symlink-install
source install/setup.bash
ros2 launch uav main.launch.py
```

The default launch behavior in simulation is now GUI-on. To run headless instead, use either of these:

```bash
export SAE_SIM_GUI=0
ros2 launch uav main.launch.py
```

```bash
export SAE_SIM_HEADLESS=1
ros2 launch uav main.launch.py
```

Useful launch overrides:

- `mission_name:=...` overrides the mission configured in `src/uav/launch/launch_params.yaml`
- `payload_name:=...` selects which payload entity a payload mission binds to
- `px4_path:=...` points the launch at a non-default PX4 checkout

Example:

```bash
ros2 launch uav main.launch.py mission_name:=payload_retreat payload_name:=payload_0
```

For hardware payload missions, make sure `pigpiod` is already running or enabled as a service before launching.

### Legacy Manual Launch
The older separate-terminal workflow below is kept for reference, but it is no longer the recommended path for SAE sim bringup.

0. Follow Setup instructions [here](../../sim/sae%20aero/gazebo%20harmonic/README.md). This should involve copying over our custom setup scripts into your local `PX4-Autopilot` folder.
1. **From PX4-Autopilot**: 

   Launch PX4 in standalone mode (this spawns the simulator drone):

   ```bash
   bash standalone_px4_cmd.sh
   ```

2. **From PX4-Autopilot**: 

   Launch Gazebo:

   ```bash
   bash standalone_gazebo_cmd.sh
   ```

3. **From Anywhere**:

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

   Source the workspace setup script (this ensure that ros2 has the most recently-built versions of our workspace packages):

   ```bash
   source install/setup.bash
   ```

   Bridge the camera feed:

   ```bash
   ros2 run ros_gz_bridge parameter_bridge /camera@sensor_msgs/msg/Image[gz.msgs.Image
   ```
   For information about the `ros_gz_bridge` package and its `parameter_bridge` executable, see https://gazebosim.org/docs/harmonic/ros2_integration/.

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
