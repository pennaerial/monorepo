# How to setup and run Gazebo and PX4 sim in standalone mode with ROS2 connection

This ReadMe documents the procedure for running the Gazebo simulator with a custom world in standalone mode alongside an instance of the PX4 SITL simulator, using ROS2 to send control data to PX4.

## Prerequisites
- A machine running Ubuntu 22.04 or higher
- PX4-Autopilot installation: https://docs.px4.io/main/en/dev_setup/dev_env.html
- Gazebo installation: https://gazebosim.org/docs/all/getstarted/
- ROS2 installation: 

## Setup PX4 and Gazebo for standalone mode
Verify the directory ~/.simulation-gazebo exists.

FROM THIS DIRECTORY, copy the custom models and worlds into the directory above.

```bash
cd {path_to_monorepo}/sim/sae\ aero/gazebo\ harmonic/
cp -R models/* ~/.simulation-gazebo/models
cp -R worlds/* ~/.simulation-gazebo/worlds
```

Copy the scripts standalone_gazebo_cmd.sh and standalone_px4_cmd.sh into the PX4-Autopilot repo:

```bash
cp standalone_gazebo_cmd.sh /{path_to_px4_repo}/PX4-Autopilot/
cp standalone_px4_cmd.sh /{path_to_px4_repo}/PX4-Autopilot/
```

## Run the startup scripts
Start QGroundControl

Navigate to the location of the PX4-Autopilot repo.
```bash
bash standalone_gazebo_cmd.sh
bash standalone_px4_cmd.sh
```

Establish the middleware connection:

```bash
MicroXRCEAgent udp4 -p 8888
```

Run the ROS control script:

```bash
cd {ros_workspace}
colcon build
source install/setup.bash
ros2 launch {package_name} {launch_file}
```

