# Quickstart

Sim-first onboarding for the UAV framework.

## Prerequisites
- ROS 2 sourced on the machine
- A built workspace
- The `PX4-Autopilot` submodule initialized (`git submodule update --init` at the repo root)
- For sim, Gazebo and `MicroXRCEAgent` available on `PATH`
- `pydantic>=2,<3` installed in the Python environment that will run `uav`

## Build And Source
```bash
source /opt/ros/jazzy/setup.bash
colcon build --packages-select sim uav payload
source install/setup.bash
```

If your environment is missing the UAV runtime dependency, install it before launch:

```bash
python3 -m pip install "pydantic>=2,<3"
```
