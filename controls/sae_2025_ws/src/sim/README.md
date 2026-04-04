# `sim` Package

## Extra Dependency
```bash
sudo apt install ros-humble-tf-transformations
```

## Current SAE Sim Launch Behavior
The SAE sim stack is launched through:

```bash
ros2 launch uav main.launch.py
```

Simulation now defaults to launching Gazebo with the GUI enabled.

To force headless mode, set either:

```bash
export SAE_SIM_GUI=0
```

or

```bash
export SAE_SIM_HEADLESS=1
```

before running the launch command.

Notes:
- Gazebo is launched directly with `gz sim`.
- The render engine is forced to `ogre`.
- A workspace-owned Gazebo `server.config` is used.
- GUI startup scrubs OpenCV Qt plugin env vars before launching Gazebo.
