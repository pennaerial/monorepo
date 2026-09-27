# `sim` Package

Gazebo Harmonic simulation backend — world generation, multi-vehicle spawning, and stage configuration.

## Extra Dependency
```bash
sudo apt install ros-jazzy-tf-transformations
python3 -m pip install "pydantic>=2,<3"
```

## Current SAE Sim Launch Behavior
Simulation now defaults to launching Gazebo with the GUI enabled.

Notes:
- Gazebo is launched directly with `gz sim`.
- The render engine is forced to `ogre`.
- A workspace-owned Gazebo `server.config` is used.
- GUI startup scrubs OpenCV Qt plugin env vars before launching Gazebo.
