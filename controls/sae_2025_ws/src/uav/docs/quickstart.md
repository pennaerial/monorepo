# Quickstart

Sim-first onboarding for the UAV framework.

## Prerequisites
- ROS 2 Humble sourced on the machine
- A built workspace
- A PX4-Autopilot checkout, usually at `~/PX4-Autopilot`
- For sim, Gazebo and `MicroXRCEAgent` available on `PATH`

## Build And Source
```bash
source /opt/ros/humble/setup.bash
cd /home/ubuntu/monorepo/controls/sae_2025_ws
colcon build --packages-select sim uav payload
source install/setup.bash
```

## Single Vehicle
Launch the default single-vehicle stack:

```bash
ros2 launch uav main.launch.py
```

Useful overrides on the legacy single-vehicle path:
- `mission_name:=hover`
- `vehicle_name:=uav`
- `px4_path:=~/PX4-Autopilot`
- `params_file:=$(pwd)/src/uav/launch/launch_params_hardware.yaml`

What success looks like:
- Gazebo starts
- the selected mission loads
- the mission node appears under `/<vehicle>/mission`
- UAV missions start PX4 SITL when `sim:=true`
- logs move past startup and the mode manager begins running the mission

## Fleet Launch
Launch a multi-vehicle sim fleet by passing a fleet YAML file:

```bash
ros2 launch uav fleet.launch.py \
  fleet_file:=$(pwd)/src/uav/uav/fleets/example_fleet.yaml
```

What success looks like:
- the sim backend/world starts once
- each managed vehicle starts in its own namespace
- UAV entries attach to backend controllables and start PX4 SITL
- payload entries start payload runtime only
- missions that need vision also bring up camera and vision nodes

## Where Things Live
- Missions: `src/uav/uav/missions/*.yaml`
- Sim stages: `src/sim/sim/simulations/<world>/<stage>.yaml`
- Fleet YAMLs: `src/uav/uav/fleets/*.yaml`

## Notes
- `main.launch.py` is the legacy single-vehicle compatibility path
- `fleet.launch.py` is the entry point for multi-vehicle orchestration
- `vehicle_name` is the namespace/identity used by the runtime
