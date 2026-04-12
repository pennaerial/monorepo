# `sim` Package

## Extra Dependency
```bash
sudo apt install ros-humble-tf-transformations
python3 -m pip install "pydantic>=2,<3"
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

## Multi-Vehicle Backend Contract
`src/sim/launch/sim.launch.py` now accepts an optional `backend_json:=...` launch
argument. The JSON object selects a canonical stage YAML by `world_name` plus
optional `mission_stage`, then applies any `world_overrides` on top of it.

Recommended shape:

```json
{
  "world_name": "sae",
  "mission_stage": "payload_retreat"
}
```

Notes:
- omitting `mission_stage` resolves `base.yaml`
- the canonical stage YAML should define spawnable props under `world.params.entities` and vehicles under `world.params.controllables`
- UAV controllables should carry an explicit `px4_airframe_id`; for valid values, inspect `PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`
- `world_overrides` should follow the same `entities` / `controllables` split and is still available for incidental stage tweaks, but it should not be used to supply PX4 airframe identity
- legacy embedded `backend.world` input is intentionally no longer supported
- legacy top-level spawnable records are no longer supported; use `world.params.entities`
- legacy single-UAV spawning remains available for older launch paths that still pass `spawn_uav_model:=true`
