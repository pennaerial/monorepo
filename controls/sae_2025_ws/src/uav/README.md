# UAV Package

This package owns mission launch, mission runtime, vehicle wrappers, camera wiring, and vision-node orchestration for the SAE autonomy stack.

## Start Here

Use one of these launch paths:

| Use case | Entry point | Main config |
| --- | --- | --- |
| Single vehicle, local dev, legacy-compatible bringup | `ros2 launch uav main.launch.py` | `src/uav/launch/launch_params.yaml` or `src/uav/launch/launch_params_hardware.yaml` |
| Multi-vehicle sim orchestration | `ros2 launch uav fleet.launch.py fleet_file:=...` | `src/uav/uav/fleets/*.yaml` |

Common commands:

```bash
# Single-vehicle sim
ros2 launch uav main.launch.py

# Fleet sim
ros2 launch uav fleet.launch.py \
  fleet_file:=$(pwd)/src/uav/uav/fleets/example_fleet.yaml
```

## Package Map

| Path | What it is |
| --- | --- |
| `src/uav/launch/` | Launch entrypoints and single-vehicle launch params |
| `src/uav/uav/missions/` | Bundled mission YAMLs |
| `src/uav/uav/fleets/` | Bundled fleet YAMLs |
| `src/uav/uav/runtime/` | Mission loading, mode manager, runtime bootstraps |
| `src/uav/uav/vehicles/` | UAV and payload vehicle adapters |
| `src/uav/uav/vision_nodes/` | Vision node executables used by mission-driven camera pipelines |

## First Read

- [Quickstart](docs/quickstart.md): build, source, first sim launch, first fleet launch
- [Config Reference](docs/config.md): launch params, fleet schema, backend-owned vs runtime-owned settings
- [Ops Notes](docs/ops.md): VTOL/QGC notes and short troubleshooting
- [Payload README](../payload/README.md): payload hardware and `pigpiod`
- [Sim README](../sim/README.md): sim backend behavior and stage/world details

## High-Signal Gotchas

- Mission and fleet loading need `pydantic>=2,<3`, and AprilTag-based payload missions also need `apriltag`:
  ```bash
  python3 -m pip install "pydantic>=2,<3" apriltag
  ```
- `auto_launch: false` does not block startup. It leaves the mission idle until `/<vehicle>/mode_manager/start_mission` is called.
- Sim launches default to GUI. Use `SAE_SIM_GUI=0` or `SAE_SIM_HEADLESS=1` for headless runs.
- Fleet files attach to backend controllables that already exist in the selected sim stage. They do not define PX4 airframes, PX4 namespaces, PX4 instance indices, or payload controller plugins.
