# UAV Package

This package owns mission launch, mission runtime, vehicle wrappers, and camera wiring for the SAE autonomy stack.

## Package Map

| Path | What it is |
| --- | --- |
| `src/uav/launch/` | Launch entrypoints |
| `src/uav/missions/` | Bundled mission YAMLs |
| `src/uav/uav/vehicles/` | UAV vehicle adapters |

## First Read

- [Quickstart](docs/quickstart.md): build and source
- [Config Reference](docs/config.md): PX4 airframe ID location
- [Ops Notes](docs/ops.md): VTOL/QGC notes and mission startup
- [Sim README](../sim/README.md): sim dependencies and Gazebo behavior

## High-Signal Gotchas

- Mission loading needs `pydantic>=2,<3`:
  ```bash
  python3 -m pip install "pydantic>=2,<3"
  ```
- `auto_launch: false` does not block startup. It leaves the mission idle until `/<vehicle>/mode_manager/start_mission` is called.
