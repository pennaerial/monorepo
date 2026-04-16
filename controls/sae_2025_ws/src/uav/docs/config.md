# Config

This repo has two config layers:
- `launch_params*.yaml` for single-vehicle launch defaults
- fleet YAMLs for multi-vehicle orchestration

## Single-Vehicle Launch Params

### `src/uav/launch/launch_params.yaml`
Sim-first defaults for `main.launch.py`.

Key fields:
- `mission_name`
- `vehicle_name`
- `auto_launch`
- `airframe`
- `custom_airframe_model`
- `px4_instance`
- `force_camera`
- `save_vision_milliseconds`
- `camera_mount_offsets`
- `camera_input_transport`
- `camera_rotate_degrees`
- `camera_preprocess_hook`
- `servo_only`
- `payload_controller`
- `sim`

### `src/uav/launch/launch_params_hardware.yaml`
Hardware defaults for `main.launch.py`.

Same shape as the sim file, but with hardware-oriented defaults such as:
- `sim: false`
- `mission_name: basic`
- `force_camera: false`
- `camera_input_transport: compressed`
- `camera_rotate_degrees: 180.0`
- `payload_controller: GPIOController`

`use_camera` is still accepted on the legacy single-vehicle path as a deprecated alias for `force_camera`.

## Fleet YAML Shape

Fleet launch reads one file with three top-level sections:

```yaml
backend:
  kind: sim  # use `hardware` or `real` on real hardware
  world_name: sae
  mission_stage: payload_retreat
  world_overrides:
    params:
      entities:
        dlz:
          path_to_sdf: ~/.simulation-gazebo/models/dlz/model.sdf
          position: [0.0, 0.0, -0.1]
          rpy: [0.0, 0.0, 0.0]
      controllables:
        uav_0:
          kind: uav
          model: gz_standard_vtol
          px4_airframe_id: 4004
          path_to_sdf: ~/.simulation-gazebo/models/standard_vtol/model.sdf

defaults:
  auto_launch: true
  debug: false
  vision_debug: false
  force_camera: false
  save_vision_milliseconds: 0
  servo_only: false
  camera_mount_offsets: [0.0, 0.0, 0.0]
  camera_input_transport: raw
  camera_rotate_degrees: 0.0
  camera_preprocess_hook: ""

vehicles:
  - name: uav_0
    mission: hover
    controllable: uav_0
```

## Backend-Owned Vs Runtime-Owned

Backend-owned:
- what exists in the world
- where it spawns
- the backend model string
- the PX4 airframe ID for UAV controllables

Runtime-owned:
- which mission runs on which controllable
- auto-start behavior
- debug and vision flags
- manual camera override
- camera transport/rotation/preprocess settings
- namespace and PX4 instance in the single-vehicle compatibility path

The backend should define spawnable props under `world.params.entities` and UAV/payload vehicles under `world.params.controllables`. UAV controllables should include `px4_airframe_id`. The fleet layer attaches to those controllables; it should not redefine their PX4 identity. For real hardware fleets, `backend.kind` may be either `hardware` or `real`; both map to the same runtime path.

## Unsupported Fleet Keys

Fleet YAMLs do not accept these keys:
- `backend.world`
- `airframe`
- `custom_airframe_model`
- `model`
- `sim`
- `px4_namespace`
- `px4_instance`
- `payload_controller`

`kind` is optional on each vehicle entry. If present, it must match the mission target.

## Where To Look For PX4 Airframe IDs

PX4 autostart IDs live in:

`~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes`

Use the numeric ID in `px4_airframe_id`.
