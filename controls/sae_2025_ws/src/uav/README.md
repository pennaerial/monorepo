## Launch
The main entry point for the integrated SAE stack is:

```bash
ros2 launch uav main.launch.py
```

AprilTag-based payload missions require the Python `apriltag` package on the machine running the vision nodes:

```bash
python3 -m pip install apriltag
```

Hardware payload missions launched through this entry point also require the payload-side `pigpiod` setup on the Pi that runs `GPIOController`. Follow [../payload/README.md](../payload/README.md) once before the first hardware run.

Useful overrides:
- `mission_name:=...` to override the mission configured in `launch/launch_params.yaml`
- `uav_name:=...` to run a UAV stack in a different vehicle namespace
- `payload_name:=...` to bind payload missions to a different payload entity
- `px4_path:=...` to use a non-default PX4 checkout

Mission start behavior:
- `launch/launch_params.yaml` and `launch/launch_params_hardware.yaml` both support `auto_launch: true|false`
- `auto_launch: true` lets the mission manager start itself when ready
- `auto_launch: false` brings up the stack but leaves the mission idle until `/<vehicle>/mode_manager/start_mission` is called
- `/<vehicle>/mode_manager/start_mission` now works for both UAV and payload missions

Simulation launch behavior:
- GUI is now the default
- `SAE_SIM_GUI=0` forces headless mode
- `SAE_SIM_HEADLESS=1` also forces headless mode
- UAV sim launches start Gazebo, spawn the configured aircraft model into the SAE world, attach PX4 SITL to that existing model, and run the UAV mission runtime
- Payload sim launches start Gazebo, spawn the configured aircraft model into the SAE world for context, and run the payload mission/runtime only; they do not start PX4 SITL

Example:

```bash
SAE_SIM_HEADLESS=1 ros2 launch uav main.launch.py mission_name:=payload_drive_to_apriltag payload_name:=payload_0
```

When launching VTOL in fixed wing mode, make sure these parameters are configured in QGC:
- CBRK_VTOLARMING: 159753
- FW_W_EN: ENABLED
- RWTO_TKOFF: ENABLED
- NAV_FORCE_VT: 0

OR if you want to permanently set these (you probably should), add these 4 lines to your 4004_gz_standard vtol file located at:
~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4004_gz_standard_vtol

ADD THESE:
param set-default CBRK_VTOLARMING 159753
param set-default FW_W_EN 1
param set-default RWTO_TKOFF 1
param set-default NAV_FORCE_VT 0

MAKE SURE THESE ARE SET:
param set-default FW_THR_MAX 1
param set-default VT_F_TRANS_THR 1
