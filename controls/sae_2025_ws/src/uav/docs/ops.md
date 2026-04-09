# UAV Ops Notes

Operational notes for common bringup and troubleshooting cases.

## VTOL Fixed-Wing / QGC

When launching VTOL in fixed-wing mode, make sure these PX4 parameters are set in QGC:

- `CBRK_VTOLARMING = 159753`
- `FW_W_EN = 1`
- `RWTO_TKOFF = 1`
- `NAV_FORCE_VT = 0`

If you want to make that permanent in sim, add these lines to `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/4004_gz_standard_vtol`:

```bash
param set-default CBRK_VTOLARMING 159753
param set-default FW_W_EN 1
param set-default RWTO_TKOFF 1
param set-default NAV_FORCE_VT 0
```

## Troubleshooting

### AprilTag missions fail immediately or no detector is available

Install `apriltag` on the machine running the vision nodes:

```bash
python3 -m pip install apriltag
```

### `auto_launch: false` and the mission never starts

This is expected. The stack still launches, but the mission stays idle until `/<vehicle>/mode_manager/start_mission` is called. This applies to both UAV and payload missions.

```bash
ros2 service call /<vehicle>/mode_manager/start_mission std_srvs/srv/Trigger "{}"
```

### Sim came up headless or with the wrong GUI mode

GUI is the default. Either of these forces headless mode:

- `SAE_SIM_GUI=0`
- `SAE_SIM_HEADLESS=1`

### Payload hardware issues

For `pigpiod`, `GPIOController`, and payload hardware bringup details, use [payload/README.md](../../payload/README.md).
