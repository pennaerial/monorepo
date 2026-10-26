# iarc

Flight modes and path planning for IARC Mission 10.

- `iarc.CircleMode`: flies a circle with changing speed (`iarc/modes/CircleMode.py`)
- `iarc/planning/circle.py`: the circle reference math, unit tested in `test/test_circle.py`

## Run in sim

Four UAVs flying the `iarc_circle` mission in the `iarc` world:

```bash
ros2 launch uav uavs_sitl.launch.py world:=iarc mission:=iarc_circle
```

`num_uavs:=N` changes the number of UAVs. To give each UAV its own mission, pass a comma-separated list in uav order, e.g. `mission:=iarc_circle,basic,basic,basic`. Each UAV's LOCAL frame starts at its own spawn point.
