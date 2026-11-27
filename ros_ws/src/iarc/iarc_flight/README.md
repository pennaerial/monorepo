# iarc_flight

Trajectories, controllers and flight modes for IARC Mission 10.

- `trajectories/`: what to fly. A `Trajectory` gives the reference position, velocity, acceleration and yaw at time `t`. Has `Circle` (changing speed, laps).
- `controllers/`: how to track it. A `Controller` turns the vehicle state and a trajectory into the setpoint sent to PX4. Has `feedforward` (send the reference, PX4 tracks it) and `mpc` (linear MPC with acceleration and speed limits, needs `cvxpy`).
- `modes/track_mode.py`: `iarc_flight.TrackMode` flies a trajectory with a controller, both picked by `type` in the mission:

```yaml
circle:
  mode: iarc_flight.TrackMode
  params:
    trajectory: {type: circle, radius: 5, laps: 2}
    controller: {type: mpc}
```

When the trajectory is done it logs the position error and controller step time.

## Stopping

`iarc_flight.StopMode` brakes to v = 0 as fast as the tilt and vertical accel limits allow, at the height the stop began. Each update it solves a stop MPC and completes once speed stays under `stopped_speed` for `stopped_time`.

To make a `TrackMode` stoppable, list what can stop it in `stop_on` and map its `stop` transition to a `StopMode`. An unmapped `stop` crashes the mode manager on the first trigger:

```yaml
circle:
  mode: iarc_flight.TrackMode
  params:
    trajectory: {...}
    controller: {type: mpc}
    stop_on: [service, rtk_loss]
  transitions: {complete: land, stop: stop}
stop:
  mode: iarc_flight.StopMode
  transitions: {complete: land}
```

The stoppable `TrackMode` solves the stop MPC alongside tracking every update. When a trigger fires, that stop setpoint is sent at once from the callback and the mode returns `stop`; `StopMode` keeps braking with the same solver. It never resumes tracking.

- `service`: `ros2 service call /uav_0/mpc_stop std_srvs/srv/Trigger`
- `rtk_loss`: `vehicle_gps_position` reports a fix below RTK fixed, nothing for `gps_timeout` (default 1 s), or no RTK fix when the mode starts.

PX4 SITL only reports a 3D fix, so in sim set `gps_topic: spoof/vehicle_gps_position` and run `gps_spoof`, as `iarc_rtk_stop` does (it also stops on the service):

```bash
ros2 launch uav uav_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_rtk_stop
ros2 run iarc_flight gps_spoof --ros-args -r __ns:=/uav_0    # publishes rtk_fixed
ros2 param set /uav_0/gps_spoof fix rtk_float                # brakes, then lands
```

`fix` is one of `no_fix`, `2d`, `3d`, `dgps`, `rtk_float`, `rtk_fixed`, `extrapolated`, or `off` (silent, so `gps_timeout` fires).

## Run in sim

IARC missions live in `iarc_flight/missions` (`ros2 run pennair_cli pennair mission iarc_flight` lists them).

Start QGroundControl first, then four UAVs flying the circle in the `iarc` world:

```bash
ros2 launch uav uavs_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_circle      # feedforward
ros2 launch uav uavs_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_circle_mpc  # MPC
```

For a single UAV use `uav_sitl.launch.py` with the same arguments, e.g.:

```bash
ros2 launch uav uav_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_circle_mpc
```

`num_uavs:=N` changes the number of UAVs. To give each UAV its own mission, pass a comma-separated list in uav order, e.g. `mission:=iarc_circle_mpc,iarc_circle,iarc_circle,iarc_circle`. Each UAV's LOCAL frame starts at its own spawn point.
