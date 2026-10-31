# iarc_flight

Trajectories, controllers and flight modes for IARC Mission 10.

- `trajectories/`: what to fly. A `Trajectory` gives the reference position, velocity, acceleration and yaw at time `t`. Has `Circle` (changing speed, laps).
- `controllers/`: how to track it. A `Controller` turns the vehicle state and a trajectory into the setpoint sent to PX4. Has `feedforward` (send the reference, PX4 tracks it) and `mpc` (linear MPC with acceleration and speed limits, needs `cvxpy`).
- `modes/TrackMode.py`: `iarc_flight.TrackMode` flies a trajectory with a controller, both picked by `type` in the mission:

```yaml
circle:
  mode: iarc_flight.TrackMode
  params:
    trajectory: {type: circle, radius: 5, laps: 2}
    controller: {type: mpc}
```

When the trajectory is done it logs the position error and controller step time.

## Run in sim

IARC missions live in `iarc_flight/missions` (`pennair mission iarc_flight` lists them).

Start QGroundControl first, then four UAVs flying the circle in the `iarc` world:

```bash
ros2 launch uav uavs_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_circle      # feedforward
ros2 launch uav uavs_sitl.launch.py world:=iarc mission_package:=iarc_flight mission:=iarc_circle_mpc  # MPC
```

`num_uavs:=N` changes the number of UAVs. To give each UAV its own mission, pass a comma-separated list in uav order, e.g. `mission:=iarc_circle_mpc,iarc_circle,iarc_circle,iarc_circle`. Each UAV's LOCAL frame starts at its own spawn point.
