# Drone-to-drone topics

Each drone has its own namespace (`uav_0`, `uav_1`, ...), set by `UXRCE_DDS_NS_IDX` on
the Pixhawk and `ns_id` at launch.

On a Pi, `uav_hardware.launch.py` also puts the drone in its own ROS domain, taken from
the hostname (`air-06` is domain 6), and starts `zenoh-bridge-ros2dds`.

Topics under `/uav_<n>/shared/` reach the laptop and the other drones. Everything else
stays on the drone.

```python
# in a node running in the uav_1 namespace
self.create_publisher(DroneState, "shared/state", qos)       # /uav_1/shared/state
self.create_subscription(DroneState, "/uav_0/shared/state", callback, qos)
```

Use best-effort QoS with depth 1 for state that is sent often, and reliable for
things that must arrive, like a map.

## Starting a mission from another machine

Service calls don't pass through the bridge. Join the drone's domain instead
(6 for `air-06`):

```bash
ROS_DOMAIN_ID=6 ros2 service call /uav_1/mode_manager/start_mission std_srvs/srv/Trigger
```

## Shells on the Pi

`ros2 topic` only sees the drone's nodes with the same domain set:

```bash
export ROS_DOMAIN_ID=$((10#$(hostname | cut -d- -f2)))
```

## Options

| Launch argument | |
| --- | --- |
| `peers` | Other drones' Pi numbers, e.g. `peers:=4,6`. Only needed if the drones don't find each other. |

## Watching from a laptop

Over Ethernet to any drone:

```bash
zenoh-bridge-ros2dds -c ros_ws/src/common/uav/config/zenoh_bridge.json5 -e tcp/192.168.50.6:7447
ros2 topic echo /uav_1/shared/state
```
