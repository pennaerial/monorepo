# custom_sim_bridge

Pluginlib-based ROS 2 bridge plugins that translate between ROS 2 services and Gazebo Harmonic transport for the SAE 2025 simulation environment.

For general information on pluginlib, see the [official guide](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Pluginlib.html).

## Architecture

Each bridge plugin is an `rclcpp::Node` subclass loaded at runtime by the generic `bridge_loader` executable. This avoids recompilation when adding new bridges — register a new class in `plugins.xml` and implement it as a shared library.

```
ROS 2 Service call
       │
       ▼
 AttachDetachBridge (rclcpp::Node)
       │  converts request via convert.hpp
       ▼
 Gazebo transport service call (gz-transport13)
       │  converts response via convert.hpp
       ▼
ROS 2 Service response
```

## Bridge Plugins

### AttachDetachBridge

Bridges the `sim_interfaces/srv/AttachDetach` ROS 2 service to the `gz.custom_msgs.AttachDetachRequest/Response` Gazebo service. Intended to be used alongside the `DynamicDetachableJoint` Gazebo plugin.

**Capabilities:**
- Exposes a ROS 2 service that forwards attach/detach commands to Gazebo
- Converts between ROS 2 and protobuf message types transparently
- 5-second timeout on Gazebo service calls with proper error propagation

**Parameters:**

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `ros_service_name` | `string` | `""` | ROS 2 service endpoint to advertise |
| `gz_service_name` | `string` | `""` | Gazebo service endpoint to forward requests to |
- These are the necessary parameters to specify in order to use the AttachDetachBridge. If not specified, the param values will default to `""`, and ROS will crash because `""` is an invalid name for a communication endpoint.

**Usage:**

Launch via the generic `bridge_loader` executable. The plugin class name is passed as a positional argument (the `custom_sim_bridge::` namespace is prepended automatically). ROS parameters follow after `--ros-args`:

```bash
ros2 run custom_sim_bridge bridge_loader AttachDetachBridge \
  --ros-args \
  -p ros_service_name:=/standard_vtol_0/attach_detach \
  -p gz_service_name:=/model/standard_vtol/dynamic_detachable_joint/attach_detach
```

Or from a launch file:

```python
Node(
    package="custom_sim_bridge",
    executable="bridge_loader",
    arguments=["AttachDetachBridge"],
    parameters=[{
        "ros_service_name": "/standard_vtol_0/attach_detach",
        "gz_service_name": "/model/standard_vtol/dynamic_detachable_joint/attach_detach",
    }],
)
```

**Example ROS 2 service call:**

```bash
ros2 service call /standard_vtol_0/attach_detach sim_interfaces/srv/AttachDetach \
  '{child_model_name: "payload", child_link_name: "payload_link", command: "attach"}'
```

## Adding New Bridges

1. Implement a new `rclcpp::Node` subclass in `src/`.
2. Export it with `PLUGINLIB_EXPORT_CLASS` and register it in `plugins.xml`.
3. Add the source file to the `custom_sim_bridge_plugins` target in `CMakeLists.txt`.

The `bridge_loader` executable will be able to instantiate it without any further changes.

## Building

Built as part of the colcon workspace:

```bash
colcon build --packages-select custom_sim_bridge
```

Depends on `custom_gz_msgs` for protobuf definitions and `sim_interfaces` for ROS 2 service types.
