# custom_gz_msgs

Custom Gazebo Harmonic protobuf message definitions for the SAE 2025 simulation environment.

For general information on creating Gazebo messages, see the [official guide](https://gazebosim.org/api/msgs/10/index.html).

## Messages

All messages are defined in the `gz.custom_msgs` namespace and live under `proto/gz/custom_msgs/`.

### AttachDetachRequest / AttachDetachResponse

Protobuf service message pair used by the `DynamicDetachableJoint` plugin to attach and detach child models at runtime.

**AttachDetachRequest fields:**

| Field | Type | Description |
|-------|------|-------------|
| `child_model_name` | `string` | Name of the child model to attach or detach |
| `child_link_name` | `string` | Specific link on the child model |
| `command` | `string` | `"attach"` or `"detach"` |

**AttachDetachResponse fields:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | `bool` | Whether the operation succeeded |
| `message` | `string` | Status or error description |

**Example usage with `gz service`:**

```bash
gz service -s /model/standard_vtol/dynamic_detachable_joint/attach_detach \
  --reqtype gz.custom_msgs.AttachDetachRequest \
  --reptype gz.custom_msgs.AttachDetachResponse \
  --req 'child_model_name: "payload", child_link_name: "payload_link", command: "attach"' \
  --timeout 1000
```

## Building

Built as part of the colcon workspace:

```bash
colcon build --packages-select custom_gz_msgs
```

The build generates a C++ protobuf library (`custom_gz_msgs-msgs`) and installs headers to `include/gz/custom_msgs/`. An environment hook automatically adds the installed descriptor path to `GZ_DESCRIPTOR_PATH` so Gazebo can resolve the custom message types at runtime.

Consumed by `custom_gz_plugins` (plugin implementation) and `custom_sim_bridge` (ROS 2 bridge).
