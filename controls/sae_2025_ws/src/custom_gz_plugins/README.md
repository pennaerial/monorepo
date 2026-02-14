# custom_gz_plugins

Custom Gazebo Harmonic system plugins for the SAE 2025 simulation environment.

For general information on creating Gazebo system plugins, see the [official guide](https://gazebosim.org/api/sim/8/createsystemplugins.html).

## Plugins

### DynamicDetachableJoint

A runtime attach/detach system plugin. Unlike the built-in `DetachableJoint`, this plugin does not require child models to be specified in SDF — models can be attached and detached dynamically via a Gazebo service.

**Capabilities:**
- Attach a child model's link to a parent model's link with a fixed joint at runtime
- Detach previously attached models
- Distance-based safety check before attachment
- Publishes attachment state changes to a topic

**SDF Parameters:**

| Parameter | Required | Default | Description |
|-----------|----------|---------|-------------|
| `<parent_link>` | Yes | — | Link on the parent model to attach children to |
| `<attach_distance>` | No | `0.1` | Max distance (m) between links to allow attachment |
| `<service_name>` | No | `/model/{name}/dynamic_detachable_joint/attach_detach` | Service topic for requests |
| `<output_topic>` | No | `/model/{name}/dynamic_detachable_joint/state` | Topic for state updates |

**SDF Usage:**

```xml
<plugin filename="DynamicDetachableJoint"
        name="gz::sim::systems::DynamicDetachableJoint">
  <parent_link>base_link</parent_link>
  <attach_distance>0.25</attach_distance>
</plugin>
```

**Service Interface:**

The plugin uses a custom protobuf message (`gz.custom_msgs.AttachDetachRequest`) with fields:
- `child_model_name` — name of the model to attach/detach
- `child_link_name` — specific link on the child model
- `command` — `"attach"` or `"detach"`

**Example Commands:**

Attach a payload model to the parent:

```bash
gz service -s /model/standard_vtol/dynamic_detachable_joint/attach_detach \
  --reqtype gz.custom_msgs.AttachDetachRequest \
  --reptype gz.custom_msgs.AttachDetachResponse \
  --req 'child_model_name: "payload", child_link_name: "payload_link", command: "attach"' \
  --timeout 1000
```

Detach the payload:

```bash
gz service -s /model/standard_vtol/dynamic_detachable_joint/attach_detach \
  --reqtype gz.custom_msgs.AttachDetachRequest \
  --reptype gz.custom_msgs.AttachDetachResponse \
  --req 'child_model_name: "payload", child_link_name: "payload_link", command: "detach"' \
  --timeout 1000
```

Listen to state changes:

```bash
gz topic -e -t /model/standard_vtol/dynamic_detachable_joint/state
```

## Building

Built as part of the colcon workspace:

```bash
colcon build --packages-select custom_gz_plugins
```

Depends on `custom_gz_msgs` for the protobuf service definitions.
