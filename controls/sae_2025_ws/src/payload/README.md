# Payload Package
Payload node responsible for receiving and converting drive commands to movement (in real and sim)

Payload for SAE Advanced Class 2026
- two wheel diff drive payload
- cam mounted in front
- ball caster in back

### Extra Dependencies
`sudo apt install ros-humble-generate-parameter-library`
- uses `generate_parameter_library` to load in node parameters as C++ structs


If running multiple instances of payload, they should differ by launch arguments, but share the same node parameters

### Launch Arguments

| Argument | Default | Description |
|----------|---------|-------------|
| `payload_name` | `payload_0` | Node name and topic namespace for this payload. Must match the Gazebo entity name when using `SimController`. |
| `controller` | `""` | Override the controller from `payload_params.yaml`. Leave empty to use the value in the config. Options: `GPIOController`, `SimController`. |
| `use_camera` | `true` | Launch camera-related nodes. When `false`, no camera nodes are started. |
| `camera_rotation` | `0.0` | Clockwise degrees to rotate the camera image before publishing to `/<payload_name>/camera`. |

### Payload Parameters
Defined payload parameters (defined in `config/payload_params.schema.yaml`):
- `controller`: defines the controller to use. Options are `GPIOController` (real hardware) and `SimController` (Gazebo). \
DEFAULT VALUE: `GPIOController`

### Launch

```bash
# Default — single payload with camera
ros2 launch payload payload.launch.py

# Override payload name
ros2 launch payload payload.launch.py payload_name:=payload_1

# Rotate camera 90° clockwise
ros2 launch payload payload.launch.py camera_rotation:=90.0

# Disable camera
ros2 launch payload payload.launch.py use_camera:=false
```

### Camera Pipeline (GPIOController)

When `use_camera:=true` and `controller=GPIOController`, two extra nodes are launched:

```
v4l2_camera  →  /<payload_name>/camera_raw  →  image_rotate  →  /<payload_name>/camera
```

- `v4l2_camera` captures at 640×480 and publishes to `camera_raw`
- `image_rotate` (from the `tools` package) rotates by `camera_rotation` degrees and publishes the final stream to `/<payload_name>/camera`

### Sim Camera Pipeline (SimController)

A `ros_gz_bridge` node bridges the Gazebo camera and camera_info topics into:
- `/<payload_name>/camera`
- `/<payload_name>/camera_info`
