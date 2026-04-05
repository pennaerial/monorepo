# tools

Developer tools for the SAE 2025 workspace.

---

## keyboard_teleop

Keyboard-driven teleoperation for a payload. Publishes `payload_interfaces/DriveCommand` and `payload_interfaces/ServoCommand` messages at 10 Hz.

### Run

```bash
ros2 run tools keyboard_teleop --ros-args -p payload_name:=payload_0
```

The `payload_name` parameter is **required** — the node will error out if it is left empty.

### Controls

| Key | Action |
|-----|--------|
| `w` | Forward |
| `s` | Reverse |
| `a` | Turn left |
| `d` | Turn right |
| Any other key | Stop (zero command) |
| `m` / `M` | Increase / decrease linear speed by 0.1 |
| `n` / `N` | Increase / decrease angular speed by 0.1 |
| `[` / `]` | Decrease / increase servo angle |
| `p` / `P` | Increase / decrease servo step by 5.0 |
| `Ctrl-C` | Quit |

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `payload_name` | string | `""` | Payload name used to derive `/{payload_name}/cmd_drive` and `/{payload_name}/servo` (**required**) |

---

## apriltag_debug

Subscribes to a camera topic, runs AprilTag detection, and opens a live `cv2.imshow` window showing detected tags with their corners, IDs, distances, and 3-axis pose arrows. This should be used for debugging a pi cam output from your laptop, by connecting to the pi local hotspot, having the pi's ROS_DOMAIN_ID=0, and subscribing to the camera stream and running the cv imshow natively (not on pi).

### Run

```bash
# Raw image
ros2 run tools apriltag_debug --ros-args -p topic:=/camera/image_raw -p compressed:=false

# Compressed image
ros2 run tools apriltag_debug --ros-args -p topic:=/camera/image_raw/compressed -p compressed:=true
```

### Visualization

Each detected tag is annotated with:
- **Green border** — tag corners
- **Red dot** — tag center
- **Yellow text** — tag ID
- **Orange text** — distance and x/y/z translation in meters
- **Colored arrows** — pose axes projected onto the image (red = X, green = Y, blue = Z)

### Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `topic` | string | `""` | Camera topic to subscribe to. Defaults to `/camera/image_raw` (raw) or `/camera/image_raw/compressed` (compressed) if left empty |
| `compressed` | bool | `false` | Set `true` to subscribe to `sensor_msgs/CompressedImage` instead of `sensor_msgs/Image` |
| `tag_size_m` | double | `0.1` | Physical side length of the AprilTag in meters (used for pose estimation) |
| `camera_fx` | double | `600.0` | Camera intrinsic — focal length X |
| `camera_fy` | double | `600.0` | Camera intrinsic — focal length Y |
| `camera_cx` | double | `320.0` | Camera intrinsic — principal point X |
| `camera_cy` | double | `240.0` | Camera intrinsic — principal point Y |

> **Note:** The default intrinsics are a rough estimate for a 640×480 camera. For accurate pose estimation, supply the real values from your camera's calibration.
