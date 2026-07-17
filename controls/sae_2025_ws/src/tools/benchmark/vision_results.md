# Vision Benchmarking Done on Raspberry Pi

Done by: Yuzhi

Hardware:

- Raspberry Pi 4B (4 cpu cores) w/ heatsink
- 8GB ram
- Pi cam module 3

## Test 1: Standalone camera_node, YUYV, 1280x1080

steps:

- ran the command `ros2 run camera_ros camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080`
- After initialization was complete, I ran `benchmark.py` for around 10 seconds. Here are the results

```
[  10.9s] polling... tracking 2 process(es), 10 sample(s)

══════════════════════════════════════════════════════════════════════════════════════════════════════════════
                                         ROS PROCESS RESOURCE REPORT
                                         (sorted by avg CPU%, top 2)
══════════════════════════════════════════════════════════════════════════════════════════════════════════════
  # PROCESS                                            PID SAMPLES  CPU avg  CPU max   MEM avg   MEM max THR avg FDS avg      IO R      IO W  CPU/THR
──────────────────────────────────────────────────────────────────────────────────────────────────────────────
  1 /home/penn/dependencies_ws/install/camera_ros…    2739      10    33.7%    38.7%     85.0M     85.0M    23.0      68    0.0B/s    0.0B/s     1.5%
    └ /home/penn/dependencies_ws/install/camera_ros/lib/camera_ros/camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2739]
  2 /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run …    2736      10     0.0%     0.0%     27.6M     27.6M     3.0      11    0.0B/s    0.0B/s     0.0%
    └ /usr/bin/python3 /opt/ros/jazzy/bin/ros2 run camera_ros camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2736]
──────────────────────────────────────────────────────────────────────────────────────────────────────────────

LIKELY BOTTLENECK SIGNALS
  * Highest avg CPU  : /home/penn/dependencies_ws/install/camera_ros/lib/camera_ros/camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2739] (33.7%)
  * Highest avg RAM  : /home/penn/dependencies_ws/install/camera_ros/lib/camera_ros/camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2739] (85.0MB)
  * Most threads     : /home/penn/dependencies_ws/install/camera_ros/lib/camera_ros/camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2739] (23.0 avg)
  * Heaviest IO      : /home/penn/dependencies_ws/install/camera_ros/lib/camera_ros/camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080 [pid 2739] (R 0.0B/s, W 0.0B/s)
══════════════════════════════════════════════════════════════════════════════════════════════════════════════

```
