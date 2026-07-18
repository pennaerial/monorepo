# Vision Benchmarking Done on Raspberry Pi

Date: 7/17/2026

Specs:
- Raspberry Pi 4B (4 cpu cores) w/ heatsink
- 8GB ram
- Pi cam module 3
- Ubuntu 24.04 / ROS Jazzy
- default RMW (FastDDS)

**NOTE:** in the below tests, /camera_node/image_raw topic is the UAV camera node, and /camera/image_raw is the camera_ros camera_node ros package. Sorry for the confusion

**NOTE:** A lot of the tests used ros2 topic hz, which I only realized afterwards is really really inefficient, because its written in rclpy and consumed more resources than I expected. In the future we can make a lightweight C++ hz calculator and put that in tools package

## Test 1: Standalone camera_node, YUYV, 1280x1080

steps:

- ran the command `ros2 run camera_ros camera_node --ros-args -p format:=YUYV -p width:=1280 -p height:=1080`
- After initialization was complete, I ran `benchmark.py` for around 10 seconds. Here are the results

```
[  10.9s] polling... tracking 2 process(es), 10 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 2 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 camera_node --ros-args -p…   2954    33.5/38.0%           85/85  23.0    68  R0.0B/s/W0.0B/s    1.5%
  2 python3 ros2 run camera_r…   2951      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (33.5%)
  * Highest avg RAM : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (85.2MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

## Test 2: Standalone camera_ros camera_node + rqt raw image stream, YUYV, 1280x1080

steps:

- Hooked up pi to laptop via ethernet
- ran same command as last test on pi
- on laptop ran rqt and subscribed to /camera/image_raw. Moved pi cam around just in case

```
==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 2 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 camera_node --ros-args -p…   3088   96.5/175.7%           85/85  23.0    63  R0.0B/s/W0.0B/s    4.2%
  2 python3 ros2 run camera_r…   3085      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (96.5%)
  * Highest avg RAM : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (85.2MB)
  * Bursty CPU      : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (avg 96.5% vs max 175.7%) -> possible stalls/callbacks
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

```
[  61.2s] polling... tracking 2 process(es), 49 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 2 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 camera_node --ros-args -p…   3392    85.9/92.5%           85/85  23.0    65  R0.0B/s/W0.0B/s    3.7%
  2 python3 ros2 run camera_r…   3389      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (85.9%)
  * Highest avg RAM : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (85.2MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

## Test 3: camera_ros camera_node + rqt compressed image stream

- Same as test 2 but subscribed to compressed stream instead

```
==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 2 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 camera_node --ros-args -p…   3356  180.9/189.0%         117/117  26.0    65  R0.0B/s/W0.0B/s    7.0%
  2 python3 ros2 run camera_r…   3353      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (180.9%)
  * Highest avg RAM : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (116.7MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (26.0 avg)
  * Heaviest IO     : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (R 0.0B/s, W 0.0B/s)
==========================================================================================

```

```
[  61.3s] polling... tracking 2 process(es), 49 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 2 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 camera_node --ros-args -p…   3526  183.6/190.6%         117/117  26.0    65  R0.0B/s/W0.0B/s    7.1%
  2 python3 ros2 run camera_r…   3523      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (183.6%)
  * Highest avg RAM : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (116.7MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (26.0 avg)
  * Heaviest IO     : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

**NOTE:** I forgot to record the topic hz when doing tests 1-3 but by my estimate and just looking at rqt image stream it was about 30fps. Will do retests soon.

UPDATE: confirmed. Standalone node does about 30 fps with just ros2 topic hz. After starting rqt it slowly decays to ~24-25 after around 30 seconds and hovers

## Test 4: camera_ros camera_node + uav camera node, no rqt

- Ran camera_ros node same as before
- ran uav camera node to perform a simple 180 degree image rotation and republish raw images. input_transport is raw, meaning it subscribes to camera_ros's raw image stream

```
ros2 run uav camera --ros-args -p rotate_degrees:=180.0 -p input_transport:=raw -p input_raw_topic:=image_raw -r image_raw:=/camera/image_raw -r camera:=/camera_node/image_raw
```

```
[  61.9s] polling... tracking 4 process(es), 41 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 4 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 python3 camera --ros-args…   4202  135.5/138.9%         187/192  18.0    27  R0.0B/s/W0.0B/s    7.5%
  2 camera_node --ros-args -p…   4176    74.5/88.3%           96/96  23.0    65  R0.0B/s/W0.0B/s    3.2%
  3 python3 ros2 run camera_r…   4173      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  4 python3 ros2 run uav came…   4181      0.0/0.0%           27/27   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (135.5%)
  * Highest avg RAM : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (187.5MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

## Test 5: camera_ros camera_node + uav camera_node + rqt view compressed

- same as test 4 procedure but add rqt for compressed stream viewing

```
[  62.6s] polling... tracking 6 process(es), 44 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 6 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 python3 camera --ros-args…   4267  124.3/129.6%         196/200  18.0    27  R0.0B/s/W0.0B/s    6.9%
  2 camera_node --ros-args -p…   4241    64.3/68.6%           97/97  23.0    65  R0.0B/s/W0.0B/s    2.8%
  3 python3 ros2 topic hz com…   4374     8.5/10.2%           74/75  15.0    28  R0.0B/s/W0.0B/s    0.6%
  4 python3 -c from ros2cli.d…   4347      0.5/1.4%           67/67  15.0    27  R0.0B/s/W0.0B/s    0.0%
  5 python3 ros2 run camera_r…   4238      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  6 python3 ros2 run uav came…   4246      0.0/0.0%           27/27   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (124.3%)
  * Highest avg RAM : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (196.5MB)
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (R 0.0B/s, W 0.0B/s)
==========================================================================================
```

ran: `ros2 topic hz /camera/compressed`:

```
average rate: 10.182
	min: 0.087s max: 0.244s std dev: 0.01412s window: 696
```

## Test 6: camera_ros camera_node + uav camera_node + payload_apriltag_node (vision node), no rqt

steps:
- same camera_ros setup
- same uav camera_node setup
- run: `ros2 run uav payload_april_tag_node --ros-args -r camera:=/camera_node/image_raw`

```
[  63.2s] polling... tracking 9 process(es), 35 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 9 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 python3 camera --ros-args…   4551  113.0/139.0%         195/197  18.0    27  R0.0B/s/W0.0B/s    6.3%
  2 camera_node --ros-args -p…   4525  103.0/110.7%           98/98  23.0    65  R0.0B/s/W0.0B/s    4.5%
  3 python3 ros2 topic hz ima…   4587    47.1/55.1%           81/87  15.0    28  R0.0B/s/W0.0B/s    3.1%
  4 python3 payload_april_tag…   4555    24.3/32.7%         185/188  15.0    27  R0.0B/s/W0.0B/s    1.6%
  5 python3 ros2 topic hz ima…   4590    13.8/25.0%           79/87  15.0    28  R0.0B/s/W0.0B/s    0.9%
  6 python3 -c from ros2cli.d…   4347      0.5/1.2%           68/68  15.0    27  R0.0B/s/W0.0B/s    0.0%
  7 python3 ros2 run camera_r…   4522      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  8 python3 ros2 run uav came…   4530      0.0/0.0%           27/27   3.0    11  R0.0B/s/W0.0B/s    0.0%
  9 python3 ros2 run uav payl…   4552      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (113.0%)
  * Highest avg RAM : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (194.8MB)
  * Bursty CPU      : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (avg 113.0% vs max 139.0%) -> possible stalls/callbacks
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (R 0.0B/s, W 0.0B/s)
==========================================================================================
ros2 topic hz /camera/image_raw
average rate: 7.544
	min: 0.031s max: 0.656s std dev: 0.09174s window: 580

ros2 topic hz /camera_node/image_raw
average rate: 1.102
	min: 0.073s max: 4.119s std dev: 0.84050s window: 83
```

## Test 7: camera_ros camera_node + uav camera + payload_color_square_node (vision_node) w/ debug stream + rqt view

We ran this combo the most for sure lmao \
vision node cmd: `ros2 run uav payload_color_square_node --ros-args -p debug:=true`

```

[  64.3s] polling... tracking 10 process(es), 35 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 10 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 python3 payload_color_squ…   4848   97.1/110.7%         194/194  18.0    27  R0.0B/s/W0.0B/s    5.4%
  2 camera_node --ros-args -p…   4710   83.5/104.0%         101/101  23.0    65  R0.0B/s/W0.0B/s    3.6%
  3 python3 camera --ros-args…   4825    73.0/98.9%         197/198  18.0    27 R0.0B/s/W124.9B/s    4.1%
  4 python3 ros2 topic hz ima…   4740    25.4/37.7%           80/88  15.0    28  R0.0B/s/W0.0B/s    1.7%
  5 python3 ros2 topic hz com…   4786     7.6/10.0%           75/76  15.0    28  R0.0B/s/W0.0B/s    0.5%
  6 python3 ros2 topic hz ima…   4745      2.2/6.8%           82/82  15.0    28  R0.0B/s/W0.0B/s    0.1%
  7 python3 -c from ros2cli.d…   4347      0.5/1.1%           68/68  15.0    27  R0.0B/s/W0.0B/s    0.0%
  8 python3 ros2 run camera_r…   4706      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  9 python3 ros2 run uav came…   4822      0.0/0.0%           27/27   3.0    11  R0.0B/s/W0.0B/s    0.0%
 10 python3 ros2 run uav payl…   4844      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : python3 payload_color_square_node --ros-args -p debug:=true (97.1%)
  * Highest avg RAM : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (196.7MB)
  * Bursty CPU      : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (avg 73.0% vs max 98.9%) -> possible stalls/callbacks
  * Most threads    : camera_node --ros-args -p format:=YUYV -p width:=1280 -p he… (23.0 avg)
  * Heaviest IO     : python3 camera --ros-args -p rotate_degrees:=180.0 -p input… (R 0.0B/s, W 124.9B/s)
==========================================================================================
ros2 topic hz /camera/image_raw # CAMERA_ROS CAMERA_NODE
average rate: 4.821
	min: 0.029s max: 2.866s std dev: 0.24050s window: 699

ros2 topic hz /camera_node/image_raw # UAV CAMERA NODE
average rate: 0.355
	min: 0.086s max: 19.669s std dev: 4.11734s window: 42

ros2 topic hz /vision/payload_color_square_node/debug_image/compressed
average rate: 7.671
	min: 0.066s max: 15.243s std dev: 0.45970s window: 1086
```

**NOTE:** According to claude, the payload_color_square_node is publishing at a somewhat higher rate because its using a stale image from its service calls to the uav camera_node.
Basically, its repeatedly calling its vision alg on the same "self.image_raw" over and over since the camera_node's image_raw doesn't update quickly (according to its topic hz numbers)

## Test 8: Payload corner navigate mission:

steps:
Camera_ros:
```
ros2 run camera_ros camera_node --ros-args -r __ns:=/payload_0 \
  -p format:=YUYV -p height:=1080 -p width:=1280 \
  --remap camera/image_raw:=camera_source \
  --remap camera/image_raw/compressed:=camera_source/compressed \
  --remap camera/camera_info:=camera_info_source   # verify this last one against `ros2 topic list`
```

uav camera_node:
```
ros2 run uav camera --ros-args -r __ns:=/payload_0 -r __node:=payload_0_camera   -p vehicle_name:=payload_0 -p input_transport:=raw   -p input_raw_topic:=camera_source -p input_compressed_topic:=camera_source/compressed   -p input_camera_info_topic:=camera_info_source -p rotate_degrees:=180.0
```

mission (This mission/mode doesn't use a vision node, it just subscribes to the camera topic in the mode itself):
```
ros2 run payload payload_mission --ros-args -r __ns:=/payload_0 \
  -p mode_map:=/home/penn/monorepo/controls/sae_2025_ws/src/payload/missions/payload_corner_navigate_real.yaml \
  -p vehicle_name:=payload_0 -p auto_launch:=false -p vision_debug:=false

```

**NOTE:** Only 23.0s because the mission lasted that long
```
[  23.0s] polling... tracking 8 process(es), 14 sample(s)   ^C

==========================================================================================
                     ROS PROCESS RESOURCE REPORT (top 8 by avg CPU%)
==========================================================================================
  # Process                       PID   CPU avg/max  Mem avg/max MB   Thr   FDs           IO R/W CPU/Thr
------------------------------------------------------------------------------------------
  1 python3 camera --ros-args…   5497  108.5/135.4%         189/193  18.0    27  R0.0B/s/W0.0B/s    6.0%
  2 camera_node --ros-args -r…   5568  106.0/191.8%          99/100  23.0    65  R0.0B/s/W0.0B/s    4.6%
  3 python3 payload_mission -…   5516   83.8/120.9%         209/217  15.6    25 R14.8KB/s/W358.7B/s    5.4%
  4 python3 ros2 service call…   5598    65.2/93.1%           53/71   4.0    12  R0.0B/s/W0.0B/s   16.3%
  5 python3 -c from ros2cli.d…   4347      0.8/2.8%           68/68  15.0    27  R0.0B/s/W0.0B/s    0.1%
  6 python3 ros2 run uav came…   5494      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  7 python3 ros2 run payload …   5513      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
  8 python3 ros2 run camera_r…   5565      0.0/0.0%           28/28   3.0    11  R0.0B/s/W0.0B/s    0.0%
------------------------------------------------------------------------------------------

Likely bottleneck signals:
  * Highest avg CPU : python3 camera --ros-args -r __ns:=/payload_0 -r __node:=pa… (108.5%)
  * Highest avg RAM : python3 payload_mission --ros-args -r __ns:=/payload_0 -p m… (209.0MB)
  * Bursty CPU      : camera_node --ros-args -r __ns:=/payload_0 -p format:=YUYV … (avg 106.0% vs max 191.8%) -> possible stalls/callbacks
  * Most threads    : camera_node --ros-args -r __ns:=/payload_0 -p format:=YUYV … (23.0 avg)
  * Heaviest IO     : python3 payload_mission --ros-args -r __ns:=/payload_0 -p m… (R 14.8KB/s, W 358.7B/s)
==========================================================================================
```


# Conclusions:

- uav CameraNode needs to be fixed at all costs
- Our entire vision pipeline is cooked and we are cooked if we don't fix immediately
- why tf does ros2 use so much compute

- What if we fade everything and switch to C++ lowkeyyyy
