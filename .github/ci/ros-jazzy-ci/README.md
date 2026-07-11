This image is the container for ROS workspace and headless simulation CI.

It starts from `ros:jazzy-ros-base` and bakes in:

- build tools used by the workspace CI jobs
- ROS package managers and test tooling
- OpenCV and ROS vision dependencies
- `apriltag`
- `pigpio` built from source
- prebuilt `px4_msgs`
- pinned PX4 SITL and Gazebo Harmonic with `ros_gz`
- Micro XRCE-DDS Agent
- Xvfb, Mesa, and `pymavlink` for headless simulation

The image supports ROS and simulation validation on `amd64` and `arm64` and the ARM artifact build.
