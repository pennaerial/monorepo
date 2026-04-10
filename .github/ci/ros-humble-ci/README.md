This image is the base container for ROS workspace CI.

It starts from `ros:humble-ros-base` and bakes in:

- build tools used by the workspace CI jobs
- ROS package managers and test tooling
- OpenCV and ROS vision dependencies
- `apriltag`
- `pigpio` built from source

The image is intentionally generic enough to serve both the x86 validation job and the ARM artifact build job.
