GitHub Actions CI assets live here.

- `ros-jazzy-ci/` builds the ROS Jazzy base image used by the ROS validation and ARM artifact workflows.
- The image is built for `amd64` and `arm64` and is meant to absorb the slow stable setup that does not need to repeat on every workflow run.
