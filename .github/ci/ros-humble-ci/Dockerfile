FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

COPY controls/sae_2025_ws/scripts/ci/common.sh /tmp/ci/common.sh
COPY controls/sae_2025_ws/scripts/ci/install_ros_ci_deps.sh /tmp/ci/install_ros_ci_deps.sh

RUN bash /tmp/ci/install_ros_ci_deps.sh \
    && rm -rf /tmp/ci \
    && rm -rf /var/lib/apt/lists/*
