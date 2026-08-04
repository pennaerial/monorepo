FROM ros:jazzy-ros-base
# Ubuntu 24.04 Image

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic
ENV ROS_DISTRO=jazzy

# For PX4's ubuntu.sh install script
ENV RUNS_IN_DOCKER=true

# monorepo specific ENV vars. Mirrors dev_env.sh for this container
ENV PENNAIR_MONOREPO_PATH=/root/monorepo
ENV PENNAIR_SAE_WS_PATH=${PENNAIR_MONOREPO_PATH}/controls/sae_2025_ws
ENV PENNAIR_GZ_MODELS_PATH=${PENNAIR_MONOREPO_PATH}/gz-models
ENV PENNAIR_PX4_PATH=${PENNAIR_MONOREPO_PATH}/PX4-Autopilot
ENV PENNAIR_PAYLOAD_CONTROLLER_PATH=${PENNAIR_MONOREPO_PATH}/payload_controller
ENV PENNAIR_DEPENDENCIES_PATH=${PENNAIR_MONOREPO_PATH}/Dependencies
ENV PENNAIR_XRCE_DDS_GEN_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-XRCE-DDS-Gen
ENV PENNAIR_XRCE_DDS_CLIENT_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-XRCE-DDS-Client
ENV PENNAIR_XRCE_DDS_MICROCDR_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-CDR

# add uv to PATH (during install_deps.sh)
ENV PATH="/root/.local/bin:${PATH}"

# Install git and clean up repository caches
RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root
ARG BRANCH=main
ARG CACHEBUST_CHECKOUT=1
RUN git clone https://github.com/pennaerial/monorepo.git -b $BRANCH

# update all monorepo submodules (managed go to up-to-date default branch  and 3rd party go to pinned tag)
WORKDIR /root/monorepo
RUN bash ci/update_submodules.sh

# install all required dependencies (rosdeps, PX4 deps, gz sim, extra apt installs, extra python deps)
RUN bash ci/install_deps.sh

# build all required ci components into image (PX4 SITL, 3rd party ROS package submodules, Dependencies modules)
RUN bash ci/image_build.sh
