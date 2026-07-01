# Sim CI image: extends the lean ROS CI image with the heavy, rarely-changing
# simulation runtime (Gazebo Harmonic + a compiled PX4 SITL) so that per-PR sim
# validation can `docker pull` a ready-to-run image instead of compiling PX4 on
# every run.
#
# amd64 only: PX4 SITL does not cross-compile sanely under QEMU emulation, and
# sim CI runs on amd64 runners.

ARG BASE_IMAGE=ghcr.io/pennaerial/sae-ros2-ci:jazzy-amd64
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
# ros_gz and PX4's gz bridge build/run against Gazebo Harmonic; sim.launch.py
# also keys off this.
ENV GZ_VERSION=harmonic

# --- 1. Gazebo Harmonic ----------------------------------------------------
# Not present in the base ROS image; installed from the official OSRF apt repo.
RUN apt-get update && apt-get install -y --no-install-recommends \
        curl gnupg lsb-release \
    && curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
        -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=amd64 signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
        > /etc/apt/sources.list.d/gazebo-stable.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        gz-harmonic \
    && rm -rf /var/lib/apt/lists/*

# --- 2. ros_gz bridge (binary) ---------------------------------------------
# Jazzy ships ros_gz as a prebuilt binary, so we no longer vendor/compile the
# ros_gz submodule. sim.launch.py runs `ros_gz_bridge parameter_bridge`, which
# this package provides. Installed from the ROS apt repo already configured in
# the base ros:jazzy image, and built against the Gazebo Harmonic above.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

# --- 3. PX4-Autopilot (compiled SITL) --------------------------------------
# Version pinned here, in the image, and kept in sync with the PX4-Autopilot
# submodule (.gitmodules: pennaerial/PX4-Autopilot @ release/1.17). Bump PX4_REF
# (and keep px4_msgs in sync) to upgrade. Cloned to /root/PX4-Autopilot so the
# launch's default px4_path (~/PX4-Autopilot) resolves with no extra config.
ARG PX4_REF=6737fe1c3754a81a983b8f5aea6797b7d1669be6
RUN git clone --recursive https://github.com/pennaerial/PX4-Autopilot.git /root/PX4-Autopilot \
    && cd /root/PX4-Autopilot \
    && git checkout --recurse-submodules ${PX4_REF} \
    # PX4's build deps only: no NuttX cross-compilers, no gazebo-classic sim tools.
    && bash ./Tools/setup/ubuntu.sh --no-nuttx --no-sim-tools \
    && make px4_sitl \
    # Record the built commit so validation can detect submodule drift.
    && git rev-parse HEAD > /root/PX4-Autopilot/PINNED_COMMIT \
    && rm -rf /var/lib/apt/lists/*

# Micro XRCE-DDS Agent (v2.4.3): PX4<->ROS2 px4_msgs/DDS bridge the mission node flies the vehicle through.
ARG XRCE_AGENT_REF=v2.4.3
# First pass (superbuild) builds Fast-CDR/Fast-DDS + the agent; second pass runs the now-present install into /usr/local.
RUN git clone -b ${XRCE_AGENT_REF} --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build \
    && cmake --build /tmp/xrce/build -j"$(nproc)" \
    && cmake --build /tmp/xrce/build --target install \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/xrce

# xvfb + mesa give gz's ogre camera rendering a headless display+GL; pymavlink backs the smoke-test GCS.
RUN apt-get update && apt-get install -y --no-install-recommends xvfb libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir --break-system-packages pymavlink
