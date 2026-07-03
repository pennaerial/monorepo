# Sim CI image: extends the lean ROS CI image with the heavy, rarely-changing
# simulation runtime (Gazebo Harmonic + a compiled PX4 SITL) so that per-PR sim
# validation can `docker pull` a ready-to-run image instead of compiling PX4 on
# every run.

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
# ros_gz and PX4's gz bridge build/run against Gazebo Harmonic; sim.launch.py
# also keys off this.
ENV GZ_VERSION=harmonic

# --- 1. PX4-Autopilot (compiled SITL + PX4 sim deps) ------------------------
# Version pinned here, in the image, and kept in sync with the PX4-Autopilot
# submodule (.gitmodules: pennaerial/PX4-Autopilot @ release/1.17). Bump PX4_REF
# (and keep px4_msgs in sync) to upgrade. Cloned to /root/PX4-Autopilot so the
# launch's default px4_path (~/PX4-Autopilot) resolves with no extra config.
ARG PX4_REF=6737fe1c3754a81a983b8f5aea6797b7d1669be6
RUN git clone --recursive https://github.com/pennaerial/PX4-Autopilot.git /root/PX4-Autopilot \
    && cd /root/PX4-Autopilot \
    && git checkout --recurse-submodules ${PX4_REF} \
    # Use PX4's simulator setup path so Gazebo Harmonic and its companion
    # dependencies match the documented developer install. Skip only NuttX.
    && RUNS_IN_DOCKER=true bash ./Tools/setup/ubuntu.sh --no-nuttx \
    && make px4_sitl \
    # Record the built commit so validation can detect submodule drift.
    && git rev-parse HEAD > /root/PX4-Autopilot/PINNED_COMMIT \
    && rm -rf /var/lib/apt/lists/*

# --- 2. ros_gz bridge (binary) ---------------------------------------------
# Jazzy ships ros_gz as a prebuilt binary, so we no longer vendor/compile the
# ros_gz submodule. sim.launch.py runs `ros_gz_bridge parameter_bridge`, which
# this package provides. Installed from the ROS apt repo already configured in
# the base ros:jazzy image, and built against the Gazebo Harmonic above.
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
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
