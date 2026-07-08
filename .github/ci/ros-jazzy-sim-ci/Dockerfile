# Simulation runtime used by ROS Sim Validate.

ARG BASE_IMAGE
FROM ${BASE_IMAGE}

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

# Keep this in sync with the PX4-Autopilot submodule.
ARG PX4_REF=6737fe1c3754a81a983b8f5aea6797b7d1669be6
RUN git clone --recursive https://github.com/pennaerial/PX4-Autopilot.git /root/PX4-Autopilot \
    && cd /root/PX4-Autopilot \
    && git checkout --recurse-submodules ${PX4_REF} \
    # Use PX4's setup path so Gazebo deps match developer installs.
    && RUNS_IN_DOCKER=true bash ./Tools/setup/ubuntu.sh --no-nuttx \
    && make px4_sitl \
    && git rev-parse HEAD > /root/PX4-Autopilot/PINNED_COMMIT \
    && rm -rf /var/lib/apt/lists/*

RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-gz \
    && rm -rf /var/lib/apt/lists/*

ARG XRCE_AGENT_REF=v2.4.3
RUN git clone -b ${XRCE_AGENT_REF} --depth 1 \
        https://github.com/eProsima/Micro-XRCE-DDS-Agent.git /tmp/xrce \
    && cmake -S /tmp/xrce -B /tmp/xrce/build \
    && cmake --build /tmp/xrce/build -j"$(nproc)" \
    && cmake --build /tmp/xrce/build --target install \
    && ldconfig /usr/local/lib \
    && rm -rf /tmp/xrce

# xvfb + mesa give Gazebo a headless display and GL.
RUN apt-get update && apt-get install -y --no-install-recommends xvfb libgl1-mesa-dri \
    && rm -rf /var/lib/apt/lists/* \
    && python3 -m pip install --no-cache-dir --break-system-packages pymavlink
