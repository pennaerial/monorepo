FROM ros:jazzy-ros-base
# Ubuntu 24.04 Image

ENV DEBIAN_FRONTEND=noninteractive
ENV GZ_VERSION=harmonic

# monorepo specific ENV vars. Mirrors dev_env.sh for this container
ENV PENNAIR_MONOREPO_PATH=/root/monorepo
ENV PENNAIR_GZ_MODELS_PATH=${PENNAIR_MONOREPO_PATH}/gz-models
ENV PENNAIR_PX4_PATH=${PENNAIR_MONOREPO_PATH}/PX4-Autopilot
ENV PENNAIR_PAYLOAD_CONTROLLER_PATH=${PENNAIR_MONOREPO_PATH}/payload_controller
ENV PENNAIR_DEPENDENCIES_PATH=${PENNAIR_MONOREPO_PATH}/Dependencies
ENV PENNAIR_XRCE_DDS_GEN_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-XRCE-DDS-Gen
ENV PENNAIR_XRCE_DDS_CLIENT_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-XRCE-DDS-Client
ENV PENNAIR_XRCE_DDS_MICROCDR_PATH=${PENNAIR_DEPENDENCIES_PATH}/Micro-CDR

# Install git and clean up repository caches
RUN apt-get update && apt-get install -y \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /root
# lets docker cache this clone layer; then we manually checkout most up-to-date
RUN git clone --recursive https://github.com/pennaerial/monorepo.git

WORKDIR /root/monorepo
ARG CACHEBUST_CHECKOUT=1 # checkout most up-to-date main branch and skip layer cache
ARG BRANCH=main
RUN git fetch origin main \
    && git checkout main \
    && git reset --hard origin/main \
    && git submodule update --init --recursive

# update all monorepo submodules (managed and 3rd party)
RUN bash ci/update_submodules.sh
