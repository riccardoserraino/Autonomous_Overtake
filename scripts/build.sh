#!/bin/bash
# Build the Docker image, then compile our ROS 2 code.
# When: the first time, after changing the Dockerfile or CARLA_VERSION, after adding new files.

cd "$(dirname "$0")/../docker"                       # all docker commands run in the docker folder
source .env                                          # read the settings
export USER_UID=$(id -u) USER_GID=$(id -g)           # your user id: files made inside belong to you
export VIDEO_GID=$(getent group video | cut -d: -f3) # your PC's GPU groups (see docker/.env)
export RENDER_GID=$(getent group render | cut -d: -f3)
export RENDER_GID=${RENDER_GID:-$USER_GID}           # (older PCs have no "render" group)

# ./scripts/build.sh code  -> skip step 1: only compile our code (~5 s). Enough after adding files.
# 1. build the image (first time ~20 min, later seconds)
if [ "$1" != code ]; then docker compose build dev; fi

# 2. compile our code + the simulator server inside a temporary container (--rm = deleted after)
docker compose run --rm dev colcon build --symlink-install \
    --base-paths src ../third_party/webdrivesim/ros2_ws/src
