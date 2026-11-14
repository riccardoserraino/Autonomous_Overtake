#!/bin/bash
# Open a terminal INSIDE Docker (Ubuntu 22.04 + ROS 2 loaded). Run it in as many tabs as you want.
# Type "exit" to leave: the container keeps running.

cd "$(dirname "$0")/../docker"                       # all docker commands run in the docker folder
source .env                                          # read the settings
export USER_UID=$(id -u) USER_GID=$(id -g)           # your user id: files made inside belong to you
export VIDEO_GID=$(getent group video | cut -d: -f3) # your PC's GPU groups (see docker/.env)
export RENDER_GID=$(getent group render | cut -d: -f3)
export RENDER_GID=${RENDER_GID:-$USER_GID}           # (older PCs have no "render" group)

docker compose up -d dev                             # start "dev" in the background, if not running
docker compose exec dev bash                         # open a terminal in it
