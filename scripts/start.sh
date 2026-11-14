#!/bin/bash
# Start a simulator connection + our modules, in the BACKGROUND (your terminal stays free).
#   ./scripts/start.sh webdrivesim            our simulator + acc
#   ./scripts/start.sh carla                  link to CARLA + acc   (start CARLA first: ./scripts/carla.sh)
#   ./scripts/start.sh webdrivesim foxglove   add extra containers by name

cd "$(dirname "$0")/../docker"                       # all docker commands run in the docker folder
source .env                                          # read the settings
export USER_UID=$(id -u) USER_GID=$(id -g)           # your user id: files made inside belong to you
export VIDEO_GID=$(getent group video | cut -d: -f3) # your PC's GPU groups (see docker/.env)
export RENDER_GID=$(getent group render | cut -d: -f3)
export RENDER_GID=${RENDER_GID:-$USER_GID}           # (older PCs have no "render" group)
export SIM=$1                                       # which simulator (passed to the containers)

# 1. choose the containers to start
if [ "$SIM" = webdrivesim ]; then
    CONTAINERS="webdrivesim acc dev"
elif [ "$SIM" = carla ]; then
    CONTAINERS="carla_bridge acc dev"
    # CARLA must already be running, otherwise the bridge has nothing to connect to
    if ! (echo > /dev/tcp/$CARLA_HOST/$CARLA_PORT) 2>/dev/null; then
        echo "CARLA is not running on $CARLA_HOST:$CARLA_PORT. Start it first: ./scripts/carla.sh"
        exit 1
    fi
else
    echo "Usage: ./scripts/start.sh webdrivesim|carla [foxglove]"
    exit 1
fi
shift                                                # the other words = extra containers
CONTAINERS="$CONTAINERS $@"

# 2. start them in the background (-d) and show what is running
xhost +local: > /dev/null 2>&1                       # allow containers to open windows on your screen
docker compose up -d $CONTAINERS
docker compose ps

if [ "$SIM" = webdrivesim ]; then echo "Open http://localhost:$WEBDRIVESIM_WEB_PORT in your browser"; fi
