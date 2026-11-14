#!/bin/bash
# Drive the car yourself (our modules are NOT started).
#   ./scripts/drive_keyboard.sh webdrivesim   in the page: step 5 "Drive & test" -> Manual, then W/A/S/D
#   ./scripts/drive_keyboard.sh carla         start CARLA first (./scripts/carla.sh); then a window opens:
#                                             press B, then W/A/S/D (H = help)

cd "$(dirname "$0")/../docker"                       # all docker commands run in the docker folder
source .env                                          # read the settings
export USER_UID=$(id -u) USER_GID=$(id -g)           # your user id: files made inside belong to you
export VIDEO_GID=$(getent group video | cut -d: -f3) # your PC's GPU groups (see docker/.env)
export RENDER_GID=$(getent group render | cut -d: -f3)
export RENDER_GID=${RENDER_GID:-$USER_GID}           # (older PCs have no "render" group)
xhost +local: > /dev/null 2>&1

if [ "$1" = carla ]; then
    export SIM=carla
    export CARLA_CAMERAS=true CARLA_RENDER=true      # the driving window shows the rgb_view camera
    docker compose stop acc 2>/dev/null              # our ACC must not steer while you drive
    docker compose up -d --force-recreate carla_bridge   # link to CARLA (recreated: it needs the camera)
    docker compose up -d dev
    echo "Waiting 20 s for the bridge to load the map and spawn the car..."
    sleep 20
    # "exec" skips the container's entrypoint, so load ROS first (source /entrypoint.sh)
    # KEYBOARD_WINDOW_SIZE (docker/.env): size of the window
    docker compose exec -e KEYBOARD_WINDOW_SIZE=$KEYBOARD_WINDOW_SIZE dev bash -c "source /entrypoint.sh && \
        ros2 launch carla_manual_control carla_manual_control.launch.py role_name:=ego_vehicle"
else
    export SIM=webdrivesim
    docker compose up -d webdrivesim                    # our simulator, in the background
    echo "Open http://localhost:$WEBDRIVESIM_WEB_PORT -> step 5 'Drive & test' -> Manual"
fi
