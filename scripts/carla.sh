#!/bin/bash
# Start the CARLA simulator on your PC (outside Docker). Keep this terminal open; Ctrl+C stops CARLA.
# Settings in docker/.env: CARLA_ROOT (where you unpacked it), CARLA_QUALITY, CARLA_WINDOW, CARLA_PORT.

cd "$(dirname "$0")/../docker"
source .env

if [ ! -f "$CARLA_ROOT/CarlaUE4.sh" ]; then
    echo "CARLA not found in $CARLA_ROOT. Download and unpack it (docs/CARLA.md), or fix CARLA_ROOT in docker/.env"
    exit 1
fi

WINDOW="-RenderOffScreen"                            # no 3D window (lighter) ...
if [ "$CARLA_WINDOW" = true ]; then                  # ... unless you asked for it, in a window of CARLA_WINDOW_SIZE
    SIZE=${CARLA_WINDOW_SIZE:-1280x720}
    WINDOW="-windowed -ResX=${SIZE%x*} -ResY=${SIZE#*x}"
fi

# Start directly in CARLA_TOWN: the bridge then does not need to reload the map,
# which can take minutes (and freeze CARLA) on an integrated GPU.
# (If CARLA opens another town anyway, ./scripts/start.sh carla loads CARLA_TOWN.)
"$CARLA_ROOT/CarlaUE4.sh" "/Game/Carla/Maps/$CARLA_TOWN" \
    -carla-rpc-port=$CARLA_PORT -quality-level=$CARLA_QUALITY -nosound $WINDOW
