#!/bin/bash
# Restart ONE container after changing its code or config/params.yaml, then show its log.
#   ./scripts/restart.sh acc          (or: webdrivesim, carla_bridge, foxglove)
# Ctrl+C stops watching the log, NOT the container.

cd "$(dirname "$0")/../docker"
docker compose restart $1
docker compose logs -f --tail 20 $1
