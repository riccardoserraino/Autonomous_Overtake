#!/bin/bash
# Stop and remove all containers of the project. Your files are not touched.
# (CARLA runs outside Docker: stop it with Ctrl+C in its terminal.)

cd "$(dirname "$0")/../docker"
docker compose down
