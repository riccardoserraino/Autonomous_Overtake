#!/bin/bash
# Rebuild the simulator page after changing web/src (needs Node.js 18+), and install it in the ROS package.
set -e
cd "$(dirname "$0")/../web"
[ -d node_modules ] || npm install
node build.mjs
cp dist/webdrivesim.html ../ros2_ws/src/webdrivesim_bringup/web/index.html
echo "Copied to ros2_ws/src/webdrivesim_bringup/web/index.html (with --symlink-install no ROS rebuild is needed)."
