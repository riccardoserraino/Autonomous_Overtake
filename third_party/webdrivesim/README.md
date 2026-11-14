← [Main README](../../README.md) · [WebDriveSim guide](../../docs/WEBDRIVESIM.md)

# WebDriveSim — the simulator's files

How to use the simulator (start it, create maps and scenarios): **[WebDriveSim guide](../../docs/WEBDRIVESIM.md)**.
This folder is its source. You normally don't need to open it.

| Path | What it is | Needed to simulate? |
|---|---|---|
| `ros2_ws/src/webdrivesim_bringup/` | ROS 2 package that runs the simulator. `./scripts/start.sh webdrivesim` starts it | **yes** |
| ├ `web/index.html` | the simulator page sent to your browser | **yes** |
| ├ `webdrivesim_bringup/web_server.py` | serves the page and the worlds (port 8080) | **yes** |
| └ `launch/sim.launch.py` | starts the web server + rosbridge (page ↔ ROS, port 9090) | **yes** |
| `web/dist/webdrivesim.html` | the same page as one file: open it in a browser to design worlds without Docker | useful |
| `web/src/` | source code of the page (three.js): `main.js` interface, `sim.js` simulation, `road.js` roads, `ros.js` ROS topics, `world.js` 3D scene, `export.js` world export | only to change the simulator |
| `web/build.mjs`, `web/package*.json`, `scripts/build_web.sh` | rebuild the page after changing `web/src` (needs Node.js 18+) | only to change the simulator |
| `tools/make_worlds.mjs` | regenerates the example worlds in `maps/webdrivesim/` | rarely |

**Change the simulator:**
1. Edit `web/src/`.
2. Run `third_party/webdrivesim/scripts/build_web.sh`.
3. Run `./scripts/restart.sh webdrivesim` and reload the page.
