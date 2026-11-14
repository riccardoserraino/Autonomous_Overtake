← [Main README](../README.md) · [Docker & settings](../docker/README.md) · [WebDriveSim](../docs/WEBDRIVESIM.md) · [CARLA](../docs/CARLA.md)

# scripts/

Short commands for everything. Run them from the `acav` folder (`cd ~/Desktop/acav`).
First time only: `chmod +x scripts/*.sh third_party/webdrivesim/scripts/*.sh`.
How each script works is explained in the comments at its top.

## Commands and options

| Command | Options | Starts these containers |
|---|---|---|
| `./scripts/build.sh` | none = image + our code (first time, after a Dockerfile change)<br>`code` = only our code, in seconds (after adding files) | a temporary one, deleted at the end |
| `./scripts/start.sh <sim> [extra]` | `webdrivesim` = our simulator + our code<br>`carla` = link to CARLA + our code (start `carla.sh` first)<br>`[extra]` = more containers by name, e.g. `foxglove` | `webdrivesim` → **webdrivesim, acc, dev**<br>`carla` → **carla_bridge, acc, dev**<br>+ each extra |
| `./scripts/restart.sh <name>` | `acc` (our code), `webdrivesim`, `carla_bridge`, `foxglove` | none: restarts that one and shows its log (Ctrl+C stops watching, not the container) |
| `./scripts/stop.sh` | none | none: removes all containers |
| `./scripts/shell.sh` | none | **dev**, if not running; opens a terminal in it (`exit` to leave) |
| `./scripts/carla.sh` | none: settings from `docker/.env` | none: starts CARLA on your PC. Ctrl+C stops it |
| `./scripts/drive_keyboard.sh <sim>` | `webdrivesim` or `carla` | `webdrivesim` → **webdrivesim**<br>`carla` → **carla_bridge, dev** (and stops `acc`); window size: `KEYBOARD_WINDOW_SIZE` in `docker/.env` |

Examples:
```bash
./scripts/start.sh webdrivesim foxglove   # our simulator + our code + Foxglove
./scripts/restart.sh carla_bridge         # reconnect to CARLA and spawn our car again
./scripts/build.sh code                   # after adding a new node or launch file
```

## Stop or restart?

| Situation | Command |
|---|---|
| end of a session | `./scripts/stop.sh` (+ Ctrl+C in CARLA). Enough: nothing stays in memory |
| changed a `.py` or `params.yaml` | `./scripts/restart.sh acc` |
| changed `docker/.env` | `./scripts/stop.sh`, then `./scripts/start.sh <sim>` (`restart.sh` keeps the old settings) |
| added a new file | `./scripts/build.sh code`, then `./scripts/restart.sh acc` |
| switch simulator | `./scripts/stop.sh`, then start the other one |

## Typical session

```bash
./scripts/start.sh webdrivesim     # open http://localhost:8080
# edit ros2_ws/src/acc_lane_keeping/acc_lane_keeping/acc.py
./scripts/restart.sh acc
./scripts/stop.sh
```

---
← [Main README](../README.md) · [Docker & settings](../docker/README.md) · [WebDriveSim](../docs/WEBDRIVESIM.md) · [CARLA](../docs/CARLA.md)
