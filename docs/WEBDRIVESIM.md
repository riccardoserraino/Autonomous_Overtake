← [Main README](../README.md) · [Scripts](../scripts/README.md) · [Docker & settings](../docker/README.md) · Other guide: [CARLA](CARLA.md)

# WebDriveSim guide

Our light simulator. It runs **in your browser**, starts in seconds, and uses the same ROS 2 topic names as CARLA,
so our code runs on it unchanged.

```
browser page (3D, traffic, sensors) ◄── port 9090 ──► Docker: webdrivesim ◄── ROS 2 ──► acc (our code)
        ▲
   http://localhost:8080
```

---

## Setup (once)

1. Docker installed and `./scripts/build.sh` done ([main README → Setup](../README.md#1-setup-once)). Nothing else to download.
2. Chrome or Firefox, with **hardware acceleration on** (Chrome: Settings → System).
3. Check the GPU is used: open `chrome://gpu` → it must say *"WebGL: Hardware accelerated"*.

Any GPU works, integrated included. If it stutters, make the browser window smaller.

---

## Run a session

**1. Start** (creates `webdrivesim`, `acc`, `dev`):
```bash
./scripts/start.sh webdrivesim
```

**2. Open** http://localhost:8080 — our code drives the car (step 5 *Drive & test* is set to **ROS 2 node**).

**3. After each code change:**
```bash
./scripts/restart.sh acc
```

**4. End:**
```bash
./scripts/stop.sh
```

Reloading the page restarts the scenario. Don't run CARLA at the same time.

---

## Create your own map and scenario

**1. Start only the simulator** (without our code):
```bash
./scripts/drive_keyboard.sh webdrivesim
```
Open http://localhost:8080.

**2. Build the world** with the 5 steps at the top of the page:

| Step | What you set |
|---|---|
| 1 Vehicle | our car: compact, sedan, SUV, van |
| 2 Sensors | radar, camera, lidar, GNSS, IMU: position, range, noise |
| 3 World & road | a preset (Motorway, Superstrada, Rural) or your own: straight, curve, S-curve, hairpin, hills, speed limits |
| 4 Traffic & start | where our car starts; other cars (lane, distance, speed); random traffic with a seed |
| 5 Drive & test | try it: **Manual** (W/A/S/D), **Autopilot**, or **ROS 2 node** |

**3. Export:** press **Export world** (top right), name it e.g. `my_test`.
You get `my_test.json` (scenario) and `my_test.xodr` (road).

**4. Move both files** into `maps/webdrivesim/`:
```bash
mv ~/Downloads/my_test.json ~/Downloads/my_test.xodr maps/webdrivesim/
```

**5. Load it** — choose one:

- **Every start:** set it in `docker/.env`
  ```
  WEBDRIVESIM_WORLD=my_test.json
  ```
  then
  ```bash
  ./scripts/stop.sh
  ./scripts/start.sh webdrivesim
  ```
- **Right now, no restart:**
  ```bash
  ./scripts/shell.sh
  ros2 param set /webdrivesim_web_server world my_test.json
  ```
  The open page switches by itself.

Example worlds in [`maps/webdrivesim/`](../maps/webdrivesim/): `empty_highway`, `superstrada_traffic`, `motorway_overtake`, `rural_overtake`.

> No Docker at hand? Open `third_party/webdrivesim/web/dist/webdrivesim.html` in a browser: everything works except the ROS link.

---

## Drive with the keyboard

```bash
./scripts/drive_keyboard.sh webdrivesim
```
In the page: step 5 → **Manual**.
Keys: `W/S` throttle/brake · `A/D` steer · `Z/X` blinkers · `Tab` camera · `R` reset · `H` help.
Our `acc` is not started here. Done: `./scripts/stop.sh`.

---

## Settings (`docker/.env`)

| Setting | Meaning |
|---|---|
| `WEBDRIVESIM_WEB_PORT` | port of the page (8080) |
| `WEBDRIVESIM_ROS_PORT` | port the page uses to talk to ROS (9090) |
| `WEBDRIVESIM_WORLD` | world loaded at start |
| `WEBDRIVESIM_SYNC` | `true` = repeatable runs: time advances only when our code answers |

Apply a change:
```bash
./scripts/stop.sh
./scripts/start.sh webdrivesim
```
then reload the page. More: [docker/README.md → Change the settings](../docker/README.md#3-change-the-settings-env).

---

## Main topics

| Topic | Meaning |
|---|---|
| `/carla/ego_vehicle/odometry` | our car's position and speed |
| `/carla/ego_vehicle/waypoints` | centre line of our lane |
| `/carla/ego_vehicle/objects` | the other cars |
| `/carla/ego_vehicle/imu`, `/gnss`, `/radar_front`, `/lidar` | sensors, with noise |
| `/webdrivesim/ego_vehicle/lane_state` | lane info: offset, lane width, line types … (on CARLA our `carla_adapter` publishes it too) |
| **in:** `/carla/ego_vehicle/vehicle_control_cmd` | throttle, brake, steer (steer +1 = full right) |

Changing the simulator itself: [third_party/webdrivesim/README.md](../third_party/webdrivesim/README.md).
Which simulator when: [CARLA vs WebDriveSim](CARLA.md#carla-vs-webdrivesim).

---
← [Main README](../README.md) · [Scripts](../scripts/README.md) · [Docker & settings](../docker/README.md) · [CARLA](CARLA.md)
