# ACAV — autonomous driving on highways and superstradas

Our ROS 2 code (ACC + lane keeping) plus two simulators to test it:

- **WebDriveSim**: light, runs in the browser on any PC. Start here.
- **CARLA 0.9.16**: realistic and heavy. Best with an NVIDIA GPU. Optional.

ROS 2 and our code run inside **Docker**, so on your PC you install only Docker (plus CARLA if you want it).
Run every command from the `acav` folder.

---

## 1. Setup (once)

**Step 1 — Install Docker:** official guide https://docs.docker.com/engine/install/ubuntu/,
then run Docker without `sudo` ([docker/README.md → 1. Install Docker](docker/README.md#1-install-docker-once), step 2).

**Step 2 — Make the scripts executable:**
```bash
cd ~/Desktop/acav
chmod +x scripts/*.sh third_party/webdrivesim/scripts/*.sh
```

**Step 3 — Build** (first time ~20 minutes):
```bash
./scripts/build.sh
```

Done: WebDriveSim is ready. For CARLA, continue with section 2.

---

## 2. CARLA (optional)

> Before this, follow the full **[CARLA guide](docs/CARLA.md)**: it covers the GPU drivers and the download.
> Here is only the short version.

**Step 1 — Unpack** the downloaded `CARLA_0.9.16.tar.gz` into `~/carla`:
```bash
mkdir -p ~/carla
tar -xzf ~/Downloads/CARLA_0.9.16.tar.gz -C ~/carla
```

**Step 2 — Start CARLA** (terminal 1, keep it open; wait until it stops printing, 1–2 min):
```bash
./scripts/carla.sh
```

GPU-specific settings (quality, window, rendering): [docker/README.md → GPU profiles](docker/README.md#gpu-profiles-for-carla).
Laptop or integrated GPU: put the PC in performance mode first ([docker/README.md → Full power](docker/README.md#4-full-power-for-carla-laptop-integrated-gpu)).

---

## 3. Run a session

### On WebDriveSim
```bash
./scripts/start.sh webdrivesim
```
Open **http://localhost:8080**. Our code drives the car.

### On CARLA (two terminals)
```bash
# terminal 1
./scripts/carla.sh
```
```bash
# terminal 2 (when CARLA stopped printing)
./scripts/start.sh carla
```

### After you edit the code
```bash
./scripts/restart.sh acc
```
It shows the log; Ctrl+C stops watching, not the container.

### End the session
```bash
./scripts/stop.sh
```
On CARLA, then press **Ctrl+C in terminal 1**. Nothing stays in memory; check with `docker ps` (empty list).

> **Never run both simulators at once**: same topics, double memory. Stop one, then start the other.

---

## 4. What to run after a change

| You changed… | Run |
|---|---|
| a `.py` file or `params.yaml` | `./scripts/restart.sh acc` |
| a setting in `docker/.env` | `./scripts/stop.sh` then `./scripts/start.sh <simulator>` |
| added a new file (node, launch, config) | `./scripts/build.sh code` then `./scripts/restart.sh acc` |
| the `Dockerfile` or `CARLA_VERSION` | `./scripts/stop.sh` then `./scripts/build.sh`, then start again |
| something is stuck | `./scripts/stop.sh`, then start again |

`restart.sh` does **not** pick up `.env` changes: those need stop + start.

---

## Where to go next

| I want to… | Go to |
|---|---|
| **write the algorithms: where code goes, workflow, tests** | **[DEVELOPMENT.md](DEVELOPMENT.md)** |
| use WebDriveSim, create maps and scenarios | [docs/WEBDRIVESIM.md](docs/WEBDRIVESIM.md) |
| install and run CARLA | [docs/CARLA.md](docs/CARLA.md) |
| change settings / GPU profile (`docker/.env`) | [docker/README.md](docker/README.md#3-change-the-settings-env) |
| see every script and its options | [scripts/README.md](scripts/README.md) |
| add or change CARLA sensors, plot the measurements | [DEVELOPMENT.md → Sensors](DEVELOPMENT.md#8-sensors) |
| change the code | [`acc.py`](ros2_ws/src/acc_lane_keeping/acc_lane_keeping/acc.py), [`lane_keeping.py`](ros2_ws/src/acc_lane_keeping/acc_lane_keeping/lane_keeping.py), tuning in [`params.yaml`](ros2_ws/src/bringup/config/params.yaml) |
| ROS commands, RViz | `./scripts/shell.sh` |
| drive by hand | `./scripts/drive_keyboard.sh webdrivesim` (or `carla`; window size: `KEYBOARD_WINDOW_SIZE` in `docker/.env`) |
| test without a simulator | `python3 tests/closed_loop_test.py ros2_ws/src` |

## Folders

```
acav/
├── docs/                  WEBDRIVESIM.md, CARLA.md (one guide per simulator)
├── docker/                Dockerfile, compose.yml, .env (all settings)
├── scripts/               build, start, stop, restart, shell, carla, drive_keyboard
├── maps/                  webdrivesim/ (our worlds), carla/ (extra CARLA roads)
├── tests/                 quick test without a simulator
├── third_party/webdrivesim/   the simulator's source (rarely opened)
└── ros2_ws/src/
    ├── bringup/           launch files + config/params.yaml
    └── acc_lane_keeping/  OUR CODE: acc.py (speed / distance), lane_keeping.py (steering)
```
