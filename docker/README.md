← [Main README](../README.md) · [Scripts](../scripts/README.md) · [WebDriveSim](../docs/WEBDRIVESIM.md) · [CARLA](../docs/CARLA.md)

# docker/ — the system our code runs in

Docker builds a complete system (Ubuntu 22.04 + ROS 2 Humble) from a recipe, so nobody installs ROS on their PC.

```
Dockerfile ──(./scripts/build.sh)──► IMAGE acav:humble ──(./scripts/start.sh)──► CONTAINERS (running copies)
```

- The **image** is the installed system, built once and kept on disk.
- A **container** is a running copy of it. We run one per job (`webdrivesim`, `carla_bridge`, `acc`, `dev`).
- Your code is **not** inside the image: the `acav` folder is shared with the containers, so your edits are used immediately.
- CARLA is **not** in Docker: it runs on your PC ([CARLA guide](../docs/CARLA.md)).

You use Docker **through the [scripts](../scripts/README.md)**. This page is for installing it and changing settings.

| File | What it is | When you change it |
|---|---|---|
| `.env` | all settings: ports, maps, CARLA folder and quality | often (see [3](#3-change-the-settings-env)) |
| `Dockerfile` | the recipe of the image | to add a library, then `./scripts/build.sh` |
| `compose.yml` | the list of containers | to add a container, or for the NVIDIA option |
| `entrypoint.sh` | loads ROS in every container | never |

## 1. Install Docker (once)

**Step 1 — Install Docker Engine** with the official guide:
**https://docs.docker.com/engine/install/ubuntu/** (section *"Install using the apt repository"*).

**Step 2 — Use Docker without `sudo`:**
```bash
sudo usermod -aG docker $USER
```
Log out and back in, then check:
```bash
docker run hello-world      # prints "Hello from Docker!"
```

**Step 3 — GPU:** nothing to do. The containers already use your GPU (integrated or NVIDIA).

> **Optional, NVIDIA only** (faster RViz/Gazebo windows from Docker):
> 1. Install the NVIDIA Container Toolkit: https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html
> 2. Enable it:
>    ```bash
>    sudo nvidia-ctk runtime configure --runtime=docker
>    sudo systemctl restart docker
>    ```
> 3. In `compose.yml`, remove the `#` before `runtime: nvidia`
>    (Intel + NVIDIA laptop: also before the two lines starting with `__`).
> 4. Check inside `./scripts/shell.sh`: `glxinfo -B | grep "renderer string"` shows your GPU, not `llvmpipe`.

Done: continue with the [main README → Setup](../README.md#1-setup-once).

## 2. The containers

| Container | What it is | Created by |
|---|---|---|
| `webdrivesim` | our simulator: web page server + ROS link | `start.sh webdrivesim`, `drive_keyboard.sh webdrivesim` |
| `carla_bridge` | the link to CARLA: topics, our car, lane info | `start.sh carla`, `drive_keyboard.sh carla` |
| `acc` | **our code**: ACC + lane keeping | `start.sh webdrivesim` and `start.sh carla` |
| `dev` | an empty container to open terminals in (it only sleeps: almost no memory) | `start.sh ...`, `shell.sh` |
| `foxglove` | web visualisation (port 8765) | only if added: `start.sh webdrivesim foxglove` |

In `docker ps` they appear as `docker-acc-1`, `docker-webdrivesim-1`, … ("docker" = this folder's name).
See them with `docker ps`, or their log with `docker compose -f docker/compose.yml logs -f acc` (from the `acav` folder; Ctrl+C stops watching).

## 3. Change the settings (`.env`)

All settings are in `docker/.env`: open it, change the value after `=`, save. Then apply it:

| Setting read by | Settings | Apply with |
|---|---|---|
| **CARLA itself** (`carla.sh`) | `CARLA_ROOT`, `CARLA_PORT`, `CARLA_QUALITY`, `CARLA_WINDOW`, `CARLA_WINDOW_SIZE`, `CARLA_TOWN` | Ctrl+C in CARLA's terminal, `./scripts/carla.sh` again |
| **our containers** | `WEBDRIVESIM_*`, `CARLA_HOST`, `CARLA_PORT`, `CARLA_TOWN`, `CARLA_RENDER`, `CARLA_CAMERAS`, `ROS_DOMAIN_ID`, `FOXGLOVE_PORT` | `./scripts/stop.sh`, then `./scripts/start.sh <simulator>` |
| **keyboard driving** | `KEYBOARD_WINDOW_SIZE` | close the window (Ctrl+C), run `./scripts/drive_keyboard.sh carla` again |
| **the image** | `CARLA_VERSION` | `./scripts/stop.sh`, `./scripts/build.sh`, then start again |

`./scripts/restart.sh` is **not** enough for `.env`: a restarted container keeps the settings it was created with.
`CARLA_TOWN` and `CARLA_PORT` are in both rows: restart CARLA and our containers.

### GPU profiles for CARLA

Open `docker/.env`, replace the CARLA lines with the block for your GPU, then [restart CARLA](../docs/CARLA.md#restart-carla).

**NVIDIA GPU** — smooth 3D window:
```
CARLA_QUALITY=Epic
CARLA_WINDOW=true
CARLA_WINDOW_SIZE=1280x720
CARLA_RENDER=true
```
(`CARLA_QUALITY=Medium` if it is not smooth.)

**Integrated GPU** (Intel Iris/UHD, AMD Radeon in a Ryzen) — slow but usable:
```
CARLA_QUALITY=Low
CARLA_WINDOW=false
CARLA_RENDER=true
CARLA_CAMERAS=false
```

**Very weak GPU / still too slow** — CARLA draws nothing; physics and our code still run
(our code uses ground-truth topics, not images). Watch the car in RViz or Foxglove:
```
CARLA_QUALITY=Low
CARLA_WINDOW=false
CARLA_RENDER=false
CARLA_CAMERAS=false
```

### The settings you change most

Every setting is explained by the comment next to it in `.env`. The common ones:

| Setting | What it does |
|---|---|
| `CARLA_QUALITY` | `Low` (integrated GPU) or `Epic` (NVIDIA) graphics |
| `CARLA_WINDOW`, `CARLA_WINDOW_SIZE` | show CARLA's own 3D window (free camera), and its size. Smaller = faster |
| `CARLA_RENDER` | `false` = CARLA draws nothing at all (fastest; physics and our code still run) |
| `CARLA_CAMERAS` | `true` = also spawn cameras + radar ([sensors](../DEVELOPMENT.md#8-sensors)). Heavy |
| `KEYBOARD_WINDOW_SIZE` | size of the `drive_keyboard.sh carla` window and of the camera it shows, e.g. `1366x768`. Smaller = faster |
| `CARLA_TOWN` | CARLA map ([list](../docs/CARLA.md#maps)) |
| `WEBDRIVESIM_WORLD` | WebDriveSim world loaded at start (file in `maps/webdrivesim/`) |
| `WEBDRIVESIM_SYNC` | `true` = repeatable WebDriveSim runs |

`USER_UID`, `USER_GID`, `VIDEO_GID`, `RENDER_GID` are filled in by the scripts: leave them.

## 4. Full power for CARLA (laptop, integrated GPU)

A laptop runs in a "balanced" mode by default: the CPU and the integrated GPU (which has no own memory and
shares the RAM) are kept at a fraction of their speed to stay cool and quiet. For CARLA, give them everything.
Expect more heat, louder fans and a faster-draining battery: do it only for CARLA sessions.

**Step 1 — Charger in.** On battery the GPU is always slowed down.

**Step 2 — Performance mode.** Settings → Power → Power Mode → **Performance**, or:
```bash
powerprofilesctl set performance
powerprofilesctl get                     # shows: performance
```
On Lenovo laptops (Yoga, IdeaPad) this also switches the fan/thermal mode to "performance": the fans run faster,
so the chip can stay at high power longer before it slows down from heat. Check:
```bash
cat /sys/firmware/acpi/platform_profile  # shows: performance
```
This step gives the biggest gain. It stays until you change it back (`powerprofilesctl set balanced`).

**Step 3 — Keep the integrated GPU at full clock** (optional, small extra gain). Normally the GPU lowers its
frequency when it has little to do and raises it slowly; here its minimum is set equal to its maximum.
Undone by a reboot.

- **Intel:**
  ```bash
  for c in /sys/class/drm/card?; do [ -f $c/gt_max_freq_mhz ] && sudo tee $c/gt_min_freq_mhz < $c/gt_max_freq_mhz; done
  cat /sys/class/drm/card?/gt_cur_freq_mhz          # current frequency: now near the maximum
  ```
  Newer Intel chips (Core Ultra) have no `gt_*` files: skip this step, step 2 is enough.
- **AMD:**
  ```bash
  echo high | sudo tee /sys/class/drm/card?/device/power_dpm_force_performance_level
  ```

Watch the GPU while CARLA runs: `sudo intel_gpu_top` (Intel, package `intel-gpu-tools`) or `radeontop` (AMD).

**Step 4 — Free the RAM.** The integrated GPU uses your RAM as video memory: close browsers and other heavy
programs before starting CARLA, and run only one simulator.

## 5. Save memory on your PC

- End every session with `./scripts/stop.sh`: containers are removed and free their RAM. Then `docker ps` is empty.
- Run **one** simulator at a time, and close CARLA (Ctrl+C) when you don't use it.
- Add `foxglove` only when you need it.
- See what each container uses: `docker stats` (Ctrl+C to quit).
- Disk: every `./scripts/build.sh` can leave old unused image layers. Free them with `docker image prune`
  (keeps `acav:humble`). See the total with `docker system df`.

## 6. Change the system

**Add a Python or Ubuntu library:** add it to the `pip3 install` or `apt-get install` line in the `Dockerfile`, then `./scripts/build.sh`.

**Add a container** (e.g. a second module): in `compose.yml`, copy the `acc` block, rename it and change its `command`.
Start it with `./scripts/start.sh webdrivesim <its name>`.

**Gazebo (optional):** at the end of the `Dockerfile` there's a commented Gazebo block. Remove its `#` signs and run `./scripts/build.sh`.

---
← [Main README](../README.md) · [Scripts](../scripts/README.md) · [WebDriveSim](../docs/WEBDRIVESIM.md) · [CARLA](../docs/CARLA.md)
