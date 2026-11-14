← [Main README](../README.md) · [Scripts](../scripts/README.md) · [Docker & settings](../docker/README.md) · Other guide: [WebDriveSim](WEBDRIVESIM.md)

# CARLA guide

CARLA is a realistic simulator (Unreal Engine graphics, full physics, cameras, radar, lidar).
It runs **on your PC, outside Docker**; our containers connect to it on port 2000.

```
your PC:  CARLA 0.9.16 (./scripts/carla.sh)  ◄── port 2000 ──►  Docker: carla_bridge + acc (our code)
```

Before you start: Docker installed and `./scripts/build.sh` done ([main README → Setup](../README.md#1-setup-once)).

---

## Setup (once)

### Step 1 — Install the GPU driver

Pick **your** case.

**A) NVIDIA GPU**
```bash
sudo apt update
sudo ubuntu-drivers install
sudo apt install vulkan-tools
sudo reboot
```
After the reboot, check:
```bash
nvidia-smi                 # must show your card
vulkaninfo --summary       # must list your NVIDIA GPU
```

**B) Integrated GPU (Intel Iris/UHD, AMD Radeon in a Ryzen)**
```bash
sudo apt update
sudo apt install mesa-vulkan-drivers vulkan-tools
```
Check:
```bash
vulkaninfo --summary       # must list your GPU (not only "llvmpipe")
```

### Step 2 — Download CARLA 0.9.16

Go to https://github.com/carla-simulator/carla/releases → **0.9.16** → download **CARLA_0.9.16.tar.gz [Ubuntu]**
(~16 GB unpacked). It must be **0.9.16**, the same as the client in our Docker image.
Ubuntu 24.04 works too.

### Step 3 — Unpack it
```bash
mkdir -p ~/carla
tar -xzf ~/Downloads/CARLA_0.9.16.tar.gz -C ~/carla
```
Using another folder? Set `CARLA_ROOT=/path/to/carla` in `docker/.env`.

### Step 4 — Set the profile for your GPU

Copy the block for your GPU into `docker/.env`:
**[docker/README.md → GPU profiles](../docker/README.md#gpu-profiles-for-carla)**.
Laptop or integrated GPU: also set it to **full power**
([docker/README.md → Full power](../docker/README.md#4-full-power-for-carla-laptop-integrated-gpu)): charger in, performance mode.

### Step 5 — First test
```bash
./scripts/carla.sh
```
Wait until it stops printing (the first time it takes a few minutes). Ctrl+C closes it.
If it says *CARLA not found*, check Step 3 or `CARLA_ROOT`.

---

## Run a session

**1. Terminal 1 — start CARLA** (keep it open, wait until it stops printing):
```bash
./scripts/carla.sh
```
> Laptop with Intel + NVIDIA? Force the NVIDIA card:
> ```bash
> __NV_PRIME_RENDER_OFFLOAD=1 __VK_LAYER_NV_optimus=NVIDIA_only ./scripts/carla.sh
> ```

**2. Terminal 2 — start our side:**
```bash
./scripts/start.sh carla
```

**3. Check that our car is in the world** (look for *"Created EgoVehicle"*, Ctrl+C stops watching):
```bash
docker compose -f docker/compose.yml logs -f carla_bridge
```

**4. After each code change:**
```bash
./scripts/restart.sh acc
```

**5. End — our side first, then CARLA:**
```bash
./scripts/stop.sh
```
Then **Ctrl+C in terminal 1**. Errors printed by CARLA at Ctrl+C are normal.

Notes:
- `start.sh carla` refuses to start if CARLA isn't running yet: start it and retry.
- If CARLA opened another town, step 2 loads `CARLA_TOWN` (can take minutes).

### Restart CARLA

Needed when it froze, or after changing `CARLA_QUALITY`, `CARLA_WINDOW`, `CARLA_WINDOW_SIZE`, `CARLA_TOWN`:
```bash
./scripts/stop.sh            # 1. our side
# 2. Ctrl+C in the CARLA terminal
./scripts/carla.sh           # 3. terminal 1, wait until it stops printing
./scripts/start.sh carla     # 4. terminal 2
```

---

## Drive with the keyboard

```bash
./scripts/carla.sh                    # terminal 1
```
```bash
./scripts/drive_keyboard.sh carla     # terminal 2: a window opens. Press B, then W/A/S/D (H = help)
```
It stops `acc` (our code must not steer while you drive) and turns cameras and rendering on: the window shows
the `rgb_view` camera behind the car.

**Window size:** `KEYBOARD_WINDOW_SIZE` in `docker/.env` (e.g. `1366x768`, or `960x540` if it is slow).
The camera image gets the same size. To change it: close the window (Ctrl+C), edit `.env`, run the script again.
The first time after updating the `Dockerfile`, run `./scripts/build.sh` once, otherwise the window stays 800x600.

Back to our code:
```bash
./scripts/stop.sh
./scripts/start.sh carla
```

---

## Maps

1. Set the town in `docker/.env`, e.g.:
   ```
   CARLA_TOWN=Town04_Opt
   ```
2. Do [Restart CARLA](#restart-carla).

| Town | Road | Needs AdditionalMaps |
|---|---|---|
| `Town04` / `Town04_Opt` | highway loop, several lanes (`_Opt` = lighter) | no |
| `Town06` | long multi-lane roads | yes |
| `Town02` | two-way roads, dashed and solid lines | no |
| `Town07` | two-way rural roads | yes |

**AdditionalMaps:** download **AdditionalMaps_0.9.16.tar.gz** from the same release page, then:
```bash
tar -xzf ~/Downloads/AdditionalMaps_0.9.16.tar.gz -C ~/carla
```
and restart CARLA.

---

## If something goes wrong

- **Slow or freezing:** full power mode ([docker/README.md → 4](../docker/README.md#4-full-power-for-carla-laptop-integrated-gpu)),
  then a lighter [GPU profile](../docker/README.md#gpu-profiles-for-carla).
- **`CARLA not found`:** check `CARLA_ROOT` in `docker/.env`.
- **`start.sh carla` refuses:** CARLA isn't up yet; wait until `carla.sh` stops printing.

---

## CARLA vs WebDriveSim

| | CARLA | WebDriveSim |
|---|---|---|
| Physics | full vehicle physics | simple |
| Sensors | rendered camera images, lidar, radar | objects with noise |
| GPU | needed (NVIDIA best; integrated: slow) | any, runs in the browser |
| Start-up | minutes | seconds |
| Memory | high (several GB) | low |
| Own maps | `.xodr` roads | built in the page |

Develop in [WebDriveSim](WEBDRIVESIM.md), validate in CARLA.

---
← [Main README](../README.md) · [Scripts](../scripts/README.md) · [Docker & settings](../docker/README.md) · [WebDriveSim](WEBDRIVESIM.md)
