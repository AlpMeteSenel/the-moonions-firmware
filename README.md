# Plan Banana: tabletop lunar lander (firmware and hardware)

Part of **Plan Banana**, by team **The Moonions**, for NASA Space Apps Challenge 2026 ("Lunar Landing Site Navigator", Turin, 3rd place).

The web planner chooses where and when to land on the Moon and marks a primary zone (Z1) and a backup (Z2). This repository is the physical half: a 3D-printed lander that checks the ground during descent and diverts to Z2 when it sees a hazard on Z1.

| Repository | What it holds |
|---|---|
| [the-moonions](https://github.com/AlpMeteSenel/the-moonions) | Planner dashboard (Next.js, 3D Moon) |
| [the-moonions-backend](https://github.com/AlpMeteSenel/the-moonions-backend) | Planner backend and the trained touchdown model (Python, PyTorch) |
| **the-moonions-firmware** (this repo) | Rover Luna app for the Arduino UNO Q, plus the hardware files |

## Rover Luna (Arduino App Lab app)

It runs on an **Arduino UNO Q** with an IMX219 downward camera, a Modulino Movement IMU and a Grove OLED. The web UI is served on port 7000.

1. **Map:** take a photo of the tabletop Moon from the live view, or upload one. It's cropped to the board and saved on the board.
2. **Landing site:** paste the region JSON from the Plan Banana planner. Z1 and Z2 are placed on the map, with north rotation, and can be dragged to adjust.
3. **Mission:** the camera localises against the map (SIFT + FLANN + MAGSAC homography, optical flow between fixes). The IMU tracks heading between camera fixes. A red marker inside Z1 is treated as a hazard: Z1 becomes **NOT LANDABLE** and the target moves to Z2.

| File | Role |
|---|---|
| `python/main.py` | App, API, persistent setup, live mission loop |
| `python/matching.py` | Map matching |
| `python/landing_zone.py` | Landing-site change and hazard check |
| `python/hazard.py` | Red-hazard detection |
| `python/imu.py`, `python/map_pose.py` | Attitude and pose |
| `assets/index.html` | 3-step web UI |
| `sketch/` | MCU sketch (IMU, OLED) |

Install: in Arduino App Lab, import this folder as an app and press **Run**. To start it on boot, run `arduino-app-cli properties set default user:rover-luna`.

## Hardware (`hardware/`)

| Path | Contents |
|---|---|
| `rover_enclosure.py` | Parametric CAD (trimesh + manifold3d) for the rover casing, lid with snap ridges, OLED window, IMU cradle, camera hole, and the clip-on sky-crane descent stage |
| `stl/assembly`, `stl/assembly_descent` | Per-part STLs (rover, and rover with sky crane) |
| `print/*.gcode.3mf` | Sliced Bambu Lab A1 plates (casing about 3 h 27 min, sky crane 59 min) |
| `*.glb` | Assembled models for any 3D viewer |
| `renders/` | Blender scripts for the hero render and the 3D demo film |

## Honest limits

The hazard here is a coloured marker, not lidar. Localisation assumes a flat map. The hardware isn't space-qualified. It's a concept demonstrator for in-descent diversion.

AI coding assistants were used during development. The code and results were reviewed and tested by the team.
