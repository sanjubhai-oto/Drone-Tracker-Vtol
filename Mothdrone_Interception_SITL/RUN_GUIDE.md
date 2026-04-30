# Mothdrone Interception System - Run Guide

## Quick Start (Simulated - No PX4 Required)

```bash
cd "Mothdrone Interception System (dosan)"
pip3 install -r requirements.txt
python3 code/simulated_runner.py
```

This runs the full interception algorithm in pure Python — shows telemetry, trigger events, and target freefall.

## Full SITL Simulation (Ubuntu/Linux Recommended)

### Prerequisites

1. **PX4-Autopilot** built with SITL support:
```bash
cd ~/PX4-Autopilot
make px4_sitl_default
```

2. **Gazebo (gz sim)** installed:
```bash
# Ubuntu 22.04
sudo apt install gz-garden

# Or newer versions
sudo apt install gz-harmonic
```

3. **MAVSDK-Python**:
```bash
pip3 install mavsdk
```

4. **QGroundControl** (for telemetry viewing):
```bash
# Download from https://docs.qgroundcontrol.com/master/en/getting_started/download_and_install.html
```

### Running the Full Simulation

#### Option 1: One-Command Launch (Automated)

```bash
cd "Mothdrone Interception System (dosan)"
python3 launch_mothdrone.py
```

This will:
1. Start Gazebo with the Mothdrone world
2. Spawn hunter and target VTOLs
3. Start PX4 SITL for both vehicles
4. Run the MAVSDK interception controller
5. Show live telemetry

#### Option 2: Manual Step-by-Step

**Terminal 1 - Start Gazebo:**
```bash
cd ~/PX4-Autopilot/Tools/simulation/gz
python3 simulation-gazebo --world mothdrone_world --headless
```

**Terminal 2 - Start Hunter PX4:**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_standard_vtol ./build/px4_sitl_default/bin/px4 -i 0
```

**Terminal 3 - Start Target PX4:**
```bash
cd ~/PX4-Autopilot
PX4_SYS_AUTOSTART=4001 PX4_SIM_MODEL=gz_standard_vtol ./build/px4_sitl_default/bin/px4 -i 1
```

**Terminal 4 - Start QGC Telemetry Forwarder:**
```bash
cd "Mothdrone Interception System (dosan)"
python3 code/telemetry_forwarder.py
```

**Terminal 5 - Run Mission Controller:**
```bash
cd "Mothdrone Interception System (dosan)"
python3 code/mothdrone_controller.py
```

### macOS Notes

PX4 SITL on macOS is **experimental**. The simulated runner (`simulated_runner.py`) is the recommended approach on macOS.

If you want to try SITL on macOS:
1. Install dependencies: `pip3 install cmake ninja mavsdk pymavlink`
2. Build PX4: `make px4_sitl_default` (may fail due to missing system libs)
3. Gazebo on macOS runs headless only (`gz sim -s`)
4. Use the simulated runner for algorithm validation

## Telemetry Viewing

### QGroundControl
1. Open QGroundControl
2. It will auto-detect MAVLink on UDP 14550
3. You'll see both vehicles on the map
4. Switch between vehicles using the dropdown

### MAVProxy (Command Line)
```bash
mavproxy.py --master=udp:127.0.0.1:14540 --out=udp:127.0.0.1:14550
```

## Live 3D Trajectory Graph

Run this in a separate terminal before or during SITL:

```bash
cd "Mothdrone Interception System (dosan)"
python3 code/live_trajectory_server.py
```

Open:

```text
http://127.0.0.1:8790/live_trajectory.html
```

The live page refreshes from `mothdrone_telemetry.json` every 0.5 s. The controller writes that file during flight, not only at mission end.

Static images from the latest verified run:

- `outputs/graphs/mothdrone_trajectory_3d.svg`
- `outputs/graphs/mothdrone_trajectory_path.svg`
- `outputs/graphs/mothdrone_mission_graph.svg`

## Mission Parameters

Edit `configs/mission_config.json` to change:
- Kill zone radius (default: 25m)
- Vision confirmation radius (default: 35m)
- Takeoff altitude (default: 20m)
- Breakaway altitude (default: 40m)
- Hunter max speed (current verified: 5 m/s)
- Target speed (current verified: 2 m/s)

## Troubleshooting

| Issue | Solution |
|-------|----------|
| "px4 binary not found" | Build PX4: `make px4_sitl_default` |
| "gz sim not found" | Install Gazebo: `sudo apt install gz-garden` |
| "mavsdk not found" | Install: `pip3 install mavsdk` |
| QGC not showing telemetry | Check firewall, ensure UDP 14550 is open |
| Hunter not moving | Check offboard mode is enabled in PX4 params |
| Target not moving | Ensure velocity setpoints are being sent |

## Algorithm Flow

```
1. SPAWN:    Hunter at (0,0), Target at 40m east
2. TAKEOFF:  Both arm, enter offboard, and climb past 20m
3. PURSUE:   Target runs 2m/s mission; hunter listens and chases target
4. CONFIRM:  Vision lock required when range < 35m
5. TRIGGER:  At range <= 25m with vision lock → MOTOR STOP
6. FREEFALL: Target receives SITL-only kill and falls/disarms
7. IMPACT:   Target hits ground, disarms
8. BREAKAWAY: Hunter climbs to 40m
9. RTL:      Hunter returns to launch and lands
```

## Files Reference

| File | Purpose |
|------|---------|
| `launch_mothdrone.py` | One-command launcher for full sim |
| `code/mothdrone_controller.py` | MAVSDK mission controller |
| `code/simulated_runner.py` | Pure Python simulation (no PX4) |
| `code/telemetry_forwarder.py` | QGC telemetry bridge |
| `code/live_trajectory_server.py` | Live browser trajectory graph |
| `worlds/mothdrone_world.sdf` | Gazebo world with both VTOLs |
| `configs/mission_config.json` | Mission parameters |
| `code/vtol_proximity_guidance.py` | Core guidance algorithm |

## Safety Notice

This system is for **SITL simulation only**. The motor-stop/freefall logic is:
- Implemented as simulation physics
- Triggered via MAVSDK commands in SITL
- **NEVER** tested on real hardware
- **NEVER** used for actual C-UAS operations without legal review
