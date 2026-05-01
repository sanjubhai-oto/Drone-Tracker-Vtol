#!/usr/bin/env python3
"""
X500 Gimbal Drone Tracker System - Main Launcher
Spawns 2 PX4 SITL X500 gimbals, runs Gazebo, starts MAVSDK control, streams QGC telemetry.

Usage:
    python3 launch_x500_gimbal.py

Requirements:
    - PX4-Autopilot built and available at ~/PX4-Autopilot
    - Gazebo (gz sim) installed
    - MAVSDK-Python: pip install mavsdk
    - QGroundControl running (optional, for telemetry view)
"""

import asyncio
import subprocess
import time
import os
import signal
import sys
from pathlib import Path

# Configuration
PX4_DIR = Path(os.environ.get("X500_GIMBAL_PX4_DIR", str(Path.home() / "PX4-Autopilot"))).expanduser()
PX4_BUILD_DIR = PX4_DIR / "build" / "px4_sitl_default"
PX4_GZ_DIR = PX4_DIR / "Tools" / "simulation" / "gz"
PX4_GZ_PLUGINS = PX4_BUILD_DIR / "src" / "modules" / "simulation" / "gz_plugins"
PX4_GZ_SERVER_CONFIG = PX4_DIR / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"
LOCAL_GZ_STORE = Path.home() / ".simulation-gazebo"
GZ_WORLD = os.environ.get("X500_GIMBAL_GZ_WORLD", "default")
EXPLICIT_GZ = os.environ.get("X500_GIMBAL_EXPLICIT_GZ", "0") == "1"
GZ_AUTO_FOLLOW = os.environ.get("X500_GIMBAL_GZ_AUTO_FOLLOW", "0") == "1"
HUNTER_UDP = "udpin://127.0.0.1:14540"
TARGET_UDP = "udpin://127.0.0.1:14541"
QGC_UDP = "udp://127.0.0.1:14550"
PACKAGE_DIR = Path(__file__).resolve().parent
LOG_DIR = PACKAGE_DIR / "logs"
LOG_DIR.mkdir(exist_ok=True)
sys.path.insert(0, str(PACKAGE_DIR / "code"))

class SimulationManager:
    def __init__(self):
        self.processes = []

    def _gz_env(self):
        env = os.environ.copy()
        env["GZ_IP"] = "127.0.0.1"
        env["PX4_GZ_MODELS"] = str(PX4_GZ_DIR / "models")
        env["PX4_GZ_WORLDS"] = str(PX4_GZ_DIR / "worlds")
        env["PX4_GZ_PLUGINS"] = str(PX4_GZ_PLUGINS)
        env["GZ_SIM_RESOURCE_PATH"] = os.pathsep.join([
            str(LOCAL_GZ_STORE / "models"),
            str(LOCAL_GZ_STORE / "worlds"),
            str(PX4_GZ_DIR / "models"),
            str(PX4_GZ_DIR / "worlds"),
            str(PACKAGE_DIR / "models"),
            str(PACKAGE_DIR / "worlds"),
        ])
        existing_plugin_path = env.get("GZ_SIM_SYSTEM_PLUGIN_PATH", "")
        plugin_paths = [str(PX4_GZ_PLUGINS)]
        if existing_plugin_path:
            plugin_paths.append(existing_plugin_path)
        env["GZ_SIM_SYSTEM_PLUGIN_PATH"] = os.pathsep.join(plugin_paths)
        if PX4_GZ_SERVER_CONFIG.exists():
            env["GZ_SIM_SERVER_CONFIG_PATH"] = str(PX4_GZ_SERVER_CONFIG)
        else:
            server_config = LOCAL_GZ_STORE / "server.config"
            if server_config.exists():
                env["GZ_SIM_SERVER_CONFIG_PATH"] = str(server_config)
        return env

    def start_gazebo_world(self):
        """Start Gazebo explicitly before PX4, useful on Linux when PX4-started GZ has no sensors."""
        world_path = PX4_GZ_DIR / "worlds" / f"{GZ_WORLD}.sdf"
        if not world_path.exists():
            world_path = PACKAGE_DIR / "worlds" / "px4_default" / "default.sdf"
        gz_cmd = ["gz", "sim", "-r", str(world_path)]
        print(f"[LAUNCHER] Starting explicit Gazebo world: {world_path}")
        proc = subprocess.Popen(
            gz_cmd,
            cwd=PACKAGE_DIR,
            env=self._gz_env(),
            stdout=open(LOG_DIR / "gazebo.log", "w"),
            stderr=open(LOG_DIR / "gazebo.err.log", "w"),
        )
        self.processes.append(proc)
        time.sleep(8)
        
    def start_px4_instance(self, instance_id: int, pose: str, standalone: bool, model: str = "x500_gimbal"):
        """Start a PX4 SITL instance and let PX4 spawn a X500 gimbal in Gazebo."""
        build_dir = PX4_BUILD_DIR
        working_dir = build_dir / f"instance_{instance_id}"
        working_dir.mkdir(exist_ok=True)
        
        env = self._gz_env()
        env["PX4_SIM_MODEL"] = f"gz_{model}"
        # PX4 airframe 4019 is gz_x500_gimbal.
        env["PX4_SYS_AUTOSTART"] = "4019"
        env["PX4_GZ_WORLD"] = GZ_WORLD
        env["PX4_GZ_MODEL_POSE"] = pose
        if not GZ_AUTO_FOLLOW:
            env["PX4_GZ_NO_FOLLOW"] = "1"
        if instance_id == 1:
            # Target is 100m ahead of hunter. A wide diagonal follow camera keeps
            # both X500 gimbals in frame while the hunter closes the range.
            env["PX4_GZ_FOLLOW_OFFSET_X"] = "-70"
            env["PX4_GZ_FOLLOW_OFFSET_Y"] = "-60"
            env["PX4_GZ_FOLLOW_OFFSET_Z"] = "45"
        if standalone:
            env["PX4_GZ_STANDALONE"] = "1"
        
        px4_cmd = [
            str(build_dir / "bin" / "px4"),
            "-i", str(instance_id),
            "-d", str(build_dir / "etc")
        ]
        
        role = "hunter" if instance_id == 0 else "target"
        print(f"[LAUNCHER] Starting PX4 {role} instance {instance_id} ({model}) pose={pose} standalone={standalone}...")
        proc = subprocess.Popen(
            px4_cmd,
            cwd=working_dir,
            env=env,
            stdout=open(LOG_DIR / f"{role}_px4.log", "w"),
            stderr=open(LOG_DIR / f"{role}_px4.err.log", "w")
        )
        self.processes.append(proc)
        time.sleep(8 if instance_id == 0 else 4)
        
    def start_mavlink_router(self):
        """Start mavlink-router to forward telemetry to QGC."""
        # Forward both instances to QGC on 14550
        router_cmd = [
            "python3", "-m", "pymavlink.tools.mavproxy",
            f"--master={HUNTER_UDP}",
            f"--out={QGC_UDP}",
            f"--out=udp:127.0.0.1:14551"  # Target also to QGC
        ]
        print("[LAUNCHER] Starting MAVLink router for QGC...")
        proc = subprocess.Popen(router_cmd)
        self.processes.append(proc)
        
    def cleanup(self):
        """Kill all spawned processes."""
        print("[LAUNCHER] Cleaning up...")
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                proc.kill()
        subprocess.run(["pkill", "-f", "mavsdk_server"], capture_output=True)
        subprocess.run(["pkill", "-f", "px4_sitl_default/bin/px4"], capture_output=True)
        subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
        
async def run_mission():
    """Run the interception mission."""
    from x500_gimbal_controller import X500GimbalTrackerController

    # This launcher is SITL-only. It enables the target motor-stop/freefall event
    # in the simulator so target does not enter PX4 RTL after the trigger.
    os.environ["X500_GIMBAL_SITL_TARGET_KILL"] = "1"
    
    controller = X500GimbalTrackerController(
        hunter_addr=HUNTER_UDP,
        target_addr=TARGET_UDP
    )
    
    print("[MISSION] Connecting to vehicles...")
    await controller.connect()
    
    print("[MISSION] Arming and taking off both X500 gimbals to 22m...")
    await controller.arm_and_takeoff_both(altitude_m=22.0)
    
    print("[MISSION] Starting interception...")
    await controller.run_interception()
    
    print("[MISSION] Mission complete. Closing connections...")
    await controller.disconnect()

def main():
    manager = SimulationManager()
    
    def signal_handler(sig, frame):
        print("\n[LAUNCHER] Interrupted by user")
        manager.cleanup()
        sys.exit(0)
        
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        if EXPLICIT_GZ:
            manager.start_gazebo_world()

        # Step 1: Start PX4 hunter. PX4 launches Gazebo GUI and spawns X500 gimbal.
        manager.start_px4_instance(0, "0,0,0,0,0,0", standalone=EXPLICIT_GZ, model="x500_gimbal")

        # Step 2: Start PX4 target 100m east in standalone mode; same Gazebo world, second X500 gimbal.
        manager.start_px4_instance(1, "0,100,0,0,0,0", standalone=True, model="x500_gimbal")
        
        # Step 3: Start MAVLink router for QGC
        # manager.start_mavlink_router()  # Uncomment if mavproxy available
        
        print("[LAUNCHER] Simulation running. Waiting 20s for systems to stabilize...")
        time.sleep(20)
        
        # Step 4: Run MAVSDK mission
        print("[LAUNCHER] Starting MAVSDK mission controller...")
        asyncio.run(run_mission())
        
        # Keep running until user stops
        print("[LAUNCHER] Mission complete. Press Ctrl+C to stop simulation.")
        while True:
            time.sleep(1)
            
    except Exception as e:
        print(f"[LAUNCHER] Error: {e}")
        manager.cleanup()
        raise

if __name__ == "__main__":
    main()
