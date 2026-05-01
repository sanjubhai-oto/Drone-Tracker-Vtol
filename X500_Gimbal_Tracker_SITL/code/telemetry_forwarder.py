#!/usr/bin/env python3
"""
MAVLink Telemetry Forwarder for QGroundControl
Forwards telemetry from multiple PX4 SITL instances to QGC.

Usage:
    python3 telemetry_forwarder.py

This creates a MAVLink proxy that:
1. Connects to hunter on UDP 14540
2. Connects to target on UDP 14541
3. Forwards both streams to QGC on UDP 14550
4. Logs all telemetry to file for replay/analysis
"""

import asyncio
import time
import json
from pathlib import Path
from pymavlink import mavutil


class TelemetryForwarder:
    """Forward MAVLink telemetry from multiple drones to QGC."""

    def __init__(self):
        self.connections = {}
        self.qgc_connection = None
        self.log_file = Path("telemetry_log.jsonl")
        self.running = False

    def connect_to_drone(self, name: str, address: str):
        """Connect to a drone via MAVLink."""
        print(f"[FORWARDER] Connecting to {name} at {address}...")
        conn = mavutil.mavlink_connection(address)
        self.connections[name] = conn
        print(f"[FORWARDER] Connected to {name}")
        return conn

    def connect_to_qgc(self, address: str = "udpout:127.0.0.1:14550"):
        """Connect to QGroundControl."""
        print(f"[FORWARDER] Connecting to QGC at {address}...")
        self.qgc_connection = mavutil.mavlink_connection(address)
        print("[FORWARDER] QGC connection ready")

    def forward_message(self, drone_name: str, msg):
        """Forward a MAVLink message to QGC and log it."""
        if self.qgc_connection and msg:
            try:
                self.qgc_connection.mav.send(msg)
            except Exception as e:
                print(f"[FORWARDER] Forward error: {e}")

        # Log important messages
        if msg.get_type() in ['GLOBAL_POSITION_INT', 'ATTITUDE', 'HEARTBEAT', 'VFR_HUD']:
            log_entry = {
                "timestamp": time.time(),
                "drone": drone_name,
                "msg_type": msg.get_type(),
                "data": msg.to_dict(),
            }
            with open(self.log_file, "a") as f:
                f.write(json.dumps(log_entry) + "\n")

    def run(self):
        """Main forwarding loop."""
        self.running = True

        # Connect to drones
        hunter_conn = self.connect_to_drone("hunter", "udpin:127.0.0.1:14540")
        target_conn = self.connect_to_drone("target", "udpin:127.0.0.1:14541")

        # Connect to QGC
        self.connect_to_qgc()

        print("[FORWARDER] Telemetry forwarding active. Press Ctrl+C to stop.")
        print("[FORWARDER] Open QGroundControl to see live telemetry from both drones.")

        try:
            while self.running:
                # Check for messages from each drone
                for name, conn in self.connections.items():
                    msg = conn.recv_match(blocking=False)
                    if msg:
                        self.forward_message(name, msg)

                time.sleep(0.01)  # 100Hz check rate

        except KeyboardInterrupt:
            print("\n[FORWARDER] Stopping...")
            self.running = False

        # Cleanup
        for conn in self.connections.values():
            conn.close()
        if self.qgc_connection:
            self.qgc_connection.close()

        print(f"[FORWARDER] Telemetry log saved to {self.log_file}")


def main():
    forwarder = TelemetryForwarder()
    forwarder.run()


if __name__ == "__main__":
    main()
