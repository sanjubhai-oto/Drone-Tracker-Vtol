#!/usr/bin/env python3
"""Preflight checks for the Linux PX4/Gazebo SITL bundle."""

from __future__ import annotations

import os
import shutil
from pathlib import Path


def fail(message: str) -> None:
    print(f"[CHECK] FAIL: {message}")
    raise SystemExit(1)


def ok(message: str) -> None:
    print(f"[CHECK] OK: {message}")


def main() -> None:
    px4_dir = Path(os.environ.get("X500_GIMBAL_PX4_DIR", str(Path.home() / "PX4-Autopilot"))).expanduser()
    build_dir = px4_dir / "build" / "px4_sitl_default"
    px4_bin = build_dir / "bin" / "px4"
    gz_dir = px4_dir / "Tools" / "simulation" / "gz"
    gz_plugins = build_dir / "src" / "modules" / "simulation" / "gz_plugins"
    gz_server_config = px4_dir / "src" / "modules" / "simulation" / "gz_bridge" / "server.config"
    model_sdf = gz_dir / "models" / "x500_gimbal" / "model.sdf"
    default_world = gz_dir / "worlds" / "default.sdf"

    if not px4_dir.exists():
        fail(f"PX4 directory not found: {px4_dir}. Set X500_GIMBAL_PX4_DIR=/path/to/PX4-Autopilot")
    ok(f"PX4 directory: {px4_dir}")

    if not px4_bin.exists():
        fail(f"PX4 SITL binary missing: {px4_bin}. Run: cd {px4_dir} && make px4_sitl_default")
    ok(f"PX4 binary: {px4_bin}")

    if not model_sdf.exists():
        fail(f"PX4 Gazebo x500_gimbal model missing: {model_sdf}")
    ok(f"PX4 x500_gimbal model: {model_sdf}")

    if not default_world.exists():
        fail(f"PX4 Gazebo default world missing: {default_world}")
    ok(f"PX4 default world: {default_world}")

    if not gz_plugins.exists():
        fail(f"PX4 Gazebo plugin directory missing: {gz_plugins}. Rebuild with: cd {px4_dir} && make px4_sitl_default")
    ok(f"PX4 Gazebo plugins: {gz_plugins}")

    if not gz_server_config.exists():
        fail(f"PX4 Gazebo server config missing: {gz_server_config}. Rebuild with: cd {px4_dir} && make px4_sitl_default")
    ok(f"PX4 Gazebo server config: {gz_server_config}")

    gz = shutil.which("gz")
    if not gz:
        fail("Gazebo executable `gz` not found. Install Gazebo Harmonic/Garden and ensure `gz sim` works.")
    ok(f"Gazebo executable: {gz}")

    try:
        import mavsdk  # noqa: F401
    except Exception as exc:
        fail(f"Python mavsdk import failed: {exc}. Run: python3 -m pip install --user -r requirements.txt")
    ok("Python mavsdk import")

    print("[CHECK] Environment looks ready for X500 Gimbal Tracker SITL.")


if __name__ == "__main__":
    main()
