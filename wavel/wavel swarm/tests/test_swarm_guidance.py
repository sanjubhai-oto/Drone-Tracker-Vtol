#!/usr/bin/env python3
"""Smoke tests for Wavel swarm formation and object confirmation."""

from __future__ import annotations

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "code"))

from simulated_swarm_runner import run_simulation  # noqa: E402
from swarm_guidance import SwarmConfig  # noqa: E402


def main() -> None:
    cfg = SwarmConfig(drone_count=5, min_separation_m=6.0)
    rows = run_simulation(cfg)
    min_sep = min(row["min_separation_m"] for row in rows)
    final = rows[-1]
    assert final["confirmed_count"] == cfg.drone_count, final["confirmed_count"]
    assert min_sep >= cfg.min_separation_m - 0.75, min_sep
    assert all(len(v) > 0 for v in final["mesh_links"].values()), final["mesh_links"]
    print(f"SWARM TEST PASS rows={len(rows)} min_sep={min_sep:.2f} confirmed={final['confirmed_count']}")


if __name__ == "__main__":
    main()

