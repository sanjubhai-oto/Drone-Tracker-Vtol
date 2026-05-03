#!/usr/bin/env python3
"""Generate dependency-free SVG/HTML plots for Wavel swarm telemetry."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TELEMETRY = ROOT / "wavel_swarm_telemetry.json"
OUT_DIR = ROOT / "outputs" / "graphs"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if abs(src_max - src_min) < 1e-9:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) * (dst_max - dst_min) / (src_max - src_min)


def main() -> None:
    rows = json.loads(TELEMETRY.read_text(encoding="utf-8"))
    drones = rows[-1]["drones"]
    all_n = [d["north_m"] for row in rows for d in row["drones"]] + [rows[-1]["target"]["north_m"]]
    all_e = [d["east_m"] for row in rows for d in row["drones"]] + [rows[-1]["target"]["east_m"]]
    min_n, max_n = min(all_n) - 12, max(all_n) + 12
    min_e, max_e = min(all_e) - 12, max(all_e) + 12
    colors = ["#f97316", "#38bdf8", "#22c55e", "#eab308", "#a78bfa"]

    def xy(n: float, e: float) -> tuple[float, float]:
        return (
            scale(e, min_e, max_e, 70, 1120),
            scale(n, min_n, max_n, 650, 80),
        )

    parts = [
        '<svg xmlns="http://www.w3.org/2000/svg" width="1200" height="720" viewBox="0 0 1200 720">',
        '<rect width="1200" height="720" fill="#05080d"/>',
        '<text x="32" y="44" fill="#f8fafc" font-family="Arial" font-size="28">Wavel Swarm Dynamic Following And Object Confirmation</text>',
        '<text x="32" y="74" fill="#94a3b8" font-family="Arial" font-size="16">Non-destructive simulated object tag; spacing and mesh communication preserved.</text>',
    ]

    for grid in range(0, 121, 20):
        x1, y1 = xy(min_n + grid, min_e)
        x2, y2 = xy(min_n + grid, max_e)
        parts.append(f'<line x1="{x1:.1f}" y1="{y1:.1f}" x2="{x2:.1f}" y2="{y2:.1f}" stroke="#1e293b"/>')

    for drone_id in range(1, len(drones) + 1):
        pts = []
        for row in rows:
            drone = next(d for d in row["drones"] if d["drone_id"] == drone_id)
            x, y = xy(drone["north_m"], drone["east_m"])
            pts.append(f"{x:.1f},{y:.1f}")
        color = colors[(drone_id - 1) % len(colors)]
        parts.append(f'<polyline points="{" ".join(pts)}" fill="none" stroke="{color}" stroke-width="4" stroke-linecap="round" stroke-linejoin="round"/>')
        x, y = xy(rows[-1]["drones"][drone_id - 1]["north_m"], rows[-1]["drones"][drone_id - 1]["east_m"])
        parts.append(f'<circle cx="{x:.1f}" cy="{y:.1f}" r="8" fill="{color}"/>')
        parts.append(f'<text x="{x + 10:.1f}" y="{y - 8:.1f}" fill="{color}" font-family="Arial" font-size="14">W{drone_id}</text>')

    tx, ty = xy(rows[-1]["target"]["north_m"], rows[-1]["target"]["east_m"])
    parts.append(f'<rect x="{tx - 16:.1f}" y="{ty - 16:.1f}" width="32" height="32" fill="#ef4444" opacity="0.85"/>')
    parts.append(f'<text x="{tx + 22:.1f}" y="{ty + 5:.1f}" fill="#fecaca" font-family="Arial" font-size="16">object B</text>')
    parts.append(f'<text x="32" y="690" fill="#cbd5e1" font-family="Arial" font-size="16">min separation: {min(r["min_separation_m"] for r in rows):.1f} m | confirmed: {rows[-1]["confirmed_count"]}/{len(drones)}</text>')
    parts.append("</svg>")
    svg = "\n".join(parts)
    (OUT_DIR / "wavel_swarm_path.svg").write_text(svg, encoding="utf-8")
    (OUT_DIR / "wavel_swarm_path.html").write_text(f"<!doctype html><meta charset='utf-8'><title>Wavel Swarm</title>{svg}", encoding="utf-8")
    print(OUT_DIR / "wavel_swarm_path.svg")


if __name__ == "__main__":
    main()

