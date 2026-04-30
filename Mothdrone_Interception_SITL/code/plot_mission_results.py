#!/usr/bin/env python3
"""Generate dependency-free SVG/HTML mission graphs from mothdrone_telemetry.json."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TELEMETRY = ROOT / "mothdrone_telemetry.json"
OUT_DIR = ROOT / "outputs" / "graphs"


def scale(value: float, src_min: float, src_max: float, dst_min: float, dst_max: float) -> float:
    if abs(src_max - src_min) < 1e-9:
        return (dst_min + dst_max) / 2.0
    return dst_min + (value - src_min) * (dst_max - dst_min) / (src_max - src_min)


def polyline(points: list[tuple[float, float]], color: str, width: int = 3) -> str:
    payload = " ".join(f"{x:.1f},{y:.1f}" for x, y in points)
    return f'<polyline points="{payload}" fill="none" stroke="{color}" stroke-width="{width}" />'


def circle(x: float, y: float, r: int, color: str) -> str:
    return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{color}" />'


def text(x: float, y: float, label: str, size: int = 14, color: str = "#d7e0ea") -> str:
    return f'<text x="{x:.1f}" y="{y:.1f}" fill="{color}" font-size="{size}" font-family="Arial">{label}</text>'


def axis_box(x: int, y: int, w: int, h: int, title: str) -> str:
    return "\n".join(
        [
            f'<rect x="{x}" y="{y}" width="{w}" height="{h}" fill="#0b1117" stroke="#334155" />',
            text(x + 12, y + 24, title, 16, "#f8fafc"),
            f'<line x1="{x + 48}" y1="{y + h - 36}" x2="{x + w - 24}" y2="{y + h - 36}" stroke="#64748b" />',
            f'<line x1="{x + 48}" y1="{y + 36}" x2="{x + 48}" y2="{y + h - 36}" stroke="#64748b" />',
        ]
    )


def main() -> None:
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    data = json.loads(TELEMETRY.read_text())
    if not data:
        raise SystemExit("No telemetry rows")

    t0 = data[0]["time"]
    times = [row["time"] - t0 for row in data]
    hunter_e = [row["hunter_e"] for row in data]
    hunter_n = [row["hunter_n"] for row in data]
    target_e = [row["target_e"] for row in data]
    target_n = [row["target_n"] for row in data]
    ranges = [row["range_m"] for row in data]
    hunter_alt = [row["hunter_alt"] for row in data]
    target_alt = [row["target_alt"] for row in data]

    event_idx = min(range(len(data)), key=lambda i: abs(ranges[i] - 25.0))

    width, height = 1200, 760
    bg = '<rect width="100%" height="100%" fill="#05080d" />'
    header = text(28, 42, "Mothdrone VTOL SITL Result - Hunter follows target and triggers at 25m", 24, "#f8fafc")

    # Path chart.
    px, py, pw, ph = 40, 74, 540, 430
    min_e, max_e = min(hunter_e + target_e), max(hunter_e + target_e)
    min_n, max_n = min(hunter_n + target_n), max(hunter_n + target_n)
    pad_e = max(8.0, (max_e - min_e) * 0.08)
    pad_n = max(8.0, (max_n - min_n) * 0.08)
    min_e -= pad_e
    max_e += pad_e
    min_n -= pad_n
    max_n += pad_n
    path_area = axis_box(px, py, pw, ph, "Top-down path: hunter vs target")
    hunter_path = [
        (scale(e, min_e, max_e, px + 54, px + pw - 28), scale(n, min_n, max_n, py + ph - 42, py + 42))
        for e, n in zip(hunter_e, hunter_n)
    ]
    target_path = [
        (scale(e, min_e, max_e, px + 54, px + pw - 28), scale(n, min_n, max_n, py + ph - 42, py + 42))
        for e, n in zip(target_e, target_n)
    ]
    path_plot = "\n".join(
        [
            path_area,
            polyline(target_path, "#38bdf8", 4),
            polyline(hunter_path, "#f97316", 4),
            circle(*target_path[0], 5, "#38bdf8"),
            circle(*hunter_path[0], 5, "#f97316"),
            circle(*target_path[event_idx], 7, "#22c55e"),
            circle(*hunter_path[event_idx], 7, "#ef4444"),
            text(px + 62, py + ph - 12, f"Easting m: {min_e:.0f} to {max_e:.0f}", 12, "#94a3b8"),
        ]
    )

    # Range chart.
    rx, ry, rw, rh = 620, 74, 540, 270
    max_time = max(times)
    max_range = max(ranges) + 5
    range_area = axis_box(rx, ry, rw, rh, "Separation range over time")
    range_points = [
        (scale(t, 0, max_time, rx + 54, rx + rw - 28), scale(r, 0, max_range, ry + rh - 42, ry + 42))
        for t, r in zip(times, ranges)
    ]
    kill_y = scale(25.0, 0, max_range, ry + rh - 42, ry + 42)
    event_x = range_points[event_idx][0]
    range_plot = "\n".join(
        [
            range_area,
            f'<line x1="{rx + 54}" y1="{kill_y:.1f}" x2="{rx + rw - 28}" y2="{kill_y:.1f}" stroke="#22c55e" stroke-dasharray="8 6" />',
            text(rx + rw - 160, kill_y - 8, "25m trigger radius", 12, "#22c55e"),
            polyline(range_points, "#a78bfa", 4),
            circle(*range_points[event_idx], 7, "#ef4444"),
            f'<line x1="{event_x:.1f}" y1="{ry + 42}" x2="{event_x:.1f}" y2="{ry + rh - 42}" stroke="#ef4444" stroke-dasharray="5 5" />',
            text(rx + 62, ry + rh - 12, f"time 0-{max_time:.1f}s | final range {ranges[-1]:.1f}m", 12, "#94a3b8"),
        ]
    )

    # Altitude chart.
    ax, ay, aw, ah = 620, 380, 540, 270
    max_alt = max(hunter_alt + target_alt + [25.0])
    alt_area = axis_box(ax, ay, aw, ah, "Altitude over time")
    hunter_alt_points = [
        (scale(t, 0, max_time, ax + 54, ax + aw - 28), scale(a, 0, max_alt, ay + ah - 42, ay + 42))
        for t, a in zip(times, hunter_alt)
    ]
    target_alt_points = [
        (scale(t, 0, max_time, ax + 54, ax + aw - 28), scale(a, 0, max_alt, ay + ah - 42, ay + 42))
        for t, a in zip(times, target_alt)
    ]
    alt_plot = "\n".join(
        [
            alt_area,
            polyline(hunter_alt_points, "#f97316", 4),
            polyline(target_alt_points, "#38bdf8", 4),
            text(ax + 62, ay + ah - 12, f"hunter final {hunter_alt[-1]:.1f}m | target final {target_alt[-1]:.1f}m", 12, "#94a3b8"),
        ]
    )

    legend = "\n".join(
        [
            text(56, 548, "orange = hunter", 15, "#f97316"),
            text(56, 574, "blue = target", 15, "#38bdf8"),
            text(56, 600, "red/green markers = proximity trigger moment", 15, "#e2e8f0"),
            text(56, 640, f"Start range: {ranges[0]:.1f}m", 15, "#e2e8f0"),
            text(56, 666, f"Closest/trigger range: {min(ranges):.1f}m", 15, "#e2e8f0"),
            text(56, 692, f"Rows: {len(data)} telemetry samples", 15, "#e2e8f0"),
        ]
    )

    svg = "\n".join(
        [
            f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" viewBox="0 0 {width} {height}">',
            bg,
            header,
            path_plot,
            range_plot,
            alt_plot,
            legend,
            "</svg>",
        ]
    )

    svg_path = OUT_DIR / "mothdrone_mission_graph.svg"
    html_path = OUT_DIR / "mothdrone_mission_graph.html"
    svg_path.write_text(svg)
    html_path.write_text(f"<!doctype html><meta charset='utf-8'><title>Mothdrone Mission Graph</title>{svg}")
    print(svg_path)
    print(html_path)


if __name__ == "__main__":
    main()
