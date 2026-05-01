#!/usr/bin/env python3
"""Generate a static 3D-style trajectory SVG from X500 gimbal telemetry."""

from __future__ import annotations

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TELEMETRY = ROOT / "x500_gimbal_telemetry.json"
OUT = ROOT / "outputs" / "graphs" / "x500_gimbal_trajectory_3d.svg"


def main() -> None:
    rows = json.loads(TELEMETRY.read_text())
    if not rows:
        raise SystemExit("No telemetry rows")

    OUT.parent.mkdir(parents=True, exist_ok=True)
    w, h = 1280, 820
    he = [r["hunter_e"] for r in rows]
    hn = [r["hunter_n"] for r in rows]
    ha = [max(0.0, r["hunter_alt"]) for r in rows]
    te = [r["target_e"] for r in rows]
    tn = [r["target_n"] for r in rows]
    ta = [max(0.0, r["target_alt"]) for r in rows]
    ranges = [r["range_m"] for r in rows]
    min_e, max_e = min(he + te), max(he + te)
    min_n, max_n = min(hn + tn), max(hn + tn)
    center_e = (min_e + max_e) / 2.0
    center_n = (min_n + max_n) / 2.0
    event_idx = next(
        (i for i, r in enumerate(rows) if r.get("mission_state") == "proximity_trigger"),
        min(range(len(rows)), key=lambda i: abs(ranges[i] - 25.0)),
    )

    def project(e: float, n: float, alt: float) -> tuple[float, float]:
        sx = (e - center_e) * 10.0
        sy = (n - center_n) * 10.0
        sz = alt * 5.0
        return w * 0.48 + sx - sy * 0.58, h * 0.72 + sx * 0.16 + sy * 0.34 - sz

    def poly(es: list[float], ns: list[float], alts: list[float], color: str, width: int = 6) -> str:
        pts = " ".join(f"{project(e, n, a)[0]:.1f},{project(e, n, a)[1]:.1f}" for e, n, a in zip(es, ns, alts))
        return (
            f'<polyline points="{pts}" fill="none" stroke="{color}" stroke-width="{width}" '
            'stroke-linecap="round" stroke-linejoin="round"/>'
        )

    def circ(e: float, n: float, a: float, r: int, color: str) -> str:
        x, y = project(e, n, a)
        return f'<circle cx="{x:.1f}" cy="{y:.1f}" r="{r}" fill="{color}"/>'

    def line(e: float, n: float, a0: float, a1: float, color: str, dash: str = "") -> str:
        x0, y0 = project(e, n, a0)
        x1, y1 = project(e, n, a1)
        dash_attr = f' stroke-dasharray="{dash}"' if dash else ""
        return f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}" stroke="{color}" stroke-width="2"{dash_attr}/>'

    def text(x: float, y: float, s: str, size: int = 22, color: str = "#f8fafc") -> str:
        return f'<text x="{x:.1f}" y="{y:.1f}" fill="{color}" font-family="Arial" font-size="{size}">{s}</text>'

    def label(e: float, n: float, a: float, s: str, color: str) -> str:
        x, y = project(e, n, a)
        return text(x + 12, y - 12, s, 15, color)

    parts = [
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}" viewBox="0 0 {w} {h}">',
        '<rect width="100%" height="100%" fill="#05080d"/>',
        text(34, 48, "X500 Gimbal Tracker X500 gimbal 3D Trajectory", 30),
        text(34, 78, "Perspective plot: ground position plus altitude. Orange=hunter, Blue=target.", 16, "#94a3b8"),
    ]

    e0 = int((min_e - 10) // 10) * 10
    e1 = int((max_e + 20) // 10) * 10
    n0 = int((min_n - 20) // 10) * 10
    n1 = int((max_n + 20) // 10) * 10
    parts.append('<g stroke="#1e293b" stroke-width="1">')
    for e in range(e0, e1 + 1, 10):
        x0, y0 = project(e, n0, 0)
        x1, y1 = project(e, n1, 0)
        parts.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}"/>')
    for n in range(n0, n1 + 1, 10):
        x0, y0 = project(e0, n, 0)
        x1, y1 = project(e1, n, 0)
        parts.append(f'<line x1="{x0:.1f}" y1="{y0:.1f}" x2="{x1:.1f}" y2="{y1:.1f}"/>')
    parts.append("</g>")

    parts.extend(
        [
            line(he[-1], hn[-1], 0, ha[-1], "#f97316", "6 6"),
            line(te[-1], tn[-1], 0, ta[-1], "#38bdf8", "6 6"),
            poly(te, tn, ta, "#38bdf8"),
            poly(he, hn, ha, "#f97316"),
            circ(te[0], tn[0], ta[0], 8, "#38bdf8"),
            circ(he[0], hn[0], ha[0], 8, "#f97316"),
            circ(te[event_idx], tn[event_idx], ta[event_idx], 11, "#22c55e"),
            circ(he[event_idx], hn[event_idx], ha[event_idx], 11, "#ef4444"),
            circ(te[-1], tn[-1], ta[-1], 8, "#7dd3fc"),
            circ(he[-1], hn[-1], ha[-1], 8, "#fdba74"),
            label(te[0], tn[0], ta[0], "target start", "#38bdf8"),
            label(he[0], hn[0], ha[0], "hunter start", "#f97316"),
            label(te[event_idx], tn[event_idx], ta[event_idx], f"trigger {ranges[event_idx]:.1f}m", "#22c55e"),
            label(te[-1], tn[-1], ta[-1], "target final", "#7dd3fc"),
            label(he[-1], hn[-1], ha[-1], "hunter final", "#fdba74"),
            text(44, h - 76, f"start {ranges[0]:.1f}m | trigger/closest {ranges[event_idx]:.1f}m | final logged {ranges[-1]:.1f}m", 18, "#e2e8f0"),
            text(44, h - 46, f"hunter altitude {ha[0]:.1f}->{ha[-1]:.1f}m | target altitude {ta[0]:.1f}->{ta[-1]:.1f}m", 18, "#e2e8f0"),
            "</svg>",
        ]
    )
    OUT.write_text("\n".join(parts))
    print(OUT)


if __name__ == "__main__":
    main()
