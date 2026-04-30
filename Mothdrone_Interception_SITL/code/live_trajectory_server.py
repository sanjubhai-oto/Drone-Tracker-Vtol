#!/usr/bin/env python3
"""Serve a live Mothdrone trajectory page from the package folder."""

from __future__ import annotations

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PORT = 8790


LIVE_HTML = """<!doctype html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>Mothdrone Live 3D Trajectory</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #05080d; color: #e2e8f0; }
    header { height: 58px; display: flex; align-items: center; gap: 24px; padding: 0 24px; border-bottom: 1px solid #1e293b; }
    h1 { font-size: 20px; margin: 0; }
    .stat { color: #94a3b8; font-size: 14px; }
    main { display: grid; grid-template-columns: 1fr 360px; gap: 0; height: calc(100vh - 59px); }
    canvas { width: 100%; height: 100%; display: block; background: #08111c; }
    aside { border-left: 1px solid #1e293b; padding: 18px; background: #0b1117; }
    .row { display: flex; justify-content: space-between; border-bottom: 1px solid #1e293b; padding: 10px 0; }
    .key { color: #94a3b8; }
    .hunter { color: #f97316; }
    .target { color: #38bdf8; }
    .trigger { color: #22c55e; }
  </style>
</head>
<body>
  <header>
    <h1>Mothdrone Live 3D Trajectory</h1>
    <div class="stat">orange hunter | blue target | green trigger | height = altitude</div>
    <div id="status" class="stat">waiting telemetry</div>
  </header>
  <main>
    <canvas id="plot"></canvas>
    <aside>
      <div class="row"><span class="key">Samples</span><span id="samples">0</span></div>
      <div class="row"><span class="key">Start range</span><span id="startRange">-</span></div>
      <div class="row"><span class="key">Current range</span><span id="range">-</span></div>
      <div class="row"><span class="key">Hunter altitude</span><span id="hunterAlt" class="hunter">-</span></div>
      <div class="row"><span class="key">Target altitude</span><span id="targetAlt" class="target">-</span></div>
      <div class="row"><span class="key">Mode</span><span id="mode">-</span></div>
      <div class="row"><span class="key">State</span><span id="state">-</span></div>
      <div class="row"><span class="key">Vision lock</span><span id="vision">-</span></div>
    </aside>
  </main>
  <script>
    const canvas = document.getElementById('plot');
    const ctx = canvas.getContext('2d');
    const $ = (id) => document.getElementById(id);

    function fitCanvas() {
      const dpr = window.devicePixelRatio || 1;
      const rect = canvas.getBoundingClientRect();
      canvas.width = Math.max(1, Math.floor(rect.width * dpr));
      canvas.height = Math.max(1, Math.floor(rect.height * dpr));
      ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
    }

    function scale(v, a, b, c, d) {
      if (Math.abs(b - a) < 1e-9) return (c + d) / 2;
      return c + (v - a) * (d - c) / (b - a);
    }

    function drawPath(rows, getE, getN, color) {
      if (!rows.length) return;
      ctx.beginPath();
      rows.forEach((r, i) => {
        const x = scale(getE(r), bounds.minE, bounds.maxE, 60, view.w - 50);
        const y = scale(getN(r), bounds.minN, bounds.maxN, view.h - 60, 50);
        if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
      });
      ctx.strokeStyle = color;
      ctx.lineWidth = 4;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();
    }

    function dot(row, eKey, nKey, color, r = 6) {
      const x = scale(row[eKey], bounds.minE, bounds.maxE, 60, view.w - 50);
      const y = scale(row[nKey], bounds.minN, bounds.maxN, view.h - 60, 50);
      ctx.beginPath();
      ctx.arc(x, y, r, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
    }

    let bounds = {};
    let view = {};

    async function load() {
      try {
        const res = await fetch('/mothdrone_telemetry.json?ts=' + Date.now(), { cache: 'no-store' });
        const rows = await res.json();
        if (!Array.isArray(rows) || rows.length === 0) return;
        draw(rows);
        $('status').textContent = 'live ' + new Date().toLocaleTimeString();
      } catch (err) {
        $('status').textContent = 'waiting for mothdrone_telemetry.json';
      }
    }

    function draw(rows) {
      fitCanvas();
      const rect = canvas.getBoundingClientRect();
      view = { w: rect.width, h: rect.height };
      const es = rows.flatMap(r => [r.hunter_e, r.target_e]);
      const ns = rows.flatMap(r => [r.hunter_n, r.target_n]);
      const minE = Math.min(...es), maxE = Math.max(...es);
      const minN = Math.min(...ns), maxN = Math.max(...ns);
      const padE = Math.max(8, (maxE - minE) * 0.12);
      const padN = Math.max(8, (maxN - minN) * 0.12);
      bounds = { minE: minE - padE, maxE: maxE + padE, minN: minN - padN, maxN: maxN + padN };

      ctx.clearRect(0, 0, view.w, view.h);
      ctx.fillStyle = '#08111c';
      ctx.fillRect(0, 0, view.w, view.h);
      ctx.strokeStyle = '#334155';
      ctx.strokeRect(32, 28, view.w - 64, view.h - 60);
      draw3D(rows);

      const last = rows[rows.length - 1];
      $('samples').textContent = rows.length;
      $('startRange').textContent = rows[0].range_m.toFixed(1) + ' m';
      $('range').textContent = last.range_m.toFixed(1) + ' m';
      $('hunterAlt').textContent = last.hunter_alt.toFixed(1) + ' m';
      $('targetAlt').textContent = last.target_alt.toFixed(1) + ' m';
      $('mode').textContent = last.mode;
      $('state').textContent = last.mission_state;
      $('vision').textContent = String(last.vision_lock);
    }

    function project(e, n, alt) {
      const cx = view.w * 0.48;
      const cy = view.h * 0.72;
      const sx = (e - (bounds.minE + bounds.maxE) / 2) * 9.0;
      const sy = (n - (bounds.minN + bounds.maxN) / 2) * 9.0;
      const sz = alt * 4.2;
      return {
        x: cx + sx - sy * 0.55,
        y: cy + sx * 0.18 + sy * 0.32 - sz
      };
    }

    function draw3DPath(rows, eKey, nKey, altKey, color, width) {
      if (!rows.length) return;
      ctx.beginPath();
      rows.forEach((r, i) => {
        const q = project(r[eKey], r[nKey], Math.max(0, r[altKey]));
        if (i === 0) ctx.moveTo(q.x, q.y); else ctx.lineTo(q.x, q.y);
      });
      ctx.strokeStyle = color;
      ctx.lineWidth = width;
      ctx.lineJoin = 'round';
      ctx.lineCap = 'round';
      ctx.stroke();
    }

    function draw3DDot(row, eKey, nKey, altKey, color, radius) {
      const q = project(row[eKey], row[nKey], Math.max(0, row[altKey]));
      ctx.beginPath();
      ctx.arc(q.x, q.y, radius, 0, Math.PI * 2);
      ctx.fillStyle = color;
      ctx.fill();
    }

    function draw3DLabel(row, eKey, nKey, altKey, label, color) {
      const q = project(row[eKey], row[nKey], Math.max(0, row[altKey]));
      ctx.fillStyle = color;
      ctx.font = '13px Arial';
      ctx.fillText(label, q.x + 10, q.y - 10);
    }

    function draw3D(rows) {
      ctx.fillStyle = '#94a3b8';
      ctx.font = '14px Arial';
      ctx.fillText('3D perspective: X/Y ground path plus altitude', 48, 54);

      // Ground grid.
      ctx.strokeStyle = '#1e293b';
      ctx.lineWidth = 1;
      for (let e = Math.floor(bounds.minE / 10) * 10; e <= bounds.maxE; e += 10) {
        const a = project(e, bounds.minN, 0);
        const b = project(e, bounds.maxN, 0);
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }
      for (let n = Math.floor(bounds.minN / 10) * 10; n <= bounds.maxN; n += 10) {
        const a = project(bounds.minE, n, 0);
        const b = project(bounds.maxE, n, 0);
        ctx.beginPath(); ctx.moveTo(a.x, a.y); ctx.lineTo(b.x, b.y); ctx.stroke();
      }

      // Altitude drop/climb reference lines at final points.
      const last = rows[rows.length - 1];
      for (const spec of [
        ['hunter_e', 'hunter_n', 'hunter_alt', '#f97316'],
        ['target_e', 'target_n', 'target_alt', '#38bdf8']
      ]) {
        const top = project(last[spec[0]], last[spec[1]], Math.max(0, last[spec[2]]));
        const ground = project(last[spec[0]], last[spec[1]], 0);
        ctx.beginPath(); ctx.moveTo(ground.x, ground.y); ctx.lineTo(top.x, top.y);
        ctx.strokeStyle = spec[3]; ctx.setLineDash([5, 5]); ctx.stroke(); ctx.setLineDash([]);
      }

      draw3DPath(rows, 'target_e', 'target_n', 'target_alt', '#38bdf8', 5);
      draw3DPath(rows, 'hunter_e', 'hunter_n', 'hunter_alt', '#f97316', 5);

      const trigger = rows.find(r => r.mission_state === 'proximity_trigger');
      draw3DDot(rows[0], 'target_e', 'target_n', 'target_alt', '#38bdf8', 7);
      draw3DDot(rows[0], 'hunter_e', 'hunter_n', 'hunter_alt', '#f97316', 7);
      if (trigger) {
        draw3DDot(trigger, 'target_e', 'target_n', 'target_alt', '#22c55e', 10);
        draw3DDot(trigger, 'hunter_e', 'hunter_n', 'hunter_alt', '#ef4444', 10);
        draw3DLabel(trigger, 'target_e', 'target_n', 'target_alt', '25m trigger', '#22c55e');
      }
      draw3DDot(last, 'target_e', 'target_n', 'target_alt', '#7dd3fc', 7);
      draw3DDot(last, 'hunter_e', 'hunter_n', 'hunter_alt', '#fdba74', 7);
      draw3DLabel(last, 'target_e', 'target_n', 'target_alt', 'target ground', '#7dd3fc');
      draw3DLabel(last, 'hunter_e', 'hunter_n', 'hunter_alt', 'hunter recovery', '#fdba74');
    }

    window.addEventListener('resize', load);
    setInterval(load, 500);
    load();
  </script>
</body>
</html>
"""


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if self.path in ("/", "/live", "/live_trajectory.html"):
            body = LIVE_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()


def main() -> None:
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Live trajectory: http://127.0.0.1:{PORT}/live_trajectory.html")
    server.serve_forever()


if __name__ == "__main__":
    main()
