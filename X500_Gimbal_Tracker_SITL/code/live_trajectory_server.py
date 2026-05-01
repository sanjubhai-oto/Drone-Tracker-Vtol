#!/usr/bin/env python3
"""Serve a live X500 Gimbal Tracker trajectory page from the package folder."""

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
  <title>X500 Gimbal Tracker Live 3D Trajectory</title>
  <style>
    :root { color-scheme: dark; font-family: Arial, sans-serif; }
    body { margin: 0; background: #05080d; color: #e2e8f0; }
    header { height: 58px; display: flex; align-items: center; gap: 24px; padding: 0 24px; border-bottom: 1px solid #1e293b; }
    h1 { font-size: 20px; margin: 0; }
    .stat { color: #94a3b8; font-size: 14px; }
    main { display: grid; grid-template-columns: 1fr 420px; gap: 0; height: calc(100vh - 59px); }
    canvas { width: 100%; height: 100%; display: block; background: #08111c; }
    aside { border-left: 1px solid #1e293b; padding: 18px; background: #0b1117; overflow: auto; }
    .row { display: flex; justify-content: space-between; border-bottom: 1px solid #1e293b; padding: 10px 0; }
    .key { color: #94a3b8; }
    .hunter { color: #f97316; }
    .target { color: #38bdf8; }
    .trigger { color: #22c55e; }
    .feeds { display: grid; gap: 14px; margin-bottom: 18px; }
    .feed { border: 1px solid #1e293b; background: #05080d; }
    .feed h2 { font-size: 14px; font-weight: 700; margin: 0; padding: 10px 12px; border-bottom: 1px solid #1e293b; }
    .feed img { display: block; width: 100%; aspect-ratio: 16 / 9; object-fit: cover; background: #020617; }
  </style>
</head>
<body>
  <header>
    <h1>X500 Gimbal Tracker Live 3D Trajectory</h1>
    <div class="stat">orange hunter | blue target | green trigger | height = altitude</div>
    <div id="status" class="stat">waiting telemetry</div>
  </header>
  <main>
    <canvas id="plot"></canvas>
    <aside>
      <section class="feeds">
        <div class="feed">
          <h2>Hunter gimbal camera</h2>
          <img id="hunterFeed" src="/camera/hunter.jpg" alt="Hunter camera feed">
        </div>
        <div class="feed">
          <h2>Target gimbal camera</h2>
          <img id="targetFeed" src="/camera/target.jpg" alt="Target camera feed">
        </div>
      </section>
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
        const res = await fetch('/x500_gimbal_telemetry.json?ts=' + Date.now(), { cache: 'no-store' });
        const rows = await res.json();
        if (!Array.isArray(rows) || rows.length === 0) return;
        draw(rows);
        $('status').textContent = 'live ' + new Date().toLocaleTimeString();
      } catch (err) {
        $('status').textContent = 'waiting for x500_gimbal_telemetry.json';
      }
    }

    function refreshFeeds() {
      const ts = Date.now();
      $('hunterFeed').src = '/camera/hunter.jpg?ts=' + ts;
      $('targetFeed').src = '/camera/target.jpg?ts=' + ts;
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
    setInterval(refreshFeeds, 500);
    load();
    refreshFeeds();
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
        if self.path.startswith("/camera/hunter.jpg"):
            self._serve_camera("hunter")
            return
        if self.path.startswith("/camera/target.jpg"):
            self._serve_camera("target")
            return
        super().do_GET()

    def do_HEAD(self):
        if self.path in ("/", "/live", "/live_trajectory.html"):
            body = LIVE_HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            return
        if self.path.startswith("/camera/hunter.jpg") or self.path.startswith("/camera/target.jpg"):
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.end_headers()
            return
        super().do_HEAD()

    def _serve_camera(self, role: str) -> None:
        image_path = ROOT / "outputs" / "camera" / f"{role}_latest.jpg"
        if image_path.exists():
            body = image_path.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return

        body = self._camera_placeholder_svg(role).encode("utf-8")
        self.send_response(200)
        self.send_header("Content-Type", "image/svg+xml; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    @staticmethod
    def _camera_placeholder_svg(role: str) -> str:
        label = "Hunter gimbal camera" if role == "hunter" else "Target gimbal camera"
        port = "5600" if role == "hunter" else "5601"
        color = "#f97316" if role == "hunter" else "#38bdf8"
        return f"""<svg xmlns="http://www.w3.org/2000/svg" width="960" height="540" viewBox="0 0 960 540">
  <rect width="960" height="540" fill="#020617"/>
  <rect x="24" y="24" width="912" height="492" fill="#08111c" stroke="#1e293b" stroke-width="3"/>
  <text x="48" y="78" fill="{color}" font-family="Arial" font-size="32" font-weight="700">{label}</text>
  <text x="48" y="128" fill="#cbd5e1" font-family="Arial" font-size="22">Waiting for live camera frame.</text>
  <text x="48" y="168" fill="#94a3b8" font-family="Arial" font-size="18">If a frame forwarder writes outputs/camera/{role}_latest.jpg, this panel updates automatically.</text>
  <text x="48" y="216" fill="#94a3b8" font-family="Arial" font-size="18">Suggested UDP stream for external viewer: udp://127.0.0.1:{port}</text>
  <circle cx="760" cy="270" r="96" fill="none" stroke="{color}" stroke-width="6"/>
  <line x1="760" y1="174" x2="760" y2="366" stroke="{color}" stroke-width="4"/>
  <line x1="664" y1="270" x2="856" y2="270" stroke="{color}" stroke-width="4"/>
</svg>"""


def main() -> None:
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Live trajectory: http://127.0.0.1:{PORT}/live_trajectory.html")
    server.serve_forever()


if __name__ == "__main__":
    main()
