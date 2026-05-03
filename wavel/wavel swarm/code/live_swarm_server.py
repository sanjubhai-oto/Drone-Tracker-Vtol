#!/usr/bin/env python3
"""Serve a live Wavel swarm trajectory and camera-panel page."""

from __future__ import annotations

from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
PORT = 8795


HTML = """<!doctype html>
<html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width, initial-scale=1">
<title>Wavel Swarm Live</title>
<style>
body{margin:0;background:#05080d;color:#e2e8f0;font-family:Arial,sans-serif}
header{height:58px;display:flex;align-items:center;gap:18px;padding:0 22px;border-bottom:1px solid #1e293b}
h1{font-size:20px;margin:0}.muted{color:#94a3b8}
main{display:grid;grid-template-columns:1fr 420px;height:calc(100vh - 59px)}
canvas{width:100%;height:100%;background:#08111c}.side{border-left:1px solid #1e293b;padding:16px;overflow:auto}
.feeds{display:grid;grid-template-columns:1fr 1fr;gap:10px}.feed{border:1px solid #1e293b;background:#020617}
.feed img{width:100%;aspect-ratio:16/9;display:block;object-fit:cover}.feed div{padding:8px;font-size:13px}
.row{display:flex;justify-content:space-between;border-bottom:1px solid #1e293b;padding:9px 0}
</style></head><body>
<header><h1>Wavel Swarm Live</h1><span class="muted">formation | mesh comms | safe object confirmation</span><span id="status" class="muted">waiting</span></header>
<main><canvas id="plot"></canvas><section class="side">
<div class="feeds" id="feeds"></div>
<div class="row"><span class="muted">samples</span><span id="samples">0</span></div>
<div class="row"><span class="muted">confirmed</span><span id="confirmed">0</span></div>
<div class="row"><span class="muted">min separation</span><span id="sep">-</span></div>
<div class="row"><span class="muted">mesh links</span><span id="links">-</span></div>
</section></main>
<script>
const canvas=document.getElementById('plot'),ctx=canvas.getContext('2d'),$=id=>document.getElementById(id);
const colors=['#f97316','#38bdf8','#22c55e','#eab308','#a78bfa'];
function fit(){const d=window.devicePixelRatio||1,r=canvas.getBoundingClientRect();canvas.width=r.width*d;canvas.height=r.height*d;ctx.setTransform(d,0,0,d,0,0)}
function sc(v,a,b,c,d){return Math.abs(b-a)<1e-9?(c+d)/2:c+(v-a)*(d-c)/(b-a)}
function feeds(count){let html='';for(let i=1;i<=count;i++)html+=`<div class="feed"><img src="/camera/wavel_${i}.jpg?ts=${Date.now()}"><div>Wavel ${i} camera</div></div>`;$('feeds').innerHTML=html}
async function load(){try{const rows=await (await fetch('/wavel_swarm_telemetry.json?ts='+Date.now(),{cache:'no-store'})).json();draw(rows);$('status').textContent='live '+new Date().toLocaleTimeString()}catch(e){$('status').textContent='waiting telemetry'}}
function draw(rows){if(!rows.length)return;fit();const r=canvas.getBoundingClientRect(),last=rows[rows.length-1];if($('feeds').children.length!==last.drones.length)feeds(last.drones.length);
const ns=rows.flatMap(x=>x.drones.map(d=>d.north_m)).concat([last.target.north_m]);const es=rows.flatMap(x=>x.drones.map(d=>d.east_m)).concat([last.target.east_m]);
const minN=Math.min(...ns)-12,maxN=Math.max(...ns)+12,minE=Math.min(...es)-12,maxE=Math.max(...es)+12;
ctx.clearRect(0,0,r.width,r.height);ctx.fillStyle='#08111c';ctx.fillRect(0,0,r.width,r.height);ctx.strokeStyle='#1e293b';ctx.strokeRect(36,32,r.width-72,r.height-68);
for(let id=1;id<=last.drones.length;id++){ctx.beginPath();rows.forEach((row,i)=>{const d=row.drones.find(x=>x.drone_id===id);const x=sc(d.east_m,minE,maxE,60,r.width-60),y=sc(d.north_m,minN,maxN,r.height-55,55);if(i===0)ctx.moveTo(x,y);else ctx.lineTo(x,y)});ctx.strokeStyle=colors[(id-1)%colors.length];ctx.lineWidth=4;ctx.stroke();
const d=last.drones.find(x=>x.drone_id===id),x=sc(d.east_m,minE,maxE,60,r.width-60),y=sc(d.north_m,minN,maxN,r.height-55,55);ctx.fillStyle=colors[(id-1)%colors.length];ctx.beginPath();ctx.arc(x,y,7,0,7);ctx.fill();ctx.fillText('W'+id,x+10,y-8)}
const tx=sc(last.target.east_m,minE,maxE,60,r.width-60),ty=sc(last.target.north_m,minN,maxN,r.height-55,55);ctx.fillStyle='#ef4444';ctx.fillRect(tx-15,ty-15,30,30);ctx.fillStyle='#fecaca';ctx.fillText('object B',tx+22,ty+5);
$('samples').textContent=rows.length;$('confirmed').textContent=last.confirmed_count+'/'+last.drones.length;$('sep').textContent=last.min_separation_m.toFixed(1)+' m';$('links').textContent=Object.values(last.mesh_links).map(v=>v.length).join(',')}
setInterval(load,500);window.addEventListener('resize',load);load();
</script></body></html>"""


class Handler(SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=str(ROOT), **kwargs)

    def end_headers(self):
        self.send_header("Cache-Control", "no-store")
        super().end_headers()

    def do_GET(self):
        if self.path in ("/", "/live", "/live_swarm.html"):
            body = HTML.encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        if self.path.startswith("/camera/wavel_"):
            self._serve_camera()
            return
        super().do_GET()

    def _serve_camera(self):
        role = self.path.split("/camera/", 1)[1].split(".jpg", 1)[0]
        image = ROOT / "outputs" / "camera" / f"{role}_latest.jpg"
        if image.exists():
            body = image.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", "image/jpeg")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
            return
        body = f"<svg xmlns='http://www.w3.org/2000/svg' width='640' height='360'><rect width='640' height='360' fill='#020617'/><text x='28' y='58' fill='#38bdf8' font-family='Arial' font-size='28'>{role} camera</text><text x='28' y='104' fill='#94a3b8' font-family='Arial' font-size='18'>waiting for frame</text></svg>".encode()
        self.send_response(200)
        self.send_header("Content-Type", "image/svg+xml")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)


def main() -> None:
    server = ThreadingHTTPServer(("127.0.0.1", PORT), Handler)
    print(f"Wavel swarm live: http://127.0.0.1:{PORT}/live_swarm.html")
    server.serve_forever()


if __name__ == "__main__":
    main()

