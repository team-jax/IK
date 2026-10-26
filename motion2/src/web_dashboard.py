"""Web dashboard: shows the live wrist (x, y, z) coordinates + per-finger curl values
(index/middle/ring) from vision_sender.py in a browser.

Listens for wrist UDP packets on its own port (5006) so it doesn't compete with
mujoco_receiver.py's exclusive bind on 5005 — vision_sender.py sends each packet to
both ports. Uses only the Python stdlib (no Flask/etc.) to keep this dependency-free.

Usage:
    venv/bin/python motion2/src/web_dashboard.py
    open http://127.0.0.1:8000
"""

import json
import socket
import threading
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

UDP_IP = "127.0.0.1"
UDP_PORT = 5006
HTTP_PORT = 8000

_lock = threading.Lock()
_latest = {"x": 0.0, "y": 0.0, "z": 0.0, "curls": [0.0, 0.0, 0.0], "packets": 0}

PAGE = """<!DOCTYPE html>
<html>
<head>
<meta charset="utf-8">
<title>Wrist Position Dashboard</title>
<style>
  body { font-family: monospace; background:#111; color:#eee; text-align:center; }
  #vals { font-size: 24px; margin: 16px; }
  canvas { background:#222; border:1px solid #555; }
</style>
</head>
<body>
<h2>Right Wrist Position</h2>
<div id="vals">waiting for data...</div>
<canvas id="c" width="400" height="400"></canvas>
<div style="width:400px;margin:8px auto;text-align:left;">
  <div>검지 <span id="curl0-val">0.00</span></div>
  <div style="height:18px;background:#333;border:1px solid #555;"><div id="curl0" style="height:100%;width:0%;background:#ff9800;"></div></div>
  <div>중지 <span id="curl1-val">0.00</span></div>
  <div style="height:18px;background:#333;border:1px solid #555;"><div id="curl1" style="height:100%;width:0%;background:#4caf50;"></div></div>
  <div>약지 <span id="curl2-val">0.00</span></div>
  <div style="height:18px;background:#333;border:1px solid #555;"><div id="curl2" style="height:100%;width:0%;background:#2196f3;"></div></div>
</div>
<script>
const ctx = document.getElementById('c').getContext('2d');
async function poll() {
  try {
    const r = await fetch('/wrist.json');
    const d = await r.json();
    document.getElementById('vals').textContent =
      `x=${d.x.toFixed(3)}  y=${d.y.toFixed(3)}  z=${d.z.toFixed(3)}  (packets: ${d.packets})`;
    for (let i = 0; i < 3; i++) {
      document.getElementById(`curl${i}`).style.width = `${(d.curls[i] * 100).toFixed(0)}%`;
      document.getElementById(`curl${i}-val`).textContent = d.curls[i].toFixed(2);
    }
    ctx.clearRect(0, 0, 400, 400);
    ctx.beginPath();
    ctx.arc(d.x * 400, d.y * 400, 10, 0, 2 * Math.PI);
    ctx.fillStyle = d.z > 0 ? '#4caf50' : '#f44336';
    ctx.fill();
  } catch (e) {
    document.getElementById('vals').textContent = 'no connection';
  }
  setTimeout(poll, 100);
}
poll();
</script>
</body>
</html>
"""


class Handler(BaseHTTPRequestHandler):
    def log_message(self, fmt, *args):
        pass  # keep stdout quiet

    def do_GET(self):
        if self.path == "/wrist.json":
            with _lock:
                body = json.dumps(_latest).encode()
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)
        else:
            body = PAGE.encode()
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)


def udp_listener():
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind((UDP_IP, UDP_PORT))
    while True:
        raw, _ = sock.recvfrom(1024)
        try:
            vals = [float(v) for v in raw.decode().strip().split()]
            if len(vals) in (6, 12):  # x y z curl_index curl_middle curl_ring [+ elbow, shoulder xyz]
                x, y, z = vals[:3]
                curls = vals[3:6]
            elif len(vals) == 4:    # legacy format: x y z grip → same value on all 3 fingers
                x, y, z, grip = vals
                curls = [grip, grip, grip]
            else:
                continue
        except ValueError:
            continue
        with _lock:
            _latest["x"] = x
            _latest["y"] = y
            _latest["z"] = z
            _latest["curls"] = curls
            _latest["packets"] += 1


def main():
    threading.Thread(target=udp_listener, daemon=True).start()
    server = ThreadingHTTPServer(("127.0.0.1", HTTP_PORT), Handler)
    print(f"Dashboard: http://127.0.0.1:{HTTP_PORT}")
    print(f"Listening for wrist UDP packets on {UDP_IP}:{UDP_PORT}")
    server.serve_forever()


if __name__ == "__main__":
    main()
