"""Headless check of web_dashboard.py as a subprocess: UDP packets in, /wrist.json and the page out.

    venv/bin/python motion2/check_dashboard.py
"""
import json
import os
import socket
import subprocess
import sys
import time
import urllib.request

proc = subprocess.Popen([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), "web_dashboard.py")],
                        stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
BASE = "http://127.0.0.1:8000"


def get():
    return json.load(urllib.request.urlopen(f"{BASE}/wrist.json", timeout=2))


try:
    for _ in range(60):
        try:
            get()
            break
        except Exception:
            time.sleep(0.1)
    else:
        raise SystemExit("dashboard did not start")
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def send(raw):
        sock.sendto(raw, ("127.0.0.1", 5006))
        time.sleep(0.2)

    assert get()["packets"] == 0
    send(b"0.1 0.2 0.3 0.4 0.5 0.6")
    d = get()
    assert (d["packets"], d["x"], d["y"], d["z"], d["curls"]) == (1, 0.1, 0.2, 0.3, [0.4, 0.5, 0.6]), d
    send(b"0.1 0.2 0.3 0.4 0.5 0.6 0.7 0.8 0.9 1.0 1.1 1.2")          # 12 fields
    assert get()["packets"] == 2
    send(b"0.9 0.8 0.7 0.3")                                            # legacy 4 fields
    d = get()
    assert (d["packets"], d["x"], d["curls"]) == (3, 0.9, [0.3, 0.3, 0.3]), d
    for bad in (b"garbage", b"1 2 3", b"0.5 0.5", b""):                 # must be ignored
        send(bad)
    assert get()["packets"] == 3, "a malformed packet was counted"
    page = urllib.request.urlopen(BASE + "/", timeout=2).read().decode()
    assert "Wrist Position Dashboard" in page and "/wrist.json" in page
    print("dashboard: ok")
finally:
    proc.terminate()
    proc.wait(timeout=5)
