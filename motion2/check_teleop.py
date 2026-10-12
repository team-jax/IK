"""Teleop pipeline check, no display and no webcam: UDP packets in, cube lifted.

A thread plays the sender (60 Hz into UDP 5005) while mujoco_receiver.main() runs unchanged with a
stand-in viewer: hover, descend beside the cube, close the three fingers, lift 85 mm, hold. So the
whole path packet -> IK -> position actuators -> contact physics is exercised. Broken packets
(garbage, wrong field count, empty, NaN position) are mixed in; the receiver must shrug them off.

    venv/bin/python motion2/check_teleop.py              # the three scenarios, one process each
    venv/bin/python motion2/check_teleop.py swing-jump   # a single scenario

Scenarios:
  wrist       6-field packets (wrist and fingers only)
  arm         12-field packets whose elbow and shoulder travel with the wrist, so the elbow swing
              angle stays constant
  swing-jump  12-field packets with the shoulder and elbow held still while the wrist drops past
              them and rises back. The measured swing angle steps by 90 deg in one frame, twice
              (swivel_angle switches its reference when the shoulder-wrist axis is nearly
              vertical). The receiver has to limit the swing it asks of the arm (+-40 deg), ramp
              it, and keep it still while the fingers are closed; before that the elbow whipped,
              the IK solution hopped mid-lift and the pinched cube was thrown off the post.
"""
import contextlib
import os
import socket
import subprocess
import sys
import threading
import time

import numpy as np

SCENARIOS = ("wrist", "arm", "swing-jump")


def schedule():
    """(y, curl) per sender frame. y is the image row: 1.0 is the lowest hand (z = BASE_Z), 0.0 the
    highest (z = BASE_Z + HEIGHT_SCALE)."""
    out = []

    def seg(n, y0, y1, c0, c1):
        for i in range(n):
            out.append((y0 + (y1 - y0) * (i + 1) / n, c0 + (c1 - c0) * (i + 1) / n))

    seg(20, .5, .5, 0, 0)       # hover above the cube, hand open
    seg(120, .5, 1.0, 0, 0)     # descend beside it
    seg(40, 1.0, 1.0, 0, 0)     # settle
    seg(60, 1.0, 1.0, 0, 1)     # close the fingers
    seg(40, 1.0, 1.0, 1, 1)     # squeeze
    seg(160, 1.0, 0.0, 1, 1)    # lift
    seg(60, 0.0, 0.0, 1, 1)     # hold
    return out


def packet(scenario, y, c):
    base = f"0.5 {y:.4f} -0.3 {c:.3f} {c:.3f} {c:.3f}"
    if scenario == "wrist":
        return base
    if scenario == "arm":      # elbow and shoulder follow the wrist rigidly: constant swing angle
        e, s = (0.4, y - 0.1, -0.45), (0.6, y - 0.2, -0.3)
    else:                      # swing-jump: shoulder and elbow fixed, the wrist drops past them
        e, s = (0.4, 0.6, -0.3), (0.6, 0.3, -0.3)
    return base + " " + " ".join(f"{v:.4f}" for v in (*e, *s))


def run(scenario):
    """One scenario in this process. Raises AssertionError when the cube is not lifted."""
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import mujoco.viewer as mv
    import mujoco_receiver as m

    state = {"frames": 0, "after": 0, "done": False, "data": None, "log": []}

    class Cam:
        def __init__(self):
            self.lookat = np.zeros(3)
            self.distance = self.azimuth = self.elevation = 0.0
            self.type = 0
            self.trackbodyid = -1

    class FakeViewer:
        def __init__(self, model, data):
            self.cam = Cam()
            state["data"] = data       # created after main() has bound the UDP socket

        def is_running(self):
            state["frames"] += 1
            state["after"] += state["done"]
            d = state["data"]
            if state["frames"] % 40 == 0:
                state["log"].append((state["frames"], float(d.body("object").xpos[2]),
                                     float(d.site("end_effector").xpos[2]),
                                     float(d.joint("finger1_joint1").qpos[0])))
            return state["after"] < 80

        def lock(self):
            return contextlib.nullcontext()

        def sync(self):
            pass

        def __enter__(self):
            return self

        def __exit__(self, *exc):
            return False

    mv.launch_passive = lambda model, data, key_callback=None: FakeViewer(model, data)

    def sender():
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        while state["data"] is None:
            time.sleep(0.05)
        time.sleep(0.2)
        t0 = time.monotonic()
        for i, (y, c) in enumerate(schedule()):
            sock.sendto(packet(scenario, y, c).encode(), ("127.0.0.1", m.UDP_PORT))
            if i % 25 == 7:
                junk = [b"garbage", b"1 2 3", b"", f"nan nan nan {c:.3f} {c:.3f} {c:.3f}".encode(), b"0.5 0.5"]
                sock.sendto(junk[(i // 25) % len(junk)], ("127.0.0.1", m.UDP_PORT))
            time.sleep(max(0.0, t0 + (i + 1) / 60 - time.monotonic()))
        state["done"] = True

    threading.Thread(target=sender, daemon=True).start()
    started = time.time()
    m.main()
    d = state["data"]
    cube, ee = d.body("object").xpos, d.site("end_effector").xpos
    print(f"[{scenario}] {state['frames']} frames in {time.time() - started:.1f} s")
    print("  frame  cube_z  hand_z  finger1")
    for row in state["log"]:
        print(f"  {row[0]:5d}  {row[1]:.3f}   {row[2]:.3f}   {row[3]:+.3f}")
    print(f"  end: hand {np.round(ee, 3)}  cube {np.round(cube, 3)}")
    assert np.all(np.isfinite(d.qpos)), "state went non-finite"
    assert ee[2] > 0.33, f"hand did not reach the top (z = {ee[2]:.3f})"
    assert cube[2] > 0.30, f"cube was not lifted (z = {cube[2]:.3f}); it was knocked off or dropped"
    assert np.linalg.norm(cube[:2] - ee[:2]) < 0.02, "cube is not in the hand"


if __name__ == "__main__":
    if len(sys.argv) > 1:
        run(sys.argv[1])
        print(f"[{sys.argv[1]}] ok")
    else:
        failed = [s for s in SCENARIOS
                  if subprocess.run([sys.executable, os.path.abspath(__file__), s]).returncode != 0]
        print(f"teleop: {len(SCENARIOS) - len(failed)}/{len(SCENARIOS)} scenarios passed"
              + (f"  (failed: {', '.join(failed)})" if failed else ""))
        sys.exit(1 if failed else 0)
