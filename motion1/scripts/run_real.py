"""A delivery given in real coordinates: run it in the sim and save the joint trajectory for the robot.

    JAX_SETUP=real venv/bin/python motion1/scripts/run_real.py --cube 0.10 -0.30 20 --target 0.05 -0.27
    JAX_SETUP=real venv/bin/python motion1/scripts/run_real.py --cube ... --target ... --out motion1/artifacts/traj.json

--cube is x y yaw and --target x y in the real frame of realcoords.py (origin on the shoulder roll
motor's axis, x to the robot's left, y backwards, metres; yaw in degrees, counter-clockwise seen
from above, of a face normal from +x). --d is the shoulder axis down to the table top.

The work is humanoid_grasp_rl.real_task.plan_delivery (team-jax/SimToReal's pick_place calls the
same function on the robot computer). Only a clean run is written - delivered with no retry - since
the robot replays it open loop; --keep writes any run, marked "playable": false, for looking at, and
play_trajectory refuses it unless forced.
Exit code: 0 clean delivery (written), 1 not delivered, 4 delivered only after a retry,
3 the arm cannot serve it (2 is argparse).
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import json
import os
import sys
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--cube", type=float, nargs=3, required=True, metavar=("X", "Y", "YAW"))
parser.add_argument("--target", type=float, nargs=2, required=True, metavar=("X", "Y"))
parser.add_argument("--d", type=float, default=None, help="shoulder axis to table top, m (default: the setup's)")
parser.add_argument("--out", default=None, help="trajectory JSON (default: motion1/artifacts/trajectory.json)")
parser.add_argument("--keep", action="store_true", help="write the trajectory even if it is not playable")
args = parser.parse_args()

if os.environ.get("JAX_SETUP") != "real":
    sys.exit("real coordinates belong to the real setup: run with JAX_SETUP=real")

import humanoid_grasp_rl.env as E  # noqa: E402
from humanoid_grasp_rl.real_task import plan_delivery  # noqa: E402

r = plan_delivery(args.cube, args.target, args.d)
c, s = r["real"], r["sim"]
print(f"cube (real {c['cube'][0]:+.3f}, {c['cube'][1]:+.3f} m, {c['cube'][2]:+.1f} deg) -> sim "
      f"{[round(v, 4) for v in s['cube']]}, yaw {s['cube_yaw_deg']:+.1f} deg")
print(f"target (real {c['target'][0]:+.3f}, {c['target'][1]:+.3f} m) -> sim {[round(v, 4) for v in s['target']]}")
print(f"reach map: {'yes' if r['reach_map']['reachable'] else 'no'} ({r['reach_map']['reason']})")
if r["status"] == "unreachable":
    print(f"env: {r['reason']}")
    sys.exit(3)
print(f"env: servable, hand turned {s['hand_yaw_deg']:+.1f} deg")
frames, dt, peak = r["positions"], r["dt"], r["peak_speed_deg_s"]
print(f"sim: {'delivered' if r['delivered_in_sim'] else 'NOT delivered'}, {r['final_error_m'] * 1000:.1f} mm from "
      f"the target, {len(frames)} frames x {dt * 1000:.0f} ms = {len(frames) * dt:.1f} s; fastest joint "
      f"{peak['joint']} {peak['value']:.0f} deg/s")
if r["retries"]:
    print("sim retries: " + ", ".join(f"{x['phase']}@{x['step']} ({x['hand']})" for x in r["retries"]))
code = {"playable": 0, "not delivered": 1, "retried": 4}[r["status"]]
if not r["playable"] and not args.keep:
    print(f"not playable on the robot ({r['reason']}): no trajectory written. "
          "--keep writes it anyway, marked not playable")
    sys.exit(code)

out = Path(args.out) if args.out else Path(E.__file__).resolve().parents[2] / "artifacts" / "trajectory.json"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps(r, indent=1))
print(f"wrote {out}" + ("" if r["playable"] else " (marked not playable)"))
sys.exit(code)
