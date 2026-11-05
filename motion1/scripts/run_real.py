"""A delivery given in real coordinates: run it in the sim and save the joint trajectory for the robot.

    JAX_SETUP=real venv/bin/python motion1/scripts/run_real.py --cube 0.10 -0.30 20 --target 0.05 -0.27
    JAX_SETUP=real venv/bin/python motion1/scripts/run_real.py --cube ... --target ... --out motion1/artifacts/traj.json

--cube is x y yaw and --target x y in the real frame of realcoords.py (origin on the shoulder roll
motor's axis, x to the robot's left, y backwards, metres; yaw in degrees, counter-clockwise seen
from above, of a face normal from +x). --d is the shoulder axis down to the table top.

realcoords.plan() answers first from the reach map (numpy only, what the robot computer runs); the
env then checks the exact poses (env.reset(options=...)) and has the last word. The trajectory is
every control step's commanded joint angles (data.ctrl: 7 arm + 6 finger joints, rad, sim joint
names), which team-jax/SimToReal's play_trajectory sends to /sim2real/joint_command.

The robot plays it open loop: nothing on it knows whether the cube is still in the hand. So only a
clean run is written - delivered with no retry. A retry in the sim (the cube dropped and fetched
again) is baked into the joint angles, and replayed on the robot it repeats a slip that may not
happen there. --keep writes any run, marked "playable": false, for looking at; play_trajectory
refuses it unless forced.
Exit code: 0 clean delivery (written), 1 not delivered, 4 delivered only after a retry,
3 the arm cannot serve it (2 is argparse).
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import json
import os
import sys
from pathlib import Path

import numpy as np

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
from humanoid_grasp_rl.single_env import JaxDeliveryEnv  # noqa: E402
from humanoid_grasp_rl.demonstration import scripted_palm_place  # noqa: E402

sys.path.insert(0, str(Path(E.__file__).parent))
import realcoords as R  # noqa: E402

d = args.d if args.d is not None else E.D
plan = R.plan(tuple(args.cube), tuple(args.target), d)
print(f"cube (real {args.cube[0]:+.3f}, {args.cube[1]:+.3f} m, {args.cube[2]:+.1f} deg) -> sim "
      f"{plan['cube'].round(4).tolist()}, yaw {plan['yaw']:+.1f} deg")
print(f"target (real {args.target[0]:+.3f}, {args.target[1]:+.3f} m) -> sim {plan['target'].round(4).tolist()}")
print(f"reach map: {'yes' if plan['reachable'] else 'no'} ({plan['reason']})")
if abs(d - E.D) > R.D_TOLERANCE:
    sys.exit(f"the sim's table is D = {E.D} m below the shoulder axis, not {d}: no run")

env = JaxDeliveryEnv()
try:
    env.reset(seed=0, options={"objects": [[*plan["cube"][:2], plan["yaw"]]], "targets": [plan["target"]]})
except ValueError as e:
    print(f"env: {e}")
    sys.exit(3)
hand = float(np.degrees(env._hand_yaws[0]))
print(f"env: servable, hand turned {hand:+.1f} deg")

names = list(env._robot.arm_joints) + list(env._robot.finger_joints)
actuators = np.concatenate((env.arm_actuator_ids, env.finger_actuator_ids))
dt = float(env.model.opt.timestep * env.frame_skip)
frames = [env.data.ctrl[actuators].tolist()]
info = {}
for step in range(env.max_steps):
    _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
    frames.append(env.data.ctrl[actuators].tolist())
    if terminated or truncated:
        break
delivered = bool(info.get("is_success")) and bool(info.get("success_now"))
retries = [{"step": int(t), "phase": ph, "hand": what} for t, _, ph, what in getattr(env, "_palm_failures", [])]
playable = delivered and not retries
cube = env.data.xpos[env.object_body_ids[0]]
error = float(np.linalg.norm(cube[:2] - plan["target"]))
q = np.array(frames)
speed = np.degrees(np.abs(np.diff(q, axis=0)).max(axis=0)) / dt
print(f"sim: {'delivered' if delivered else 'NOT delivered'}, {error * 1000:.1f} mm from the target, "
      f"{len(frames)} frames x {dt * 1000:.0f} ms = {len(frames) * dt:.1f} s; fastest joint "
      f"{names[int(speed.argmax())]} {speed.max():.0f} deg/s")
if retries:
    print("sim retries: " + ", ".join(f"{r['phase']}@{r['step']} ({r['hand']})" for r in retries))
code = 0 if playable else (4 if delivered else 1)
if not playable and not args.keep:
    print("not playable on the robot (" + ("delivered only after a retry" if delivered else "not delivered")
          + "): no trajectory written. --keep writes it anyway, marked not playable")
    env.close()
    sys.exit(code)

out = Path(args.out) if args.out else Path(E.__file__).resolve().parents[2] / "artifacts" / "trajectory.json"
out.parent.mkdir(parents=True, exist_ok=True)
out.write_text(json.dumps({
    "joint_names": names, "dt": dt, "positions": frames, "unit": "rad",
    "playable": playable, "delivered_in_sim": delivered, "retries": retries, "final_error_m": error,
    "real": {"cube": args.cube, "target": args.target, "d": d},
    "sim": {"cube": plan["cube"].tolist(), "target": plan["target"].tolist(),
            "cube_yaw_deg": plan["yaw"], "hand_yaw_deg": hand},
}, indent=1))
print(f"wrote {out}" + ("" if playable else " (marked not playable)"))
env.close()
sys.exit(code)
