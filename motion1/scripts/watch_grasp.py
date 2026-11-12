"""Watch the scripted delivery in the interactive MuJoCo viewer, at real-time speed.

    venv/bin/mjpython motion1/scripts/watch_grasp.py --seed 3 --randomization-scale 1.0
    venv/bin/mjpython motion1/scripts/watch_grasp.py --layouts motion1/artifacts/layouts_10.json --seed 3
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import sys
import time
from pathlib import Path

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=0)
parser.add_argument("--randomization-scale", type=float, default=0.0)
parser.add_argument("--layouts", default=None,
                    help="a file from scripts/layouts.py, so the scene cannot shift with the map")
args = parser.parse_args()

import mujoco  # noqa: E402
from mujoco import viewer  # noqa: E402

import humanoid_grasp_rl.env as E  # noqa: E402

from humanoid_grasp_rl.demonstration import scripted_palm_place  # noqa: E402
from humanoid_grasp_rl.single_env import JaxDeliveryEnv  # noqa: E402

if args.layouts:
    sys.path.insert(0, str(Path(__file__).parent))
    from layouts import replay
    env, _ = replay(args.layouts)
else:
    env = JaxDeliveryEnv(stage="randomize", randomization_scale=args.randomization_scale)
env.reset(seed=args.seed)
period = env.model.opt.timestep * env.frame_skip
info, closed_at, pushed = {"is_success": False}, None, []
with viewer.launch_passive(env.model, env.data) as window:
    if E.VIEW:   # the real setup starts from the table's side (env.VIEW); the mouse still moves the camera
        with window.lock():
            window.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
            window.cam.lookat[:] = (E.VIEW["lookat"][0], E.VIEW["lookat"][1], E.TABLE_SURFACE_Z + E.VIEW["lookat"][2])
            window.cam.distance, window.cam.azimuth, window.cam.elevation = (
                E.VIEW["distance"], E.VIEW["azimuth"], E.VIEW["elevation"])
    for step in range(env.max_steps):
        if not window.is_running():
            closed_at = step
            break
        started = time.time()
        _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
        window.sync()
        # Ctrl+drag in the viewer pushes a body (xfrc_applied): that changes the run, so say so
        if env.data.xfrc_applied.any():
            pushed.append(step)
        time.sleep(max(0.0, period - (time.time() - started)))
        if terminated or truncated:
            break
print(f"phase {env._palm_phase}, delivered {bool(info['is_success'])}")
if closed_at is not None:
    print(f"  the window was closed at step {closed_at} of {env.max_steps} ({closed_at * period:.1f} s): "
          "the run stopped there, so this is not its result. Leave the window open to the end.")
if pushed:
    print(f"  a body was pushed with the mouse (Ctrl+drag) during the run, steps {pushed[0]}-{pushed[-1]}: "
          "the result is not the scripted one.")
env.close()
