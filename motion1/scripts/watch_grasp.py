"""Watch the scripted delivery in the interactive MuJoCo viewer, at real-time speed.

    venv/bin/mjpython motion1/scripts/watch_grasp.py --seed 3 --randomization-scale 1.0
    venv/bin/python motion1/scripts/watch_grasp.py --layouts motion1/artifacts/layouts_10.json --seed 3
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

from mujoco import viewer  # noqa: E402

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
with viewer.launch_passive(env.model, env.data) as window:
    for step in range(env.max_steps):
        if not window.is_running():
            break
        started = time.time()
        _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
        window.sync()
        time.sleep(max(0.0, period - (time.time() - started)))
        if terminated or truncated:
            break
print(f"phase {env._palm_phase}, delivered {bool(info['is_success'])}")
env.close()
