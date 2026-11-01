"""Film one pick and place, built entirely on measured poses."""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import os
import sys
from pathlib import Path

if sys.platform != "darwin":  # headless Linux has no window system; macOS renders through CGL
    os.environ.setdefault("MUJOCO_GL", "egl")

import imageio.v2 as imageio  # noqa: E402
import mujoco  # noqa: E402
import numpy as np  # noqa: E402

import humanoid_grasp_rl.env as E  # noqa: E402
from humanoid_grasp_rl import JaxDeliveryEnv, JaxStackEnv  # noqa: E402
from humanoid_grasp_rl.demonstration import scripted_palm_place  # noqa: E402

parser = argparse.ArgumentParser()
parser.add_argument("--seed", type=int, default=400)
parser.add_argument("--randomization-scale", type=float, default=0.0)
parser.add_argument("--output", type=Path, default=_paths.ROOT / "artifacts/video/delivery.mp4")
parser.add_argument("--width", type=int, default=1920)
parser.add_argument("--height", type=int, default=1080)
parser.add_argument("--distance", type=float, default=0.95)
parser.add_argument("--azimuth", type=float, default=140.0)
parser.add_argument("--elevation", type=float, default=-22.0)
parser.add_argument("--hold", type=int, default=60,
                    help="frames of the last image repeated at the end - a freeze, not more physics")
parser.add_argument("--stack", action="store_true", help="cubes to go on top of one another")
parser.add_argument("--layouts", default=None,
                    help="a file from scripts/layouts.py, so the scene cannot shift with the map")
args = parser.parse_args()

if args.layouts:
    sys.path.insert(0, str(Path(__file__).parent))
    from layouts import replay
    env, _ = replay(args.layouts)
else:
    kwargs = dict(stage="randomize", randomization_scale=args.randomization_scale)
    env = JaxStackEnv(cubes=2, **kwargs) if args.stack else JaxDeliveryEnv(**kwargs)
env.reset(seed=args.seed)
env._palm_phase = None
camera = mujoco.MjvCamera()
camera.type = mujoco.mjtCamera.mjCAMERA_FREE
camera.lookat[:] = (0.10, -0.10, 0.45)
camera.distance, camera.azimuth, camera.elevation = args.distance, args.azimuth, args.elevation
renderer = mujoco.Renderer(env.model, args.height, args.width)
args.output.parent.mkdir(parents=True, exist_ok=True)
writer = imageio.get_writer(args.output, fps=50, quality=8, macro_block_size=None)

marks = {}
peak = 0.0
for step in range(env.max_steps):
    before = (env.moving, env._palm_phase)
    _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
    if (env.moving, env._palm_phase) != before:
        marks.setdefault(f"cube{env.moving}:{env._palm_phase}", step)
    peak = max(peak, float(env.data.xpos[env.object_body_ids[env.moving], 2]))
    renderer.update_scene(env.data, camera)
    writer.append_data(renderer.render())
    if terminated or truncated:
        break
for _ in range(args.hold):
    writer.append_data(renderer.render())

cube = env.data.xpos[env.object_body_ids[env.moving]]
gap = float(np.linalg.norm(cube[:2] - env._target_positions[env.moving]))
done_once, done_now = bool(info["is_success"]), bool(info.get("success_now"))
print(f"seed {args.seed}, randomization {env.randomization_scale}: "
      f"delivered {done_once and done_now}"
      + ("" if done_once == done_now else " (done once, not at the end)")
      + f", {gap*1000:.1f} mm from the spot, carried to {(peak - E.OBJECT_Z)*1000:.0f} mm"
      + (f", retries {[(ph, t) for t, _, ph, _ in env._palm_failures]}" if env._palm_failures else ""))
print("phase starts:", marks)
writer.close()
renderer.close()
env.close()
# Read the file back: the deliverable is the video, so the video is what gets checked.
reader = imageio.get_reader(args.output)
meta = reader.get_meta_data()
frames = reader.count_frames()
reader.close()
print(f"written to {args.output}: {meta.get('size')} at {meta.get('fps')} fps, {frames} frames "
      f"({frames / meta.get('fps', 50):.1f} s, of which the last {args.hold} are a freeze)")
