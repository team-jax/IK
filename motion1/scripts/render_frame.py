"""Render the delivery scene (seed 7, the overview camera) to a PNG; prints the path."""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
from pathlib import Path

import mujoco
from PIL import Image

from humanoid_grasp_rl import JaxDeliveryEnv


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", type=Path, default=_paths.ROOT / "artifacts/scene.png")
output = parser.parse_args().out
output.parent.mkdir(parents=True, exist_ok=True)
env = JaxDeliveryEnv(render_mode="rgb_array")
env.reset(seed=7)
renderer = mujoco.Renderer(env.model, 720, 1280)
renderer.update_scene(env.data, camera="overview")
Image.fromarray(renderer.render()).save(output)
env.close()
print(output)
