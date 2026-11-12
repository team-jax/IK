"""Render the delivery scene (seed 7, the overview camera) to a PNG; prints the path."""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
from pathlib import Path

import mujoco
from PIL import Image

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv


parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--out", type=Path, default=_paths.ROOT / "artifacts/scene.png")
output = parser.parse_args().out
output.parent.mkdir(parents=True, exist_ok=True)
env = JaxDeliveryEnv(render_mode="rgb_array")
env.reset(seed=7)
renderer = mujoco.Renderer(env.model, 720, 1280)
if E.VIEW:   # the real setup: the overview camera looks past the stand's column
    camera = mujoco.MjvCamera()
    camera.type = mujoco.mjtCamera.mjCAMERA_FREE
    camera.lookat[:] = (E.VIEW["lookat"][0], E.VIEW["lookat"][1], E.TABLE_SURFACE_Z + E.VIEW["lookat"][2])
    camera.distance, camera.azimuth, camera.elevation = E.VIEW["distance"], E.VIEW["azimuth"], E.VIEW["elevation"]
else:
    camera = "overview"
renderer.update_scene(env.data, camera=camera)
Image.fromarray(renderer.render()).save(output)
env.close()
print(output)
