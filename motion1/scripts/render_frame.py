import _paths  # noqa: F401  (puts src/ on the path)
from pathlib import Path

import mujoco
from PIL import Image

from humanoid_grasp_rl import JaxDeliveryEnv


output = _paths.ROOT / "artifacts/scene.png"
output.parent.mkdir(parents=True, exist_ok=True)
env = JaxDeliveryEnv(render_mode="rgb_array")
env.reset(seed=7)
renderer = mujoco.Renderer(env.model, 720, 1280)
renderer.update_scene(env.data, camera="overview")
Image.fromarray(renderer.render()).save(output)
env.close()
print(output)
