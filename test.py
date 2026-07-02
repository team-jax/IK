import os

import gymnasium.envs.mujoco
import mujoco
import mujoco.viewer

ASSETS_DIR = os.path.join(os.path.dirname(gymnasium.envs.mujoco.__file__), "assets")
model = mujoco.MjModel.from_xml_path(os.path.join(ASSETS_DIR, "reacher.xml"))
data = mujoco.MjData(model)

with mujoco.viewer.launch_passive(model, data) as viewer:
    while viewer.is_running():
        mujoco.mj_step(model, data)
        viewer.sync()