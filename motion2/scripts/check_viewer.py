"""Viewer smoke test: opens gymnasium's reacher.xml in MuJoCo's passive viewer, to confirm that the
GUI works with this venv. A display is required and it must be awake (see motion2/README.md).

    venv/bin/mjpython motion2/scripts/check_viewer.py   # macOS
    venv/bin/python motion2/scripts/check_viewer.py     # Linux
"""
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
