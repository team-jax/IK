"""Measure how the pads close, to choose FINGER_OPEN / FINGER_CLOSED / CUBE_GRIP.

For each closure g along the path env.py defines, this poses the three fingers and measures, on
the finger collision hulls, the gap between finger 3's pad and finger 1's pad over the lowest
40 mm of the plates (the part that meets a cube standing on the table), and the lean of the
plates: the sum of the proximal and distal angles. Negative leans the tips away from the cube,
which turns the squeeze into a push down the cube's side; the grasp wants it upright or
slightly positive from the moment the pads touch the cube.

    venv/bin/python motion1/scripts/tune_grasp.py
"""
import _paths  # noqa: F401  (puts src/ on the path)
import mujoco
import numpy as np

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv

env = JaxDeliveryEnv()
env.reset(seed=0)
model, data = env.model, env.data
site = data.site("end_effector")


def hull(geom_id):
    """Corners of the finger's collision box, or the vertices of its mesh (JAX_FINGER_COLLISION=mesh)."""
    if model.geom_type[geom_id] == mujoco.mjtGeom.mjGEOM_MESH:
        mesh = model.geom_dataid[geom_id]
        vertices = model.mesh_vert[model.mesh_vertadr[mesh]:model.mesh_vertadr[mesh] + model.mesh_vertnum[mesh]]
    else:
        signs = np.array([[x, y, z] for x in (-1, 1) for y in (-1, 1) for z in (-1, 1)])
        vertices = signs * model.geom_size[geom_id]
    return data.geom_xpos[geom_id] + vertices @ data.geom_xmat[geom_id].reshape(3, 3).T


def pad_x(finger, low=-0.0175, high=0.0225):
    """x of the finger's plate in the grasp-centre frame, over a height slab above its tips."""
    points = np.vstack([hull(model.geom(f"Finger_{finger}_{k}_collision_geom").id) for k in (1, 2)])
    local = (points - site.xpos) @ site.xmat.reshape(3, 3)
    return local[(local[:, 2] > low) & (local[:, 2] < high), 0]


def gap_at(closure):
    data.qpos[env.qpos_ids[7:]] = env.finger_targets(closure)
    mujoco.mj_forward(model, data)
    return 1000 * (pad_x(3).min() - pad_x(1).max())


print(f"{'g':>5s} {'proximal':>9s} {'distal':>7s} {'lean':>7s} {'gap':>7s}")
touch, previous = None, None
for g in np.arange(0.0, 1.0001, 0.05):
    proximal, distal = E.FINGER_OPEN + g * (E.FINGER_CLOSED - E.FINGER_OPEN)
    gap = gap_at(g)
    if touch is None and gap <= 1000 * E.OBJECT_SIZE and previous is not None:
        # Interpolated between this step and the last, so the answer is not a multiple of 0.05.
        touch = previous[0] + (previous[1] - 1000 * E.OBJECT_SIZE) / (previous[1] - gap) * (g - previous[0])
    previous = (g, gap)
    print(f"{g:5.2f} {proximal:+9.3f} {distal:7.3f} {proximal + distal:+7.3f} {gap:6.1f}mm")
print(f"\nCUBE_GRIP in env.py is {E.CUBE_GRIP}; the pads are as far apart as a "
      f"{1000 * E.OBJECT_SIZE:.0f} mm cube at g = {touch:.2f}")
env.close()
