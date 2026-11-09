"""Find the arm's home pose (HOME_ARM_QPOS) for the setup JAX_SETUP picks, by the rule it was chosen by.

    venv/bin/python motion1/scripts/find_home.py --spot 0.20 -0.10 0.05                   # sim setup
    JAX_SETUP=real venv/bin/python motion1/scripts/find_home.py --spot 0.20 -0.30 0.10 --free

The home pose matters beyond where an episode starts: the controller's IK starts every solve from it
and pulls the arm's spare freedom back to it, so it picks the elbow branch and, with it, the band of
the table the arm can serve (env.py, HOME_ARM_QPOS). The rule:

1. The grasp centre (the `end_effector` site) at the home spot: --spot x y and its height above the
   table top. Palm-down there (PALM_DOWN within REACH_TILT), as in the sim setup; with --free in any
   orientation, as in the real setup, whose leaning hand turns towards the home pose's own
   orientation on the way there (env._hand_rotation).
2. Candidates by multi-start IK: from the current HOME_ARM_QPOS and from --starts random arm postures
   (seeded, so a run repeats), each solve pulled towards its own start. A solution within 0.05 rad of
   an earlier one on every joint is the same pose and is dropped.
3. Each candidate is scored by the band scripts/build_reachmap.py would save with it as the home: the
   cells where the grasp, the clear-air pose above it, the release and the carry pose above that are
   all reachable by the controller's own IK (env.reachable), with the rim trimmed.
4. The widest band wins; a tie goes to the one with more cells at least HAND_CLEAR + 0.03 from the
   home spot, where the layout rules let goals stand.

It prints every candidate and the winner and edits nothing: copy the winner into SETUPS in env.py,
then rebuild the reach map (scripts/build_reachmap.py). A band takes about half a minute to solve.
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse

import mujoco
import numpy as np

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
from humanoid_grasp_rl.demonstration import TRAVEL_CLEAR
from humanoid_grasp_rl.ik import solve_site_pose

parser = argparse.ArgumentParser(description="Find HOME_ARM_QPOS for the current JAX_SETUP.")
parser.add_argument("--spot", type=float, nargs=3, metavar=("X", "Y", "DZ"), required=True,
                    help="home grasp centre: world x, y, and its height above the table top (m)")
parser.add_argument("--free", action="store_true", help="any hand orientation at home (default: palm-down)")
parser.add_argument("--starts", type=int, default=8, help="random starting postures besides HOME_ARM_QPOS")
parser.add_argument("--seed", type=int, default=0, help="seed of the random starting postures")
args = parser.parse_args()

env = JaxDeliveryEnv(stage="place")
env.reset(seed=0)
model, ik = env.model, env._ik_data
qpos_ids, dof_ids = env.qpos_ids[:7], model.jnt_dofadr[env.arm_joint_ids]
limits = model.jnt_range[env.arm_joint_ids].copy()
limits[~model.jnt_limited[env.arm_joint_ids].astype(bool)] = (-np.pi, np.pi)
spot = np.array([args.spot[0], args.spot[1], E.TABLE_SURFACE_Z + args.spot[2]])
# The grid scripts/build_reachmap.py solves.
xs = np.round(np.arange(-0.08, 0.281, 0.02), 3)
ys = np.round(np.arange(-0.30, 0.301, 0.025), 3)


def band() -> np.ndarray:
    """build_reachmap.py's feasible cells, with E.HOME_ARM_QPOS as the home env.reachable starts from."""
    clear_z = env.clear_height(E.OBJECT_Z)

    def reachable(position):
        if np.any(position < E.WORKSPACE_LOW) or np.any(position > E.WORKSPACE_HIGH):
            return False
        return env.reachable(position)

    both = np.zeros((len(xs), len(ys)), dtype=bool)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            cell = np.array([x, y])
            pick, drop = env.pick_pose(cell), env.drop_pose(cell, E.OBJECT_Z)
            over_pick = np.array([pick[0], pick[1], max(pick[2] + TRAVEL_CLEAR, clear_z)])
            over_drop = np.array([drop[0], drop[1], max(drop[2] + TRAVEL_CLEAR, clear_z)])
            both[i, j] = (reachable(pick) and reachable(over_pick)
                          and reachable(drop) and reachable(over_drop))
    padded = np.pad(both, 1, constant_values=False)
    on_table = (xs <= E.CUBE_X_MAX)[:, None]
    return both & padded[:-2, 1:-1] & padded[2:, 1:-1] & padded[1:-1, :-2] & padded[1:-1, 2:] & on_table


def solve(start: np.ndarray):
    """The arm posture nearest `start` that puts the grasp centre on the home spot, or None."""
    ik.qpos[:] = env.data.qpos
    ik.qpos[qpos_ids] = start
    ik.qvel.fill(0.0)
    if not args.free:
        ok, _, _ = solve_site_pose(model, ik, env.site_id, env.arm_joint_ids, spot, E.PALM_DOWN, start,
                                   position_tolerance=0.001, rotation_tolerance=E.REACH_TILT, max_iterations=2000)
        return ik.qpos[qpos_ids].copy() if ok else None
    # Position alone: with the rotation rows in the solve (even at a loose tolerance) it stalls.
    jp, jr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    for _ in range(3000):
        mujoco.mj_kinematics(model, ik)
        mujoco.mj_comPos(model, ik)
        error = spot - ik.site_xpos[env.site_id]
        if np.linalg.norm(error) < 0.0005:
            return ik.qpos[qpos_ids].copy()
        mujoco.mj_jacSite(model, ik, jp, jr, env.site_id)
        j = jp[:, dof_ids]
        step = j.T @ np.linalg.solve(j @ j.T + 1e-3 * np.eye(3), error)
        ik.qpos[qpos_ids] = np.clip(ik.qpos[qpos_ids] + 0.25 * step, limits[:, 0], limits[:, 1])
    return None


def joints(q) -> str:
    return "(" + ", ".join(f"{v:.4f}" for v in q) + ")"


current = E.HOME_ARM_QPOS.copy()
starts = [current] + list(np.random.default_rng(args.seed).uniform(limits[:, 0], limits[:, 1], size=(args.starts, 7)))
candidates = []
for start in starts:
    q = solve(start)
    if q is not None and all(np.max(np.abs(q - c)) >= 0.05 for c in candidates):
        candidates.append(q)
print(f"setup {E.SETUP}: home spot {np.round(spot, 4)} ({'any orientation' if args.free else 'palm-down'}); "
      f"{len(candidates)} distinct poses from {len(starts)} starts")

results = []
for k, q in enumerate([current] + candidates):
    E.HOME_ARM_QPOS = q
    cells = np.argwhere(band())
    far = sum(np.hypot(xs[i] - spot[0], ys[j] - spot[1]) >= E.HAND_CLEAR + 0.03 for i, j in cells)
    label = "current HOME_ARM_QPOS" if k == 0 else f"candidate {k}"
    print(f"  {label:22s} usable {len(cells):3d}, of them {far:3d} >= {E.HAND_CLEAR + 0.03:.2f} m from home; "
          f"q {joints(q)}", flush=True)
    if k:
        results.append((len(cells), far, k, q))
if results:
    usable, far, k, q = max(results, key=lambda r: (r[0], r[1]))
    print(f"winner: candidate {k}, usable {usable}, far {far}: home={joints(q)}")
