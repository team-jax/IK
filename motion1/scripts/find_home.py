"""Find the arm's home pose (HOME_ARM_QPOS) for the setup JAX_SETUP picks, by the rule it was chosen by.

    venv/bin/python motion1/scripts/find_home.py --spot 0.20 -0.10 0.05                   # sim setup
    JAX_SETUP=real venv/bin/python motion1/scripts/find_home.py --spot 0.13 0.15 0.10 --free --rollouts 12

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

A band is solved one pose at a time, so it cannot tell whether the controller gets there: in the
real setup the widest band's home stalled in every delivery, the arm straightening its elbow into a
joint stop on the way out (motion1/STATUS.md, 2026-11-10). --rollouts N therefore also runs N
deliveries per candidate with the controller itself, over layouts its band allows (a cube and a goal
TARGET_MIN_SEPARATION apart, clear of the home spot as env._sample_layout requires; "layouts" counts
them, and with none every episode falls back to the fixed layout), and the ranking becomes: most
deliveries, then most cubes picked up, then most layouts, then the band. "stalled" counts episodes
whose first failure was the reach phase, "elbow" the smallest |elbow| angle seen (0 is straight).

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
parser.add_argument("--rollouts", type=int, default=0, help="deliveries per candidate run with the controller")
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

def layouts(cells) -> list:
    """(cube, goal) cell pairs env._sample_layout would accept, given this home spot."""
    points = [np.array([xs[i], ys[j]]) for i, j in cells]
    cubes = [p for p in points if np.linalg.norm(p - spot[:2]) >= E.SPAWN_CLEARANCE]
    goals = [p for p in points if np.linalg.norm(p - spot[:2]) >= E.HAND_CLEAR + 0.03]
    return [(c, g) for c in cubes for g in goals if np.linalg.norm(c - g) >= E.TARGET_MIN_SEPARATION]


def rollout(pairs, n) -> dict:
    """Deliveries with the controller from the home now in E.HOME_ARM_QPOS, over n of the pairs."""
    from humanoid_grasp_rl.demonstration import scripted_palm_place
    from humanoid_grasp_rl.single_env import JaxDeliveryEnv
    E.WORK_X_MIN = float(xs.min())   # the table must reach this candidate's farthest cells
    trial = JaxDeliveryEnv(stage="randomize")
    elbow = trial.model.jnt_qposadr[trial.arm_joint_ids[3]]
    out = dict(picked=0, delivered=0, stalled=0, refused=0, elbow=np.inf)
    rng = np.random.default_rng(args.seed)
    for k in rng.choice(len(pairs), size=min(n, len(pairs)), replace=False):
        cube, goal = pairs[k]
        try:
            trial.reset(seed=0, options={"objects": [[cube[0], cube[1], 0.0]], "targets": [list(goal)]})
        except ValueError:
            out["refused"] += 1
            continue
        lifted, info = False, {}
        for step in range(trial.max_steps):
            _, _, terminated, truncated, info = trial.step(scripted_palm_place(trial, step))
            _, touching, supported = trial._contacts
            body = trial.object_body_ids[trial.moving]
            lifted |= bool(touching[trial.moving] and not supported[trial.moving]
                           and trial.data.xpos[body][2] > E.OBJECT_Z + E.LIFT_REQUIRED)
            out["elbow"] = min(out["elbow"], abs(float(trial.data.qpos[elbow])))
            if terminated or truncated:
                break
        out["picked"] += lifted
        out["delivered"] += bool(info.get("is_success"))
        out["stalled"] += bool(trial._palm_failures and trial._palm_failures[0][2] == "reach")
    trial.close()
    out["elbow"] = float(np.degrees(out["elbow"]))
    return out


results = []
for k, q in enumerate([current] + candidates):
    E.HOME_ARM_QPOS = q
    cells = np.argwhere(band())
    far = int(sum(np.hypot(xs[i] - spot[0], ys[j] - spot[1]) >= E.HAND_CLEAR + 0.03 for i, j in cells))
    label = "current HOME_ARM_QPOS" if k == 0 else f"candidate {k}"
    pairs = layouts(cells)
    line = (f"  {label:22s} usable {len(cells):3d}, of them {far:3d} >= {E.HAND_CLEAR + 0.03:.2f} m from home, "
            f"layouts {len(pairs):4d}; q {joints(q)}")
    trial = None
    if args.rollouts and pairs:
        trial = rollout(pairs, args.rollouts)
        line += (f"\n  {'':22s} rollouts {min(args.rollouts, len(pairs))}: picked {trial['picked']}, delivered "
                 f"{trial['delivered']}, stalled {trial['stalled']}, refused {trial['refused']}, elbow >= {trial['elbow']:.0f} deg")
    print(line, flush=True)
    if k:
        score = ((trial["delivered"], trial["picked"]) if trial else (0, 0)) if args.rollouts else ()
        results.append((score + ((len(pairs),) if args.rollouts else ()) + (len(cells), far), k, q))
if results:
    score, k, q = max(results, key=lambda r: r[0])
    if score[-2] == 0:
        # No candidate serves a single cell: naming one the winner told the user to copy a home that
        # serves nothing into SETUPS (the old --spot example of the docstring did exactly that).
        print("no winner: no candidate serves any cell of the table; keep the current HOME_ARM_QPOS")
    else:
        print(f"winner: candidate {k}, score {score}: home={joints(q)}")
