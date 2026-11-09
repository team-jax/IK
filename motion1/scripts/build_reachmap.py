"""Solve the poses the palm controller actually commands, and save the feasible band.

A cell of the table is feasible when the arm can hold the grasp centre, fingers straight down,
at every pose a delivery there asks for: the grasp on a cube standing in it and the clear-air
pose above, and the release at table height and the carry pose above that. The arm loses
forward reach as it comes down, so the band is not a circle around the shoulder and cannot be
written down; it is solved, at 5 mm and 3 degrees, from the home pose, with the same IK the
controller uses.

    venv/bin/python motion1/scripts/build_reachmap.py      # rewrites src/humanoid_grasp_rl/reachmap.npz
    JAX_SETUP=real venv/bin/python motion1/scripts/build_reachmap.py   # the real setup's reachmap_real.npz
    JAX_SETUP=real venv/bin/python motion1/scripts/build_reachmap.py --yaws -45 -30 -15 0 15 30
        # maps per cube yaw and per hand turn (the cube's yaw, or a quarter more either way:
        # env.hand_turns), in reachmap_real_yaw.npz, which realcoords reads. A cube repeats every
        # 90 degrees, so -45..30 in 15 covers it; the yaw-0 map files stay as they are.

Rebuild it whenever a constant in env.py that the signature covers changes.
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse

import numpy as np

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
from humanoid_grasp_rl.demonstration import TRAVEL_CLEAR

parser = argparse.ArgumentParser()
parser.add_argument("--yaws", type=float, nargs="+", default=None,
                    help="cube yaws in degrees: build the per-yaw map instead of the yaw-0 one")
args = parser.parse_args()

env = JaxDeliveryEnv(stage="place")
env.reset(seed=0)

xs = np.round(np.arange(-0.08, 0.281, 0.02), 3)
ys = np.round(np.arange(-0.30, 0.301, 0.025), 3)


def reachable(position):
    """Inside the workspace first: the env clips every commanded pose to it, so a pose the IK
    could solve outside it is one the controller can never command."""
    if np.any(position < E.WORKSPACE_LOW) or np.any(position > E.WORKSPACE_HIGH):
        return False
    return env.reachable(position)


def solve(hand_yaw_deg):
    """grasp, place and feasible over the grid, for the hand turned hand_yaw_deg."""
    env._hand_yaws[env.moving] = np.radians(hand_yaw_deg)
    grasp = np.zeros((len(xs), len(ys)), dtype=bool)
    place = np.zeros((len(xs), len(ys)), dtype=bool)
    clear_z = env.clear_height(E.OBJECT_Z)
    for i, x in enumerate(xs):
        for j, y in enumerate(ys):
            spot = np.array([x, y])
            pick = env.pick_pose(spot)
            drop = env.drop_pose(spot, E.OBJECT_Z)
            over_pick = np.array([pick[0], pick[1], max(pick[2] + TRAVEL_CLEAR, clear_z)])
            over_drop = np.array([drop[0], drop[1], max(drop[2] + TRAVEL_CLEAR, clear_z)])
            grasp[i, j] = reachable(pick) and reachable(over_pick)
            # Setting a cube down is the harder half: the hand is lowest here with a cube under it.
            place[i, j] = reachable(drop) and reachable(over_drop)
    both = grasp & place
    # Drop the rim of the band: a cell whose neighbour the arm cannot reach is one where the arm is
    # at a joint stop, and the hand there can no longer hold the fingers straight down while it
    # moves, so a carried cube slips out. Keeping only cells with all four neighbours feasible
    # leaves the controller a pose's worth of margin in every direction.
    padded = np.pad(both, 1, constant_values=False)
    feasible = (both & padded[:-2, 1:-1] & padded[2:, 1:-1] & padded[1:-1, :-2] & padded[1:-1, 2:])
    # And a cube there must stand on the table.
    feasible &= (xs <= E.CUBE_X_MAX)[:, None]
    return grasp, place, feasible


if args.yaws:
    yaws = np.array(sorted({E.fold_yaw(y) for y in args.yaws}))
    offsets = np.array([0.0, 90.0, -90.0])   # hand yaw = cube yaw + offset (env.hand_turns)
    maps = [[solve(y + o) for o in offsets] for y in yaws]
    out = E.REACH_FILE.with_name(E.REACH_FILE.stem + "_yaw.npz")
    np.savez_compressed(out, yaws=yaws, offsets=offsets, xs=xs, ys=ys,
                        grasp=np.array([[m[0] for m in row] for row in maps]),
                        place=np.array([[m[1] for m in row] for row in maps]),
                        feasible=np.array([[m[2] for m in row] for row in maps]),
                        signature=E.map_signature())
    print(f"{out.name}: cells both grasp and place, per cube yaw and hand turn (offset from the cube yaw)")
    for y, row in zip(yaws, maps):
        counts = "  ".join(f"{o:+4.0f}: {m[2].sum():3d}" for o, m in zip(offsets, row))
        usable = E.hand_turns(y)
        union = np.any([m[2] for o, m in zip(offsets, row) if y + o in usable], axis=0)
        print(f"  yaw {y:+5.1f}  {counts}   any usable turn: {union.sum():3d}")
    env.close()
    raise SystemExit(0)

grasp, place, feasible = solve(0.0)
np.savez_compressed(E.REACH_FILE,   # the map of the setup JAX_SETUP picks (env.py)
                    xs=xs, ys=ys, grasp=grasp, place=place, feasible=feasible,
                    signature=E.map_signature())
print(f"격자 {grasp.size},  파지가능 {grasp.sum()},  놓기가능 {place.sum()},  "
      f"둘다 {feasible.sum()} ({100 * feasible.mean():.0f}%)")
idx = np.argwhere(feasible)
if len(idx):
    print(f"둘다 가능 범위: x {xs[idx[:, 0]].min():.2f}~{xs[idx[:, 0]].max():.2f}  "
          f"y {ys[idx[:, 1]].min():+.2f}~{ys[idx[:, 1]].max():+.2f}")
print("\n" + "  " + " ".join(f"{y:+.2f}"[:5] for y in ys[::2]))
for i, x in enumerate(xs):
    row = " ".join(("#" if feasible[i, j] else ("g" if grasp[i, j] else
                    ("p" if place[i, j] else "."))).center(5) for j in range(0, len(ys), 2))
    print(f"{x:+.2f} {row}")
print("\n# both, g grasp only, p release only, . neither")
env.close()
