"""Solve the poses the palm controller actually commands, and save the feasible band.

A cell of the table is feasible when the arm can hold the grasp centre, fingers straight down,
at every pose a delivery there asks for: the grasp on a cube standing in it and the clear-air
pose above, and the release at table height and the carry pose above that. The arm loses
forward reach as it comes down, so the band is not a circle around the shoulder and cannot be
written down; it is solved, at 5 mm and 3 degrees, from the home pose, with the same IK the
controller uses.

    venv/bin/python motion1/scripts/build_reachmap.py      # rewrites src/humanoid_grasp_rl/reachmap.npz

Rebuild it whenever a constant in env.py that the signature covers changes.
"""
import _paths  # noqa: F401  (puts src/ on the path)
import numpy as np

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl import JaxDeliveryEnv
from humanoid_grasp_rl.demonstration import TRAVEL_CLEAR

env = JaxDeliveryEnv(stage="place")
env.reset(seed=0)

xs = np.round(np.arange(-0.08, 0.281, 0.02), 3)
ys = np.round(np.arange(-0.30, 0.301, 0.025), 3)
grasp = np.zeros((len(xs), len(ys)), dtype=bool)
place = np.zeros((len(xs), len(ys)), dtype=bool)


def reachable(position):
    """Inside the workspace first: the env clips every commanded pose to it, so a pose the IK
    could solve outside it is one the controller can never command."""
    if np.any(position < E.WORKSPACE_LOW) or np.any(position > E.WORKSPACE_HIGH):
        return False
    return env.reachable(position)


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
np.savez_compressed(_paths.ROOT / "src/humanoid_grasp_rl/reachmap.npz",
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
