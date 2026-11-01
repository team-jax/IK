"""Split the misses into the ones that never picked the cube up and the ones that lost it.

Two failures that both read as a large final error need different fixes, and lumping them
made a release problem look like a reach problem and back again. So this records, per seed
and per cube, whether the cube was ever carried, how close it came to its spot while it was
in hand, where it was when the hand actually came off it, how far it moved after that (at
the end, and at most), and whether the task was done at the end - not only whether it was
done at some point.

    venv/bin/python motion1/scripts/classify_failures.py --scales 1.0 --seeds 100     # delivery, seeds 0-99
    venv/bin/python motion1/scripts/classify_failures.py --stack --scales 1.0 --seeds 50   # two-cube tower, seeds 0-49
    venv/bin/python motion1/scripts/classify_failures.py --layouts motion1/artifacts/layouts_10.json   # replay a file from layouts.py

The default is 12 seeds per scale; the 100 and 50 above are the runs behind the numbers in README.md.
A tower is two cubes at most (see stack_env.py), so --cubes stays at its default of 2.
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import sys
from pathlib import Path

import numpy as np

parser = argparse.ArgumentParser()
parser.add_argument("--scales", type=float, nargs="+", default=[1.0])
parser.add_argument("--seeds", type=int, default=12)
parser.add_argument("--first-seed", type=int, default=0, help="so a long run can be sharded")
parser.add_argument("--stack", action="store_true", help="cubes to go on top of one another")
parser.add_argument("--cubes", type=int, default=2, help="how tall the tower is, with --stack")
parser.add_argument("--layouts", default=None,
                    help="a file from scripts/layouts.py; the scales are then ignored")
args = parser.parse_args()

import humanoid_grasp_rl.env as E  # noqa: E402
from humanoid_grasp_rl.single_env import JaxDeliveryEnv  # noqa: E402
from humanoid_grasp_rl.stack_env import JaxStackEnv  # noqa: E402
from humanoid_grasp_rl.demonstration import scripted_palm_place  # noqa: E402

sys.path.insert(0, str(Path(__file__).parent))
from layouts import replay  # noqa: E402

if args.layouts:
    env, seeds = replay(args.layouts)
    scales = [env.randomization_scale]
    seeds = [s for s in seeds if args.first_seed <= s < args.first_seed + args.seeds]
else:
    kwargs = dict(stage="randomize", randomization_scale=args.scales[0])
    env = JaxStackEnv(cubes=args.cubes, **kwargs) if args.stack else JaxDeliveryEnv(**kwargs)
    scales = args.scales
    seeds = list(range(args.first_seed, args.first_seed + args.seeds))


def off_a_face(rotation):
    return float(np.degrees(np.arccos(np.clip(np.abs(rotation[2, :]).max(), 0.0, 1.0))))


for scale in scales:
    env.randomization_scale = scale
    print(f"\nrandomization {scale}"
          + (f", replaying {args.layouts}" if args.layouts else ""))
    print(f"{'seed':>4s} {'picked':>6s} {'best carry':>10s} {'at release':>10s} "
          f"{'off face':>8s} {'final':>8s} {'moved after':>11s}  verdict")
    tally = {}
    for seed in seeds:
        env.reset(seed=seed)
        goal = env._target_positions[env.moving].copy()
        picked, best, angle = {}, {}, {}
        release_at, moved_max = {}, {}
        info = {}
        for step in range(env.max_steps):
            _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
            m = env.moving
            cube = env.data.xpos[env.object_body_ids[m]]
            _, touching, supported = env._contacts
            in_hand = bool(touching[m] and not supported[m])
            phase = getattr(env, "_palm_phase", None)
            if in_hand and cube[2] > E.OBJECT_Z + E.LIFT_REQUIRED:   # the env's own lifted test
                picked[m] = True
            if phase in ("cross", "lower") and in_hand:
                best[m] = min(best.get(m, np.inf), float(np.linalg.norm(cube[:2] - goal)))
            # Where the cube was when the hand really came off it. A retry that lifts the
            # cube again forgets the earlier release; the record is the last one.
            if phase in ("let go", "back") and not touching[m] and m not in release_at:
                release_at[m] = cube[:2].copy()
                angle[m] = off_a_face(env.data.xmat[env.object_body_ids[m]].reshape(3, 3))
                moved_max[m] = 0.0
            elif phase in ("reach", "down", "grip", "up", "cross", "lower") and m in release_at:
                del release_at[m], moved_max[m]
            for i, at in release_at.items():
                moved_max[i] = max(moved_max[i], float(np.linalg.norm(
                    env.data.xpos[env.object_body_ids[i]][:2] - at)))
            if terminated or truncated:
                break
        m = env.moving
        cube = env.data.xpos[env.object_body_ids[m]]
        final = float(np.linalg.norm(cube[:2] - goal))
        released = float(np.linalg.norm(release_at[m] - goal)) if m in release_at else None
        moved_end = {i: float(np.linalg.norm(env.data.xpos[env.object_body_ids[i]][:2] - at))
                     for i, at in release_at.items()}
        ever, now = bool(info.get("is_success")), bool(info.get("success_now"))
        ok = ever and now
        failures = getattr(env, "_palm_failures", [])
        verdict = ("delivered" if ok else
                   "never picked up" if not picked.get(m) else
                   "lost on release" if released is not None and moved_max.get(m, 0.0) > 0.02 else
                   "placed short")
        tally[verdict] = tally.get(verdict, 0) + 1
        note = ""
        if ever and not now:
            note += "  done once, not at the end"
        if failures:
            note += "  retries: " + ",".join(f"{ph}@{t}" for t, _, ph, _ in failures)
        others = max((v for i, v in moved_max.items() if i != m), default=0.0)
        if others > 0.005:
            # A cube below rocks while the one above lies down on it; the peak and where
            # it ends are different facts.
            ends = max((v for i, v in moved_end.items() if i != m), default=0.0)
            note += f"  earlier cube moved {others * 1000:.0f}mm at most, {ends * 1000:.0f}mm at the end"
        if info.get("layout_fell_back"):
            note += "  [fixed-layout fallback]"
        if env.n_objects > 2:
            lower = [float(np.linalg.norm(env.data.xpos[env.object_body_ids[i]][:2] - goal)) * 1000
                     for i in range(1, env.n_objects - 1)]
            note += "  lower cubes " + "/".join(f"{v:.1f}" for v in lower) + "mm"
        print(f"{seed:4d} {str(bool(picked.get(m))):>6s} {best.get(m, np.inf) * 1000:9.1f}mm "
              f"{(released if released is not None else float('nan')) * 1000:9.1f}mm "
              f"{angle.get(m, float('nan')):7.1f}deg {final * 1000:7.1f}mm "
              f"{moved_end.get(m, 0.0) * 1000:6.1f}/{moved_max.get(m, 0.0) * 1000:.1f}mm  {verdict}{note}")
    print("  " + ",  ".join(f"{v} {k}" for k, v in sorted(tally.items(), key=lambda x: -x[1])),
          flush=True)
env.close()
