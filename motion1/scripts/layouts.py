"""Freeze the cube and goal spots to a file, so a change to the map cannot move them.

Every comparison in this project was being drawn from the reach map at reset, which means a
change that grows the map also changes what "seed 4" is. Captured once and replayed, a run
answers the question it was asked.

    venv/bin/python motion1/scripts/layouts.py --scale 1.0 --out motion1/artifacts/layouts_10.json
    from layouts import replay; env, seeds = replay("motion1/artifacts/layouts_10.json")
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
import json
from pathlib import Path

import numpy as np

from humanoid_grasp_rl.single_env import JaxDeliveryEnv
from humanoid_grasp_rl.stack_env import JaxStackEnv

ENVS = {"JaxDeliveryEnv": JaxDeliveryEnv, "JaxStackEnv": JaxStackEnv}


def replay(path, **kwargs):
    """An env that serves the saved layouts, one per seed, whatever the map now says."""
    saved = json.loads(Path(path).read_text())
    layouts = {int(k): v for k, v in saved["layouts"].items()}

    class Replayed(ENVS[saved.get("env_class", "JaxDeliveryEnv")]):
        def _sample_layout(self):
            entry = layouts[int(self._replay_seed)]
            return np.array(entry["objects"]), np.array(entry["targets"])

        def reset(self, *, seed=None, options=None):
            self._replay_seed = 0 if seed is None else seed
            return super().reset(seed=seed, options=options)

    options = dict(saved["env"])
    options.update(kwargs)
    return Replayed(**options), sorted(layouts)


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument("--scale", type=float, default=1.0)
    parser.add_argument("--seeds", type=int, default=12)
    parser.add_argument("--first-seed", type=int, default=0)
    parser.add_argument("--out", type=Path, required=True)
    parser.add_argument("--stack", action="store_true", help="cubes to go on top of one another")
    parser.add_argument("--no-validate", action="store_true",
                        help="skip the per-layout IK check, to measure what it is worth")
    parser.add_argument("--cubes", type=int, default=2, help="how tall the tower is, with --stack")
    args = parser.parse_args()

    kwargs = dict(stage="randomize", randomization_scale=args.scale)
    env = JaxStackEnv(cubes=args.cubes, **kwargs) if args.stack else JaxDeliveryEnv(**kwargs)
    env.validate_layouts = not args.no_validate
    layouts = {}
    for seed in range(args.first_seed, args.first_seed + args.seeds):
        env.reset(seed=seed)
        layouts[seed] = {
            "objects": env._initial_object_positions[:, :2].round(6).tolist(),
            "targets": env._target_positions.round(6).tolist(),
        }
    args.out.parent.mkdir(parents=True, exist_ok=True)
    args.out.write_text(json.dumps({
        "env_class": type(env).__name__,
        "env": {"stage": "randomize", "randomization_scale": args.scale,
                **({"cubes": args.cubes} if args.stack else {})},
        "layouts": layouts,
    }, indent=1))
    env.close()
    print(f"{len(layouts)} layouts at randomization {args.scale} written to {args.out}"
          + (f"  ({env.layout_fallbacks} fell back to the fixed layout)" if getattr(env, "layout_fallbacks", 0) else ""))
    for seed, entry in layouts.items():
        print(f"  seed {seed:2d}  cubes {np.round(entry['objects'], 3).tolist()}"
              f"  goal {np.round(entry['targets'][env.moving], 3)}")
