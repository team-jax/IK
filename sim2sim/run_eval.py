"""MuJoCo sim-to-sim check of Jungwoo's chain, as deploy/MUJOCO_SIM2SIM.md section 3 asks.

    venv/bin/python sim2sim/run_eval.py                 # all conditions, writes sim2sim/results/
    venv/bin/python sim2sim/run_eval.py --cond base     # one condition

Spots: the seven required cube positions paired with the seven required cube angles (3.5), then RANDOM_N spots drawn
(seed SEED) uniformly in the training box x -14..+2, y -28..+28 cm, kept only inside reach_mask_v65 (10), angle
U(-45, 45) deg. The same spot list runs under every condition:
  base     40 g cube, matched MuJoCo physics (build_scene.py)
  m30/m50  30 g / 50 g cube
  yawerr   base + a camera error U(-3, +3) deg on the cube angle input (43-44), drawn once per spot
  plain    base on MuJoCo's default contact / friction settings (the "before" of the physics change)
  diag_*   diagnostics: base without the 1 rad/s joint velocity limit; base with impratio 100 instead of 10
Each spot writes results/<cond>/<nn>_x.._y.._yaw../summary.json and trace.npz; results/<cond>/summary.json and
results/summary.json collect them.
"""

import argparse
import json
import math
import os
import sys

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chain as C  # noqa: E402

SEED, RANDOM_N = 0, 20
REQUIRED_SPOTS_CM = [(0, 0), (2, 28), (2, -28), (-8, 18), (-8, -18), (-14, -8), (-14, 6)]
REQUIRED_YAWS = [0.0, 20.0, -20.0, 35.0, -35.0, 45.0, -45.0]
CONDITIONS = {
    "base": {},
    "m30": {"mass": 0.030},
    "m50": {"mass": 0.050},
    "yawerr": {"yaw_err": True},
    "plain": {"physics": "plain"},
    # diagnostics, not adopted: each compatibility setting's own effect
    "diag_novlim": {"velocity_limit": None},
    "diag_imp100": {"impratio": 100.0},
}


def spots():
    tables = C.Tables()
    out = [(x / 100, y / 100, yaw) for (x, y), yaw in zip(REQUIRED_SPOTS_CM, REQUIRED_YAWS)]
    for x, y, _ in out:
        assert tables.valid(x, y), (x, y)
    rng = np.random.default_rng(SEED)
    while len(out) < len(REQUIRED_SPOTS_CM) + RANDOM_N:
        x, y = rng.uniform(-0.14, 0.02), rng.uniform(-0.28, 0.28)
        if tables.valid(x, y):
            out.append((round(float(x), 4), round(float(y), 4), round(float(rng.uniform(-45, 45)), 2)))
    return out


def run_condition(name, spec, spot_list, out_dir, model_cache):
    rng = np.random.default_rng(SEED + 1)
    errs = rng.uniform(-3.0, 3.0, len(spot_list))
    rows = []
    for k, (x, y, yaw) in enumerate(spot_list):
        kw = {"mass": spec.get("mass", 0.040), "physics": spec.get("physics", "matched"),
              "tables": model_cache["tables"], "policy": model_cache["policy"]}
        if "velocity_limit" in spec:
            kw["velocity_limit"] = spec["velocity_limit"]
        if "impratio" in spec:
            kw["impratio"] = spec["impratio"]
        if spec.get("yaw_err"):
            kw["yaw_obs_err_deg"] = float(errs[k])
        s, tr = C.run(x, y, yaw, **kw)
        s["condition"] = name
        s["yaw_obs_err_deg"] = kw.get("yaw_obs_err_deg", 0.0)
        tag = f"{k:02d}_x{x * 100:+.1f}_y{y * 100:+.1f}_yaw{yaw:+.1f}"
        d = os.path.join(out_dir, name, tag)
        os.makedirs(d, exist_ok=True)
        with open(os.path.join(d, "summary.json"), "w") as f:
            json.dump(s, f, indent=1)
        np.savez_compressed(os.path.join(d, "trace.npz"), phases=np.array(C.PHASES), joint_names=np.array(C.ISAAC), **tr)
        rows.append(s)
        ok = "DONE" if s["done"] else f"FAIL ({s['fail_reason']})"
        t_done = s["phase_t_s"]["DONE"]
        print(f"  {name:6s} {tag:32s} {ok:52s} close->done "
              f"{'-' if t_done is None else f'{t_done - s['phase_t_s']['GRASP_LIFT']:.2f} s'}  "
              f"shift {s['cube_shift_before_grasp_mm']:5.1f} mm", flush=True)
    return rows


def aggregate(rows):
    done = [r for r in rows if r["done"]]
    t = [r["phase_t_s"]["DONE"] - r["phase_t_s"]["GRASP_LIFT"] for r in done]
    q = lambda v, p: None if not v else round(float(np.percentile(v, p)), 3)
    tilt = [r["done_state"]["cube_tilt_deg"] for r in done]
    pads = np.array([r["done_state"]["pad_N"] for r in done]) if done else np.zeros((0, 6))
    fails = {}
    for r in rows:
        if not r["done"]:
            fails.setdefault(r["fail_reason"], []).append(r["spot_cm"] + [r["cube_yaw_deg"]])
    return {
        "runs": len(rows), "done": len(done),
        "handover_t_s": [min(r["phase_t_s"]["INSERT"] for r in rows), max(r["phase_t_s"]["INSERT"] for r in rows)],
        "handover_timeouts": sum(r["handover_by"] != "converged" for r in rows),
        "close_to_done_s": {"min": q(t, 0), "median": q(t, 50), "max": q(t, 100)},
        "cube_shift_before_grasp_mm": {"median": q([r["cube_shift_before_grasp_mm"] for r in rows], 50),
                                       "max": q([r["cube_shift_before_grasp_mm"] for r in rows], 100)},
        "raise_insert_cube_robot_force_max_n": round(max(max(r["raise_cube_robot_force_max_n"], r["insert_cube_robot_force_max_n"]) for r in rows), 3),
        "cube_tilt_at_done_deg": {"median": q(tilt, 50), "p90": q(tilt, 90), "max": q(tilt, 100)},
        "cube_turn_in_hand_after_lift_deg": {"median": q([r["cube_turn_in_hand_after_lift_deg"] for r in done], 50),
                                             "max": q([r["cube_turn_in_hand_after_lift_deg"] for r in done], 100)},
        "cube_rise_at_done_m": {"min": q([r["done_state"]["cube_rise_m"] for r in done], 0),
                                "median": q([r["done_state"]["cube_rise_m"] for r in done], 50)},
        "pad_N_at_done_median": [round(float(v), 2) for v in np.median(pads, axis=0)] if len(pads) else None,
        "still_up_2s_after_done": sum(bool(r.get("still_up_at_end")) for r in done),
        "max_joint_speed_rad_s": round(max(r["max_joint_speed_rad_s"] for r in rows), 3),
        "failures": fails,
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cond", nargs="*", default=list(CONDITIONS))
    ap.add_argument("--out", default=os.path.join(HERE, "results"))
    args = ap.parse_args()
    spot_list = spots()
    cache = {"tables": C.Tables(), "policy": C.Policy()}
    os.makedirs(args.out, exist_ok=True)
    with open(os.path.join(args.out, "spots.json"), "w") as f:
        json.dump([{"dx_m": x, "dy_m": y, "cube_yaw_deg": yaw} for x, y, yaw in spot_list], f, indent=1)
    allsum = {}
    path = os.path.join(args.out, "summary.json")
    if os.path.exists(path):
        allsum = json.load(open(path))
    for name in args.cond:
        print(f"[{name}] {len(spot_list)} spots", flush=True)
        rows = run_condition(name, CONDITIONS[name], spot_list, args.out, cache)
        agg = aggregate(rows)
        agg["physics"] = rows[0].get("physics") if rows else None
        with open(os.path.join(args.out, name, "summary.json"), "w") as f:
            json.dump({"aggregate": agg, "runs": rows}, f, indent=1)
        allsum[name] = agg
        print(f"[{name}] done {agg['done']}/{agg['runs']}, close->done {agg['close_to_done_s']}, "
              f"tilt {agg['cube_tilt_at_done_deg']}, failures {list(agg['failures'])}", flush=True)
    with open(path, "w") as f:
        json.dump(allsum, f, indent=1)


if __name__ == "__main__":
    main()
