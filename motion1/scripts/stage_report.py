"""Where deliveries fail, stage by stage, with what the arm and hand were doing.

classify_failures.py names the end state of an episode ("placed short", ...). This splits the
episode into the stages a delivery goes through and records the first one that went wrong:

    grasp      reach, down, grip   - the hand closes on the cube
    lift       up                  - the cube rises with the hand
    transport  cross               - the cube is carried over its spot
    place      lower, settle       - the cube is set down on the spot
    release    let go, back        - the fingers open and the hand withdraws

A stage fails when its phase runs out of time or the cube leaves the hand in it (the controller's
own failure record, env._palm_failures), or, for the last stage, when the episode ends without the
cube delivered. Per episode it also records, over the stages with the cube in hand:

    slip       how far the cube moved in the hand (hand frame, 3D) from the end of the grip to the
               moment it was lost or set down, mm; slip/s the same over the time it was carried, mm/s
    F1 F2 F3   each finger's mean normal force on the cube while it was carried, N (first + last link)
    F1 time    share of the carried time finger 1 touched the cube (> 0.05 N)
    prox       share of the fingers' normal force taken by the first links (the rest by the last links)
    tq p / d   largest finger torque at the first / last joints while gripping and carrying, N m;
               mean p / d the mean |torque| of the same joints over the carried time
               These are sim joint torques: the sim servos are not limited, and how an MG90S's output
               reaches a finger joint on the robot is not known (motion1/STATUS.md)
    arm err    largest |commanded - actual| arm joint angle, degrees
    rot err    largest hand orientation error against the commanded grasp orientation, degrees

    JAX_SETUP=real venv/bin/python motion1/scripts/stage_report.py --seeds 50
    JAX_SETUP=real venv/bin/python motion1/scripts/stage_report.py --seeds 50 --set demonstration.CROSS_RATE=0.0005

--set MODULE.NAME=VALUE replaces a module constant (modules: env, demonstration) before the run, for
comparison experiments on the same seeds. Only constants read at run time take effect; a constant
other constants were computed from at import (HAND_TILT -> PALM_DOWN) does not carry through. An env
constant that demonstration imports by name (REACH_RESCUE, CUBE_GRIP, ...) is set in both. It edits
nothing on disk.
"""
import _paths  # noqa: F401  (puts src/ on the path)
import argparse
from collections import Counter

import mujoco
import numpy as np
from scipy.spatial.transform import Rotation

import humanoid_grasp_rl.demonstration as demonstration
import humanoid_grasp_rl.env as env_module
from humanoid_grasp_rl.demonstration import scripted_palm_place
from humanoid_grasp_rl.single_env import JaxDeliveryEnv

STAGES = {"reach": "grasp", "down": "grasp", "grip": "grasp", "up": "lift", "cross": "transport",
          "lower": "place", "settle": "place", "let go": "release", "back": "release", "park": "park"}
ORDER = ("grasp", "lift", "transport", "place", "release")
CARRIED = ("up", "cross", "lower")


def apply_overrides(pairs):
    modules = {"env": env_module, "demonstration": demonstration}
    for pair in pairs:
        name, value = pair.split("=", 1)
        module, constant = name.split(".", 1)
        if module not in modules or not hasattr(modules[module], constant):
            raise SystemExit(f"--set {pair}: no constant {constant} in {module}")
        old = getattr(modules[module], constant)
        new = float(value) if old is None else type(old)(float(value))   # None: an unset rate (SLOW_LOWER_RATE)
        setattr(modules[module], constant, new)
        # demonstration imports some env constants by name (REACH_RESCUE, CUBE_GRIP, ...) and reads its
        # own copy: set only in env, those changed nothing the controller saw.
        if module == "env" and getattr(demonstration, constant, None) is old and old is not None:
            setattr(demonstration, constant, new)


def finger_forces(env, cube_body) -> np.ndarray:
    """Normal force of each finger on the cube, N: [finger 1, 2, 3], and of the first links alone."""
    m, d = env.model, env.data
    per, first, f6 = np.zeros(3), 0.0, np.zeros(6)
    for i in range(d.ncon):
        c = d.contact[i]
        b1, b2 = m.geom_bodyid[c.geom1], m.geom_bodyid[c.geom2]
        if cube_body not in (b1, b2):
            continue
        name = m.body(b2 if b1 == cube_body else b1).name
        if name.startswith("Finger_"):
            mujoco.mj_contactForce(m, d, i, f6)
            per[int(name[7]) - 1] += f6[0]
            first += f6[0] if name.endswith("_1") else 0.0
    return per, first


def run_episode(env, seed):
    fallbacks_before = env.layout_fallbacks
    env.reset(seed=seed)
    m, d = env.model, env.data
    moving = env.moving
    cube_body = env.object_body_ids[moving]
    joints = [m.actuator(i).name for i in range(m.nu)]
    proximal = [i for i, n in enumerate(joints) if n.startswith("finger") and n.endswith("joint1_act")]
    distal = [i for i, n in enumerate(joints) if n.startswith("finger") and n.endswith("joint2_act")]
    dt = env.model.opt.timestep * env.frame_skip
    rec = dict(arm_err=0.0, rot_err=0.0, tq_p=0.0, tq_d=0.0, slip=np.nan)
    forces, first_total, all_total, carried = np.zeros(3), 0.0, 0.0, 0
    f1_on = 0
    tq_sum = np.zeros(2)
    gripped_at, failures_at_grip = None, 0
    last_phase, info = None, {}
    for step in range(env.max_steps):
        _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
        phase = env._palm_phase
        site_rot = d.site_xmat[env.site_id].reshape(3, 3)
        in_hand = site_rot.T @ (d.xpos[cube_body] - d.site_xpos[env.site_id])   # cube in the hand frame
        if phase in ("grip",) + CARRIED:
            rec["tq_p"] = max(rec["tq_p"], float(np.abs(d.actuator_force[proximal]).max()))
            rec["tq_d"] = max(rec["tq_d"], float(np.abs(d.actuator_force[distal]).max()))
        if phase in CARRIED:
            arm = np.abs(d.ctrl[env.arm_actuator_ids] - d.qpos[env.qpos_ids[:7]])
            rec["arm_err"] = max(rec["arm_err"], float(np.degrees(arm.max())))
            wanted = env._hand_rotation(env._pose_target_position)
            rec["rot_err"] = max(rec["rot_err"], float(np.degrees(np.linalg.norm(
                Rotation.from_matrix(wanted.T @ site_rot).as_rotvec()))))
            _, touching, supported = env._contacts
            if touching[moving] and not supported[moving] and gripped_at is not None and np.isnan(rec["slip"]):
                per, first = finger_forces(env, cube_body)
                forces += per
                first_total += first
                all_total += per.sum()
                carried += 1
                f1_on += per[0] > 0.05
                tq_sum += (np.abs(d.actuator_force[proximal]).mean(), np.abs(d.actuator_force[distal]).mean())
        if last_phase == "grip" and phase == "up":
            gripped_at, failures_at_grip = in_hand.copy(), len(env._palm_failures)
        # Ended by a failure after this grip, not one before it: a reach that timed out first had
        # latched the slip at 0 on the grip and left the carry's forces unmeasured.
        if gripped_at is not None and np.isnan(rec["slip"]) and (
                len(env._palm_failures) > failures_at_grip or phase in ("settle", "let go", "back")):
            rec["slip"] = float(np.linalg.norm(in_hand - gripped_at)) * 1000.0
        last_phase = phase
        if terminated or truncated:
            break
    delivered = bool(info.get("is_success"))
    if env._palm_failures:
        _, _, phase, held = env._palm_failures[0]
        stage, how = STAGES.get(phase, phase), f"{phase} ({held})"
    elif not delivered:
        stage, how = STAGES.get(env._palm_phase, env._palm_phase), f"ended in {env._palm_phase}"
    else:
        stage, how = None, ""
    rec.update(seed=seed, delivered=delivered, stage=stage, how=how,
               retries=len(env._palm_failures), fallback=env.layout_fallbacks > fallbacks_before,
               carried_s=carried * dt, F=forces / max(carried, 1), f1_time=f1_on / max(carried, 1),
               tq_mean=tq_sum / carried if carried else np.full(2, np.nan),
               prox=first_total / all_total if all_total > 0 else np.nan,
               slip_rate=rec["slip"] / (carried * dt) if carried else np.nan)
    return rec


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seeds", type=int, default=20)
    parser.add_argument("--first-seed", type=int, default=0)
    parser.add_argument("--yaw", type=float, default=0.0, help="cube yaw range, degrees (env.cube_yaw_max)")
    parser.add_argument("--set", action="append", default=[], metavar="MODULE.NAME=VALUE")
    args = parser.parse_args()
    apply_overrides(args.set)

    env = JaxDeliveryEnv(stage="randomize", randomization_scale=1.0)
    env.cube_yaw_max = args.yaw
    print(f"setup {env_module.SETUP}, seeds {args.first_seed}-{args.first_seed + args.seeds - 1}"
          + (f", yaw +-{args.yaw:g}" if args.yaw else "") + (f", set {' '.join(args.set)}" if args.set else ""))
    print(f"{'seed':>4s} {'result':>10s} {'first failure':>22s} {'slip mm':>7s} {'slip/s':>6s} {'F1':>5s} "
          f"{'F2':>5s} {'F3':>5s} {'F1 time':>7s} {'prox':>5s} {'tq p':>5s} {'tq d':>5s} {'arm err':>7s} {'rot err':>7s}")
    rows = []
    for seed in range(args.first_seed, args.first_seed + args.seeds):
        r = run_episode(env, seed)
        rows.append(r)
        result = "delivered" + ("*" if r["retries"] else "") if r["delivered"] else "failed"
        print(f"{seed:4d} {result:>10s} {(r['stage'] or '') + ' ' + r['how']:>22s} {r['slip']:7.1f} {r['slip_rate']:6.1f} "
              f"{r['F'][0]:5.2f} {r['F'][1]:5.2f} {r['F'][2]:5.2f} {r['f1_time']:7.2f} {r['prox']:5.2f} "
              f"{r['tq_p']:5.3f} {r['tq_d']:5.3f} {r['arm_err']:7.2f} {r['rot_err']:7.2f}"
              + ("  [fixed-layout fallback]" if r["fallback"] else ""))
    env.close()
    first = Counter(r["stage"] for r in rows if r["stage"])
    clean = sum(r["delivered"] and not r["retries"] for r in rows)
    print(f"\ndelivered {sum(r['delivered'] for r in rows)}/{len(rows)} "
          f"({clean} without a retry); first failure by stage: "
          + ", ".join(f"{s} {first[s]}" for s in ORDER + ("park",) if first[s])
          + f"; fixed-layout fallbacks {sum(r['fallback'] for r in rows)}")
    med = lambda key: np.nanmedian([r[key] for r in rows]) if rows else np.nan
    F = np.array([r["F"] for r in rows if r["carried_s"] > 0])
    print(f"medians: slip {med('slip'):.1f} mm at {med('slip_rate'):.1f} mm/s over {med('carried_s'):.2f} s carried; "
          f"F1/F2/F3 {np.median(F[:, 0]) if len(F) else np.nan:.2f}/{np.median(F[:, 1]) if len(F) else np.nan:.2f}/"
          f"{np.median(F[:, 2]) if len(F) else np.nan:.2f} N; finger 1 in contact {med('f1_time'):.2f} of the carry; "
          f"first links take {med('prox'):.2f} of the force; mean finger torque in the carry, first/last joints "
          f"{np.nanmedian([r['tq_mean'][0] for r in rows]):.3f}/{np.nanmedian([r['tq_mean'][1] for r in rows]):.3f} N m")
    print(f"finger torque, sim joints (largest of all episodes): first joints {max(r['tq_p'] for r in rows):.3f} N m, "
          f"last joints {max(r['tq_d'] for r in rows):.3f} N m")


if __name__ == "__main__":
    main()
