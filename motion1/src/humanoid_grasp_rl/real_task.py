"""A delivery given in real coordinates, run in the sim: the joint trajectory the robot replays.

    import os; os.environ["JAX_SETUP"] = "real"      # before anything from this package is imported
    from humanoid_grasp_rl.real_task import plan_delivery
    result = plan_delivery((0.10, -0.30, 20.0), (0.05, -0.27))

cube = (x, y, yaw) and target = (x, y) in the real frame of realcoords.py (origin on the shoulder roll
motor's axis, x to the robot's left, y backwards, metres; yaw in degrees, counter-clockwise seen from
above, of a face normal from +x). d is the shoulder axis down to the table top (default: the setup's).

realcoords.plan() answers first from the reach map; the env then checks the exact poses
(env.reset(options=...)) and has the last word. The trajectory is every control step's commanded
joint angles (data.ctrl: 7 arm + 6 finger joints, rad, sim joint names).

The robot replays it open loop: nothing on it knows whether the cube is still in the hand. So a run
is playable only when it delivered with no retry: a retry in the sim (the cube dropped and fetched
again) is baked into the joint angles and would repeat, on the robot, a slip that may not happen there.

Used by scripts/run_real.py and by team-jax/SimToReal's pick_place.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

import humanoid_grasp_rl.env as E
from humanoid_grasp_rl.demonstration import scripted_palm_place
from humanoid_grasp_rl.single_env import JaxDeliveryEnv

sys.path.insert(0, str(Path(__file__).parent))
import realcoords as R  # noqa: E402

# status -> what it means; "playable" is the only one a robot may run
STATUSES = {
    "playable": "delivered in the sim with no retry",
    "retried": "delivered only after a retry",
    "not delivered": "not delivered in the sim",
    "unreachable": "the arm cannot serve it",
}


def plan_delivery(cube, target, d: float | None = None) -> dict:
    """Run the delivery in the sim. The result always has "status", "reason" and the plan; a run that
    happened also has the trajectory ("joint_names", "dt", "positions") and what came of it."""
    if E.SETUP != "real":
        raise RuntimeError("real coordinates belong to the real setup: set JAX_SETUP=real before "
                           "importing humanoid_grasp_rl")
    d = E.D if d is None else float(d)
    plan = R.plan(tuple(cube), tuple(target), d)
    out = {
        "real": {"cube": [float(v) for v in cube], "target": [float(v) for v in target], "d": d},
        "sim": {"cube": plan["cube"].tolist(), "target": plan["target"].tolist(), "cube_yaw_deg": plan["yaw"]},
        "reach_map": {"reachable": plan["reachable"], "reason": plan["reason"]},
    }
    if abs(d - E.D) > R.D_TOLERANCE:
        return {**out, "status": "unreachable",
                "reason": f"the sim's table is D = {E.D} m below the shoulder axis, not {d}"}
    env = JaxDeliveryEnv()
    try:
        try:
            env.reset(seed=0, options={"objects": [[*plan["cube"][:2], plan["yaw"]]], "targets": [plan["target"]]})
        except ValueError as e:
            return {**out, "status": "unreachable", "reason": str(e)}
        out["sim"]["hand_yaw_deg"] = float(np.degrees(env._hand_yaws[0]))
        # The grasp the env chose (env._grasp_turns): half-turned about the hand's axis, and how many
        # degrees the finger gap is off the cube's faces.
        out["sim"]["hand_half_turned"] = bool(env._hand_flips[0])
        out["sim"]["grasp_off_faces_deg"] = float(env._misalign[0])
        names = list(env._robot.arm_joints) + list(env._robot.finger_joints)
        actuators = np.concatenate((env.arm_actuator_ids, env.finger_actuator_ids))
        dt = float(env.model.opt.timestep * env.frame_skip)
        frames = [env.data.ctrl[actuators].tolist()]
        info = {}
        for step in range(env.max_steps):
            _, _, terminated, truncated, info = env.step(scripted_palm_place(env, step))
            frames.append(env.data.ctrl[actuators].tolist())
            if terminated or truncated:
                break
        delivered = bool(info.get("is_success")) and bool(info.get("success_now"))
        retries = [{"step": int(t), "phase": ph, "hand": what}
                   for t, _, ph, what in getattr(env, "_palm_failures", [])]
        error = float(np.linalg.norm(env.data.xpos[env.object_body_ids[0]][:2] - plan["target"]))
        status = "playable" if delivered and not retries else ("retried" if delivered else "not delivered")
        q = np.array(frames)
        speed = np.degrees(np.abs(np.diff(q, axis=0)).max(axis=0)) / dt
        return {**out, "status": status, "reason": STATUSES[status],
                "playable": status == "playable", "delivered_in_sim": delivered, "retries": retries,
                "final_error_m": error, "joint_names": names, "dt": dt, "positions": frames, "unit": "rad",
                "peak_speed_deg_s": {"joint": names[int(speed.argmax())], "value": float(speed.max())}}
    finally:
        env.close()
