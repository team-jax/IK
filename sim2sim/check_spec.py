"""Check the MuJoCo port against numbers printed in Jungwoo's deploy folder (no physics run).

    venv/bin/python sim2sim/check_spec.py        # prints "spec ok" or fails on the first mismatch

1. layout: shoulder pitch axis on world z 1.2125 along world y through Shoulder_Pitch (0, 0.16, 1.2125), table top
   0.735 (MUJOCO_SIM2SIM.md 4), robot base quat (0.7071, 0, 0, -0.7071)
2. forward kinematics: the 8 recorded Isaac hand-over states (handover_state_23025_v3_example.json) give the same
   hand position / orientation and fingertip midpoint here (Isaac z is the sim layout's, +0.0575 to this frame)
3. lookup tables: README 7.1's knot table at the reference cube, 7.2's goal pose, 7.3's four reach-in deltas, 8.1's
   theta examples, 10's reach ranges
4. policy: the numpy actor equals the ONNX file's weights byte for byte and the meta normalizer
"""

import json
import os
import sys

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chain as C  # noqa: E402
import ptload  # noqa: E402


def close(a, b, tol, what):
    a, b = np.asarray(a, float), np.asarray(b, float)
    err = float(np.abs(a - b).max())
    assert err <= tol, f"{what}: max error {err:.4g} > {tol} ({a.round(3)} vs {b.round(3)})"
    return err


def main():
    m = mujoco.MjModel.from_xml_path(C.SCENE)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)

    # 1. layout
    j = jid(C.ISAAC[0])
    close(d.xaxis[j], (0, 1, 0), 1e-6, "shoulder pitch axis direction")
    close(d.xanchor[j][[0, 2]], (0.0, 1.2125), 1e-6, "shoulder pitch axis x, z")
    close(d.xpos[bid("Shoulder_Pitch")], (0.0, 0.16, 1.2125), 1e-6, "Shoulder_Pitch frame")
    close(d.xquat[bid("base_link")], (0.5 ** 0.5, 0, 0, -0.5 ** 0.5), 1e-9, "base quat")
    t = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
    close(d.geom_xpos[t][2] + m.geom_size[t][2], 0.735, 1e-9, "table top")
    close(d.geom_xpos[t][0] + m.geom_size[t][0], -0.225, 1e-9, "table front edge")
    print("layout: shoulder pitch axis z 1.2125 along world y, table top 0.735, front edge x -0.225")

    # 2. forward kinematics vs the Isaac hand-over states
    ex = json.load(open(os.path.join(C.DEPLOY, "handover_state_23025_v3_example.json")))
    worst = 0.0
    for e in ex["envs"]:
        for n, v in zip(ex["joint_names_arm"] + ex["joint_names_fingers"], e["arm_q_deg"] + e["finger_q_deg"]):
            d.qpos[m.jnt_qposadr[jid(n)]] = np.radians(v)
        mujoco.mj_kinematics(m, d)
        off = np.array([0.0, 0.01, 0.0]) - np.array(e["robot_base_pos_m"])
        tips = np.mean([d.xpos[bid(n)] for n in C.TIP_LINKS], axis=0)
        worst = max(worst, close(d.xpos[bid(C.HAND)], np.array(e["hand_pos_m"]) + off, 5e-4, "hand position"),
                    close(tips, np.array(e["fingertip_mid_pos_m"]) + off, 5e-4, "fingertip midpoint"))
        q = d.xquat[bid(C.HAND)]
        q = q if np.dot(q, e["hand_quat_wxyz"]) >= 0 else -q
        close(q, e["hand_quat_wxyz"], 5e-4, "hand orientation")
    print(f"forward kinematics: {len(ex['envs'])} Isaac hand-over states within {worst * 1000:.2f} mm")

    # 3. lookup tables
    T = C.Tables()
    e_ = C.E_TABLE
    close(e_, -0.005, 1e-12, "table blend e")
    S = T.raise_knots(0, 0, e_)
    knots = np.vstack([np.zeros(8), S[0], S])
    readme_71 = {5.0: (-34.09, 0, 1.37, 62.41, -5.14, 57.47, -9.18, -170.1),
                 6.2: (-29.27, 0, 1.40, 62.26, -5.01, 52.70, -9.29, -170.1),
                 8.0: (-21.14, 0, 1.34, 59.71, -5.20, 47.25, -9.22, -170.1)}
    for tk, row in readme_71.items():
        close(np.degrees(C.catmull_rom(C.P_T, C.P_Q, tk) + C.catmull_rom(C.S_T, knots, tk)), row, 0.006, f"7.1 knot {tk}")
    close(np.degrees(C.GOAL_BASE + S[5]), (-23.04, 0, 1.34, 57.61, -5.20, 46.55, -9.02, -170.1), 0.006, "7.2 goal")
    readme_73 = {(0, 0): (7.28, 0, 0.18, -1.92, -0.04, -5.34, 12.53, 0),
                 (0, -0.10): (7.72, 0, -2.10, -2.79, -0.29, -4.92, 11.46, 0),
                 (0, 0.20): (10.10, 0, 5.97, -5.68, -0.67, -3.06, 16.82, 0),
                 (0.02, -0.28): (16.89, 0, -20.90, -14.19, -6.12, 4.49, 3.08, 0)}
    for (x, y), row in readme_73.items():
        dq = T.final_pose(x, y, e_) - T.raise_knots(x, y, e_)[5]
        dq[6] += C.ROLL_IN
        close(np.degrees(dq), row, 0.006, f"7.3 delta at {(x, y)}")
    for (x, y), th in {(0, 0): 0.0, (0, -0.10): 15.9, (0, 0.20): -33.8, (0.02, -0.28): 57.6, (0.02, 0.28): -54.6}.items():
        close(T.theta_deg(x, y), th, 0.06, f"8.1 theta at {(x, y)}")
    ranges = {0.02: (-0.28, 0.28), 0.0: (-0.26, 0.28), -0.04: (-0.24, 0.26), -0.08: (-0.18, 0.22),
              -0.12: (-0.16, 0.16), -0.14: (-0.10, 0.08)}
    for x, (lo, hi) in ranges.items():
        ys = [y for y in np.round(np.arange(-0.30, 0.3001, 0.02), 2) if T.valid(x, y)]
        close((min(ys), max(ys)), (lo, hi), 1e-9, f"10 reach range at x {x}")
    print("lookup tables: 7.1 knots, 7.2 goal, 7.3 deltas, 8.1 theta, 10 reach ranges as the README")

    # 4. policy
    a = ptload.load_actor(os.path.join(C.DEPLOY, "grasp_lift_23025.pt"))
    meta = json.load(open(os.path.join(C.DEPLOY, "grasp_lift_23025_meta.json")))
    close(a["mean"], meta["obs_normalizer"]["mean"], 0, "normalizer mean")
    close(a["std"], meta["obs_normalizer"]["std"], 0, "normalizer std")
    onnx = open(os.path.join(C.DEPLOY, "grasp_lift_23025.onnx"), "rb").read()
    for k, (w, b) in enumerate(a["layers"]):
        assert w.astype(np.float32).tobytes() in onnx and b.astype(np.float32).tobytes() in onnx, f"layer {k} not in ONNX"
    assert [w.shape for w, _ in a["layers"]] == [(256, 45), (256, 256), (11, 256)]
    print("policy: 45 -> 256 -> 256 -> 11, weights byte-identical to the ONNX file, normalizer = meta")
    print("spec ok")


if __name__ == "__main__":
    main()
