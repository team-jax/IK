"""IK demo for the 7-DOF arm (arm/arm.urdf + arm/arm.xml).

Solves inverse kinematics with ikpy for a target end-effector position, then applies the
resulting joint angles to the matching MuJoCo model (arm/arm.xml) to validate that the
simulated end-effector site actually reaches the target.

Usage:
    venv/bin/python arm/ik.py                   # solve + validate, no GUI
    venv/bin/python arm/ik.py --view             # also open the MuJoCo passive viewer (static pose)
    venv/bin/python arm/ik.py --target 0.3 0.2 0.5    # custom world-space target (meters)
    venv/bin/mjpython arm/ik.py --interactive      # open the viewer, type "x y z" at the prompt to move the arm live
"""

import argparse
import os
import time

import mujoco
import mujoco.viewer
import numpy as np
from ikpy.chain import Chain

ARM_DIR = os.path.dirname(os.path.abspath(__file__))
URDF_PATH = os.path.join(ARM_DIR, "arm.urdf")
MJCF_PATH = os.path.join(ARM_DIR, "arm.xml")

# ikpy's chain is rooted at the fixed Shoulder_Pitch frame of the final URDF. The arm reaches
# toward local -x, so the column is placed at world x=0.50 (shoulder at 0.34, 0.85 m up, from
# make_arm_urdf.py's BASE_LINK_WORLD + SHOULDER_OFFSET) and the pedestal sits in front at x=0.12.
# Keep the frame unmirrored (BASE_YAW=0); world targets go through world_to_local().
BASE_POS_WORLD = np.array([0.34, 0.0, 0.85])
BASE_YAW = 0.0
_C, _S = np.cos(BASE_YAW), np.sin(BASE_YAW)
BASE_ROT = np.array([[_C, -_S, 0.0], [_S, _C, 0.0], [0.0, 0.0, 1.0]])


def world_to_local(p_world):
    return BASE_ROT.T @ (np.asarray(p_world) - BASE_POS_WORLD)


def local_to_world(p_local):
    return BASE_ROT @ np.asarray(p_local) + BASE_POS_WORLD


# At the all-zero pose the arm hangs straight down and several joint axes become aligned
# (a kinematic singularity), which stalls ikpy's gradient-based solver for many targets.
# Seed with a palm-down pose (end frame z == world +z, fingers down) above the cube at world
# [0.12, 0, 0.277], picked by multi-start IK for the largest margin to all joint limits.
SEED_POSE = [0.0, -0.578, 0.204, 0.336, 1.385, -0.44, -0.923, 0.461, 0.0]


def build_chain():
    return Chain.from_urdf_file(
        URDF_PATH,
        active_links_mask=[False, True, True, True, True, True, True, True, False],
    )


def solve_ik(chain, target_world, seed=None):
    # A redundant 7DOF chain has multiple joint solutions for nearly the same end-effector
    # position. Always seeding from the same fixed SEED_POSE lets SLSQP land on a different
    # local minimum almost every call, which looks like the arm jittering even when the target
    # barely moves. Passing the previous frame's solution as the seed keeps the solver in the
    # same solution branch, so smooth target motion produces smooth joint motion.
    target_local = world_to_local(target_world)
    initial_position = SEED_POSE if seed is None else [0.0, *seed, 0.0]
    joint_angles = chain.inverse_kinematics(target_local, initial_position=initial_position)
    # drop the two fixed-link placeholders (base + end-effector) ikpy includes in its output
    return joint_angles[1:-1]


def apply_pose(model, data, joint_angles):
    data.qpos[: len(joint_angles)] = joint_angles
    mujoco.mj_forward(model, data)
    return data.site("end_effector").xpos.copy()


def print_result(target_world, achieved):
    error = np.linalg.norm(achieved - np.asarray(target_world))
    print(f"target world pos:   {np.asarray(target_world)}")
    print(f"achieved world pos: {achieved}")
    print(f"position error:     {error * 1000:.2f} mm")
    return error


def view_static(model, data):
    # This viewer is for kinematic validation, not dynamics, so skip mj_step entirely —
    # stepping would integrate gravity with no actuator to hold the pose, making the
    # arm sag/swing away from the solved IK angles instead of displaying them statically.
    with mujoco.viewer.launch_passive(model, data) as viewer:
        while viewer.is_running():
            viewer.sync()
            time.sleep(0.02)


def run_interactive(chain, model, data):
    """Open the viewer and re-solve IK for each "x y z" target typed at the prompt.

    launch_passive renders on its own thread, so reading input() on the main thread
    doesn't block the window. Like view_static, this never calls mj_step — each new
    target snaps the arm directly to the newly solved pose. viewer.lock() guards the
    qpos write against the render thread reading data mid-update.
    """
    with mujoco.viewer.launch_passive(model, data) as viewer:
        print("목표 위치를 'x y z' 형태로 입력하세요 (m 단위, 종료: q)")
        while viewer.is_running():
            try:
                raw = input("target x y z> ").strip()
            except EOFError:
                break
            if raw.lower() in ("q", "quit", "exit"):
                break
            if not raw:
                continue

            parts = raw.split()
            if len(parts) != 3:
                print("3개의 숫자를 공백으로 구분해 입력하세요. 예: 0.3 0.2 0.5")
                continue
            try:
                target = [float(p) for p in parts]
            except ValueError:
                print("숫자로 입력하세요. 예: 0.3 0.2 0.5")
                continue

            joint_angles = solve_ik(chain, target)
            with viewer.lock():
                achieved = apply_pose(model, data, joint_angles)
            viewer.sync()
            print_result(target, achieved)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--target", type=float, nargs=3, default=[0.3, 0.2, 0.5],
                         help="world-space target position (x y z) in meters")
    parser.add_argument("--view", action="store_true", help="open the MuJoCo passive viewer")
    parser.add_argument("--interactive", action="store_true",
                         help="open the viewer and solve IK live for targets typed at the prompt")
    args = parser.parse_args()

    chain = build_chain()
    joint_angles = solve_ik(chain, args.target)
    print("solved joint angles (rad):")
    for link, angle in zip(chain.links[1:-1], joint_angles):
        print(f"  {link.name:16s} {angle:+.4f}")

    model = mujoco.MjModel.from_xml_path(MJCF_PATH)
    data = mujoco.MjData(model)
    achieved = apply_pose(model, data, joint_angles)
    print_result(args.target, achieved)

    if args.interactive:
        run_interactive(chain, model, data)
    elif args.view:
        view_static(model, data)


if __name__ == "__main__":
    main()
