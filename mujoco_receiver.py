"""MuJoCo UDP receiver: drive the 7DOF arm + 3-finger gripper with real-time wrist coordinates
and per-finger curl values, under contact physics — pick up the cube on the pedestal with your
own hand.

Usage:
    venv/bin/mjpython mujoco_receiver.py   # macOS
    venv/bin/python mujoco_receiver.py      # Linux

Receives right wrist position + index/middle/ring finger curls from vision_sender.py via UDP,
converts wrist position to arm workspace coordinates, solves IK, and feeds the joint angles to
position actuators in arm/arm.xml while mj_step integrates contact physics. Each curl value
(0=extended..1=curled) drives its own gripper finger: index → finger1, middle → finger2,
ring → finger3. Legacy 4-field packets ("x y z grip") are still accepted and apply the single
grip value to all three fingers.

Unlike the earlier kinematic version, joint angles are NOT written into qpos directly — with
contacts enabled that would teleport the fingers through the cube. Instead every target goes to
a <position> actuator (see arm/arm.xml) and the physics loop lets the servos pull the arm there,
so the fingers can actually push on and hold the cube. The arm therefore follows the hand with
a slight servo lag; that is expected.

Camera: on startup the free camera is aimed at the pedestal/workspace. Press 'T' in the viewer
to toggle a tracking camera that keeps the gripper centered as it moves; 'T' again returns to
the fixed view.

Lateral/depth centering: MediaPipe's x=0.5 (frame center) is not necessarily where the arm
should consider "centered" — a person's right wrist naturally rests off to one side of the
frame — and MediaPipe's z (hip-relative depth) has a large per-person baseline (measured -0.8
to -1.9 for the same user), which unmapped would command targets behind the arm base. The
receiver auto-calibrates both the lateral center and the neutral depth to wherever the wrist is
on the very first packet received; press 'C' in the viewer window at any time to recalibrate to
the wrist's current position instead. Solved targets are clamped to the WORKSPACE_* box.
"""

import os
import socket
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)
from arm.ik import BASE_POS_WORLD, SEED_POSE, build_chain  # noqa: E402

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

ARM_XML = os.path.join(_DIR, "arm", "arm.xml")

# MediaPipe → arm world-space coordinate mapping. Tune these at runtime.
#
# Geometry budget (see .omc/plans/autopilot-impl.md): the commandable gripper_base height is
# [BASE_Z, BASE_Z + HEIGHT_SCALE] and the open fingertips hang ~0.06 below it. The cube rests
# at center z=0.465 on the pedestal (top z=0.45), so BASE_Z must sit below 0.465 minus some
# margin or the gripper can never descend around the cube. BASE_Z=0.40 leaves 0.065 of margin.
LATERAL_SCALE = 0.6   # x∈[0,1] → arm y-axis: (x-center)*LATERAL_SCALE
HEIGHT_SCALE = 0.5    # y∈[0,1] → arm z-axis: (1-y)*HEIGHT_SCALE + BASE_Z
DEPTH_SCALE = 0.3     # z (depth) → arm x-axis: (z-center)*DEPTH_SCALE around FORWARD_BASE
FORWARD_BASE = 0.12   # world-x when the hand is at its calibrated neutral depth — on the pedestal
BASE_Z = 0.40         # world-z when wrist is at the bottom of the image

# MediaPipe's pose z is hip-relative with a per-person/per-pose baseline: measured live it sat
# anywhere from -0.8 to -1.9 for the same user. Mapping it through DEPTH_SCALE without removing
# that baseline put the target at world x ≈ -0.1..-0.46 — BEHIND the arm base, the opposite side
# from the pedestal — so the arm reached backwards and looked "stuck" from the workspace view.
# Depth is therefore auto-calibrated exactly like the lateral center: the first packet's z
# becomes neutral depth ('C' re-arms both). The solved target is additionally clamped to the
# reachable workspace box below so no packet can ever command a pose outside it.
WORKSPACE_X = (0.0, 0.35)    # forward/back, pedestal at 0.12
WORKSPACE_Y = (-0.35, 0.35)  # lateral
WORKSPACE_Z = (0.30, 0.90)   # height; arm base at z=0.8, pedestal top at 0.45

# Gripper: each of the 3 fingers has a proximal + distal joint sharing one curl axis (see
# arm/arm.xml). Each finger's curl∈[0,1] (0=open,1=closed) scales both of its joint targets; the
# distal joint curls less than the proximal one so the finger bends in a natural-looking arc
# rather than a straight hinge.
# MAX_CURL=1.3 initially overshot: fingertips passed the centerline before grip reached 1.0 and
# started spreading out again on the far side (measured empirically by sweeping grip and tracking
# fingertip convergence — see arm/AGENTS.md). 0.78 lands the tips at the centerline at grip=1.0.
# With contacts on, a finger blocked by the cube simply stops short of its target and the servo
# keeps pressing — that residual force is what holds the cube.
MAX_CURL = 0.78  # radians, proximal joint target fully closed
DISTAL_SCALE = 0.6
# Tracked hand finger → gripper finger, in packet order (index, middle, ring).
FINGER_ORDER = ["finger1", "finger2", "finger3"]

# IK joint order = actuator name prefixes in arm/arm.xml ("{joint}_act"). Name-based actuator
# access keeps this robust against qpos index shifts (the cube's freejoint sits after the arm
# joints in qpos, but ctrl indices don't enter into it).
ARM_JOINTS = [
    "shoulder_pitch", "shoulder_roll", "upper_arm_roll", "elbow",
    "wrist_yaw", "wrist_pitch", "wrist_roll",
]

# Physics pacing: 5 substeps of 0.002s per rendered frame → 500Hz physics, 100Hz render/UDP
# loop, without running the Python loop itself at 500Hz.
N_SUBSTEPS = 5

# Startup free-camera framing: lookat between the pedestal (x=0.12, top z=0.45) and the arm's
# workspace so both the cube and the hand sit in frame.
CAM_LOOKAT = [0.12, 0.0, 0.55]
CAM_DISTANCE = 1.6
CAM_AZIMUTH = 150
CAM_ELEVATION = -15


# Above this position error, the palm-down solution is treated as unreachable and the
# position-only fallback kicks in (see solve_ik_palm_down).
PALM_DOWN_ERR_TOL = 0.02  # meters


def _fk_error(chain, full_q, target_world):
    fk = chain.forward_kinematics(full_q)
    return float(np.linalg.norm(np.asarray(target_world) - (fk[:3, 3] + BASE_POS_WORLD)))


def solve_ik_palm_down(chain, target_world, seed=None):
    """arm.ik.solve_ik + a palm-down constraint, with a position-only fallback.

    Position-only IK leaves the wrist orientation arbitrary — measured at the grasp pose it
    left the palm pointing sideways (gripper local z ≈ [0.53, 0.82, -0.22]), so the fingers
    splayed next to the cube instead of descending around it. orientation_mode="Z" pins the
    end link's local z-axis to world +z (at qpos=0 the arm hangs straight down with local z =
    world z), which keeps the fingers pointing down; the redundant 7DOF chain has DOF to spare
    for position(3) + one axis(2).

    The constraint is NOT satisfiable everywhere in the workspace though: near/high targets
    around y=0 need an elbow bend beyond its 2.4435 rad limit to keep the hand vertical
    (measured 54–153 mm position error, unchanged by max_iter — a joint-limit wall, not a
    convergence issue). Low grasp-zone targets solve fine. So: try palm-down first; if the
    position error exceeds PALM_DOWN_ERR_TOL, re-solve position-only and keep the better one —
    fingers stay vertical where it matters (around the cube) and the arm still tracks the hand
    everywhere else instead of freezing. Same world→local shift and warm-start seeding as
    arm.ik.solve_ik (unmodified — this wrapper lives here to keep that file untouched).
    """
    target_local = np.asarray(target_world) - BASE_POS_WORLD
    initial_position = SEED_POSE if seed is None else [0.0, *seed, 0.0]
    joint_angles = chain.inverse_kinematics(
        target_local,
        initial_position=initial_position,
        target_orientation=[0, 0, 1],
        orientation_mode="Z",
    )
    err = _fk_error(chain, joint_angles, target_world)
    if err > PALM_DOWN_ERR_TOL:
        fallback = chain.inverse_kinematics(target_local, initial_position=initial_position)
        if _fk_error(chain, fallback, target_world) < err:
            joint_angles = fallback
    return joint_angles[1:-1]


def apply_arm_ctrl(data, joint_angles):
    for joint, angle in zip(ARM_JOINTS, joint_angles):
        data.actuator(f"{joint}_act").ctrl[0] = angle


def apply_grip(data, curls):
    for finger, curl in zip(FINGER_ORDER, curls):
        curl = max(0.0, min(1.0, curl))
        data.actuator(f"{finger}_joint1_act").ctrl[0] = curl * MAX_CURL
        data.actuator(f"{finger}_joint2_act").ctrl[0] = curl * MAX_CURL * DISTAL_SCALE


def parse_packet(raw):
    """Decode one UDP packet → (x_norm, y_norm, z_m, (curl_index, curl_middle, curl_ring)).

    6 fields = current per-finger format; 4 fields = legacy single-grip format, fanned out to
    all three fingers so an old sender still works.
    """
    vals = [float(v) for v in raw.decode().strip().split()]
    if len(vals) == 6:
        return vals[0], vals[1], vals[2], (vals[3], vals[4], vals[5])
    if len(vals) == 4:
        return vals[0], vals[1], vals[2], (vals[3], vals[3], vals[3])
    raise ValueError(f"expected 4 or 6 fields, got {len(vals)}")


def main():
    model = mujoco.MjModel.from_xml_path(ARM_XML)
    data = mujoco.MjData(model)
    frame_dt = model.opt.timestep * N_SUBSTEPS

    chain = build_chain()
    # Start from the same non-singular seed arm/ik.py uses, not zeros — the all-zero pose is a
    # kinematic singularity (see arm/ik.py's SEED_POSE comment) that IK can fail to escape once
    # it's used as a warm-start seed.
    last_good_qpos = np.array(SEED_POSE[1:-1])
    last_curls = (0.0, 0.0, 0.0)  # holds steady if a packet is missing/malformed

    # Pre-pose the arm at the seed AND point the servos there before the first step: starting
    # at qpos=0 (the singular straight-down pose) with servos pulling toward a distant target
    # makes the arm slam through the pedestal on startup. Fingers start open (ctrl=0).
    for joint, angle in zip(ARM_JOINTS, last_good_qpos):
        data.joint(joint).qpos[0] = angle
    apply_arm_ctrl(data, last_good_qpos)
    apply_grip(data, last_curls)
    mujoco.mj_forward(model, data)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT} ...")

    # None triggers auto-calibration on the first packet; pressing 'C' re-arms it so the next
    # received x_norm/z_m become the new lateral center and neutral depth.
    calib = {"x_center": None, "z_center": None}
    # 'T' toggles gripper-tracking camera. The key callback runs on the viewer's own thread
    # before/while `viewer` exists, so it only flips the flag — the main loop applies the
    # camera change when `applied` disagrees with `want`.
    cam_track = {"want": False, "applied": False}

    def key_callback(keycode):
        if keycode == ord("C"):
            calib["x_center"] = None
            calib["z_center"] = None
            print("[Calibration] next wrist position will become the new center/depth")
        elif keycode == ord("T"):
            cam_track["want"] = not cam_track["want"]
            print(f"[Camera] gripper tracking {'ON' if cam_track['want'] else 'OFF'}")

    gripper_body_id = model.body("gripper_base").id

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        # Re-aim the default free camera at the workspace (see CAM_* comment above).
        with viewer.lock():
            viewer.cam.lookat[:] = CAM_LOOKAT
            viewer.cam.distance = CAM_DISTANCE
            viewer.cam.azimuth = CAM_AZIMUTH
            viewer.cam.elevation = CAM_ELEVATION

        while viewer.is_running():
            frame_start = time.monotonic()

            if cam_track["want"] != cam_track["applied"]:
                with viewer.lock():
                    if cam_track["want"]:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_TRACKING
                        viewer.cam.trackbodyid = gripper_body_id
                        viewer.cam.distance = 0.8
                    else:
                        viewer.cam.type = mujoco.mjtCamera.mjCAMERA_FREE
                        viewer.cam.trackbodyid = -1
                        viewer.cam.lookat[:] = CAM_LOOKAT
                        viewer.cam.distance = CAM_DISTANCE
                cam_track["applied"] = cam_track["want"]

            # Drain the UDP queue and keep only the newest packet: the sender ticks at camera
            # rate independent of this loop, and applying stale queued samples would add lag.
            raw = None
            while True:
                try:
                    raw, _ = sock.recvfrom(1024)
                except BlockingIOError:
                    break

            if raw is not None:
                try:
                    x_norm, y_norm, z_m, last_curls = parse_packet(raw)

                    if calib["x_center"] is None:
                        calib["x_center"] = x_norm
                        calib["z_center"] = z_m
                        print(f"[Calibration] center x = {x_norm:.3f}, neutral depth z = {z_m:.3f}")

                    target_world = [
                        np.clip(FORWARD_BASE + (z_m - calib["z_center"]) * DEPTH_SCALE, *WORKSPACE_X),
                        np.clip((x_norm - calib["x_center"]) * LATERAL_SCALE, *WORKSPACE_Y),
                        np.clip((1.0 - y_norm) * HEIGHT_SCALE + BASE_Z, *WORKSPACE_Z),
                    ]

                    try:
                        joint_angles = solve_ik_palm_down(chain, target_world, seed=last_good_qpos)
                        if not np.any(np.isnan(joint_angles)):
                            last_good_qpos = joint_angles
                        else:
                            joint_angles = last_good_qpos
                    except Exception:
                        joint_angles = last_good_qpos

                    # ctrl targets only — mj_step below moves the arm via the servos.
                    apply_arm_ctrl(data, joint_angles)
                    apply_grip(data, last_curls)

                except ValueError:
                    pass
                except Exception as e:
                    print(f"Error: {e}")

            with viewer.lock():
                for _ in range(N_SUBSTEPS):
                    mujoco.mj_step(model, data)

            viewer.sync()
            leftover = frame_dt - (time.monotonic() - frame_start)
            if leftover > 0:
                time.sleep(leftover)


if __name__ == "__main__":
    main()
