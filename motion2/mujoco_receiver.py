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
from scipy.optimize import least_squares

_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)
from arm.ik import BASE_ROT, SEED_POSE, build_chain, local_to_world, world_to_local  # noqa: E402

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

ARM_XML = os.path.join(_DIR, "arm", "arm.xml")

# MediaPipe → arm world-space coordinate mapping. Tune these at runtime.
#
# The commanded point is the end-effector site: the grasp center between the three fingers. The
# 30 mm cube sits on a narrow post (top z=0.25) at world (0.12, 0, 0.265). The fingertips hang
# 17.5 mm below the site (measured), so with the site at BASE_Z (5 mm above the cube's bottom face,
# 10 mm below its centre) they reach 12.5 mm below that face, beside the post, and the flat finger
# pads meet the cube's sides.
# The commandable height is [BASE_Z, BASE_Z + HEIGHT_SCALE]; above ~z=0.35 near the body the arm
# can no longer keep the fingers pointing straight down (measured by sweeping the IK), so the top
# is capped there.
LATERAL_SCALE = 0.6   # x∈[0,1] → arm y-axis: (x-center)*LATERAL_SCALE
HEIGHT_SCALE = 0.085  # y∈[0,1] → arm z-axis: (1-y)*HEIGHT_SCALE + BASE_Z
DEPTH_SCALE = 0.3     # z (depth) → arm x-axis: (z-center)*DEPTH_SCALE around FORWARD_BASE
FORWARD_BASE = 0.12   # world x of the cube / pedestal centre
BASE_Z = 0.255        # grasp-centre height whose finger pads sit beside the cube

# MediaPipe's pose z is hip-relative with a per-person/per-pose baseline: measured live it sat
# anywhere from -0.8 to -1.9 for the same user. Mapping it through DEPTH_SCALE without removing
# that baseline put the target at world x ≈ -0.1..-0.46 — BEHIND the arm base, the opposite side
# from the pedestal — so the arm reached backwards and looked "stuck" from the workspace view.
# Depth is therefore auto-calibrated exactly like the lateral center: the first packet's z
# becomes neutral depth ('C' re-arms both). The solved target is additionally clamped to the
# reachable workspace box below so no packet can ever command a pose outside it.
# Box trimmed to where the arm's IK keeps the fingers pointing down (swept in sim; outside it the
# solver fails with 10-50 mm error and each failed solve takes ~150 ms, stalling the loop):
WORKSPACE_X = (0.03, 0.21)    # forward/back; cube / neutral grasp centre is at x=0.12
WORKSPACE_Y = (-0.2, 0.2)     # lateral
WORKSPACE_Z = (0.255, 0.34)   # height

# Gripper: each finger has a proximal + distal joint. Each curl∈[0,1] interpolates the proximal
# joint from FINGER_OPEN to FINGER_CLOSED; the distal joint follows by DISTAL_SCALE for a natural
# arc. The finger plates stand 30 mm apart at proximal angle 0 (finger 1/2 at x=-15 mm, finger 3
# at x=+15 mm from the grasp centre), exactly the 30 mm cube, so OPEN is slightly negative: it
# opens the gap to ~75-83 mm at the cube's height (87 mm at the tips; measured with the arm at
# BASE_Z) for the approach, and CLOSED > 0 presses past contact.
# With contacts on, a finger blocked by the cube simply stops short of its target and the servo
# keeps pressing — that residual force is what holds the cube.
FINGER_OPEN = -0.25    # radians, proximal joint at curl 0
FINGER_CLOSED = 0.15   # radians, proximal joint target at curl 1
DISTAL_SCALE = 0.1     # distal angle per radian closed from FINGER_OPEN; kept small so the pads stay flat on the cube (0.3 left them tilted and the lift slid off)
# Tracked hand finger → gripper finger, in packet order (index, middle, ring).
FINGER_ORDER = ["finger1", "finger2", "finger3"]

# IK joint order = actuator name prefixes in arm/arm.xml ("{joint}_act"). Name-based actuator
# access keeps this robust against qpos index shifts (the cube's freejoint sits after the arm
# joints in qpos, but ctrl indices don't enter into it).
ARM_JOINTS = [
    "shoulder_roll", "shoulder_yaw", "upper_arm_roll", "elbow",
    "wrist_pitch", "wrist_yaw", "hand_roll",
]

# Physics pacing: 5 substeps of 0.002s per rendered frame → 500Hz physics, 100Hz render/UDP
# loop, without running the Python loop itself at 500Hz.
N_SUBSTEPS = 5

# Startup free-camera framing: lookat between the pedestal (x=0.12, top z=0.25) and the shoulder
# (z=0.85) so the cube, the arm and the column all sit in frame.
CAM_LOOKAT = [0.2, 0.0, 0.5]
CAM_DISTANCE = 1.9
CAM_AZIMUTH = 150
CAM_ELEVATION = -15


# Above this position error, the palm-down solution is treated as unreachable and the
# position-only fallback kicks in (see solve_ik_palm_down).
PALM_DOWN_ERR_TOL = 0.02  # meters
# The full-orientation solve is unreachable above this height (see solve_ik_palm_down) and a
# failed SLSQP run costs ~100 ms, so it is only attempted at or below it.
ORIENT_ERR_TOL = 0.1  # rad (~6 deg) allowed between end frame and the aligned target
FULL_ORIENT_MAX_Z = 0.34  # meters


def _fk_error(chain, full_q, target_world):
    fk = chain.forward_kinematics(full_q)
    return float(np.linalg.norm(np.asarray(target_world) - local_to_world(fk[:3, 3])))


def _orient_err(chain, full_q):
    """Angle (rad) between the end frame and the aligned target frame BASE_ROT.T (local)."""
    rot = chain.forward_kinematics(full_q)[:3, :3]
    cos = (np.trace(rot.T @ BASE_ROT.T) - 1.0) / 2.0
    return float(np.arccos(np.clip(cos, -1.0, 1.0)))


def solve_ik_palm_down(chain, target_world, seed=None):
    """arm.ik.solve_ik + a palm-down constraint, with a position-only fallback.

    Position-only IK leaves the wrist orientation arbitrary, so the fingers can splay next to
    the cube instead of descending around it. Candidates, tried in order until one reaches the
    target within PALM_DOWN_ERR_TOL:
      1. orientation_mode="all" with the end frame aligned to the world axes: fingers pointing
         straight down AND the gripper yawed so finger1 lies along world +x. The three fingers
         sit only ~4 mm clear of the 30 mm cube, so a random yaw can land a finger on the cube
         top (the cube got launched out of the scene in testing); fixing yaw avoids that.
      2. position only, warm-started from the previous pose so the orientation drifts smoothly.
    (orientation_mode="Z" was dropped: on this arm it averaged ~60 ms per solve vs ~20 ms for
    position-only and converged less often, stalling the 100 Hz loop.)
    The arm's joint limits make the full-orientation pose unreachable for targets above
    roughly z=0.35 near the body — measured 10-35 mm error there — so it is skipped above FULL_ORIENT_MAX_Z and higher targets use position-only; the arm
    still tracks the hand instead of freezing. If no candidate meets the tolerance the lowest-error one wins.
    Same world->local shift and warm-start seeding as arm.ik.solve_ik (unmodified — this
    wrapper lives here to keep that file untouched).
    """
    target_local = world_to_local(target_world)
    warm = SEED_POSE if seed is None else [0.0, *seed, 0.0]
    # (initial_position, ikpy kwargs). The full-orientation solve is retried from SEED_POSE
    # because a warm start that drifted into a bad-yaw branch (after a position-only fallback)
    # otherwise keeps failing even where an aligned solution exists.
    candidates = [(warm, {})]
    if target_world[2] <= FULL_ORIENT_MAX_Z:
        full = dict(target_orientation=BASE_ROT.T, orientation_mode="all")
        candidates = [(warm, full)] + ([(SEED_POSE, full)] if seed is not None else []) + candidates
    best, best_err = None, float("inf")
    for initial_position, kwargs in candidates:
        angles = chain.inverse_kinematics(target_local, initial_position=initial_position, **kwargs)
        err = _fk_error(chain, angles, target_world)
        # ikpy's "all" mode minimizes position + orientation jointly, so position alone can be
        # met while the yaw is far off; reject those so the retry/fallback runs instead.
        aligned = err <= PALM_DOWN_ERR_TOL and (
            "orientation_mode" not in kwargs or _orient_err(chain, angles) <= ORIENT_ERR_TOL)
        if err < best_err:
            best, best_err = angles, err
        if aligned:
            break
    return best[1:-1]


# --- Whole-arm tracking -----------------------------------------------------------------------
# The 7DOF arm is redundant for a 3D wrist target: the elbow can swing on a circle around the
# shoulder-wrist axis (the "swivel" angle) without moving the wrist. Following only the wrist
# leaves that DOF to whatever the solver's warm start happens to give. When the sender also
# provides the shoulder and elbow landmarks, the change in the user's swivel angle is imposed on
# the robot as a soft secondary task, so the robot's elbow swings out/in like the user's.
# Only the CHANGE is tracked (relative to the first packet / 'C' recalibration): the robot's
# shoulder sits above its hand while a person's sits beside it, so absolute elbow directions do
# not correspond between the two.
SHOULDER_LINK, ELBOW_LINK, WRIST_LINK = 2, 4, 6   # chain.links indices (0 = base)
SWIVEL_GAIN = 1.0         # robot swivel change per unit human swivel change
# Past about 40 deg the arm solver no longer follows a held swing continuously. Measured at the
# cube's spot while lifting the wrist 1 mm at a time: the largest joint step is 0.013-0.018 rad up
# to +-40 deg, 0.05 rad at +50, and a hop of 0.22 rad at -70 / 0.48 rad at +70 (an elbow flip; the
# hand jerks and the pinched cube is thrown, see check_teleop.py swing-jump).
SWIVEL_LIMIT = np.radians(40)
# How fast the commanded swing may change. The measured swing can jump by tens of degrees in one
# frame: swivel_angle() switches its reference when the shoulder-wrist axis is nearly vertical (an
# arm hanging down), and a noisy elbow near a straight arm does the same. Straight into the IK that
# is a step the servos cannot follow. In the swing-jump scenario of check_teleop.py, with the swing
# clipped to 40 deg, the hand strays 7.9 mm, tilts 7.1 deg and the servos lag 0.45 rad; ramped at
# this rate: 2.5 mm, 3.9 deg, 0.10 rad. (At the old 70 deg limit the hand strayed 37 mm and
# knocked the cube off its post.)
SWIVEL_MAX_RATE = np.radians(90)    # rad/s: a full 40 deg swing takes ~0.5 s
# Even a ramped swing is a null-space motion of the whole arm, and the cube is pinched between plates
# 30.1 mm apart. Measured during the lift with the swing kept moving: at 5 / 10 / 20 deg/s the cube
# creeps 5 / 9 / 15 mm sideways in the pinch, at 45 deg/s and up it is thrown; with the swing held
# still it stays within 2 mm. The swing is only the elbow's style, so while the fingers are closing
# or closed (mean curl above this) the commanded swing is frozen, and it picks up again, rate
# limited, once they open.
SWIVEL_HOLD_CURL = 0.3
ELBOW_DEPTH_GAIN = 0.5    # MediaPipe z is noisy; damp the depth part of the arm vectors
W_POS = 30.0              # wrist position residual weight (m)
W_SWIVEL = 0.5            # swivel residual weight (rad)
W_ORIENT = 0.05           # palm-down orientation weight (soft; Frobenius difference)
W_REG = 0.05              # smoothness: stay near the previous solution (rad)
ARM_POS_TOL = 0.02        # meters; above this the arm solve is rejected for the fallback solver
_BOUNDS = None


def swivel_angle(shoulder, elbow, wrist):
    """Angle (rad) of the elbow around the shoulder→wrist axis, measured from world-down."""
    a = np.asarray(wrist, float) - np.asarray(shoulder, float)
    a = a / (np.linalg.norm(a) + 1e-9)
    ref = np.array([0.0, 0.0, -1.0])
    ref = ref - a * (a @ ref)
    if np.linalg.norm(ref) < 0.2:       # axis nearly vertical: use world +x instead
        ref = np.array([1.0, 0.0, 0.0])
        ref = ref - a * (a @ ref)
    ref = ref / np.linalg.norm(ref)
    e = np.asarray(elbow, float) - np.asarray(shoulder, float)
    e = e - a * (a @ e)
    return float(np.arctan2(a @ np.cross(ref, e), ref @ e))


def human_swivel(shoulder_img, elbow_img, wrist_img):
    """Swivel of the user's arm from three MediaPipe pose landmarks (x right, y down, z away
    from camera), after the same axis mapping as the wrist target in main()."""
    def to_world(p):
        return np.array([p[2] * ELBOW_DEPTH_GAIN, p[0], -p[1]])
    return swivel_angle(to_world(shoulder_img), to_world(elbow_img), to_world(wrist_img))


def _wrap(a):
    return float(np.arctan2(np.sin(a), np.cos(a)))


def robot_swivel(chain, q):
    frames = chain.forward_kinematics(np.concatenate(([0.0], q, [0.0])), full_kinematics=True)
    p = [local_to_world(frames[i][:3, 3]) for i in (SHOULDER_LINK, ELBOW_LINK, WRIST_LINK)]
    return swivel_angle(*p)


def solve_ik_arm(chain, target_world, swivel_target, seed):
    """Wrist position + elbow swivel + soft palm-down, solved jointly over the 7 joints.

    swivel_target: desired robot swivel angle (rad), or None to leave the elbow free.
    Returns (angles, wrist_pos_err_m, achieved_swivel_rad).
    """
    global _BOUNDS
    if _BOUNDS is None:
        b = np.array([l.bounds for l in chain.links[1:-1]], float)
        _BOUNDS = (b[:, 0], b[:, 1])
    target_local = world_to_local(target_world)
    seed = np.asarray(seed, float)

    def residual(q):
        frames = chain.forward_kinematics(np.concatenate(([0.0], q, [0.0])), full_kinematics=True)
        end = frames[-1]
        res = [W_POS * (end[:3, 3] - target_local)]
        if swivel_target is not None:
            p = [local_to_world(frames[i][:3, 3]) for i in (SHOULDER_LINK, ELBOW_LINK, WRIST_LINK)]
            res.append([W_SWIVEL * _wrap(swivel_angle(*p) - swivel_target)])
        res.append(W_ORIENT * (end[:3, :3] - BASE_ROT.T).ravel())
        res.append(W_REG * (q - seed))
        return np.concatenate(res)

    x0 = np.clip(seed, _BOUNDS[0] + 1e-6, _BOUNDS[1] - 1e-6)
    sol = least_squares(residual, x0, bounds=_BOUNDS, max_nfev=40)
    full = np.concatenate(([0.0], sol.x, [0.0]))
    return sol.x, _fk_error(chain, full, target_world), robot_swivel(chain, sol.x)


def apply_arm_ctrl(data, joint_angles):
    for joint, angle in zip(ARM_JOINTS, joint_angles):
        data.actuator(f"{joint}_act").ctrl[0] = angle


def apply_grip(data, curls):
    for finger, curl in zip(FINGER_ORDER, curls):
        curl = max(0.0, min(1.0, curl))
        proximal = FINGER_OPEN + curl * (FINGER_CLOSED - FINGER_OPEN)
        data.actuator(f"{finger}_joint1_act").ctrl[0] = proximal
        data.actuator(f"{finger}_joint2_act").ctrl[0] = (proximal - FINGER_OPEN) * DISTAL_SCALE


def parse_packet(raw):
    """Decode one UDP packet → (x_norm, y_norm, z_m, curls, arm).

    12 fields = whole-arm format "x y z c_i c_m c_r ex ey ez sx sy sz" (elbow and shoulder
    landmarks appended);
    6 fields = per-finger format; 4 fields = legacy single-grip format, fanned out to all three
    fingers. `arm` is ((elbow xyz), (shoulder xyz)) or None when the packet has no arm landmarks.
    """
    vals = [float(v) for v in raw.decode().strip().split()]
    if len(vals) == 12:
        return (vals[0], vals[1], vals[2], (vals[3], vals[4], vals[5]),
                (tuple(vals[6:9]), tuple(vals[9:12])))
    if len(vals) == 6:
        return vals[0], vals[1], vals[2], (vals[3], vals[4], vals[5]), None
    if len(vals) == 4:
        return vals[0], vals[1], vals[2], (vals[3], vals[3], vals[3]), None
    raise ValueError(f"expected 4, 6 or 12 fields, got {len(vals)}")


def main():
    model = mujoco.MjModel.from_xml_path(ARM_XML)
    data = mujoco.MjData(model)
    frame_dt = model.opt.timestep * N_SUBSTEPS

    chain = build_chain()
    # Start from the same non-singular seed arm/ik.py uses, not zeros — the all-zero pose is a
    # kinematic singularity (see arm/ik.py's SEED_POSE comment) that IK can fail to escape once
    # it's used as a warm-start seed.
    last_good_qpos = np.array(SEED_POSE[1:-1])
    robot_neutral_swivel = robot_swivel(chain, last_good_qpos)  # swivel the user's neutral maps to
    last_curls = (0.0, 0.0, 0.0)  # holds steady if a packet is missing/malformed

    # Pre-pose the arm at the seed AND point the servos there before the first step: starting
    # at qpos=0 (the singular straight-down pose) with servos pulling toward a distant target
    # makes the arm slam through the pedestal on startup. Fingers start open (curl 0).
    for joint, angle in zip(ARM_JOINTS, last_good_qpos):
        data.joint(joint).qpos[0] = angle
    # Fingers start open too: at proximal angle 0 the finger plates are exactly one cube width
    # apart, so starting there would begin the simulation with the cube in contact.
    for finger in FINGER_ORDER:
        data.joint(f"{finger}_joint1").qpos[0] = FINGER_OPEN
    apply_arm_ctrl(data, last_good_qpos)
    apply_grip(data, last_curls)
    mujoco.mj_forward(model, data)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT} ...")

    # None triggers auto-calibration on the first packet; pressing 'C' re-arms it so the next
    # received x_norm/z_m become the new lateral center and neutral depth.
    # "delta"/"t" are the swing actually commanded (rate-limited, see SWIVEL_MAX_RATE) and when it
    # was last updated.
    calib = {"x_center": None, "z_center": None, "swivel": None, "delta": 0.0, "t": 0.0}
    # 'T' toggles gripper-tracking camera. The key callback runs on the viewer's own thread
    # before/while `viewer` exists, so it only flips the flag — the main loop applies the
    # camera change when `applied` disagrees with `want`.
    cam_track = {"want": False, "applied": False}

    def key_callback(keycode):
        if keycode == ord("C"):
            calib["x_center"] = None
            calib["z_center"] = None
            calib["swivel"] = None
            print("[Calibration] next wrist position will become the new center/depth")
        elif keycode == ord("T"):
            cam_track["want"] = not cam_track["want"]
            print(f"[Camera] gripper tracking {'ON' if cam_track['want'] else 'OFF'}")

    gripper_body_id = model.body("MG996R").id  # the hand: the servo body the three fingers hang from

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
                    x_norm, y_norm, z_m, last_curls, arm = parse_packet(raw)

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
                        joint_angles = None
                        if arm is not None:
                            elbow, shoulder = arm
                            h_sw = human_swivel(shoulder, elbow, (x_norm, y_norm, z_m))
                            now = time.monotonic()
                            if calib["swivel"] is None:
                                calib["swivel"], calib["delta"], calib["t"] = h_sw, 0.0, now
                            want = np.clip(_wrap(h_sw - calib["swivel"]) * SWIVEL_GAIN,
                                           -SWIVEL_LIMIT, SWIVEL_LIMIT)
                            # Ramp to the wanted swing (after a pause cap the step at 0.1 s worth),
                            # and keep it still while the hand is closed on something.
                            gripping = float(np.mean(last_curls)) > SWIVEL_HOLD_CURL
                            step = 0.0 if gripping else SWIVEL_MAX_RATE * min(now - calib["t"], 0.1)
                            calib["delta"] += float(np.clip(want - calib["delta"], -step, step))
                            calib["t"] = now
                            delta = calib["delta"]
                            angles, err, _ = solve_ik_arm(
                                chain, target_world, robot_neutral_swivel + delta, last_good_qpos)
                            if err <= ARM_POS_TOL:
                                joint_angles = angles
                        if joint_angles is None:
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
