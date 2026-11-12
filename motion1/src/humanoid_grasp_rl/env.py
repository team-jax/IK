from __future__ import annotations

import os
import warnings

from pathlib import Path
from typing import Literal

import gymnasium as gym
import mujoco
import numpy as np
from gymnasium import spaces
from scipy.spatial.transform import Rotation, Slerp

from humanoid_grasp_rl.assets import robot_parts
from humanoid_grasp_rl.ik import solve_site_pose

Stage = Literal["place", "randomize"]

# --- The scene ---------------------------------------------------------------------------
# Two setups, picked with the JAX_SETUP environment variable; heights in metres above the floor.
# "sim" (the default, where every result in README was measured): the shoulder axis at 0.85 with
# the robot's stand on the floor, and the table at 0.25 - where the arm holds the hand palm-down
# over a wide patch (swept in sim; the patch shrinks fast below 0.20 and above 0.38).
# "real": the team's setup. The table top is 735 mm above the floor (measured 2026-11-04) and the
# robot stands on the floor as V3's URDF builds it, the shoulder axis 1212.5 mm up, so D is 477.5 mm
# (user 2026-11-10, as JaxIsaacLab's deploy_2026-11-05 has it; the CAD figure was 1207.5). The
# robot faces the table as in JaxIsaacLab (user 2026-11-12): turned -90 degrees about the vertical
# through its shoulder (robot_yaw; JaxIsaacLab's base quat 0.7071, 0, 0, -0.7071), so the stand's
# FRONT side faces world -x, where the table is, and the arm hangs beside the column on its +y side.
# The table's near edge is STAND_GAP from the column's FRONT face (user 2026-11-11, kept), which
# leaves it 14 cm from the shoulder axis (JaxIsaacLab: 22.5 cm). On 2026-11-11 the table was on the
# side the shoulder bracket sticks out to, the robot not turned. Below its shoulder the arm cannot hold the fingers
# straight down, so the hand leans HAND_TILT degrees about its finger-gap axis; one of fingers 1
# and 2 then hangs 36 mm below the grasp centre, and the grasp moves 21.5 mm up the hand's axis to
# keep it off the table. It is the scene to try a new hand in (motion1/STATUS.md).
# d is D, the drop from the shoulder rotation axis to the table top (the reach band depends on it
# directly). cube_size and cube_mass: the real cube is 50 mm (user, 2026-11-05), its mass not
# measured (0.05 kg). In both setups the table's near edge is STAND_GAP from the stand's surface
# facing it at the table top, stand_surface_x, sliced from V3's stand (URDF_1108's base_link mesh):
# a column 80 mm square, the same at every height between the foot plate (0 to 0.05 above the
# floor) and the shoulder bracket (from 1.15 up). Not turned (sim setup) its face is at x 0.45 and
# the plate reaches x -0.05; turned -90 degrees (real setup) the face is at x 0.30, the column
# spans y -0.19 to -0.11 and the plate reaches x -0.16 under the table (V2's stand narrowed
# upwards: 0.4085). home is the arm's rest pose
# (HOME_ARM_QPOS, found by scripts/find_home.py).
SETUPS = {
    "sim": dict(table_z=0.25, d=0.60, cube_size=0.05, cube_mass=0.05, robot_yaw=0.0, stand_surface_x=0.45,
                hand_tilt=0.0, grasp_dz=0.0,
                home=(-0.3178, 0.1606, -0.0103, 1.5426, -0.1588, -1.2235, -0.0061)),
    "real": dict(table_z=0.735, d=0.4775, cube_size=0.05, cube_mass=0.05, robot_yaw=-90.0, stand_surface_x=0.30,
                 hand_tilt=-40.0, grasp_dz=0.0215, pad_friction=2.0, hand_turn_rate=30.0, episode_steps=1600,
                 lower_rate=0.001, travel_rate=0.002, carry_rate=0.0004, place_align=0.006,
                 hand_flip=True, grasp_misalign=30.0, hand_footprint=(0.059, 0.025, 0.077), empty_clear=0.07,
                 reach_rescue=True, slow_lower_rate=0.0005,
                 home=(0.4727, 0.1093, 2.4948, -0.9200, -1.5708, 1.4812, 0.1006)),
}
SETUP = os.environ.get("JAX_SETUP", "sim")
if SETUP not in SETUPS:
    raise ValueError(f"JAX_SETUP={SETUP!r}: the setups are {', '.join(SETUPS)}")
TABLE_SURFACE_Z = SETUPS[SETUP]["table_z"]
D = SETUPS[SETUP]["d"]
SHOULDER_Z = round(TABLE_SURFACE_Z + D, 6)   # rounded: exactly 0.85 and 1.2125
ROBOT_YAW = SETUPS[SETUP]["robot_yaw"]
# The shoulder frame's world x: motion2's generator puts it there (BASE_LINK_WORLD + SHOULDER_OFFSET),
# and turning the robot (assets.robot_parts) keeps it there. run_tests.sh checks it on the model.
SHOULDER_X = 0.34
# The table's near edge: STAND_GAP from the robot's stand (the user's figure, 2026-11-04, confirmed
# 2026-11-05 and, for the real setup, 2026-11-11; sizing the table from the reach map alone had left
# 68.5 mm).
STAND_GAP = 0.10
STAND_SURFACE_X = SETUPS[SETUP]["stand_surface_x"]
TABLE_NEAR_X = round(STAND_SURFACE_X - STAND_GAP, 6)
# 50 mm, not the 30 mm the pedestal demo in motion2 uses. The finger plates reach only 17 mm
# below the grasp centre, so on a table the pads cannot go below a cube's bottom face and
# meet it over its upper part; a 30 mm cube then slips or is shot out, a 40 mm one holds in
# four of five closings, a 50 mm one in five of five. Set per setup (cube_size, cube_mass).
OBJECT_SIZE = SETUPS[SETUP]["cube_size"]
OBJECT_MASS = SETUPS[SETUP]["cube_mass"]
OBJECT_HALF_SIZE = OBJECT_SIZE / 2
OBJECT_Z = TABLE_SURFACE_Z + OBJECT_HALF_SIZE
# The nearest a cube spot may be to the robot and still stand on the table: a spot is a cell
# jittered by up to 0.01 (_draw_spot). The reach map leaves out the cells past it (in the sim
# setup none of its grid is).
CUBE_X_MAX = round(TABLE_NEAR_X - 0.01 - OBJECT_HALF_SIZE, 6)
# Each cube has its own goal spot marked on the table in the cube's colour.
TARGET_TOLERANCE = 0.05
TARGET_MIN_SEPARATION = 0.15
OBJECT_COLORS = ((0.90, 0.25, 0.20), (0.95, 0.70, 0.15),
                 (0.30, 0.65, 0.35), (0.35, 0.45, 0.90))

# --- The hand ----------------------------------------------------------------------------
# The three finger plates are thin in the hand's x axis and hang below the grasp centre (the
# `end_effector` site, between them). Finger 1 and 2 stand on one side, finger 3 on the
# other, and a closure g in [0, 1] takes the proximal and distal joints from OPEN to CLOSED.
# The pads' gap along that path was measured on the hull geometry (`scripts/tune_grasp.py`):
# 90 mm at g = 0, which clears a 50 mm cube by 20 mm a side, 50 mm at g = 0.86, 44 mm at g = 1.
#
# The path is chosen for the pad angle, not the gap alone. Each plate leans by the sum of its
# two joint angles, and a plate whose tip leans away from the cube turns the squeeze into a
# push *down* the cube's side: the cube slid out of the pads as soon as the hand rose. Here
# the plates are upright at g = 0.70 and lean slightly inwards from the moment they touch the
# cube (+3 degrees at contact, +6 at g = 1), so the squeeze holds the cube up instead.
FINGER_OPEN = np.array([-0.26, 0.0])        # (proximal, distal) radians
FINGER_CLOSED = np.array([-0.19, 0.30])
# The closure whose pads are as far apart as the cube. Past it the servos squeeze, and the
# force is what holds it.
CUBE_GRIP = 0.86
# --- Grasp geometry (see scripts/tune_grasp.py for how they were measured) ----------------
# How far the grasp centre sits above the cube's centre, along the hand's axis, when the fingers
# close. The plates end 17.5 mm below it, so at 0 they stop 7.5 mm above the table; the leaning
# hand of the real setup needs 21.5 mm (see SETUPS).
GRASP_DZ = SETUPS[SETUP]["grasp_dz"]
# The same once the cube is held and lifted.
HELD_DZ = GRASP_DZ
# How much stiffer the friction constraints are than the normal ones. At 1 (MuJoCo's default) a
# pressed cube creeps down the pads; the elliptic cone with a large ratio holds it.
IMPRATIO = 30
# Whether fingers 1 and 2 are held at the same angle (see _scene_xml).
COUPLE_FINGERS = True
# The cube's torsional friction, which resists the pads twisting it about the vertical. Each
# pad meets it at one point, and the three points are not in line, so the squeeze is a small
# couple as well as a clamp.
CUBE_TORSION = 0.02
# The sliding friction between the pads and the cube, measured on the real ones (2026-11-04).
# MuJoCo uses the larger of the two geoms' coefficients at a contact (read back from data.contact
# in 3.10.0), so in the sim setup the finger pads get the same value (PAD_FRICTION), and the cube's
# contact with the table then takes the table's 1. It was 2.0 until 2026-11-04; at 0.4 a carried
# cube slips out of the pads more often (see CROSS_RATE in demonstration.py).
CUBE_FRICTION = 0.4
# The finger pads' own coefficient, on every finger collision geom (first and last links). The sim
# setup keeps the measured 0.4. The real setup has 2.0: the team will cover the fingers' inner faces
# with a material of 2.0 or more (user, 2026-11-10; not measured yet). Only the pads change, so
# pad-cube contacts take 2.0 while the cube keeps 0.4 against the table (1) and another cube (0.4).
PAD_FRICTION = SETUPS[SETUP].get("pad_friction", CUBE_FRICTION)
# How fast the commanded hand orientation may turn, degrees a second (None: no limit). The real
# setup's home holds the hand turned far from the grasp orientation, and _hand_rotation makes that
# turn over the few centimetres between home and the reach band, so at the reach speed hand_roll
# was commanded at up to 1750 deg/s in one step and the whole trajectory had to be played about 30
# times slower on the robot (SimToReal plays it at peak joint speed / 30 deg/s). At 30 deg/s the
# hand turns while it travels and the scripted reach waits for the turn (hand_turn_lag). The sim
# setup's home is inside the band, where the orientation does not change, so it has no limit.
HAND_TURN_RATE = SETUPS[SETUP].get("hand_turn_rate")
# How many control steps an episode (one cube moved) may take. The real setup's slower turns and
# descents and carry need more than the sim setup's 700 (motion1/STATUS.md 2026-11-10 (4), (5)).
EPISODE_STEPS = SETUPS[SETUP].get("episode_steps", 700)
# How far the bottom of a held cube hangs under the grasp centre: its half-size, plus how far the
# grasp centre sits above the cube's centre (HELD_DZ runs along the leaning hand's axis), plus the
# few millimetres it sits low in the pads.
HAND_TILT = SETUPS[SETUP]["hand_tilt"]
HELD_BELOW_GC = OBJECT_HALF_SIZE + HELD_DZ * float(np.cos(np.radians(HAND_TILT))) + 0.003
# How far the grasp centre has to be from a delivered cube's spot for the episode to count.
HAND_CLEAR = 0.10
# Consecutive steps a cube must stay in hand and clear of the table before it counts as
# carried. Carrying is read from contacts, not from the cube's height: every height proxy
# tried before it was satisfied by something that is not a grasp.
GRASP_HOLD_STEPS = 10
LIFT_REQUIRED = 0.03
SPAWN_CLEARANCE = 0.12
SUCCESS_HOLD_STEPS = 20
# Control stays at 50 Hz with 10 mm steps: contact-rich grasping needs the fast loop.
CONTROL_FRAME_SKIP = 10
POSITION_ACTION_SCALE = 0.01
# How far the commanded pose may get ahead of the hand: three steps' worth. Enough that the
# arm is always pulling, small enough that a target cannot run off to somewhere the arm will
# never reach and leave the controller waiting on it.
TARGET_LEASH = 0.03
# The grip channel is a rate, not a target, so a policy cannot flutter the fingers.
GRIP_RATE = 0.10
# Where the grasp centre may be commanded. The top is where the arm can still hold the
# fingers straight down in the middle of the table. The heights follow the table (0.24 and 0.42
# in the sim setup; rounded so that they are exactly those numbers there).
WORKSPACE_LOW = np.array([-0.08, -0.32, round(TABLE_SURFACE_Z - 0.01, 4)])
WORKSPACE_HIGH = np.array([0.28, 0.32, round(TABLE_SURFACE_Z + 0.17, 4)])
# How far from straight down the fingers may lean, in radians, for a pose to count as reachable.
# A plate leaning away from the cube turns the squeeze into a push down its side (see FINGER_OPEN),
# and the lean margin the closing path leaves at contact is about 0.04, so a pose the solver can
# only reach 0.05 rad off - which this tolerance once allowed - drops the cube while carrying it.
REACH_TILT = 0.02
# The palm-down home pose: grasp centre at (0.20, -0.10, 0.30), at the near edge of the table,
# fingers straight down. From multi-start IK, picked among the poses that give the widest band
# when the controller's own IK is started from them: palm-down needs the shoulder roll near its
# stop, the elbow can bend either way, and the damped-least-squares solve stays on the branch
# it starts on, so the home picks the branch - and with it which side of the table the arm
# serves (here y from -0.28 to 0.05, x from -0.02 to 0.24). The arm starts here and the
# spare joint freedom is pulled back towards it. The real setup's home puts the grasp centre at
# (0.160, -0.1496, table + 0.0998) (scripts/find_home.py --spot 0.16 -0.15 0.10, candidate 6, the hand
# leaning and wrist_pitch at its -90 degree stop), on the side the open hand slides out to from a tower:
# with home on the +y side (0.20, 0.15) the way back passed the tower and 12 of 50 stackings lost their
# top cube (2026-11-12). It is inside HAND_HOLD_X, so the hand never turns towards home's orientation.
HOME_ARM_QPOS = np.array(SETUPS[SETUP]["home"])
# The hand's orientation for grasping. Palm down is the end-effector frame aligned with the world,
# so the finger gap runs along x; the real setup leans it HAND_TILT degrees about that axis.
_TILT = np.radians(HAND_TILT)
PALM_DOWN = np.array([[1.0, 0.0, 0.0],
                      [0.0, np.cos(_TILT), -np.sin(_TILT)],
                      [0.0, np.sin(_TILT), np.cos(_TILT)]])
# The hand turned half a turn about its own axis: fingers 1-2 and finger 3 change sides of the cube and
# the lean stays as it is. The real setup's home pose holds the hand 90 degrees from it and 100 from the
# plain grasp of a square cube (74 from the nearest grasp, a plain one turned 45).
HALF_TURN = np.diag([-1.0, -1.0, 1.0])
# Which grasps of a cube the hand may take besides the one square to its faces (hand_turns), in the
# real setup. The arm reaches the leaning hand turned about the vertical only from about -30 to 0
# degrees over the band, and half-turned from about -15 to +15 (motion1/STATUS.md 2026-11-10 (7)), so a
# cube turned much past that could not be gripped at all: 63 of 100 turned layouts were never picked
# up. HAND_FLIP lets the hand take the half-turned grasp; GRASP_MISALIGN lets the finger gap be up to
# that many degrees off the faces (in GRASP_MISALIGN_STEP steps), the closing fingers squaring the
# cube or holding it by its edges. The fewest degrees off come first, then the least turn from home.
HAND_FLIP = SETUPS[SETUP].get("hand_flip", False)
GRASP_MISALIGN = SETUPS[SETUP].get("grasp_misalign", 0.0)
GRASP_MISALIGN_STEP = 7.5
# The open hand's footprint below the top of a cube standing beside the one it comes down on, at the
# grasp pose, in the hand's frame turned by its yaw: half its extent along the finger gap, and how far
# it reaches either side across it (behind, and on the side the hand leans to). Measured on the finger
# collision boxes of the real setup's leaning hand (half-turned or not), 2026-11-10: +-59 mm along the
# gap, 25 mm behind and 77 mm on the lean side. A cube standing inside it, with its own half extent and
# 10 mm more, is struck by the fingers coming down. None in the sim setup, whose upright hand
# stack_env's CLEAR_DX and CLEAR_DY keep the cubes out of.
HAND_FOOTPRINT = SETUPS[SETUP].get("hand_footprint")
# How high above the top of the highest cube standing the empty hand travels to the next cube (real
# setup). While the leaning hand turns from its home orientation a finger hangs up to 58 mm below the
# grasp centre, and the arm sags up to 30 mm under a target on the move; at the carry height
# (clear_height) the open hand swept the base cube aside in 21 of 50 turned stackings.
EMPTY_CLEAR = SETUPS[SETUP].get("empty_clear")
# Real setup: a reach that stops getting closer to its cube (less than REACH_STALL_PROGRESS in
# REACH_STALL_STEPS control steps, while still farther than REACH_STALL_NEAR from it) is finished in
# joint space instead, to the angles the reach check solves from home, no joint faster than
# JOINT_MOVE_RATE, and handed back to the pose control there. Followed pose by pose from home, the hand
# turning about the vertical took the arm onto the elbow's and shoulder_yaw's stops with the grasp
# centre 30 cm short and the hand 140 degrees off: 11 of 100 turned deliveries timed out reaching.
REACH_RESCUE = SETUPS[SETUP].get("reach_rescue", False)
REACH_STALL_STEPS = 25
REACH_STALL_PROGRESS = 0.010
REACH_STALL_NEAR = 0.02
JOINT_MOVE_RATE = 30.0   # degrees per second

# Each setup has its own reach map; the sim setup's keeps the name it always had.
REACH_FILE = Path(__file__).with_name("reachmap.npz" if SETUP == "sim" else f"reachmap_{SETUP}.npz")


def _load_reach():
    return np.load(REACH_FILE) if REACH_FILE.is_file() else None


_REACH = _load_reach()
if _REACH is not None and _REACH["feasible"].any():
    REACH_XS, REACH_YS = _REACH["xs"], _REACH["ys"]
    REACH_FEASIBLE = _REACH["feasible"]
else:  # before scripts/build_reachmap.py has run, or after it found no cell: every grid cell on the table
    # is allowed, so that the env still builds and the map can be built (an empty map, or the full grid
    # past the real setup's table edge at x 0.20, left the scene unbuildable, 2026-11-12)
    REACH_XS = np.round(np.arange(0.00, 0.261, 0.02), 3)
    REACH_YS = np.round(np.arange(-0.25, 0.251, 0.025), 3)
    REACH_FEASIBLE = np.ones((len(REACH_XS), len(REACH_YS)), dtype=bool) & (REACH_XS <= CUBE_X_MAX)[:, None]
REACH_CELLS = np.argwhere(REACH_FEASIBLE)
# The table is sized from the map: anything beyond it is surface the arm can never use.
if len(REACH_CELLS):
    WORK_X_MIN = float(REACH_XS[REACH_CELLS[:, 0].min()])
    WORK_X_MAX = float(REACH_XS[REACH_CELLS[:, 0].max()])
    WORK_Y_MAX = float(np.abs(REACH_YS[REACH_CELLS[:, 1]]).max())
else:
    WORK_X_MIN, WORK_X_MAX, WORK_Y_MAX = 0.0, 0.26, 0.25
# How far towards home the hand keeps its grasp orientation (_hand_rotation). WORK_X_MAX is a cell
# centre: spots are drawn up to 0.01 past it (_draw_spot, stack_env), and the leaning hand holds its
# grasp centre up to GRASP_DZ sin HAND_TILT further out still. Turning towards the real setup's home
# (x 0.13) from WORK_X_MAX itself left the hand up to 34 degrees off the grasp the layout was checked
# for at a pick and 44 at a drop, and a gripped cube was thrown (motion1/STATUS.md 2026-11-10 (8)).
# The sim setup's home lies inside the band, so its hand never turns towards home.
HAND_HOLD_X = WORK_X_MAX + 0.01 + max(GRASP_DZ, HELD_DZ) * abs(np.sin(np.radians(HAND_TILT)))


def map_signature() -> np.ndarray:
    """Everything the reach map's answer depends on, flattened, so a map built under other
    constants can be told from the current ones. The sim setup's map was built before the setups
    existed and has no room for the shoulder height and the hand's lean, which are fixed there."""
    return np.concatenate([
        [TABLE_SURFACE_Z, OBJECT_SIZE, GRASP_DZ, HELD_DZ, REACH_TILT],
        WORKSPACE_LOW, WORKSPACE_HIGH, HOME_ARM_QPOS,
        [] if SETUP == "sim" else [SHOULDER_Z, HAND_TILT],
    ]).astype(np.float64)


if _REACH is not None:
    if "signature" not in _REACH.files:
        warnings.warn(f"{REACH_FILE.name} carries no build signature; run scripts/build_reachmap.py")
    elif (_REACH["signature"].shape != map_signature().shape
          or not np.allclose(_REACH["signature"], map_signature())):
        warnings.warn(f"{REACH_FILE.name} was built under different constants than the ones now in "
                      "force; run scripts/build_reachmap.py before trusting any layout")


def fold_yaw(degrees: float) -> float:
    """A cube's yaw in degrees, folded to [-45, 45): a cube looks the same every 90 degrees, and this
    is the turn of the hand that needs the least."""
    return float((degrees + 45.0) % 90.0 - 45.0)


def hand_turns(cube_yaw_deg: float) -> list[float]:
    """Hand yaws (degrees) that grip a cube turned cube_yaw_deg: the cube looks the same every
    90 degrees, so the hand may also turn a quarter more either way and close on the other two
    faces. Least turn first. An unturned cube keeps the hand as it always was (0 only), so every
    result without yaw stays what it was."""
    y = fold_yaw(cube_yaw_deg)
    return [0.0] if y == 0.0 else sorted((y, y + 90.0, y - 90.0), key=abs)


def _snap_to_band(xy: np.ndarray) -> np.ndarray:
    """The nearest spot the arm can both pick from and place on."""
    cells = np.column_stack((REACH_XS[REACH_CELLS[:, 0]], REACH_YS[REACH_CELLS[:, 1]]))
    return cells[int(np.argmin(np.linalg.norm(cells - xy, axis=1)))]


# The fixed layout, in the order cubes are used. The band the arm serves is lopsided (home picks
# the elbow branch, and with it y from -0.30 to about 0.05), so the spots are written down and
# then snapped onto the map, which moves them with it if a pose change moves the band. Goals
# stand HAND_CLEAR + 30 mm away from the home pose, so the withdrawal ends with the hand clear.
NOMINAL_OBJECTS = np.array([[0.10, -0.25], [0.00, -0.20]])
NOMINAL_TARGETS = np.array([[0.02, -0.04], [0.10, -0.10]])


def nominal_layout(n: int) -> tuple[np.ndarray, np.ndarray]:
    """Fixed cube and goal spots for n objects, snapped onto the feasible map."""
    objects = np.array([_snap_to_band(s) for s in NOMINAL_OBJECTS[:n]])
    targets = np.array([_snap_to_band(s) for s in NOMINAL_TARGETS[:n]])
    return objects, targets


class JaxGraspEnv(gym.Env[np.ndarray, np.ndarray]):
    """One arm and hand of the final URDF (7 joints + 3 two-joint fingers) at a table.

    The action is the one the scripted controller speaks: a step of the grasp centre's
    position and a rate for the grip. The hand's orientation is held palm-down.
    """
    metadata = {"render_modes": ["human", "rgb_array"], "render_fps": 50}
    n_objects = 2
    # Whether a sampled layout is checked, with IK at the poses the controller will
    # command, before it is used. Off only to measure what the check is worth.
    validate_layouts = True
    # Cube yaw (degrees about the vertical): each cube is drawn in +-cube_yaw_max per episode and
    # the hand grasps the moving cube turned by its yaw. A cube looks the same every 90 degrees, so
    # 45 covers every orientation. 0 (the default) keeps every cube square to the axes.
    cube_yaw_max = 0.0

    def __init__(
        self,
        stage: Stage = "place",
        render_mode: str | None = None,
        max_steps: int | None = None,
        randomization_scale: float = 1.0,
    ):
        super().__init__()
        if stage not in ("place", "randomize"):
            raise ValueError(f"unknown stage: {stage}")
        if not 0.0 <= randomization_scale <= 1.0:
            raise ValueError("randomization_scale must be between 0 and 1")
        self.stage, self.render_mode = stage, render_mode
        self.randomization_scale = randomization_scale
        # What the scripted delivery needs: reaching, coming down on the cube, closing,
        # lifting, crossing, lowering, letting the cube lie down flat, opening and then 20
        # steps of stillness with the hand clear before the task counts it.
        self.max_steps = max_steps if max_steps is not None else EPISODE_STEPS
        self._robot = robot_parts(finger_sliding_friction=PAD_FRICTION, shoulder_z=SHOULDER_Z,
                                  base_yaw_deg=ROBOT_YAW)
        self.model = mujoco.MjModel.from_xml_string(self._scene_xml())
        self.data = mujoco.MjData(self.model)
        self.frame_skip = CONTROL_FRAME_SKIP
        arm, fingers = self._robot.arm_joints, self._robot.finger_joints
        self.arm_joint_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in arm])
        self.finger_joint_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_JOINT, n) for n in fingers])
        controlled = np.concatenate((self.arm_joint_ids, self.finger_joint_ids))
        self.qpos_ids = self.model.jnt_qposadr[controlled]
        self.qvel_ids = self.model.jnt_dofadr[controlled]
        self.arm_actuator_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_act") for n in arm])
        self.finger_actuator_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_ACTUATOR, f"{n}_act") for n in fingers])
        self.site_id = self._id(mujoco.mjtObj.mjOBJ_SITE, "end_effector")
        self.hand_body_id = self._id(mujoco.mjtObj.mjOBJ_BODY, "MG996R")
        self.target_geom_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_GEOM, f"target_{i}") for i in range(self.n_objects)])
        self.object_body_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_BODY, f"object_{i}") for i in range(self.n_objects)])
        self.object_joint_ids = np.array([self._id(mujoco.mjtObj.mjOBJ_JOINT, f"object_{i}_joint") for i in range(self.n_objects)])
        self.object_geom_ids = np.array([self.model.body_geomadr[b] for b in self.object_body_ids])
        self.table_geom_id = self._id(mujoco.mjtObj.mjOBJ_GEOM, "table_top")
        # Every geom of the hand: the finger plates are the only ones that collide, and
        # "a hand geom touches the cube" is how carrying and letting go are told apart.
        self._is_hand_geom = np.zeros(self.model.ngeom, dtype=bool)
        for body in range(self.model.nbody):
            ancestor = body
            while ancestor != 0 and ancestor != self.hand_body_id:
                ancestor = self.model.body_parentid[ancestor]
            if ancestor == self.hand_body_id:
                start = self.model.body_geomadr[body]
                self._is_hand_geom[start : start + self.model.body_geomnum[body]] = True
        self.arm_ranges = self.model.jnt_range[self.arm_joint_ids].copy()
        self.finger_ranges = self.model.jnt_range[self.finger_joint_ids].copy()
        self.action_space = spaces.Box(-1.0, 1.0, shape=(4,), dtype=np.float32)
        # joints 13 + 13 velocities, previous action 4, grasp centre 3, and per object:
        # position 3, velocity 3, hand-to-object 3, object-to-goal 3, goal xy 2, contact 2.
        self.observation_space = spaces.Box(
            -10.0, 10.0, shape=(13 + 13 + 4 + 3 + 16 * self.n_objects,), dtype=np.float32)
        self._renderer = None
        self._viewer = None
        self._ik_data = mujoco.MjData(self.model)
        self._step_count = self._stable_steps = 0
        self._previous_action = np.zeros(4, dtype=np.float64)
        self._grip = 0.0
        self._arm_targets = HOME_ARM_QPOS.copy()
        self._pose_target_position = np.zeros(3, dtype=np.float64)
        self._initial_pose_position = np.zeros(3, dtype=np.float64)
        self._initial_object_positions = np.zeros((self.n_objects, 3), dtype=np.float64)
        self._target_positions = nominal_layout(self.n_objects)[1]
        # Which cube the arm moves this episode, and how high each cube's spot sits.
        # Delivery moves cube 0 onto a spot on the table; stacking moves cube 1 onto cube 0,
        # a cube's height up. Everything that tests "is it on its spot" reads the height
        # from here rather than assuming the table.
        self.moving = 0
        self._target_heights = np.full(self.n_objects, OBJECT_Z)
        self._cube_yaws = np.zeros(self.n_objects)
        self._hand_yaws = np.zeros(self.n_objects)   # the turn chosen for each cube (_grasp_turns)
        self._hand_flips = np.zeros(self.n_objects, dtype=bool)   # and whether it is half-turned
        self._misalign = np.zeros(self.n_objects)    # and how many degrees it is off the cube's faces
        self._joint_goal = None   # arm angles a joint-space move is heading for (start_joint_move)
        self.layout_fallbacks = getattr(self, "layout_fallbacks", 0)
        self._placed_latch = np.zeros(self.n_objects, dtype=bool)
        # The pads' closing curve: where each finger joint goes for a closure g.
        self._rest_grasp_centre = self._home_grasp_centre()
        self._rest_rotation = self._home_hand_rotation()
        self._commanded_rotation, self.hand_turn_lag = self._rest_rotation.copy(), 0.0
        self._contacts = (np.zeros(self.n_objects), np.zeros(self.n_objects, dtype=bool),
                          np.zeros(self.n_objects, dtype=bool))

    def _id(self, kind: mujoco.mjtObj, name: str) -> int:
        value = mujoco.mj_name2id(self.model, kind, name)
        if value < 0:
            raise ValueError(f"model is missing {name}")
        return value

    def _home_grasp_centre(self) -> np.ndarray:
        probe = mujoco.MjData(self.model)
        probe.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        mujoco.mj_kinematics(self.model, probe)
        return probe.site_xpos[self.site_id].copy()

    def _home_hand_rotation(self) -> np.ndarray:
        probe = mujoco.MjData(self.model)
        probe.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        mujoco.mj_kinematics(self.model, probe)
        return probe.site_xmat[self.site_id].reshape(3, 3).copy()

    def grasp_rotation(self) -> np.ndarray:
        """PALM_DOWN, half-turned about the hand's own axis if the grasp chosen for the moving cube is
        (HALF_TURN), and turned about the vertical by its hand yaw, so the finger gap lies across two
        of its faces (or GRASP_MISALIGN at most off them). Exactly PALM_DOWN for an unturned cube in
        the sim setup."""
        yaw = self._hand_yaws[self.moving]
        hand = PALM_DOWN @ HALF_TURN if self._hand_flips[self.moving] else PALM_DOWN
        if yaw == 0.0:
            return hand
        return Rotation.from_euler("z", yaw).as_matrix() @ hand

    def _hand_rotation(self, position) -> np.ndarray:
        """The hand's orientation for a commanded grasp centre: the grasp rotation over the reach
        band and every pose held over it (to HAND_HOLD_X), and turning towards the home pose's own
        orientation between there and home. In the sim setup home lies inside the band, so the hand
        never turns towards home."""
        home_x = self._rest_grasp_centre[0]
        if home_x <= HAND_HOLD_X or position[0] <= HAND_HOLD_X:
            return self.grasp_rotation()
        s = min((position[0] - HAND_HOLD_X) / (home_x - HAND_HOLD_X), 1.0)
        return Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([self.grasp_rotation(), self._rest_rotation])))(s).as_matrix()

    def _turning_hand_rotation(self, position) -> np.ndarray:
        """_hand_rotation, approached at no more than HAND_TURN_RATE from the last commanded
        orientation; hand_turn_lag is what is still left to turn, radians."""
        wanted = self._hand_rotation(position)
        if HAND_TURN_RATE is None:
            self.hand_turn_lag = 0.0
            return wanted
        current = self._commanded_rotation
        angle = Rotation.from_matrix(current.T @ wanted).magnitude()
        step = np.radians(HAND_TURN_RATE) * self.model.opt.timestep * self.frame_skip
        if angle > step:
            wanted = Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([current, wanted])))(step / angle).as_matrix()
        self.hand_turn_lag = max(0.0, angle - step)
        self._commanded_rotation = wanted
        return wanted

    def _scene_xml(self) -> str:
        robot = self._robot
        layout_objects, layout_targets = nominal_layout(self.n_objects)
        # The table runs from 10 cm beyond the farthest cube spots to its near edge (TABLE_NEAR_X).
        near_edge, far_edge = TABLE_NEAR_X, WORK_X_MIN - 0.10
        # A spot is a cell jittered by up to 0.01 (_draw_spot), and the nearest cubes must still stand on the table.
        if WORK_X_MAX + 0.01 + OBJECT_HALF_SIZE > near_edge:
            raise ValueError(f"cube spots reach x = {WORK_X_MAX + 0.01 + OBJECT_HALF_SIZE:.3f}, past the table edge at {near_edge:.3f}")
        table_x, table_half_x = (near_edge + far_edge) / 2, (near_edge - far_edge) / 2
        table_half_y = WORK_Y_MAX + 0.08
        lift = TABLE_SURFACE_Z - 0.25   # the cameras were placed for the sim setup's table
        solref, solimp = robot.contact_solref, robot.contact_solimp
        targets = "".join(
            f'    <geom name="target_{i}" type="cylinder" pos="{layout_targets[i][0]} {layout_targets[i][1]} {TABLE_SURFACE_Z + 0.001}" '
            f'size="{TARGET_TOLERANCE} .001" rgba="{OBJECT_COLORS[i][0]} {OBJECT_COLORS[i][1]} {OBJECT_COLORS[i][2]} .40" '
            f'contype="0" conaffinity="0"/>\n' for i in range(self.n_objects))
        cubes = "".join(
            f'    <body name="object_{i}" pos="{layout_objects[i][0]} {layout_objects[i][1]} {OBJECT_Z}">\n'
            f'      <freejoint name="object_{i}_joint"/>\n'
            f'      <geom type="box" size="{OBJECT_HALF_SIZE} {OBJECT_HALF_SIZE} {OBJECT_HALF_SIZE}" mass="{OBJECT_MASS:g}" '
            f'friction="{CUBE_FRICTION} {CUBE_TORSION} .002" condim="4" contype="1" conaffinity="3" solref="{solref}" solimp="{solimp}" '
            f'rgba="{OBJECT_COLORS[i][0]} {OBJECT_COLORS[i][1]} {OBJECT_COLORS[i][2]} 1"/>\n'
            f'    </body>\n' for i in range(self.n_objects))
        # Fingers 1 and 2 stand side by side on the same side of the cube and are driven by the same
        # command; left independent, the contact loads let one lean further than the other and
        # their pads then press the cube's face with opposing friction, a couple that rolls it.
        equality = ("  <equality>\n" + "".join(
            f'    <joint joint1="finger1_joint{k}" joint2="finger2_joint{k}" polycoef="0 1 0 0 0" solref="0.004 1"/>\n'
            for k in (1, 2)) + "  </equality>\n") if COUPLE_FINGERS else ""
        return f"""<mujoco model="jax arm pick and place">
  <compiler angle="radian" meshdir="{robot.mesh_dir}" balanceinertia="true"/>
  <!-- An elliptic friction cone with a high impratio, not MuJoCo's pyramidal default: on the
       default, friction constraints are as soft as the normal ones and a pressed cube creeps
       out of the fingers. -->
  <option timestep="0.002" gravity="0 0 -9.81" integrator="implicitfast" cone="elliptic" impratio="{IMPRATIO}"/>
  <default>
    <joint damping="0.2" frictionloss="0.05"/>
    <geom contype="0" conaffinity="0"/>
  </default>
  <visual>
    <global azimuth="145" elevation="-20" offwidth="1920" offheight="1080"/>
    <quality shadowsize="4096" offsamples="8"/>
    <map shadowclip=".5" shadowscale=".7"/>
  </visual>
  <asset>
{robot.assets}
    <texture name="floor_tex" type="2d" builtin="checker" width="512" height="512" rgb1=".16 .18 .21" rgb2=".22 .24 .28"/>
    <material name="floor_mat" texture="floor_tex" texrepeat="12 12" reflectance=".08"/>
  </asset>
  <worldbody>
    <light name="key" pos="1.2 .9 2.4" dir="-.4 -.35 -1" diffuse=".7 .68 .65" specular=".25 .25 .25" castshadow="true"/>
    <light name="fill" pos="-.6 -1.2 2.0" dir=".3 .5 -1" diffuse=".28 .3 .34" specular="0 0 0" castshadow="false"/>
    <camera name="overview" pos="-0.9 -1.0 {0.9 + lift:g}" xyaxes="0.75 -0.66 0 0.2 0.23 0.95"/>
    <camera name="side" pos="0.1 -1.3 {0.7 + lift:g}" xyaxes="1 0 0 0 0.25 1"/>
    <camera name="top" pos="0.12 0 {1.6 + lift:g}" xyaxes="0 -1 0 1 0 0"/>
    <geom name="floor" type="plane" size="2 2 .1" material="floor_mat" contype="1" conaffinity="3"/>
    <body name="table" pos="{table_x} 0 {TABLE_SURFACE_Z - 0.025}">
      <geom name="table_top" type="box" size="{table_half_x} {table_half_y} .025" rgba=".45 .32 .2 1"
            friction="1 .01 .001" contype="1" conaffinity="3" solref="{solref}" solimp="{solimp}"/>
      <geom name="table_leg_a" type="box" pos="{table_half_x - 0.03} {table_half_y - 0.03} {-(TABLE_SURFACE_Z - 0.025) / 2}" size=".02 .02 {(TABLE_SURFACE_Z - 0.025) / 2}" rgba=".35 .25 .15 1"/>
      <geom name="table_leg_b" type="box" pos="{table_half_x - 0.03} {-(table_half_y - 0.03)} {-(TABLE_SURFACE_Z - 0.025) / 2}" size=".02 .02 {(TABLE_SURFACE_Z - 0.025) / 2}" rgba=".35 .25 .15 1"/>
      <geom name="table_leg_c" type="box" pos="{-(table_half_x - 0.03)} {table_half_y - 0.03} {-(TABLE_SURFACE_Z - 0.025) / 2}" size=".02 .02 {(TABLE_SURFACE_Z - 0.025) / 2}" rgba=".35 .25 .15 1"/>
      <geom name="table_leg_d" type="box" pos="{-(table_half_x - 0.03)} {-(table_half_y - 0.03)} {-(TABLE_SURFACE_Z - 0.025) / 2}" size=".02 .02 {(TABLE_SURFACE_Z - 0.025) / 2}" rgba=".35 .25 .15 1"/>
    </body>
{targets}{robot.column}
{cubes}  </worldbody>
  <actuator>
{robot.actuators}
  </actuator>
{equality}</mujoco>"""

    # --- Reset and layouts ------------------------------------------------------------------
    def reset(self, *, seed: int | None = None, options: dict | None = None):
        """options, to set the scene instead of drawing it (sim world, metres and degrees):
        {"objects": [[x, y, yaw], ...], "targets": [[x, y], ...]}, one entry per cube. The
        layout is checked like a drawn one (`_servable`) and a ValueError says if the arm cannot
        serve it."""
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        if options and ("objects" in options or "targets" in options):
            objects, targets = self._layout_from(options)
        else:
            # Drawn before the layout, which is checked with the hand turned for them. No draw at
            # 0, so the random stream and every layout stay what they were without yaw.
            limit = np.radians(self.cube_yaw_max)
            self._cube_yaws = (self.np_random.uniform(-limit, limit, self.n_objects) if limit > 0
                               else np.zeros(self.n_objects))
            self._hand_yaws = self._cube_yaws.copy()
            self._hand_flips[:], self._misalign[:] = False, 0.0
            objects, targets = self._sample_layout()
        self._target_positions = targets
        self._target_heights = self._target_heights_for(objects, targets)
        for i, joint_id in enumerate(self.object_joint_ids):
            adr = self.model.jnt_qposadr[joint_id]
            half = self._cube_yaws[i] / 2
            self.data.qpos[adr : adr + 7] = (*objects[i], OBJECT_Z, np.cos(half), 0.0, 0.0, np.sin(half))
        self.model.geom_pos[self.target_geom_ids, :2] = targets
        self._arm_targets = HOME_ARM_QPOS.copy()
        self._commanded_rotation, self.hand_turn_lag = self._rest_rotation.copy(), 0.0
        self._joint_goal = None
        self.data.ctrl[self.arm_actuator_ids] = self._arm_targets
        self._grip = 0.0
        self._set_grip(0.0)
        # The fingers start where their servos are told to be: open.
        self.data.qpos[self.qpos_ids[7:]] = self.data.ctrl[self.finger_actuator_ids]
        self._previous_action.fill(0)
        self._lifted_latch = np.zeros(self.n_objects, dtype=bool)
        self._placed_latch = np.zeros(self.n_objects, dtype=bool)
        self._hold_steps = np.zeros(self.n_objects, dtype=int)
        self._success_latch = False
        self._success_now = False
        self._step_count = self._stable_steps = 0
        mujoco.mj_forward(self.model, self.data)
        self._contacts = self._contact_features()
        self._pose_target_position = self.data.site_xpos[self.site_id].copy()
        self._initial_pose_position = self._pose_target_position.copy()
        self._initial_object_positions = self.data.xpos[self.object_body_ids].copy()
        return self._observation(), self._info(False)

    def _layout_from(self, options: dict) -> tuple[np.ndarray, np.ndarray]:
        """The scene reset(options=...) asks for, checked."""
        objects = np.asarray(options["objects"], dtype=np.float64).reshape(-1, 3)
        targets = np.asarray(options["targets"], dtype=np.float64).reshape(-1, 2)
        if len(objects) != self.n_objects or len(targets) != self.n_objects:
            raise ValueError(f"this task has {self.n_objects} cube(s): give that many objects and targets")
        self._cube_yaws = np.radians([fold_yaw(y) for y in objects[:, 2]])
        self._hand_yaws = self._cube_yaws.copy()
        self._hand_flips[:], self._misalign[:] = False, 0.0
        objects = objects[:, :2].copy()
        if not self._servable(objects, targets):
            raise ValueError(f"the arm cannot serve this layout (cubes {objects.tolist()}, yaw "
                             f"{np.degrees(self._cube_yaws).round(1).tolist()} deg, targets {targets.tolist()})")
        return objects, targets

    def _moving_cubes(self) -> list[int]:
        """Which cubes the arm carries in an episode. One task moves one cube."""
        return [self.moving]

    def _target_heights_for(self, objects, targets) -> np.ndarray:
        """Resting height of each cube's spot: on the table unless the task says otherwise."""
        del objects, targets
        return np.full(self.n_objects, OBJECT_Z)

    def _nominal_layout(self) -> tuple[np.ndarray, np.ndarray]:
        """The fixed layout this task varies around. Overridden where the task needs it."""
        return nominal_layout(self.n_objects)

    def clear_height(self, standing_centre_z: float) -> float:
        """Grasp-centre height at which the hand, carrying a cube, clears the cubes now standing.

        `standing_centre_z` is the centre height of the highest cube the carried one has to
        pass over; the table counts as a cube. A held cube hangs HELD_BELOW_GC under the grasp
        centre, so the centre rides above that cube's top by this and a margin. It is the cubes
        standing *now* that matter, not the tower's final height: asked to clear the finished
        tower from the start, the hand was sent above where the arm can hold the fingers down.
        """
        return max(standing_centre_z, OBJECT_Z) + OBJECT_HALF_SIZE + HELD_BELOW_GC + 0.012

    def standing_height(self, moving: int, heights: np.ndarray) -> float:
        """Centre height of the highest cube other than `moving`, once the cubes before it are
        stacked and the ones after it still lie on the table."""
        others = [heights[j] if j < moving else OBJECT_Z for j in range(self.n_objects) if j != moving]
        return max(others, default=OBJECT_Z)

    def pick_pose(self, spot, height: float = 0.0) -> np.ndarray:
        """Grasp centre that puts the fingers around a cube standing at this spot."""
        return np.array([spot[0], spot[1], OBJECT_Z + height]) + GRASP_DZ * self.grasp_rotation()[:, 2]

    def drop_pose(self, spot, height: float) -> np.ndarray:
        """Grasp centre that sets a held cube on this spot, `height` being its resting centre."""
        return np.array([spot[0], spot[1], height + 0.004]) + HELD_DZ * self.grasp_rotation()[:, 2]

    def _servable(self, objects, targets) -> bool:
        """Can the arm hold every pose this layout asks for?

        The reach map answers only for cell centres at table height, and the sampler jitters
        off the cells, so every layout is checked at the poses it will actually get - the
        grasp on each cube that moves, the clear-air pose above it, the release at the
        height it goes to and the carry pose above that - with the hand inside the workspace.
        """
        heights = self._target_heights_for(objects, targets)
        moving = self.moving
        try:
            for i in self._moving_cubes():
                self.moving = i
                # The first grasp (least off the faces, then least turn) that holds every pose and
                # comes down clear of the other cubes; the cube keeps it.
                for turn, half_turned, off in self._grasp_turns(i):
                    self._hand_yaws[i], self._hand_flips[i], self._misalign[i] = np.radians(turn), half_turned, off
                    if (self._poses_servable(i, objects, targets, heights)
                            and self._footprint_clear(i, objects, targets)):
                        break
                else:
                    self._hand_yaws[i], self._hand_flips[i], self._misalign[i] = self._cube_yaws[i], False, 0.0
                    return False
            return True
        finally:
            self.moving = moving

    def _grasp_turns(self, i) -> list[tuple[float, bool, float]]:
        """The grasps to try on cube i, as (hand yaw in degrees, half-turned, degrees off its faces).

        The sim setup: square to its faces only (hand_turns), least turn first. The real setup also
        has the half-turned hand (HAND_FLIP) and grasps up to GRASP_MISALIGN off the faces; fewest
        degrees off first, then the least turn from the home pose's hand, which is the time and the
        sweep of the turn on the way to the cube."""
        cube = np.degrees(self._cube_yaws[i])
        if not (HAND_FLIP or GRASP_MISALIGN):
            return [(turn, False, 0.0) for turn in hand_turns(cube)]
        y = fold_yaw(cube)
        offs = np.arange(-GRASP_MISALIGN, GRASP_MISALIGN + 1e-9, GRASP_MISALIGN_STEP) if GRASP_MISALIGN else [0.0]
        flips = (False, True) if HAND_FLIP else (False,)
        options = {(round(float(face + off), 6), flip, round(float(off), 6))
                   for face in (y, y + 90.0, y - 90.0) for off in offs for flip in flips}

        def turn_from_home(option):
            hand = PALM_DOWN @ HALF_TURN if option[1] else PALM_DOWN
            rotation = Rotation.from_euler("z", option[0], degrees=True).as_matrix() @ hand
            return float(Rotation.from_matrix(self._rest_rotation.T @ rotation).magnitude())
        return sorted(options, key=lambda o: (abs(o[2]), round(turn_from_home(o), 9), o[0], o[1]))

    def _footprint_clear(self, i, objects, targets) -> bool:
        """Does the open hand, coming down on cube i in the grasp chosen for it, stay clear of every
        other cube standing then (HAND_FOOTPRINT, plus that cube's half extent across the hand's axes
        and 10 mm)? The cubes moved before i stand on their spots, the rest where they spawned. Always
        true in the sim setup, whose stacking keeps the cubes apart with stack_env's spacing."""
        if HAND_FOOTPRINT is None:
            return True
        half_gap, behind, ahead = HAND_FOOTPRINT
        c, s = np.cos(-self._hand_yaws[i]), np.sin(-self._hand_yaws[i])
        moved = [j for j in self._moving_cubes() if j < i]
        for j in range(self.n_objects):
            if j == i:
                continue
            r = (targets[j] if j in moved else objects[j])[:2] - np.asarray(objects[i])[:2]
            x, y = c * r[0] - s * r[1], s * r[0] + c * r[1]
            turn = self._cube_yaws[j] - self._hand_yaws[i]
            half = OBJECT_HALF_SIZE * (abs(np.cos(turn)) + abs(np.sin(turn))) + 0.010
            if not (abs(x) >= half_gap + half or y >= ahead + half or y <= -(behind + half)):
                return False
        return True

    def empty_hand_height(self, standing_centre_z: float) -> float:
        """Grasp-centre height the empty hand travels at to the next cube, over the cubes standing then
        (EMPTY_CLEAR above the highest one's top); 0 where nothing else stands or in the sim setup."""
        if EMPTY_CLEAR is None or self.n_objects < 2:
            return 0.0
        return standing_centre_z + OBJECT_HALF_SIZE + EMPTY_CLEAR

    def _poses_servable(self, i, objects, targets, heights) -> bool:
        from humanoid_grasp_rl.demonstration import TRAVEL_CLEAR
        clear_z = self.clear_height(self.standing_height(i, heights))
        pick = self.pick_pose(objects[i])
        drop = self.drop_pose(targets[i], heights[i])
        travel = max(pick[2] + TRAVEL_CLEAR, drop[2] + TRAVEL_CLEAR, clear_z)
        approach = max(pick[2] + TRAVEL_CLEAR, clear_z, self.empty_hand_height(self.standing_height(i, heights)))
        # Every pose the state machine holds: the grasp and the clear-air pose above it, the
        # carry height over the pick (where the lift ends) and over the spot, and the release.
        poses = (pick, np.array([pick[0], pick[1], approach]),
                 np.array([pick[0], pick[1], travel]),
                 drop, np.array([drop[0], drop[1], travel]))
        for position in poses:
            if np.any(position < WORKSPACE_LOW) or np.any(position > WORKSPACE_HIGH):
                return False
            if not self.reachable(position):
                return False
        return True

    def reachable(self, position) -> bool:
        """Can the arm hold the grasp centre here, in the grasp rotation, from the home pose?"""
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        self._ik_data.qvel.fill(0.0)
        ok, _, _ = solve_site_pose(
            self.model, self._ik_data, self.site_id, self.arm_joint_ids, position, self.grasp_rotation(),
            HOME_ARM_QPOS, position_tolerance=0.005, rotation_tolerance=REACH_TILT, max_iterations=120)
        return ok

    def holds_path(self, points) -> bool:
        """Can the pose control follow these grasp centres in turn? Solved as it solves them: from
        its current arm angles, each point seeded with the last, in the orientation it will command
        there (_hand_rotation). reachable() solves each pose afresh from home, on another joint
        branch: on the real setup's table it passed withdrawals along which the arm, continuing
        from the release, ran its elbow into the stop and the hand sank onto the cube."""
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = self._arm_targets
        for p in points:
            self._ik_data.qvel.fill(0.0)
            ok, _, _ = solve_site_pose(
                self.model, self._ik_data, self.site_id, self.arm_joint_ids, p, self._hand_rotation(p),
                HOME_ARM_QPOS, position_tolerance=0.005, rotation_tolerance=REACH_TILT, max_iterations=120)
            if not ok:
                return False
        return True

    def _reachable_cell(self, xy: np.ndarray) -> bool:
        """Is this spot inside the IK-verified band, for both grasping and releasing?"""
        i = int(np.argmin(np.abs(REACH_XS - xy[0])))
        j = int(np.argmin(np.abs(REACH_YS - xy[1])))
        return bool(REACH_FEASIBLE[i, j])

    def _draw_spot(self) -> np.ndarray:
        """A cell of the feasible map, jittered inside the cell so spots stay continuous."""
        i, j = REACH_CELLS[self.np_random.integers(len(REACH_CELLS))]
        return np.array([
            REACH_XS[i] + self.np_random.uniform(-0.01, 0.01),
            REACH_YS[j] + self.np_random.uniform(-0.0125, 0.0125),
        ])

    def _sample_layout(self) -> tuple[np.ndarray, np.ndarray]:
        """Cube and goal spots for this episode, interpolated from the fixed layout.

        randomization_scale blends the drawn spot with the nominal one, so scale 0 is the
        fixed scene and scale 1 spreads over the whole reachable band while every
        intermediate value stays reachable too.
        """
        nominal_objects, nominal_targets = self._nominal_layout()
        if self.stage != "randomize":
            if HAND_FLIP or GRASP_MISALIGN:
                self._servable(nominal_objects, nominal_targets)   # its grasp chosen, as for a fallback
            return nominal_objects, nominal_targets
        blend = self.randomization_scale
        for _ in range(50):
            objects = np.array([
                nominal_objects[i] + blend * (self._draw_spot() - nominal_objects[i])
                for i in range(self.n_objects)
            ])
            targets = np.array([
                nominal_targets[i] + blend * (self._draw_spot() - nominal_targets[i])
                for i in range(self.n_objects)
            ])
            spots = np.vstack((objects, targets))
            gaps = [np.linalg.norm(spots[a] - spots[b])
                    for a in range(len(spots)) for b in range(a + 1, len(spots))]
            # Cubes start clear of the hand, and goals clear of where the hand ends up: the
            # withdrawal goes home, and the task counts a delivery only once the hand is
            # HAND_CLEAR from its spot.
            home = self._rest_grasp_centre[:2]
            cubes_clear = np.linalg.norm(objects - home, axis=1).min() >= SPAWN_CLEARANCE
            goals_clear = np.linalg.norm(targets - home, axis=1).min() >= HAND_CLEAR + 0.03
            # Blending can land between two feasible cells, and the nearest-cell lookup
            # cannot tell, so the poses the layout asks for are solved outright.
            if (min(gaps) >= TARGET_MIN_SEPARATION and cubes_clear and goals_clear
                    and all(self._reachable_cell(p) for p in spots)
                    and (not self.validate_layouts or self._servable(objects, targets))):
                return objects, targets
        self.layout_fallbacks += 1
        if HAND_FLIP or GRASP_MISALIGN:
            # The fixed layout gets its grasp chosen too; left as drawn, a turned cube there was gripped
            # with the hand turned by the cube's own yaw, often where the arm cannot hold it.
            self._servable(nominal_objects, nominal_targets)
        return nominal_objects, nominal_targets

    # --- Contacts and the task ----------------------------------------------------------------
    def _resting_on_support(self, i: int) -> bool:
        """Is cube i held up from below - by the table or another cube - rather than merely
        touching something?

        `supported` in the contact features is any non-hand contact at all, but a controller
        deciding the cube has landed needs more: the side of a tower or a brush against the
        forearm is a contact too. This asks for a contact with the table or a cube whose
        normal is near vertical and whose point is below the cube's centre.
        """
        own = self.object_geom_ids[i]
        centre_z = float(self.data.xpos[self.object_body_ids[i], 2])
        for k in range(self.data.ncon):
            contact = self.data.contact[k]
            if own not in (contact.geom1, contact.geom2):
                continue
            other = contact.geom2 if contact.geom1 == own else contact.geom1
            if self._is_hand_geom[other]:
                continue
            if other != self.table_geom_id and self.model.geom_bodyid[other] not in self.object_body_ids:
                continue
            if abs(contact.frame[2]) > 0.7 and contact.pos[2] < centre_z:
                return True
        return False

    def finger_targets(self, closure: float) -> np.ndarray:
        """The six finger joint targets (finger1 j1, j2, finger2 j1, j2, finger3 j1, j2)."""
        g = float(np.clip(closure, 0.0, 1.0))
        pose = FINGER_OPEN + g * (FINGER_CLOSED - FINGER_OPEN)
        return np.tile(pose, 3)

    def _set_grip(self, closure: float) -> None:
        self.data.ctrl[self.finger_actuator_ids] = self.finger_targets(closure)

    @property
    def grasp_centre(self) -> np.ndarray:
        return self.data.site_xpos[self.site_id]

    def _update_arm_targets(self) -> None:
        """Solve the arm angles for the commanded pose, continuing from the last solve.

        Seeded from the measured joint angles instead, the solver restarts every step from
        wherever the arm has drooped to, and the spare joint freedom wanders - the same
        commanded pose stops producing the same angles, so holding still does not hold still.
        During a joint-space move (start_joint_move) the angles step towards its goal instead.
        """
        if self._joint_goal is not None:
            self._step_joint_move()
            return
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = self._arm_targets
        self._ik_data.qvel.fill(0.0)
        solve_site_pose(
            self.model, self._ik_data, self.site_id, self.arm_joint_ids,
            self._pose_target_position, self._turning_hand_rotation(self._pose_target_position), HOME_ARM_QPOS,
            position_tolerance=0.0005, rotation_tolerance=0.004, max_iterations=60)
        self._arm_targets = self._ik_data.qpos[self.qpos_ids[:7]].copy()

    @property
    def joint_moving(self) -> bool:
        return self._joint_goal is not None

    def start_joint_move(self, position) -> None:
        """Move the arm in joint space to the angles that hold the grasp centre at `position` in the
        grasp rotation, solved from home as reachable() solves them, no joint faster than
        JOINT_MOVE_RATE. The pose control takes over again where the angles are reached. The move goes
        to the solver's last angles even where the solve stops short of its tolerances: what it is for
        is leaving the joint branch the reach is stuck on. Refusing such a move turned three turned
        deliveries of seeds 100-199 (111, 126, 127) from clean into three reach timeouts; where the
        angles are far off (32-42 mm on a re-pick) the reach just times out as it would have."""
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        self._ik_data.qvel.fill(0.0)
        solve_site_pose(
            self.model, self._ik_data, self.site_id, self.arm_joint_ids, position, self.grasp_rotation(),
            HOME_ARM_QPOS, position_tolerance=0.0005, rotation_tolerance=0.004, max_iterations=400)
        self._joint_goal = self._ik_data.qpos[self.qpos_ids[:7]].copy()

    def cancel_joint_move(self) -> None:
        self._joint_goal = None

    def _step_joint_move(self) -> None:
        """One control step of a joint-space move. The commanded pose and the hand's commanded
        orientation follow the arm, so the pose control resumes from where the arm is."""
        step = np.radians(JOINT_MOVE_RATE) * self.model.opt.timestep * self.frame_skip
        delta = self._joint_goal - self._arm_targets
        largest = float(np.abs(delta).max())
        if largest > step:
            delta = delta * (step / largest)
        else:
            self._joint_goal = None
        self._arm_targets = self._arm_targets + delta
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = self._arm_targets
        mujoco.mj_kinematics(self.model, self._ik_data)
        self._pose_target_position = self._ik_data.site_xpos[self.site_id].copy()
        self._commanded_rotation = self._ik_data.site_xmat[self.site_id].reshape(3, 3).copy()
        self.hand_turn_lag = float(Rotation.from_matrix(
            self._commanded_rotation.T @ self._hand_rotation(self._pose_target_position)).magnitude())

    def step(self, action: np.ndarray):
        action = np.clip(np.asarray(action, dtype=np.float64), -1.0, 1.0)
        # The target moves from where it was, not from where the hand got to: read off the
        # hand, a zero action asks for wherever gravity has just pulled it and the arm walks
        # downwards. Integrating the target leaves a standing error that does not grow.
        self._target_request = self._pose_target_position + POSITION_ACTION_SCALE * action[:3]
        self._pose_target_position = np.clip(self._target_request, WORKSPACE_LOW, WORKSPACE_HIGH)
        # Keep the target on a short leash: integrating it stops the arm drifting down, but
        # it also lets the target walk away from a hand that cannot follow - into a corner the
        # arm will not reach, or through a cube it is pressing on. Held within TARGET_LEASH of
        # the hand, the pose the arm chases stays reachable and the force it presses with
        # stays bounded at roughly the servo gain times the leash.
        offset = self._pose_target_position - self.grasp_centre
        stretch = float(np.linalg.norm(offset))
        if stretch > TARGET_LEASH:
            self._pose_target_position = self.grasp_centre + offset * (TARGET_LEASH / stretch)
        self._update_arm_targets()
        # Hold the arm up against gravity with a feedforward term rather than with position
        # error: a position servo answers gravity by sitting low, and re-reading the target
        # from the drooping hand turns that standing error into a drift. Only the arm gets it;
        # a cube that floated would not be a cube.
        self.data.qfrc_applied[self.qvel_ids[:7]] = self.data.qfrc_bias[self.qvel_ids[:7]]
        self.data.ctrl[self.arm_actuator_ids] = self._arm_targets
        self._grip = float(np.clip(self._grip + GRIP_RATE * action[3], 0.0, 1.0))
        self._set_grip(self._grip)
        mujoco.mj_step(self.model, self.data, nstep=self.frame_skip)
        self._step_count += 1

        self._update_latches()
        self._previous_action = action.copy()

        success = self._success()
        self._success_now = success
        self._stable_steps = self._stable_steps + 1 if success else 0
        # The episode runs to its limit either way; success is latched once it has held for
        # SUCCESS_HOLD_STEPS, and that step is the only one that pays.
        reward = 0.0
        if self._stable_steps >= SUCCESS_HOLD_STEPS and not self._success_latch:
            self._success_latch = True
            reward = 1.0
        dropped = bool(np.any(self.data.xpos[self.object_body_ids, 2] < TABLE_SURFACE_Z - 0.10))
        terminated = dropped
        truncated = self._step_count >= self.max_steps
        if self.render_mode == "human":
            self.render()
        info = self._info(self._success_latch)
        info.update({"placed_objects": int(self._placed_latch.sum()),
                     "grasped_objects": int(self._lifted_latch.sum())})
        return self._observation(), float(reward), terminated, truncated, info

    def _contact_features(self) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Per cube: how opposed the hand contacts are, and what is touching it.

        `opposition` is the deepest pair of inward contact normals from hand geoms, mapped
        to [0, 1]: a hand laid against a face measures 0, a hand closed around the cube
        measures 1. `supported` means something that is not a hand - the table, the other
        cube - is also in contact, which is what separates carrying a cube from pushing or
        stacking it.
        """
        opposition = np.zeros(self.n_objects)
        touching = np.zeros(self.n_objects, dtype=bool)
        supported = np.zeros(self.n_objects, dtype=bool)
        if self.data.ncon == 0:
            return opposition, touching, supported
        pairs = self.data.contact.geom
        normals = self.data.contact.frame[:, :3]
        for i, own in enumerate(self.object_geom_ids):
            mine = pairs[:, 0] == own
            rows = mine | (pairs[:, 1] == own)
            if not rows.any():
                continue
            other = np.where(mine, pairs[:, 1], pairs[:, 0])[rows]
            is_hand = self._is_hand_geom[other]
            touching[i] = bool(is_hand.any())
            supported[i] = bool((~is_hand).any())
            if is_hand.sum() < 2:
                continue
            # MuJoCo aims the normal from the first geom to the second; flip the ones that
            # start at the cube so every normal points into it.
            inward = np.where(mine[rows, None], -normals[rows], normals[rows])[is_hand]
            opposition[i] = float(np.clip(-(inward @ inward.T).min(), 0.0, 1.0))
        return opposition, touching, supported

    def _update_latches(self) -> None:
        """Advance what the task remembers about each cube - that it was carried, and that it
        was set on its spot - once per step, from contacts."""
        objects = self.data.xpos[self.object_body_ids]
        _, touching, supported = self._contacts = self._contact_features()
        # Carried, not nudged: in hand and clear of the table for a fifth of a second.
        carried = (touching & ~supported) & (objects[:, 2] - OBJECT_Z > LIFT_REQUIRED)
        self._hold_steps = np.where(carried, self._hold_steps + 1, 0)
        self._lifted_latch |= self._hold_steps >= GRASP_HOLD_STEPS
        # Hysteresis: a cube counts as placed once it settles inside the tighter radius and
        # only stops counting outside the looser one, so the flag does not chatter while the
        # fingers come off it.
        mask, settled = self._placed_object_mask()
        enter = mask & settled & (np.linalg.norm(objects[:, :2] - self._target_positions, axis=1)
                                  < TARGET_TOLERANCE * 0.8)
        self._placed_latch = (self._placed_latch & mask) | enter

    def _success(self) -> bool:
        v = self.data.cvel[self.object_body_ids, 3:]
        _, touching, _ = self._contacts
        # The placed mask already carries "was carried here, is on its spot, is on the table
        # and nothing is holding it". What success adds is that the hand is out of the shot.
        goals = np.column_stack((self._target_positions, self._target_heights))
        hand_clear = bool(np.all(np.linalg.norm(self.grasp_centre[None, :] - goals, axis=1) > HAND_CLEAR))
        # Let go means let go: no hand geom on any cube. Distance alone was satisfiable with
        # the fingers still touching. And flat on a face, not merely within reach of its
        # spot: a cube left on an edge is not delivered, whatever the latch says.
        frames = self.data.xmat[self.object_body_ids].reshape(-1, 3, 3)
        flat = np.abs(frames[:, 2, :]).max(axis=1) > np.cos(np.radians(4.0))
        return bool(np.all(self._placed_latch) and hand_clear and not touching.any()
                    and np.all(flat) and np.all(np.linalg.norm(v, axis=1) < 0.08))

    def _placed_object_mask(self) -> tuple[np.ndarray, np.ndarray]:
        """Carried here, then rested on its spot: the cube is held up from below, on its spot.

        Resting on its spot counts whether or not the hand is still around it; the hand has
        to let go for the episode to succeed, but `_success` asks that separately. Two
        conditions, because stillness gates entry only: staying placed tolerates more
        movement than entering it, so the flag does not chatter as the fingers come off.
        """
        positions = self.data.xpos[self.object_body_ids]
        speeds = np.linalg.norm(self.data.cvel[self.object_body_ids, 3:], axis=1)
        held_up = np.array([self._resting_on_support(i) for i in range(self.n_objects)])
        resting = (
            self._lifted_latch
            & held_up
            & (speeds < 0.25)
            & (np.linalg.norm(positions[:, :2] - self._target_positions, axis=1) < TARGET_TOLERANCE)
            & (np.abs(positions[:, 2] - self._target_heights) < OBJECT_HALF_SIZE)
        )
        return resting, speeds < 0.08

    def _observation(self) -> np.ndarray:
        ranges = np.vstack((self.arm_ranges, self.finger_ranges))
        midpoint = ranges.mean(axis=1)
        half_range = np.maximum((ranges[:, 1] - ranges[:, 0]) * 0.5, 1e-6)
        joint_position = (self.data.qpos[self.qpos_ids] - midpoint) / half_range
        joint_velocity = self.data.qvel[self.qvel_ids] / 10.0
        object_positions = self.data.xpos[self.object_body_ids]
        # Linear velocity belongs in the observation: there is no frame stacking, so without
        # it the policy cannot tell a rising cube from a falling one.
        object_velocities = self.data.cvel[self.object_body_ids, 3:]
        # Relative vectors as well as absolute positions: everything this task turns on is a
        # difference - hand to cube, cube to goal.
        hand_to_object = object_positions - self.grasp_centre[None, :]
        goals_xyz = np.column_stack((self._target_positions, self._target_heights))
        object_to_goal = goals_xyz - object_positions
        opposition, touching, _ = self._contacts
        obs = np.concatenate((
            joint_position, joint_velocity, self._previous_action, self.grasp_centre / 2.0,
            np.column_stack((
                object_positions / 2.0, object_velocities / 2.0, hand_to_object / 2.0,
                object_to_goal / 2.0, self._target_positions / 2.0,
                opposition, touching.astype(np.float64))).reshape(-1),
        ))
        return np.clip(obs, -10.0, 10.0).astype(np.float32)

    def _info(self, success: bool) -> dict:
        # `is_success` latches the first time the task is done; `success_now` says whether it
        # still holds at this step, which is what a finished tower has to show.
        return {"is_success": success, "success_now": bool(getattr(self, "_success_now", False)),
                "stage": self.stage, "stable_steps": self._stable_steps}

    def render(self):
        if self.render_mode == "rgb_array":
            if self._renderer is None:
                self._renderer = mujoco.Renderer(self.model, 480, 640)
            self._renderer.update_scene(self.data)
            return self._renderer.render()
        if self.render_mode == "human":
            if self._viewer is None:
                from mujoco import viewer
                self._viewer = viewer.launch_passive(self.model, self.data)
            self._viewer.sync()

    def close(self):
        if self._renderer is not None:
            self._renderer.close()
        if self._viewer is not None:
            self._viewer.close()
