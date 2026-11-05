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
# "real": the team's setup as measured (2026-11-04), the shoulder axis 1207.5 mm and the table top
# 735 mm above the floor, so the stand floats 357.5 mm up. 472.5 mm below its shoulder the arm
# cannot hold the fingers straight down anywhere (0 usable cells over a 0.74 x 1.0 m grid), so the
# hand leans 40 degrees about its finger-gap axis. One of fingers 1 and 2 then hangs 36 mm below
# the grasp centre, the grasp moves 21.5 mm up the hand's axis to keep it off the table, and a
# carried cube slips out: it is the scene to try a new hand in (motion1/STATUS.md).
# d is D, the drop from the shoulder rotation axis to the table top; the real 0.4725 is CAD's 1207.5
# - 735, taken at the shoulder motor's corner rather than its axis, so a measured D replaces it (the
# reach band depends on it directly). cube_size and cube_mass: the real cube is 50 mm (user,
# 2026-11-05), its mass not measured (0.05 kg). stand_surface_x is the stand's surface facing
# the table at the table top (the stand, the URDF's base_link mesh, narrows upwards), read from the
# mesh; home is the arm's rest pose (HOME_ARM_QPOS, found by scripts/find_home.py's criterion).
SETUPS = {
    "sim": dict(table_z=0.25, d=0.60, cube_size=0.05, cube_mass=0.05, stand_surface_x=0.4085,
                hand_tilt=0.0, grasp_dz=0.0,
                home=(-0.3266, 0.1621, -0.0089, 1.5628, -0.1595, -1.2349, -0.0079)),
    "real": dict(table_z=0.735, d=0.4725, cube_size=0.05, cube_mass=0.05, stand_surface_x=0.4297,
                 hand_tilt=-40.0, grasp_dz=0.0215,
                 home=(0.1010, 0.3778, 0.1976, 1.8326, -0.7061, -1.5708, -0.0016)),
}
SETUP = os.environ.get("JAX_SETUP", "sim")
if SETUP not in SETUPS:
    raise ValueError(f"JAX_SETUP={SETUP!r}: the setups are {', '.join(SETUPS)}")
TABLE_SURFACE_Z = SETUPS[SETUP]["table_z"]
D = SETUPS[SETUP]["d"]
SHOULDER_Z = round(TABLE_SURFACE_Z + D, 6)   # rounded: exactly 0.85 and 1.2075
# The table's near edge stands STAND_GAP from the robot's stand, as on the real setup (the
# user's figure, 2026-11-04, confirmed 2026-11-05; sizing the table from the reach map alone had
# left 68.5 mm).
STAND_GAP = 0.10
STAND_SURFACE_X = SETUPS[SETUP]["stand_surface_x"]
# 50 mm, not the 30 mm the pedestal demo in motion2 uses. The finger plates reach only 17 mm
# below the grasp centre, so on a table the pads cannot go below a cube's bottom face and
# meet it over its upper part; a 30 mm cube then slips or is shot out, a 40 mm one holds in
# four of five closings, a 50 mm one in five of five. Set per setup (cube_size, cube_mass).
OBJECT_SIZE = SETUPS[SETUP]["cube_size"]
OBJECT_MASS = SETUPS[SETUP]["cube_mass"]
OBJECT_HALF_SIZE = OBJECT_SIZE / 2
OBJECT_Z = TABLE_SURFACE_Z + OBJECT_HALF_SIZE
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
# cube (+2 degrees at contact, +6 at g = 1), so the squeeze holds the cube up instead.
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
# in 3.10.0), so the finger pads get the same value (robot_parts in __init__), and the cube's
# contact with the table then takes the table's 1. It was 2.0 until 2026-11-04; at 0.4 a carried
# cube slips out of the pads more often (see CROSS_RATE in demonstration.py).
CUBE_FRICTION = 0.4
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
# serves (here y from -0.30 to about 0.05, x from -0.04 to 0.24). The arm starts here and the
# spare joint freedom is pulled back towards it. The real setup's home puts the grasp centre at
# (0.20, -0.30, table + 0.10), found by the same kind of multi-start search with the hand leaning.
HOME_ARM_QPOS = np.array(SETUPS[SETUP]["home"])
# The hand's orientation for grasping. Palm down is the end-effector frame aligned with the world,
# so the finger gap runs along x; the real setup leans it HAND_TILT degrees about that axis.
_TILT = np.radians(HAND_TILT)
PALM_DOWN = np.array([[1.0, 0.0, 0.0],
                      [0.0, np.cos(_TILT), -np.sin(_TILT)],
                      [0.0, np.sin(_TILT), np.cos(_TILT)]])

# Each setup has its own reach map; the sim setup's keeps the name it always had.
REACH_FILE = Path(__file__).with_name("reachmap.npz" if SETUP == "sim" else f"reachmap_{SETUP}.npz")


def _load_reach():
    return np.load(REACH_FILE) if REACH_FILE.is_file() else None


_REACH = _load_reach()
if _REACH is not None:
    REACH_XS, REACH_YS = _REACH["xs"], _REACH["ys"]
    REACH_FEASIBLE = _REACH["feasible"]
else:  # before scripts/build_reachmap.py has run: every grid cell is allowed
    REACH_XS = np.round(np.arange(0.00, 0.261, 0.02), 3)
    REACH_YS = np.round(np.arange(-0.25, 0.251, 0.025), 3)
    REACH_FEASIBLE = np.ones((len(REACH_XS), len(REACH_YS)), dtype=bool)
REACH_CELLS = np.argwhere(REACH_FEASIBLE)
# The table is sized from the map: anything beyond it is surface the arm can never use.
if len(REACH_CELLS):
    WORK_X_MIN = float(REACH_XS[REACH_CELLS[:, 0].min()])
    WORK_X_MAX = float(REACH_XS[REACH_CELLS[:, 0].max()])
    WORK_Y_MAX = float(np.abs(REACH_YS[REACH_CELLS[:, 1]]).max())
else:
    WORK_X_MIN, WORK_X_MAX, WORK_Y_MAX = 0.0, 0.26, 0.25


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
        self.max_steps = max_steps if max_steps is not None else 700
        self._robot = robot_parts(finger_sliding_friction=CUBE_FRICTION, shoulder_z=SHOULDER_Z)
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
        self.layout_fallbacks = getattr(self, "layout_fallbacks", 0)
        self._placed_latch = np.zeros(self.n_objects, dtype=bool)
        # The pads' closing curve: where each finger joint goes for a closure g.
        self._rest_grasp_centre = self._home_grasp_centre()
        self._rest_rotation = self._home_hand_rotation()
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
        """PALM_DOWN turned about the vertical by the moving cube's yaw, so the finger gap lies
        across the cube's faces. Exactly PALM_DOWN for an unturned cube."""
        yaw = self._cube_yaws[self.moving]
        if yaw == 0.0:
            return PALM_DOWN
        return Rotation.from_euler("z", yaw).as_matrix() @ PALM_DOWN

    def _hand_rotation(self, position) -> np.ndarray:
        """The hand's orientation for a commanded grasp centre: the grasp rotation over the reach
        band, and turning towards the home pose's own orientation between the band's near edge and
        home. In the sim setup home lies inside the band, so the hand never turns towards home."""
        home_x = self._rest_grasp_centre[0]
        if home_x <= WORK_X_MAX or position[0] <= WORK_X_MAX:
            return self.grasp_rotation()
        s = min((position[0] - WORK_X_MAX) / (home_x - WORK_X_MAX), 1.0)
        return Slerp([0.0, 1.0], Rotation.from_matrix(np.stack([self.grasp_rotation(), self._rest_rotation])))(s).as_matrix()

    def _scene_xml(self) -> str:
        robot = self._robot
        layout_objects, layout_targets = nominal_layout(self.n_objects)
        # The table runs from 10 cm beyond the farthest cube spots to STAND_GAP short of the stand.
        near_edge, far_edge = STAND_SURFACE_X - STAND_GAP, WORK_X_MIN - 0.10
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
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        self.data.qpos[self.qpos_ids[:7]] = HOME_ARM_QPOS
        # Drawn before the layout, which is checked with the hand turned for them. No draw at 0,
        # so the random stream and every layout stay what they were without yaw.
        limit = np.radians(self.cube_yaw_max)
        self._cube_yaws = (self.np_random.uniform(-limit, limit, self.n_objects) if limit > 0
                           else np.zeros(self.n_objects))
        objects, targets = self._sample_layout()
        self._target_positions = targets
        self._target_heights = self._target_heights_for(objects, targets)
        for i, joint_id in enumerate(self.object_joint_ids):
            adr = self.model.jnt_qposadr[joint_id]
            half = self._cube_yaws[i] / 2
            self.data.qpos[adr : adr + 7] = (*objects[i], OBJECT_Z, np.cos(half), 0.0, 0.0, np.sin(half))
        self.model.geom_pos[self.target_geom_ids, :2] = targets
        self._arm_targets = HOME_ARM_QPOS.copy()
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
        from humanoid_grasp_rl.demonstration import TRAVEL_CLEAR
        joints = self.arm_joint_ids
        heights = self._target_heights_for(objects, targets)
        for i in self._moving_cubes():
            clear_z = self.clear_height(self.standing_height(i, heights))
            pick = self.pick_pose(objects[i])
            drop = self.drop_pose(targets[i], heights[i])
            travel = max(pick[2] + TRAVEL_CLEAR, drop[2] + TRAVEL_CLEAR, clear_z)
            # Every pose the state machine holds: the grasp and the clear-air pose above it, the
            # carry height over the pick (where the lift ends) and over the spot, and the release.
            poses = (pick, np.array([pick[0], pick[1], max(pick[2] + TRAVEL_CLEAR, clear_z)]),
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
        """
        self._ik_data.qpos[:] = self.data.qpos
        self._ik_data.qpos[self.qpos_ids[:7]] = self._arm_targets
        self._ik_data.qvel.fill(0.0)
        solve_site_pose(
            self.model, self._ik_data, self.site_id, self.arm_joint_ids,
            self._pose_target_position, self._hand_rotation(self._pose_target_position), HOME_ARM_QPOS,
            position_tolerance=0.0005, rotation_tolerance=0.004, max_iterations=60)
        self._arm_targets = self._ik_data.qpos[self.qpos_ids[:7]].copy()

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
