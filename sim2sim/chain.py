"""Jungwoo's one-arm chain (JaxIsaacLab artifacts/deploy_2026-11-05) run in MuJoCo.

Hanging arm -> raise (scripted path + lookup tables, 7.1) -> wait until still (7.2) -> reach in 4.5 cm (7.3) ->
grasp_lift_23025 policy with the scripted close force loop and lift IK (7.4-7.6) -> L return (7.7) -> done (7.8).
Section numbers are deploy/README.md's. Every rule follows that README; where it was silent the Isaac code it names
(render_front_grasp.py, v2_front_*.py) was read and is cited at the spot. Joint values use the V2 sign convention of
deploy/robot_v3 (the scene is built from that URDF, so no sign flips anywhere).

    from chain import run
    summary, trace = run(dx=0.0, dy=0.0, yaw_deg=20.0)
"""

import math
import os

import mujoco
import numpy as np

import ptload

HERE = os.path.dirname(os.path.abspath(__file__))
SCENE = os.path.join(HERE, "jax_v3_scene.xml")
DEPLOY = os.path.join(HERE, "deploy")

DT = 1.0 / 60.0                     # control
SUBSTEPS = 2                        # physics 120 Hz
CUBE_REF = np.array([-0.285, 0.26])
CUBE_HALF = 0.025
TABLE_TOP = 0.735
OBS_ORIGIN = np.array([0.0, 0.0, 0.3625])          # V2 base origin (4)
DZ = (TABLE_TOP - OBS_ORIGIN[2]) - 0.435           # -0.0625 (7.1)
E_TABLE = DZ + 0.0575                              # -0.005 (7.1)

# Isaac joint order (5.1), arm bundle (8) and finger bundle (6) (5.2)
ISAAC = ("shoulder_pitch_to_shoulder_roll", "shoulder_roll_to_shoulder_yaw", "shoulder_yaw_to_upper_arm",
         "upper_arm_to_under_arm", "under_arm_to_wrist_pitch", "wrist_pitch_to_wrist_yaw", "wrist_yaw_to_hand_base",
         "hand_base_to_finger_1_1", "hand_base_to_finger_2_1", "hand_base_to_rotation_base", "finger_1_1_to_finger_1_2",
         "finger_1_2_to_finger_2_2", "rotation_base_to_finger_3_1", "finger_3_1_to_finger_3_2")
ARM_IDS = (0, 1, 2, 3, 4, 5, 6, 9)
FINGER_IDS = (7, 8, 12, 10, 11, 13)
PAD_LINKS = ("Finger_1_1", "Finger_2_1", "Finger_3_1", "Finger_1_2", "Finger_2_2", "Finger_3_2")   # 6.1
TIP_LINKS = ("Finger_1_2", "Finger_2_2", "Finger_3_2")
HAND = "MG996R"

ARM_KP = 100.0
S_FIX = np.radians([1.9, 0.0, 0.0, 2.1, 0.0, 0.7, -0.2, 0.0])                     # 5.4
GOAL_BASE = np.radians([-7.3, 0.0, 2.5, 73.7, -5.6, 14.7, -6.6, -170.1])         # 7.2
Q_L = np.radians([1.9, 0.0, 7.2, 92.1, 0.0, -1.9, -0.2])                          # 7.7
ROT_BODY = math.radians(-170.0)                                                   # 7.4
ROLL_IN = math.radians(12.0)                                                      # 7.3
THUMB_OPEN = math.radians(-30.0)
HAND_SHAPE = np.radians([0.0, 0.0, -30.0, 0.0, 0.0, 0.0])
FIRST_MAX = np.radians([30.0, 30.0, 0.0])

# 7.1 path P (deg), knots t = 0, 5.0, 5.6, ..., 8.0; s_fix is already in it
P_T = np.array([0.0, 5.0, 5.6, 6.2, 6.8, 7.4, 8.0])
P_Q = np.radians(np.array([
    [0, 0, 0, 0, 0, 0, 0, 0],
    [-26.9, 0, 2.5, 91.2, -5.6, 21.5, -6.8, -170.1],
    [-22.7, 0, 2.5, 89.1, -5.6, 19.4, -6.8, -170.1],
    [-18.5, 0, 2.5, 86.5, -5.6, 17.7, -6.8, -170.1],
    [-14.2, 0, 2.5, 83.5, -5.6, 16.5, -6.8, -170.1],
    [-9.8, 0, 2.5, 79.9, -5.6, 15.7, -6.8, -170.1],
    [-5.4, 0, 2.5, 75.8, -5.6, 15.4, -6.8, -170.1]], dtype=float))
S_T = np.array([0.0, 2.0, 5.0, 5.6, 6.2, 6.8, 7.4, 8.0])

# 6.3 RA12P
RA12P_GF = np.array([5, 55, 100, 165, 260, 350, 470, 590, 750, 975, 1290, 1585, 1920, 2465, 3240, 4000], float)
RA12P_KOHM = np.array([60, 19.5, 11.5, 8.8, 7.5, 6.8, 5.4, 4.4, 4.0, 3.5, 3.1, 2.9, 2.7, 2.6, 2.55, 2.5], float)

VELOCITY_LIMIT = 1.0                # rad/s, robot joints (5.3)
# The scene's MuJoCo settings (build_scene.py: 1/1200 s step, contact time constant 0.002 s, elliptic cone,
# impratio 10) stand in for PhysX's near-rigid contact and friction. Chain(physics="plain") puts MuJoCo's defaults back
# (1/120 s, 0.02 s, pyramidal, impratio 1) for the before/after comparison.
PLAIN = {"timestep": 1.0 / 120.0, "contact_tc": 0.02, "cone": "pyramidal", "impratio": 1.0}

PHASES = ("RAISE", "SETTLE", "INSERT", "GRASP_LIFT", "RETURN", "DONE")


def smoothstep(x):
    x = min(max(x, 0.0), 1.0)
    return x * x * (3.0 - 2.0 * x)


def catmull_rom(ts, qs, t):
    """7.1: cubic Hermite through the knots, interior slopes (Q[k+1]-Q[k-1])/(t[k+1]-t[k-1]), end slopes 0."""
    if t >= ts[-1]:
        return qs[-1].copy()
    if t <= ts[0]:
        return qs[0].copy()
    k = int(np.searchsorted(ts, t, side="right") - 1)
    k = min(k, len(ts) - 2)
    slope = lambda i: np.zeros_like(qs[0]) if i in (0, len(ts) - 1) else (qs[i + 1] - qs[i - 1]) / (ts[i + 1] - ts[i - 1])
    h = ts[k + 1] - ts[k]
    x = (t - ts[k]) / h
    return ((2 * x**3 - 3 * x**2 + 1) * qs[k] + (x**3 - 2 * x**2 + x) * h * slope(k)
            + (-2 * x**3 + 3 * x**2) * qs[k + 1] + (x**3 - x**2) * h * slope(k + 1))


class Tables:
    """Lookup tables of the deploy folder, bilinear in the cube offset (dx, dy), blended to the table height e (7.1)."""

    def __init__(self):
        ld = lambda n: ptload.load(os.path.join(DEPLOY, n + ".pt"))
        self.raise_ = {k: ld(f"raise_grid_v65_{k}") for k in ("basec", "m3c", "p3c")}
        self.final = {k: ld(f"pregrasp_grid_v65_{k}") for k in ("basec", "m3c", "p3c")}
        self.theta = ld("hand_theta_v65")
        self.mask = ld("reach_mask_v65")
        self.gx = self.raise_["basec"]["gx"].astype(float)
        self.gy = self.raise_["basec"]["gy"].astype(float)
        for d in list(self.raise_.values()) + list(self.final.values()) + [self.theta, self.mask]:
            assert np.allclose(d["gx"], self.gx) and np.allclose(d["gy"], self.gy)

    def _cell(self, dx, dy):
        """Lower-left node as v2_front_grasp_lift.py's mask test, floor((x - gx0) / step), with 1e-4 cell slack so a
        spot on a grid line (e.g. y -18 cm) is not pushed into the cell below by float32 grid spacing."""
        x = min(max(dx, self.gx[0]), self.gx[-1])
        y = min(max(dy, self.gy[0]), self.gy[-1])
        fx = (x - self.gx[0]) / (self.gx[1] - self.gx[0])
        fy = (y - self.gy[0]) / (self.gy[1] - self.gy[0])
        i = min(int(math.floor(fx + 1e-4)), len(self.gx) - 2)
        j = min(int(math.floor(fy + 1e-4)), len(self.gy) - 2)
        u = min(max((x - self.gx[i]) / (self.gx[i + 1] - self.gx[i]), 0.0), 1.0)
        v = min(max((y - self.gy[j]) / (self.gy[j + 1] - self.gy[j]), 0.0), 1.0)
        return i, j, u, v

    def _bilinear(self, grid, dx, dy):
        i, j, u, v = self._cell(dx, dy)
        g = grid.astype(float)
        return ((1 - u) * (1 - v) * g[i, j] + u * (1 - v) * g[i + 1, j] + (1 - u) * v * g[i, j + 1] + u * v * g[i + 1, j + 1])

    def _blend(self, tables, k, dx, dy, e):
        s = self._bilinear(tables["basec"]["dq"][k], dx, dy)
        if e < 0:
            s = s + (e / -0.03) * (self._bilinear(tables["m3c"]["dq"][k], dx, dy) - s)
        elif e > 0:
            s = s + (e / 0.03) * (self._bilinear(tables["p3c"]["dq"][k], dx, dy) - s)
        return s

    def raise_knots(self, dx, dy, e):
        return np.stack([self._blend(self.raise_, k, dx, dy, e) for k in range(6)])      # S_0..S_5 (rad)

    def final_pose(self, dx, dy, e):
        return self._blend(self.final, -1, dx, dy, e)                                     # F (rad)

    def theta_deg(self, dx, dy):
        return float(self._bilinear(self.theta["theta_deg"], dx, dy))

    def valid(self, dx, dy):
        """10: the four grid nodes around (dx, dy) are all valid. A spot on a grid line lies in two cells (four on a
        node); v2_front_grasp_lift.py's floor() then picks one by float32 rounding, so here any cell containing the
        spot with four valid nodes counts. That is the closed union of the valid cells, the ranges README 10 lists
        (x +2 cm: y -28..+28 cm)."""
        if not (self.gx[0] - 1e-9 <= dx <= self.gx[-1] + 1e-9 and self.gy[0] - 1e-9 <= dy <= self.gy[-1] + 1e-9):
            return False
        fx = (dx - self.gx[0]) / (self.gx[1] - self.gx[0])
        fy = (dy - self.gy[0]) / (self.gy[1] - self.gy[0])
        m = self.mask["valid"]
        for i in {math.floor(fx - 1e-4), math.floor(fx + 1e-4)}:
            for j in {math.floor(fy - 1e-4), math.floor(fy + 1e-4)}:
                if 0 <= i <= len(self.gx) - 2 and 0 <= j <= len(self.gy) - 2:
                    if m[i, j] and m[i + 1, j] and m[i, j + 1] and m[i + 1, j + 1]:
                        return True
        return False


class Policy:
    def __init__(self):
        a = ptload.load_actor(os.path.join(DEPLOY, "grasp_lift_23025.pt"))
        self.mean, self.std, self.layers = a["mean"], a["std"], a["layers"]

    def __call__(self, obs):
        x = (obs - self.mean) / (self.std + 0.01)
        for k, (w, b) in enumerate(self.layers):
            x = w @ x + b
            if k < 2:
                x = np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))      # ELU(1)
        return x


def sensor_reading(force_n):
    """6.3: RA12P + 10 kOhm divider + 10-bit ADC, no sensor randomization (evaluation)."""
    gf = force_n * 1000.0 / 9.80665
    out = np.zeros_like(gf)
    on = gf >= RA12P_GF[0]
    lg = np.log(np.clip(gf[on], RA12P_GF[0], RA12P_GF[-1]))
    res = np.exp(np.interp(lg, np.log(RA12P_GF), np.log(RA12P_KOHM)))
    out[on] = 10.0 / (res + 10.0)
    return np.round(np.clip(out, 0, 1) * 1023) / 1023


def quat_mul(a, b):
    w1, x1, y1, z1 = a
    w2, x2, y2, z2 = b
    return np.array([w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2, w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
                     w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2, w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2])


def axis_angle(q):
    q = q / np.linalg.norm(q)
    if q[0] < 0:
        q = -q
    s = np.linalg.norm(q[1:])
    if s < 1e-9:
        return 2.0 * q[1:]
    return 2.0 * math.atan2(s, q[0]) * q[1:] / s


def quat_yaw(q):
    """Cube yaw: angle of its x axis about world z (as v2_front_grasp_lift.py)."""
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    return math.atan2(R[3], R[0])


def quat_tilt_deg(q):
    R = np.zeros(9)
    mujoco.mju_quat2Mat(R, q)
    up = np.abs(R.reshape(3, 3)[2])          # world z seen in cube axes: tilt of the face nearest to up
    return math.degrees(math.acos(min(1.0, up.max())))


def quat_angle_deg(a, b):
    return math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(a, b))))))


class Chain:
    def __init__(self, dx, dy, yaw_deg, mass=0.040, yaw_obs_err_deg=0.0, e=E_TABLE, model=None, tables=None, policy=None,
                 velocity_limit=VELOCITY_LIMIT, physics="matched", impratio=None):
        self.m = model or mujoco.MjModel.from_xml_path(SCENE)
        self.d = mujoco.MjData(self.m)
        self.scratch = mujoco.MjData(self.m)
        self.tables = tables or Tables()
        self.policy = policy or Policy()
        m = self.m
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        self.qadr = np.array([m.jnt_qposadr[jid(n)] for n in ISAAC])
        self.vadr = np.array([m.jnt_dofadr[jid(n)] for n in ISAAC])
        self.act = np.array([mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_ACTUATOR, n) for n in ISAAC])
        self.lo = np.array([m.jnt_range[jid(n)][0] for n in ISAAC])
        self.hi = np.array([m.jnt_range[jid(n)][1] for n in ISAAC])
        self.obs_lo, self.obs_hi = self.lo.copy(), self.hi.copy()            # 8.1: V2 limits
        self.obs_hi[1] = math.radians(45.0)
        self.obs_lo[3], self.obs_hi[3] = math.radians(-105.0), math.radians(105.0)
        self.arm = np.array(ARM_IDS)
        self.fing = np.array(FINGER_IDS)
        self.pad_body = [bid(n) for n in PAD_LINKS]
        self.tip_body = [bid(n) for n in TIP_LINKS]
        self.hand_body = bid(HAND)
        self.cube_body = bid("cube")
        self.cube_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "cube")
        self.table_geom = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "table")
        self.cube_q = m.jnt_qposadr[jid("cube")]
        self.cube_v = m.jnt_dofadr[jid("cube")]
        self.robot_bodies = set(range(bid("base_link"), m.nbody))
        self.robot_bodies.discard(self.cube_body)
        self.pad_of_body = {b: i for i, b in enumerate(self.pad_body)}

        # cube: spot, yaw, mass (inertia scales with the mass of a uniform cube)
        self.dx, self.dy, self.yaw_deg, self.e = float(dx), float(dy), float(yaw_deg), float(e)
        self.velocity_limit = velocity_limit
        if physics == "plain":
            m.opt.timestep = PLAIN["timestep"]
            m.geom_solref[:] = (PLAIN["contact_tc"], 1.0)
            m.opt.cone = mujoco.mjtCone.mjCONE_PYRAMIDAL
            m.opt.impratio = PLAIN["impratio"]
        else:
            assert physics == "matched", physics
        if impratio is not None:
            m.opt.impratio = impratio
        self.inner = int(round((DT / SUBSTEPS) / m.opt.timestep))
        assert abs(self.inner * m.opt.timestep - DT / SUBSTEPS) < 1e-12
        self.physics = {"mode": physics, "mujoco_dt": float(m.opt.timestep), "contact_tc": float(m.geom_solref[0, 0]),
                        "cone": "elliptic" if m.opt.cone == mujoco.mjtCone.mjCONE_ELLIPTIC else "pyramidal",
                        "impratio": float(m.opt.impratio), "velocity_limit": velocity_limit}
        m.body_mass[self.cube_body] = mass
        m.body_inertia[self.cube_body] = mass * (2 * CUBE_HALF) ** 2 / 6.0
        mujoco.mj_setConst(m, self.d)
        self.d.qpos[self.cube_q:self.cube_q + 3] = [CUBE_REF[0] + dx, CUBE_REF[1] + dy, TABLE_TOP + CUBE_HALF]
        h = math.radians(yaw_deg) / 2
        self.d.qpos[self.cube_q + 3:self.cube_q + 7] = [math.cos(h), 0, 0, math.sin(h)]
        self.d.qpos[self.qadr] = 0.0                       # hanging arm (3): every joint 0
        self.d.ctrl[self.act] = 0.0
        mujoco.mj_forward(m, self.d)

        # 7.1 / 7.3 / 8.1 constants of this cube
        self.S = self.tables.raise_knots(dx, dy, e)
        self.S_knots = np.vstack([np.zeros(8), self.S[0], self.S])      # (0: 0), (2.0: S0), (5.0: S0), ..., (8.0: S5)
        self.goal = GOAL_BASE + self.S[5]
        self.F = self.tables.final_pose(dx, dy, e)
        self.dq_in = self.F - self.S[5]
        self.dq_in[6] += ROLL_IN
        self.theta = self.tables.theta_deg(dx, dy)
        psi = math.radians(yaw_deg + yaw_obs_err_deg - self.theta)     # measured once before the raise
        self.yaw_obs = np.array([math.sin(4 * psi), math.cos(4 * psi)])

        self.target = np.zeros(14)          # joint targets, Isaac order
        self.trace = {k: [] for k in ("t", "phase", "q", "qd", "target", "cube_pos", "cube_quat", "pad_normal",
                                      "pad_obs", "tip_mid", "hand_pos", "hand_quat", "cube_robot_force",
                                      "cube_table_force")}
        self.t = 0.0
        self.phase = "RAISE"
        self.events = {}

    # ------------------------------------------------------------------ state
    def q(self):
        return self.d.qpos[self.qadr].copy()

    def qd(self):
        return self.d.qvel[self.vadr].copy()

    def cube_pos(self):
        return self.d.xpos[self.cube_body].copy()

    def cube_quat(self):
        return self.d.xquat[self.cube_body].copy()

    def tip_mid(self):
        return np.mean([self.d.xpos[b] for b in self.tip_body], axis=0)

    def contacts(self):
        """Cube-on-pad-link force vectors (world, 6.2), total robot-cube force magnitude, cube-table force."""
        f_links = np.zeros((6, 3))
        robot_n, table_n = 0.0, 0.0
        f6 = np.zeros(6)
        for i in range(self.d.ncon):
            c = self.d.contact[i]
            g1, g2 = c.geom1, c.geom2
            if self.cube_geom not in (g1, g2):
                continue
            other = g2 if g1 == self.cube_geom else g1
            mujoco.mj_contactForce(self.m, self.d, i, f6)
            fw = c.frame.reshape(3, 3).T @ f6[:3]          # force on geom2 (normal points geom1 -> geom2)
            f_on_other = fw if other == g2 else -fw
            if other == self.table_geom:
                table_n += np.linalg.norm(f_on_other)
                continue
            b = self.m.geom_bodyid[other]
            if b in self.robot_bodies:
                robot_n += np.linalg.norm(f_on_other)
            if b in self.pad_of_body:
                f_links[self.pad_of_body[b]] += f_on_other
        return f_links, robot_n, table_n

    def pad_forces(self, f_links):
        """6.2: pad normal = -f_x in the link frame, clamped at 0 (all six, as JaxV2PadSensorGraspEnv.pad_forces);
        the policy observation and the done check use the thumb's force magnitude (JaxV2FrontGraspEnv.pad_forces)."""
        normal = np.zeros(6)
        for k, b in enumerate(self.pad_body):
            R = self.d.xmat[b].reshape(3, 3)
            normal[k] = max(-(R.T @ f_links[k])[0], 0.0)
        obs = normal.copy()
        obs[2] = np.linalg.norm(f_links[2])
        obs[5] = np.linalg.norm(f_links[5])
        return normal, obs

    def gravity_torque(self):
        """tau_g(q) of the 8 arm joints at the measured joint positions, zero velocity and acceleration."""
        s = self.scratch
        s.qpos[:] = self.d.qpos
        s.qvel[:] = 0.0
        mujoco.mj_kinematics(self.m, s)
        mujoco.mj_comPos(self.m, s)
        out = np.zeros(self.m.nv)
        mujoco.mj_rne(self.m, s, 0, out)
        return out[self.vadr[self.arm]]

    # ------------------------------------------------------------------ stepping
    def _record(self, f_links=None, robot_n=0.0, table_n=0.0):
        if f_links is None:
            f_links, robot_n, table_n = self.contacts()
        normal, obs = self.pad_forces(f_links)
        tr = self.trace
        tr["t"].append(self.t)
        tr["phase"].append(PHASES.index(self.phase))
        tr["q"].append(self.q())
        tr["qd"].append(self.qd())
        tr["target"].append(self.target.copy())
        tr["cube_pos"].append(self.cube_pos())
        tr["cube_quat"].append(self.cube_quat())
        tr["pad_normal"].append(normal)
        tr["pad_obs"].append(obs)
        tr["tip_mid"].append(self.tip_mid())
        tr["hand_pos"].append(self.d.xpos[self.hand_body].copy())
        tr["hand_quat"].append(self.d.xquat[self.hand_body].copy())
        tr["cube_robot_force"].append(robot_n)
        tr["cube_table_force"].append(table_n)
        return normal, obs

    def step(self, sent=None, blend=None):
        """One control step: two physics steps with the drive targets; `blend(k)` gives per-substep targets."""
        for k in range(SUBSTEPS):
            ctrl = blend(k) if blend is not None else (self.target if sent is None else sent)
            self.d.ctrl[self.act] = ctrl
            for _ in range(self.inner):
                q_old = self.d.qpos[self.qadr].copy()
                mujoco.mj_step(self.m, self.d)
                if self.velocity_limit:
                    # PhysX max joint velocity (velocity_limit_sim=1.0 for arm and fingers in Jungwoo's robot config).
                    # MuJoCo has none: clamp the new joint velocity and redo the hinge position update
                    # q' = q + dt v' (semi-implicit Euler) with it, i.e. the clamp inside the integrator.
                    v = self.d.qvel[self.vadr]
                    if (np.abs(v) > self.velocity_limit).any():
                        vc = np.clip(v, -self.velocity_limit, self.velocity_limit)
                        self.d.qvel[self.vadr] = vc
                        self.d.qpos[self.qadr] = q_old + self.m.opt.timestep * vc
        self.t += DT
        return self._record()

    # ------------------------------------------------------------------ phases
    def raise_and_settle(self, max_s=11.0):
        """7.1 raise and 7.2 hand-over condition (render_front_grasp.py raise loop with --raise_policy nominal)."""
        s = np.zeros(8)                   # filter state (unclamped, no gravity term)
        rate = 0.5 * DT
        trace_q = []
        calm = 0
        self._record()
        for k in range(round(max_s / DT)):
            t = (k + 1) * DT
            self.phase = "RAISE" if t <= 8.0 else "SETTLE"
            q_cmd = catmull_rom(P_T, P_Q, t) + catmull_rom(S_T, self.S_knots, t)
            new = 0.85 * s + 0.15 * q_cmd
            new[:7] = s[:7] + np.clip(new[:7] - s[:7], -rate, rate)
            s = new
            w = smoothstep(t - 4.0)
            G = w * (self.gravity_torque() / ARM_KP - S_FIX)
            self.target[self.arm] = np.clip(s, self.lo[self.arm], self.hi[self.arm]) + G
            fg = np.zeros(6)
            fg[2] = smoothstep((t - 1.0) / 3.0) * THUMB_OPEN
            self.target[self.fing] = fg
            self.step()
            q = self.q()[self.arm]
            trace_q.append(q)
            err = np.abs(q - self.goal).max()
            still = len(trace_q) > 15 and np.abs(trace_q[-1] - trace_q[-16]).max() < math.radians(0.2)
            calm = calm + 1 if (t >= 8.5 and still and err < math.radians(3.0)) else 0
            if calm >= 10:
                break
        self.events["handover_t"] = self.t
        self.events["handover_by"] = "converged" if calm >= 10 else "timeout"
        self.events["handover_err_deg"] = float(np.degrees(np.abs(self.q()[self.arm] - self.goal).max()))
        self.s_raise = s

    def insert(self):
        """7.3: 30 steps clamp(q_h + smoothstep(k/30) dq) + G, then 15 steps holding; fingers unchanged."""
        self.phase = "INSERT"
        self.events["insert_t"] = self.t
        fg = self.target[self.fing].copy()
        for k in range(45):
            w = smoothstep(min((k + 1) / 30.0, 1.0))
            G = self.gravity_torque() / ARM_KP - S_FIX
            self.target[self.arm] = np.clip(self.s_raise + w * self.dq_in, self.lo[self.arm], self.hi[self.arm]) + G
            self.target[self.fing] = fg
            self.step()
        self.events["insert_end_arm_deg"] = np.degrees(self.q()[self.arm]).round(2).tolist()
        self.events["insert_end_grav_deg"] = np.degrees(G).round(2).tolist()
        self.events["insert_end_target_deg"] = np.degrees(self.target[self.arm]).round(2).tolist()

    def grasp_lift(self, max_s=11.0, after_done_s=2.0):
        """7.4-7.8 with the policy at 60 Hz."""
        self.phase = "GRASP_LIFT"
        self.act_log = []
        t0 = self.t
        self.events["close_t"] = t0
        q_h = self.target.copy()                         # 7.4 blend source (targets at the hand-over)
        hold = self.target[self.arm[:7]].copy()          # IK base (gravity term of the last reach-in step included)
        offset = np.zeros(7)
        self.target[self.arm[7]] = ROT_BODY
        filt = np.zeros(11)
        bal_q = HAND_SHAPE.copy()
        hand_q0 = self.d.xquat[self.hand_body].copy()
        p_des = self.tip_mid()
        cube0 = self.cube_pos()
        cube_q_close = self.cube_quat()
        ret_t, ret_cnt, ret_q0, ret_frozen = -1.0, 0, None, None
        done_cnt, q_prev, done_t = 0, self.q()[self.arm[:7]], None
        blend_k = 0
        use = [0, 2, 3, 4, 5, 6]                           # 7.6: no shoulder roll
        jacp, jacr = np.zeros((3, self.m.nv)), np.zeros((3, self.m.nv))
        dof = self.vadr[self.arm[:7]][use]
        f_links, _, _ = self.contacts()
        normal, pad_obs = self.pad_forces(f_links)
        first_touch = [None, None, None]
        n_max = round(max_s / DT)
        for k in range(n_max):
            tc = self.t - t0
            # --- 8.1 observation (state after the last physics step)
            obs = self.observation(pad_obs)
            # --- policy and 8.2 post-processing; 7.7 freezes actions 7-10 during the return (as raw inputs)
            a = np.clip(self.policy(obs), -1.0, 1.0)
            if ret_frozen is not None:
                a[7:11] = ret_frozen
            filt = 0.25 * a + 0.75 * filt
            self.act_log.append(filt.copy())
            # --- 7.5 close (pad normal forces, per digit first + distal link)
            f = normal[:3] + normal[3:]
            on = f >= 0.3
            for i in range(3):
                if on[i] and first_touch[i] is None:
                    first_touch[i] = tc
            sf = max(5.0 + 4.0 * filt[7], 0.3)
            st = max(5.0 + 4.0 * filt[9], 0.3)
            want = np.array([0.5 * sf, 0.5 * sf, st]) if on.all() else np.full(3, 0.3)
            dmax = 0.15 * DT
            stp = np.where(on, np.clip(0.0012 * (want - f), -dmax, dmax), dmax)
            q1 = np.minimum(np.maximum(bal_q[:3] + stp, HAND_SHAPE[:3]), FIRST_MAX)
            q2 = HAND_SHAPE[3:] + math.radians(60.0) * np.clip(filt[[8, 8, 10]], 0.0, 1.0)
            bal_q = np.concatenate([q1, q2])
            self.target[self.fing] = bal_q
            # --- 7.7 return and 7.8 done check, in v2_front_lift_rl.py's order (_lift_return_step: trigger, target,
            # clock + dt, then _lift_done_update), all before the IK of this step
            rise = self.cube_pos()[2] - cube0[2]
            ret_cnt = ret_cnt + 1 if rise >= 0.04 else 0
            if ret_t < 0 and ret_cnt * DT >= 0.5:
                ret_t, ret_q0, ret_frozen = 0.0, hold + offset, filt[7:11].copy()
                self.events["return_t"] = self.t
                self.phase = "RETURN"
            returning = ret_t >= 0
            if returning:
                w = smoothstep(ret_t / 2.5)
                self.target[self.arm[:7]] = ret_q0 + w * (Q_L - ret_q0)
                ret_t += DT
            q7 = self.q()[self.arm[:7]]
            still = np.abs(q7 - q_prev).max() < math.radians(0.05)
            q_prev = q7
            grip = pad_obs[[0, 1, 2]] + pad_obs[[3, 4, 5]]
            ok = ret_t >= 2.5 and still and grip[2] >= 0.3 and (grip[0] >= 0.3 or grip[1] >= 0.3) and rise >= 0.04
            done_cnt = done_cnt + 1 if ok else 0
            if done_t is None and done_cnt * DT >= 0.5:
                done_t = self.t
                self.phase = "DONE"
                self.events["done_t"] = self.t
                self.events["done_state"] = self.handover_state(normal, pad_obs, t0, cube0)
            # --- arm targets: lift IK unless returning
            if not returning:
                v = np.array([0.005 * filt[3], 0.005 * filt[5], -0.035 * filt[0]])
                p_now = self.tip_mid()
                p_des = p_des + v * DT
                v = v + 2.0 * np.clip(p_des - p_now, -0.02, 0.02)
                hq = self.d.xquat[self.hand_body]
                hq_conj = hq * np.array([1, -1, -1, -1])
                omega = 2.0 * axis_angle(quat_mul(hand_q0, hq_conj))
                mujoco.mj_jacBody(self.m, self.d, jacp, jacr, self.hand_body)
                Jv, Jw = jacp[:, dof], jacr[:, dof]
                r = p_now - self.d.xpos[self.hand_body]
                skew = np.array([[0, -r[2], r[1]], [r[2], 0, -r[0]], [-r[1], r[0], 0]])
                J = np.vstack([Jv - skew @ Jw, Jw])
                A = J @ J.T + 0.02**2 * np.eye(6)
                dq = np.clip(J.T @ np.linalg.solve(A, np.concatenate([v, omega])), -0.6, 0.6)
                stepq = np.zeros(7)
                stepq[use] = dq * DT
                offset = np.clip(offset + stepq, -1.0, 1.0)
                self.target[self.arm[:7]] = np.clip(hold + offset, self.lo[self.arm[:7]], self.hi[self.arm[:7]])
            self.target[self.arm[7]] = ROT_BODY

            # --- 7.4 blend over 0.5 s, per physics step (render_front_grasp.py ramped_apply)
            def blended(sub, k0=blend_k):
                x = min(1.0, (k0 + sub + 1) * (DT / SUBSTEPS) / 0.5)
                return q_h + smoothstep(x) * (self.target - q_h)
            normal, pad_obs = self.step(blend=blended)
            blend_k += SUBSTEPS
            if done_t is not None and self.t - done_t >= after_done_s:
                break
        self.events["first_touch_s"] = first_touch
        self.events["all_touch_s"] = None if None in first_touch else max(first_touch)
        self.events["cube_close_pos"] = cube0.tolist()
        self.events["cube_close_quat"] = cube_q_close.tolist()
        self.done_t = done_t

    def observation(self, pad_obs):
        q, qd = self.q(), self.qd()
        nq = 2.0 * (q - self.obs_lo) / (self.obs_hi - self.obs_lo) - 1.0
        nq[9] = -0.7222                                 # 8.1: RotationBody read as -155 deg
        vel = 0.2 * qd
        vel[9] = 0.0
        R_base_inv = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]], float)   # base x = world -y, base y = world +x
        cube = self.cube_pos()
        cube_b = R_base_inv @ (cube - OBS_ORIGIN)
        cube_v = R_base_inv @ self.d.qvel[self.cube_v:self.cube_v + 3]
        rel = R_base_inv @ (cube - self.tip_mid())
        obs = np.concatenate([nq, vel, cube_b, cube_v, rel, sensor_reading(pad_obs)])
        obs = np.clip(obs, -5.0, 5.0)
        obs[29] -= 0.115
        obs[30] -= DZ
        return np.concatenate([obs, self.yaw_obs])

    def handover_state(self, normal, pad_obs, t0, cube0):
        cq = self.cube_quat()
        return {
            "t_after_handover_s": round(self.t - t0, 3), "episode_t_s": round(self.t, 3),
            "arm_q_deg": np.degrees(self.q()[self.arm]).round(3).tolist(),
            "finger_q_deg": np.degrees(self.q()[self.fing]).round(3).tolist(),
            "hand_pos_m": self.d.xpos[self.hand_body].round(5).tolist(),
            "hand_quat_wxyz": self.d.xquat[self.hand_body].round(5).tolist(),
            "fingertip_mid_pos_m": self.tip_mid().round(5).tolist(),
            "cube_pos_m": self.cube_pos().round(5).tolist(), "cube_quat_wxyz": cq.round(5).tolist(),
            "cube_rise_m": round(float(self.cube_pos()[2] - cube0[2]), 4),
            "cube_minus_fingertip_mid_m": (self.cube_pos() - self.tip_mid()).round(5).tolist(),
            "pad_N": pad_obs.round(3).tolist(), "pad_normal_N": normal.round(3).tolist(),
            "cube_tilt_deg": round(quat_tilt_deg(cq), 2),
            "robot_base_pos_m": [0.0, 0.01, 0.0],
        }


def run(dx, dy, yaw_deg, mass=0.040, yaw_obs_err_deg=0.0, **kw):
    c = Chain(dx, dy, yaw_deg, mass=mass, yaw_obs_err_deg=yaw_obs_err_deg, **kw)
    c.raise_and_settle()
    c.insert()
    c.grasp_lift()
    return summarize(c), {k: np.asarray(v) for k, v in c.trace.items()}


def summarize(c):
    tr = {k: np.asarray(v) for k, v in c.trace.items()}
    ev = c.events
    t = tr["t"]
    cube0 = tr["cube_pos"][0]
    close_i = int(np.searchsorted(t, ev["close_t"] - 1e-9))
    pre = slice(0, close_i + 1)
    rise = tr["cube_pos"][:, 2] - tr["cube_pos"][close_i, 2]
    lift_i = next((i for i in range(close_i, len(t)) if rise[i] >= 0.005), None)
    raise_i = int(np.searchsorted(t, ev["insert_t"] - 1e-9))
    s = {
        "spot_cm": [round(c.dx * 100, 2), round(c.dy * 100, 2)], "cube_yaw_deg": c.yaw_deg,
        "cube_mass_kg": float(c.m.body_mass[c.cube_body]), "theta_deg": round(c.theta, 2),
        "table_e_m": c.e, "valid_spot": c.tables.valid(c.dx, c.dy),
        "phase_t_s": {"RAISE": 0.0, "SETTLE": 8.0 if ev["handover_t"] > 8.0 else None, "INSERT": round(ev["insert_t"], 3),
                      "GRASP_LIFT": round(ev["close_t"], 3), "RETURN": round(ev["return_t"], 3) if "return_t" in ev else None,
                      "DONE": round(ev["done_t"], 3) if "done_t" in ev else None},
        "handover_by": ev["handover_by"], "handover_err_deg": round(ev["handover_err_deg"], 2),
        "insert_end_arm_deg": ev["insert_end_arm_deg"], "insert_end_grav_deg": ev["insert_end_grav_deg"],
        "first_touch_after_close_s": [None if v is None else round(v, 3) for v in ev["first_touch_s"]],
        "all_touch_after_close_s": None if ev["all_touch_s"] is None else round(ev["all_touch_s"], 3),
        "max_rise_m": round(float(rise[close_i:].max()), 4),
        "raise_cube_robot_force_max_n": round(float(tr["cube_robot_force"][:raise_i + 1].max()), 3),
        "insert_cube_robot_force_max_n": round(float(tr["cube_robot_force"][raise_i:close_i + 1].max()), 3),
        "lift_cube_robot_force_max_n": round(float(tr["cube_robot_force"][close_i:].max()), 3),
        "cube_shift_before_grasp_mm": round(float(np.linalg.norm(tr["cube_pos"][pre, :2] - cube0[:2], axis=1).max()) * 1000, 2),
        "cube_shift_during_grasp_mm": None if lift_i is None else round(float(np.linalg.norm(tr["cube_pos"][lift_i, :2] - tr["cube_pos"][close_i, :2])) * 1000, 2),
        "cube_turn_during_grasp_deg": None if lift_i is None else round(quat_angle_deg(tr["cube_quat"][close_i], tr["cube_quat"][lift_i]), 2),
        "max_joint_speed_rad_s": round(float(np.abs(tr["qd"]).max()), 3),
        "done": "done_t" in ev,
        "physics": c.physics,
    }
    if lift_i is not None and "done_t" in ev:
        di = int(np.searchsorted(t, ev["done_t"] - 1e-9))
        s["cube_turn_after_lift_deg"] = round(quat_angle_deg(tr["cube_quat"][lift_i], tr["cube_quat"][di]), 2)
        s["cube_turn_in_hand_after_lift_deg"] = round(in_hand_turn_deg(tr, lift_i, di), 2)
        s["cube_turn_in_hand_during_grasp_deg"] = round(in_hand_turn_deg(tr, close_i, lift_i), 2)
        s["done_state"] = ev["done_state"]
        s["still_up_at_end"] = bool(rise[-1] >= 0.04)
    s["fail_reason"] = None if s["done"] else fail_reason(s, rise, close_i, ev)
    return s


def in_hand_turn_deg(tr, i, j):
    """Rotation of the cube relative to the hand link between trace steps i and j (deg)."""
    def rel(k):
        Rh, Rc = np.zeros(9), np.zeros(9)
        mujoco.mju_quat2Mat(Rh, tr["hand_quat"][k])
        mujoco.mju_quat2Mat(Rc, tr["cube_quat"][k])
        return Rh.reshape(3, 3).T @ Rc.reshape(3, 3)
    R = rel(i).T @ rel(j)
    return math.degrees(math.acos(min(1.0, max(-1.0, (np.trace(R) - 1.0) / 2.0))))


def fail_reason(s, rise, close_i, ev):
    if s["all_touch_after_close_s"] is None:
        return "not all three digits touched the cube"
    if rise[close_i:].max() < 0.04:
        return "cube never 4 cm up"
    if "return_t" not in ev:
        return "cube not 4 cm up for 0.5 s (no L return)"
    if rise[-1] < 0.04:
        return "cube dropped after rising"
    return "done check never held 0.5 s (grip/still)"
