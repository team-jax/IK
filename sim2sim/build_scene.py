"""Build the MuJoCo scene of Jungwoo's (JaxIsaacLab) V3 deploy layout from his robot_v3 URDF.

    venv/bin/python sim2sim/build_scene.py        # writes sim2sim/jax_v3_scene.xml

Everything follows `deploy/MUJOCO_SIM2SIM.md` section 4 and `deploy/README.md` (world frame with the floor at
z = 0, the table toward world -x):
- robot `base_link` (the stand) at (0, 0.01, 0), turned -90 deg about z (quat wxyz 0.7071, 0, 0, -0.7071);
  joint axes, limits, masses, inertias and collision meshes exactly as in `deploy/robot_v3` (V2 sign convention:
  `shoulder_roll_to_shoulder_yaw` and `under_arm_to_wrist_pitch` already flipped there, so no sign changes here)
- base_link, Shoulder_Pitch and Holder are fixed (Isaac merges them); base_link has no collision (removed in the URDF)
- robot self-collision off (`self_collision=False` in Jungwoo's robot config); the robot touches the cube, table, floor
- drives: arm 8 joints kp 100 kd 8, fingers kp 40 kd 2 armature 2e-4, torque limit 100 N m, no joint friction
- table 0.80 x 1.2 x 0.04 centred (-0.625, 0, 0.715) (top 0.735), cube 50 mm 40 g at (-0.285, 0.26, 0.760)
- PhysX runs 120 Hz (the chain's controller 60 Hz, two PhysX steps each). MuJoCo needs three engine settings to
  behave like it (sim2sim/README.md, compatibility items 2-3); each was measured against plain MuJoCo:
  MUJOCO_DT 1/1200 s (ten MuJoCo steps per PhysX step) so the contact time constant can be CONTACT_TC 0.002 s
  (default 0.02 s lets the kp 40 fingers sink 6-24 mm into the 40 g cube and fling it), and an elliptic friction
  cone with IMPRATIO 10 (MuJoCo's soft friction lets the held cube creep and tilt 34 deg; PhysX friction does not).
  Friction coefficients, masses and drive gains are not changed by any of these
- friction: PhysX averages the two materials, MuJoCo takes the larger. Robot 0.4, cube 0.4, table 0.8 make every
  pair equal to the PhysX average of the static values (cube-robot 0.4, cube-table 0.8, robot-table 0.8)

Visual meshes: the robot_v3 DAE files are byte-identical to URDF_1108's, already converted to OBJ in
motion2/arm/meshes, so the scene points there. This file is generated: change this script, not the XML.
"""

import os
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
URDF = os.path.join(HERE, "deploy", "robot_v3", "urdf", "JAXV3_Assembly.urdf")
OUT = os.path.join(HERE, "jax_v3_scene.xml")
VISUAL_DIR = os.path.join("..", "motion2", "arm", "meshes")          # relative to OUT
COLLISION_DIR = os.path.join("deploy", "robot_v3", "meshes", "JAXV3_Assembly")

BASE_POS = (0.0, 0.01, 0.0)
BASE_QUAT = (0.7071067811865476, 0.0, 0.0, -0.7071067811865476)
FIXED_LINKS = ("base_link", "Shoulder_Pitch", "Holder")

ARM_JOINTS = ("shoulder_pitch_to_shoulder_roll", "shoulder_roll_to_shoulder_yaw", "shoulder_yaw_to_upper_arm",
              "upper_arm_to_under_arm", "under_arm_to_wrist_pitch", "wrist_pitch_to_wrist_yaw",
              "wrist_yaw_to_hand_base", "hand_base_to_rotation_base")
FINGER_JOINTS = ("hand_base_to_finger_1_1", "hand_base_to_finger_2_1", "rotation_base_to_finger_3_1",
                 "finger_1_1_to_finger_1_2", "finger_1_2_to_finger_2_2", "finger_3_1_to_finger_3_2")
ARM_KP, ARM_KD = 100.0, 8.0
FINGER_KP, FINGER_KD, FINGER_ARMATURE = 40.0, 2.0, 2e-4
TORQUE_LIMIT = 100.0

ROBOT_FRICTION, CUBE_FRICTION, TABLE_FRICTION = 0.4, 0.4, 0.8
TABLE_SIZE, TABLE_CENTER = (0.80, 1.2, 0.04), (-0.625, 0.0, 0.715)
CUBE_SIDE, CUBE_MASS, CUBE_REF = 0.050, 0.040, (-0.285, 0.26, 0.760)
MUJOCO_DT = 1.0 / 1200.0
CONTACT_TC = 0.002
CONE, IMPRATIO = "elliptic", 10
COLORS = {"Steel": "0.63 0.63 0.63 1", "Acetal_Resin_White": "0.9 0.9 0.88 1", "Generic": "0.25 0.25 0.27 1"}


def _look(pos, target):
    """Camera xyaxes looking from pos at target with world z up."""
    pos, target = np.asarray(pos, float), np.asarray(target, float)
    fwd = (target - pos) / np.linalg.norm(target - pos)
    x = np.cross(fwd, (0.0, 0.0, 1.0))
    x /= np.linalg.norm(x)
    y = np.cross(x, fwd)
    return f'pos="{_f(pos)}" xyaxes="{_f(x)} {_f(y)}"'


def _f(values):
    return " ".join(f"{float(v):.9g}" for v in values)


def _origin(elem):
    o = elem.find("origin") if elem is not None else None
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy


def _quat(rpy):
    x, y, z, w = Rot.from_euler("xyz", rpy).as_quat()        # URDF rpy: fixed axes x, y, z
    return (w, x, y, z)


def build():
    root = ET.parse(URDF).getroot()
    links = {L.get("name"): L for L in root.findall("link")}
    joints = root.findall("joint")
    children = {}
    for j in joints:
        children.setdefault(j.find("parent").get("link"), []).append(j)

    assets, meshes = [], set()

    def mesh_asset(kind, filename, scale):
        base = os.path.basename(filename)
        if kind == "visual":
            name, path = "v_" + base[:-4], os.path.join(VISUAL_DIR, base.replace(".dae", ".obj"))
        else:
            name, path = "c_" + base[:-4], os.path.join(COLLISION_DIR, base)
        if name not in meshes:
            meshes.add(name)
            assets.append(f'    <mesh name="{name}" file="{path}" scale="{scale}"/>')
        return name

    def geoms(link, ind):
        out = []
        L = links[link]
        for v in L.findall("visual"):
            xyz, rpy = _origin(v)
            m = v.find("geometry/mesh")
            name = mesh_asset("visual", m.get("filename"), m.get("scale", "1 1 1"))
            mat = v.find("material")
            rgba = COLORS.get(mat.get("name") if mat is not None else "", "0.5 0.5 0.5 1")
            out.append(f'{ind}<geom type="mesh" mesh="{name}" pos="{_f(xyz)}" quat="{_f(_quat(rpy))}" '
                       f'contype="0" conaffinity="0" group="1" density="0" rgba="{rgba}"/>')
        for c in L.findall("collision"):
            xyz, rpy = _origin(c)
            m = c.find("geometry/mesh")
            assert m is not None, f"{link}: only mesh collisions expected"
            name = mesh_asset("collision", m.get("filename"), m.get("scale", "1 1 1"))
            out.append(f'{ind}<geom name="{link}_col" type="mesh" mesh="{name}" pos="{_f(xyz)}" quat="{_f(_quat(rpy))}" '
                       f'contype="2" conaffinity="1" group="3" density="0" friction="{ROBOT_FRICTION} 0.005 0.0001" '
                       f'rgba="0.8 0.3 0.3 0.4"/>')
        return out

    def inertial(link, ind):
        i = links[link].find("inertial")
        xyz, rpy = _origin(i)
        assert np.allclose(rpy, 0.0), f"{link}: rotated inertial frame"
        m = float(i.find("mass").get("value"))
        I = i.find("inertia")
        full = [I.get(k) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        return f'{ind}<inertial pos="{_f(xyz)}" mass="{m:.9g}" fullinertia="{_f(full)}"/>'

    def body(link, joint, ind):
        lines = []
        if joint is None:
            lines.append(f'{ind}<body name="{link}" pos="{_f(BASE_POS)}" quat="{_f(BASE_QUAT)}">')
        else:
            xyz, rpy = _origin(joint)
            lines.append(f'{ind}<body name="{link}" pos="{_f(xyz)}" quat="{_f(_quat(rpy))}">')
        lines.append(inertial(link, ind + "  "))
        if joint is not None and joint.get("type") == "revolute":
            name = joint.get("name")
            axis = joint.find("axis").get("xyz")
            lim = joint.find("limit")
            arm = name in ARM_JOINTS
            armature = 0.0 if arm else FINGER_ARMATURE
            lines.append(f'{ind}  <joint name="{name}" type="hinge" axis="{axis}" limited="true" '
                         f'range="{lim.get("lower")} {lim.get("upper")}" armature="{armature:g}" damping="0" '
                         f'frictionloss="0"/>')
        else:
            assert joint is None or joint.get("type") == "fixed"
        lines += geoms(link, ind + "  ")
        for j in children.get(link, []):
            lines += body(j.find("child").get("link"), j, ind + "  ")
        lines.append(f"{ind}</body>")
        return lines

    robot = body("base_link", None, "    ")
    acts = []
    for name in ARM_JOINTS + FINGER_JOINTS:
        kp, kd = (ARM_KP, ARM_KD) if name in ARM_JOINTS else (FINGER_KP, FINGER_KD)
        acts.append(f'    <position name="{name}" joint="{name}" kp="{kp:g}" kv="{kd:g}" ctrllimited="false" '
                    f'forcelimited="true" forcerange="-{TORQUE_LIMIT:g} {TORQUE_LIMIT:g}"/>')
    th = [s / 2 for s in TABLE_SIZE]
    c = CUBE_SIDE / 2
    xml = f"""<!-- Generated by sim2sim/build_scene.py from deploy/robot_v3/urdf/JAXV3_Assembly.urdf (JaxIsaacLab
     artifacts/deploy_2026-11-05). Do not edit by hand. World frame of deploy/MUJOCO_SIM2SIM.md section 4. -->
<mujoco model="jax_v3_deploy_2026-11-05">
  <compiler angle="radian" autolimits="true"/>
  <option timestep="{MUJOCO_DT:.12g}" integrator="implicitfast" gravity="0 0 -9.81" cone="{CONE}" impratio="{IMPRATIO:g}"/>
  <default>
    <geom solref="{CONTACT_TC:g} 1"/>
  </default>
  <visual>
    <global offwidth="1280" offheight="960"/>
  </visual>
  <asset>
    <texture name="grid" type="2d" builtin="checker" width="512" height="512" rgb1="0.82 0.82 0.82" rgb2="0.74 0.74 0.74"/>
    <material name="grid" texture="grid" texrepeat="8 8" reflectance="0"/>
{chr(10).join(assets)}
  </asset>
  <worldbody>
    <light pos="0.5 0.5 3" dir="-0.2 -0.2 -1" diffuse="0.8 0.8 0.8"/>
    <light pos="-1.5 1.5 2.5" dir="0.5 -0.5 -1" diffuse="0.4 0.4 0.4"/>
    <camera name="front" {_look((-1.75, 1.15, 1.75), (-0.15, 0.18, 0.85))}/>
    <camera name="close" {_look((-0.75, 0.75, 1.15), (-0.30, 0.27, 0.82))}/>
    <geom name="floor" type="plane" size="3 3 0.1" material="grid" contype="1" conaffinity="3" friction="1 0.005 0.0001"/>
    <geom name="table" type="box" size="{_f(th)}" pos="{_f(TABLE_CENTER)}" rgba="0.72 0.58 0.42 1"
          contype="1" conaffinity="3" friction="{TABLE_FRICTION} 0.005 0.0001"/>
    <body name="cube" pos="{_f(CUBE_REF)}">
      <freejoint name="cube"/>
      <geom name="cube" type="box" size="{c:g} {c:g} {c:g}" mass="{CUBE_MASS:g}" rgba="0.85 0.2 0.15 1"
            contype="1" conaffinity="3" friction="{CUBE_FRICTION} 0.005 0.0001"/>
    </body>
{chr(10).join(robot)}
  </worldbody>
  <actuator>
{chr(10).join(acts)}
  </actuator>
</mujoco>
"""
    with open(OUT, "w", encoding="utf-8") as f:
        f.write(xml)
    return OUT


if __name__ == "__main__":
    path = build()
    import mujoco
    m = mujoco.MjModel.from_xml_path(path)
    print(f"wrote {os.path.relpath(path)}: {m.nbody} bodies, {m.njnt} joints, {m.nu} actuators, {m.ngeom} geoms")
