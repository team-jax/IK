"""Regenerate the IK chain and MuJoCo scene from the team's final URDF (URDF_1006/JAXV2_Assembly).

Reads ../../URDF_1006/JAXV2_Assembly_description/urdf/JAXV2_Assembly.urdf and writes
  arm.urdf   - 7-revolute-joint chain for ikpy (base_link = the fixed Shoulder_Pitch frame)
  arm.xml    - MuJoCo MJCF: the full arm + 3-finger hand, pedestal, cube, 13 position actuators
  meshes/    - OBJ visuals (converted from Collada) and the finger collision STLs
so the two models share one source of truth. Re-run after replacing the URDF package:

    venv/bin/python motion2/arm/make_arm_urdf.py

Everything tunable (hand/grasp frame, finger spacing, scene placement, servo gains) is a constant
below; do not hand-edit the generated files.
"""
import os
import shutil
import xml.etree.ElementTree as ET

import numpy as np
from scipy.spatial.transform import Rotation as Rot

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.join(HERE, "..", "..", "URDF_1006", "JAXV2_Assembly_description")
SRC = os.path.join(PKG, "urdf", "JAXV2_Assembly.urdf")
SRC_MESH = os.path.join(PKG, "meshes", "JAXV2_Assembly")
OUT_URDF = os.path.join(HERE, "arm.urdf")
OUT_XML = os.path.join(HERE, "arm.xml")
MESH_DIR = os.path.join(HERE, "meshes")

# URDF joint name -> project joint name, in chain order from the shoulder to the hand.
ARM_JOINTS = [
    ("shoulder_pitch_to_shoulder_roll", "shoulder_roll"),
    ("shoulder_roll_to_shoulder_yaw", "shoulder_yaw"),
    ("shoulder_yaw_to_upper_arm", "upper_arm_roll"),
    ("upper_arm_to_under_arm", "elbow"),
    ("under_arm_to_wrist_pitch", "wrist_pitch"),
    ("wrist_pitch_to_wrist_yaw", "wrist_yaw"),
    ("wrist_yaw_to_hand_base", "hand_roll"),
]
# (urdf joint, mujoco joint) for the two joints of each finger.
FINGER_JOINTS = [
    ("hand_base_to_finger_1_1", "finger1_joint1"), ("finger_1_1_to_finger_1_2", "finger1_joint2"),
    ("hand_base_to_finger_2_1", "finger2_joint1"), ("finger_1_2_to_finger_2_2", "finger2_joint2"),
    ("rotation_base_to_finger_3_1", "finger3_joint1"), ("finger_3_1_to_finger_3_2", "finger3_joint2"),
]
FINGER_LINKS = ["Finger_1_1", "Finger_1_2", "Finger_2_1", "Finger_2_2", "Finger_3_1", "Finger_3_2"]

# --- Hand / grasp frame -------------------------------------------------------------------------
# The MG996R (hand) frame has +z pointing back up the forearm and the three fingers extending
# along -z. The end effector is the grasp center in that frame, with its z axis opposite to the
# fingers, so "palm down" means end-frame z == world +z (the convention mujoco_receiver.py's IK
# relies on). The finger plates are thin in the hand x axis, so the opening gap is along x.
# Grasp center: midway between the finger-1/2 plane (x~-0.02) and finger 3 once it is swung to
# the far side, at the height where the proximal links meet the cube.
GRASP_CENTER_HAND = (0.002, -0.001, -0.17)
# Yaw of the end frame about the hand z axis, so a palm-down pose can put the finger gap along a
# convenient world direction without changing the arm's own joint range.
END_YAW = 0.0
# The URDF's hand_base_to_rotation_base joint only swings finger 3 around to the opposite side of
# the cube (range -pi..0). It is held at this angle in arm.xml (the real MG90S is positioned once).
ROTATION_BASE_ANGLE = -3.141593

# --- Scene --------------------------------------------------------------------------------------
# base_link (the column) stands on the floor at this world position; the shoulder frame sits at
# (-0.16, 0, 0.85) from it. The arm reaches toward local -x, so the pedestal is placed at -x.
BASE_LINK_WORLD = (0.50, 0.0, 0.0)
SHOULDER_OFFSET = (-0.16, 0.0, 0.85)   # base_link_to_shoulder_pitch in the URDF
CUBE_HALF = 0.015
PEDESTAL_POS_XY = (0.12, 0.0)
PEDESTAL_HALF_XY = 0.014   # a post narrower than the cube, so the fingers can reach below its bottom face
PEDESTAL_TOP_Z = 0.25

# The finger links weigh 0.01-0.07 kg, which makes MuJoCo's default (0.02 s) contact far too soft for
# them: the servos pushed 10-19 mm into the cube. A stiffer, harder contact on the finger geoms fixes it.
CONTACT_SOLREF = "0.006 1"
CONTACT_SOLIMP = "0.95 0.99 0.001"

# What the finger links collide as. "mesh" is the URDF's collision STL, whose convex hull meets a
# flat cube face at a single point; "box" is the axis-aligned box around that STL in the link's
# own frame, which meets it along the whole pad, so a squeezed cube cannot roll about the pinch
# axis. The plates are close enough to boxes that the proxy costs no reach.
FINGER_COLLISION = os.environ.get("JAX_FINGER_COLLISION", "box")

# Servo gains. Arm links weigh 0.2-1.1 kg, the fingers 0.01-0.07 kg, hence the very different kp.
ARM_KP, ARM_KV = 1000, 60
FINGER_KP, FINGER_KV = 5, 0.5
# Fingers 1 and 2 stand on one side of the cube and finger 3 alone on the other, so finger 3's servo
# is stiffer by the same factor to keep the squeeze balanced (otherwise the cube gets pushed away).
FINGER3_KP_SCALE = 2.0

COLLADA_NS = {"c": "http://www.collada.org/2005/11/COLLADASchema"}


# ------------------------------------------------------------------------------------------------
# URDF parsing helpers
# ------------------------------------------------------------------------------------------------
def _vec(text, default="0 0 0"):
    return np.array([float(v) for v in (text or default).split()])


def _origin(elem):
    o = elem.find("origin") if elem is not None else None
    if o is None:
        return np.zeros(3), np.zeros(3)
    return _vec(o.get("xyz")), _vec(o.get("rpy"))


def _quat_wxyz(rpy):
    x, y, z, w = Rot.from_euler("xyz", rpy).as_quat()  # URDF rpy = fixed-axis x, y, z
    return np.array([w, x, y, z])


def _fmt(values):
    return " ".join(f"{v:.9g}" for v in values)


class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {l.get("name"): l for l in root.findall("link")}
        self.joints = {j.get("name"): j for j in root.findall("joint")}
        self.children = {}
        for j in self.joints.values():
            self.children.setdefault(j.find("parent").get("link"), []).append(j)

    def inertial(self, link):
        i = self.links[link].find("inertial")
        pos, _ = _origin(i)
        m = float(i.find("mass").get("value"))
        t = i.find("inertia")
        full = [float(t.get(k)) for k in ("ixx", "iyy", "izz", "ixy", "ixz", "iyz")]
        return pos, m, full

    def joint_frame(self, joint):
        return _origin(self.joints[joint])

    def limits(self, joint):
        lim = self.joints[joint].find("limit")
        return float(lim.get("lower")), float(lim.get("upper"))

    def geom(self, link, kind):
        g = self.links[link].find(kind)
        if g is None:
            return None
        pos, rpy = _origin(g)
        mesh = g.find("geometry/mesh")
        return pos, rpy, os.path.basename(mesh.get("filename")), mesh.get("scale")


# ------------------------------------------------------------------------------------------------
# Mesh conversion
# ------------------------------------------------------------------------------------------------
def _source_values(mesh, source_id):
    source = mesh.find(f"c:source[@id='{source_id}']", COLLADA_NS)
    values = [float(v) for v in source.find("c:float_array", COLLADA_NS).text.split()]
    stride = int(source.find("c:technique_common/c:accessor", COLLADA_NS).get("stride", "1"))
    return [values[i:i + stride] for i in range(0, len(values), stride)]


def convert_dae_to_obj(dae_path, obj_path):
    """Convert the simple triangle-only Collada files emitted by the Fusion URDF exporter."""
    root = ET.parse(dae_path).getroot()
    mesh = root.find(".//c:geometry/c:mesh", COLLADA_NS)
    vertices_element = mesh.find("c:vertices", COLLADA_NS)
    position_ref = vertices_element.find("c:input[@semantic='POSITION']", COLLADA_NS).get("source")[1:]
    vertices = _source_values(mesh, position_ref)

    faces = []
    for triangles in mesh.findall("c:triangles", COLLADA_NS):
        inputs = triangles.findall("c:input", COLLADA_NS)
        stride = max(int(item.get("offset", "0")) for item in inputs) + 1
        vertex_offset = next(
            int(item.get("offset", "0")) for item in inputs if item.get("semantic") == "VERTEX")
        indices = [int(v) for v in triangles.find("c:p", COLLADA_NS).text.split()]
        vertex_indices = indices[vertex_offset::stride]
        faces.extend(zip(vertex_indices[0::3], vertex_indices[1::3], vertex_indices[2::3]))

    with open(obj_path, "w", encoding="utf-8") as out:
        out.write(f"# Generated from {os.path.basename(dae_path)} by make_arm_urdf.py\n")
        for v in vertices:
            out.write(f"v {v[0]:.9g} {v[1]:.9g} {v[2]:.9g}\n")
        for f in faces:
            out.write(f"f {f[0] + 1} {f[1] + 1} {f[2] + 1}\n")


def prepare_meshes(urdf):
    """OBJ for every visual, and a copy of the collision STL for the finger links only."""
    shutil.rmtree(MESH_DIR, ignore_errors=True)
    os.makedirs(MESH_DIR)
    for link in urdf.links:
        visual = urdf.geom(link, "visual")
        if visual:
            name = visual[2]
            convert_dae_to_obj(os.path.join(SRC_MESH, name),
                               os.path.join(MESH_DIR, name.replace(".dae", ".obj")))
    for link in FINGER_LINKS:
        name = urdf.geom(link, "collision")[2]
        shutil.copy(os.path.join(SRC_MESH, name), os.path.join(MESH_DIR, name))


# ------------------------------------------------------------------------------------------------
# arm.urdf (ikpy chain)
# ------------------------------------------------------------------------------------------------
def write_ik_urdf(urdf):
    end_rpy = (0.0, 0.0, END_YAW)
    out = [f"""<?xml version="1.0"?>
<!-- GENERATED by arm/make_arm_urdf.py from URDF_1006/JAXV2_Assembly_description — do not hand-edit.
  7-DOF arm chain for ikpy. Every joint rotates about its local z; zero pose hangs straight down.
  base_link is the fixed Shoulder_Pitch frame (the column's own offset lives in BASE_POS_WORLD,
  arm/ik.py). end_effector_link = grasp center between the fingers, z opposite to the finger
  direction, so z == world +z means fingers straight down (the palm-down convention
  mujoco_receiver.py's IK relies on). -->
<robot name="upper_arm">
  <link name="base_link"><inertial><mass value="0.5"/><inertia ixx="0.001" ixy="0" ixz="0" iyy="0.001" iyz="0" izz="0.001"/></inertial></link>
"""]
    parent = "base_link"
    for urdf_joint, name in ARM_JOINTS:
        j = urdf.joints[urdf_joint]
        child = f"{name}_link"
        xyz, rpy = urdf.joint_frame(urdf_joint)
        lower, upper = urdf.limits(urdf_joint)
        out.append(f"""
  <link name="{child}"><inertial><origin xyz="{_fmt(urdf.inertial(j.find('child').get('link'))[0])}"/><mass value="{urdf.inertial(j.find('child').get('link'))[1]:.6f}"/><inertia ixx="0.0005" ixy="0" ixz="0" iyy="0.0005" iyz="0" izz="0.0005"/></inertial></link>
  <joint name="{name}" type="revolute">
    <parent link="{parent}"/>
    <child link="{child}"/>
    <origin xyz="{_fmt(xyz)}" rpy="{_fmt(rpy)}"/>
    <axis xyz="0 0 1"/>
    <limit lower="{lower:.6f}" upper="{upper:.6f}" effort="10" velocity="3"/>
  </joint>""")
        parent = child
    out.append(f"""
  <link name="end_effector_link"/>
  <joint name="end_effector_fixed" type="fixed">
    <parent link="{parent}"/>
    <child link="end_effector_link"/>
    <origin xyz="{_fmt(GRASP_CENTER_HAND)}" rpy="{_fmt(end_rpy)}"/>
  </joint>
</robot>
""")
    with open(OUT_URDF, "w") as f:
        f.write("".join(out))


# ------------------------------------------------------------------------------------------------
# arm.xml (MuJoCo)
# ------------------------------------------------------------------------------------------------
def _mesh_name(link, kind):
    return f"{link}_{kind}"


def build_assets(urdf):
    lines = []
    for link in urdf.links:
        visual = urdf.geom(link, "visual")
        if visual:
            lines.append(f'    <mesh name="{_mesh_name(link, "visual")}" file="{visual[2].replace(".dae", ".obj")}"/>')
    for link in FINGER_LINKS:
        if FINGER_COLLISION == "box":
            continue
        name = urdf.geom(link, "collision")[2]
        lines.append(f'    <mesh name="{_mesh_name(link, "collision")}" file="{name}" scale="0.01 0.01 0.01"/>')
    return "\n".join(lines)


RGBA = {"base_link": "0.70 0.70 0.70 1", "MG996R": "0.702 0.553 0.004 1", "RotationBody": "1 1 1 1"}
FINGER_RGBA = {1: "0.8 0.82 0.93 1", 2: "0.9 0.92 0.93 1"}


def _stl_points(path):
    """Vertices of a binary or ASCII STL."""
    with open(path, "rb") as f:
        data = f.read()
    if data[:5] == b"solid" and b"facet" in data[:400]:
        text = data.decode("ascii", "ignore").split()
        return np.array([[float(text[i + 1]), float(text[i + 2]), float(text[i + 3])]
                         for i, word in enumerate(text) if word == "vertex"])
    count = int(np.frombuffer(data[80:84], dtype="<u4")[0])
    facets = np.frombuffer(data[84:84 + 50 * count],
                           dtype=np.dtype([("n", "<f4", 3), ("v", "<f4", 9), ("a", "<u2")]))
    return facets["v"].reshape(-1, 3).astype(float)


def _collision_box(urdf, link):
    """(pos, quat, half sizes) of the box around a link's collision STL, in the link's frame."""
    pos, rpy, name, scale = urdf.geom(link, "collision")
    points = _stl_points(os.path.join(SRC_MESH, name)) * float(scale.split()[0])
    low, high = points.min(axis=0), points.max(axis=0)
    rotation = Rot.from_euler("xyz", rpy)
    return pos + rotation.apply((low + high) / 2), _quat_wxyz(rpy), (high - low) / 2


def _geoms(urdf, link, indent):
    pad = " " * indent
    lines = []
    visual = urdf.geom(link, "visual")
    if visual:
        pos, rpy, _, _ = visual
        if link in FINGER_LINKS:
            rgba = FINGER_RGBA[2 if link.endswith("_2") else 1]
        else:
            rgba = RGBA.get(link, "0.098 0.098 0.098 1")
        lines.append(f'{pad}<geom type="mesh" mesh="{_mesh_name(link, "visual")}" pos="{_fmt(pos)}" '
                     f'quat="{_fmt(_quat_wxyz(rpy))}" rgba="{rgba}"/>')
    if link in FINGER_LINKS:
        contact = (f'contype="2" conaffinity="1" friction="1.0 0.02 0.001" condim="4" '
                   f'solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}" group="3"')
        if FINGER_COLLISION == "box":
            pos, quat, half = _collision_box(urdf, link)
            lines.append(f'{pad}<geom name="{link}_collision_geom" type="box" size="{_fmt(half)}" '
                         f'pos="{_fmt(pos)}" quat="{_fmt(quat)}" {contact}/>')
        else:
            pos, rpy, _, _ = urdf.geom(link, "collision")
            lines.append(f'{pad}<geom name="{link}_collision_geom" type="mesh" mesh="{_mesh_name(link, "collision")}" '
                         f'pos="{_fmt(pos)}" quat="{_fmt(_quat_wxyz(rpy))}" {contact}/>')  # group 3: hidden in the viewer
    return lines


def _body(urdf, link, joint, indent, joint_names):
    """Emit the body for `link` (reached through URDF joint `joint`), then recurse."""
    pad = " " * indent
    j = urdf.joints[joint]
    xyz, rpy = urdf.joint_frame(joint)
    mj_joint = joint_names.get(joint)
    rotation_base = joint == "hand_base_to_rotation_base"
    if rotation_base:  # held at ROTATION_BASE_ANGLE: compose the fixed rotation into the frame
        r = Rot.from_euler("xyz", rpy) * Rot.from_euler("z", ROTATION_BASE_ANGLE)
        x, y, z, w = r.as_quat()
        quat = np.array([w, x, y, z])
    else:
        quat = _quat_wxyz(rpy)
    lines = [f'{pad}<body name="{link}" pos="{_fmt(xyz)}" quat="{_fmt(quat)}">']
    if mj_joint:
        lower, upper = urdf.limits(joint)
        lines.append(f'{pad}  <joint name="{mj_joint}" type="hinge" axis="0 0 1" range="{lower:.6f} {upper:.6f}"/>')
    ipos, mass, full = urdf.inertial(link)
    lines.append(f'{pad}  <inertial pos="{_fmt(ipos)}" mass="{mass:.6f}" fullinertia="{_fmt(full)}"/>')
    lines += _geoms(urdf, link, indent + 2)
    # The swung finger-3 body goes last so the finger hinges stay in finger1, finger2, finger3 order.
    for child_joint in sorted(urdf.children.get(link, []), key=lambda c: c.get("name") == "hand_base_to_rotation_base"):
        lines += _body(urdf, child_joint.find("child").get("link"), child_joint.get("name"), indent + 2, joint_names)
    lines.append(f"{pad}</body>")
    return lines


# Gripper joints whose MJCF counterpart exists. hand_base_to_rotation_base is held fixed.
def _joint_name_map():
    m = {u: n for u, n in ARM_JOINTS}
    m.update({u: n for u, n in FINGER_JOINTS})
    return m


def robot_parts(urdf, base_link_world=BASE_LINK_WORLD):
    """The robot's MJCF pieces, for any scene to compose: (asset meshes, column body, actuators).

    The column body holds the whole arm and hand and carries the `end_effector` site (the grasp
    centre); the meshes live in MESH_DIR, so a scene needs `meshdir` pointing there. motion1's
    scene (a table with several cubes) is built from these too, so both motions share one robot.
    """
    joint_names = _joint_name_map()
    shoulder_lines = []
    for child_joint in urdf.children["Shoulder_Pitch"]:
        shoulder_lines += _body(urdf, child_joint.find("child").get("link"), child_joint.get("name"), 8, joint_names)
    # end_effector site lives in the MG996R body: insert before its first child body.
    site = (f'            <site name="end_effector" pos="{_fmt(GRASP_CENTER_HAND)}" '
            f'quat="{_fmt(_quat_wxyz((0, 0, END_YAW)))}" size="0.01" rgba="0 1 0 1"/>')
    for i, line in enumerate(shoulder_lines):
        if line.strip().startswith('<body name="MG996R"'):
            k = i + 1  # after this body's inertial + geom lines
            while not shoulder_lines[k].strip().startswith("<body"):
                k += 1
            shoulder_lines.insert(k, site)
            break
    base_geoms = "\n".join(_geoms(urdf, "base_link", 6))
    sp_geoms = "\n".join(_geoms(urdf, "Shoulder_Pitch", 8))
    spos, smass, sfull = urdf.inertial("Shoulder_Pitch")
    column = f"""    <body name="column" pos="{_fmt(np.array(base_link_world))}">
{base_geoms}
      <body name="base_link" pos="{_fmt(SHOULDER_OFFSET)}">
        <inertial pos="{_fmt(spos)}" mass="{smass:.6f}" fullinertia="{_fmt(sfull)}"/>
{sp_geoms}
{chr(10).join(shoulder_lines)}
      </body>
    </body>"""
    arm_acts = "\n".join(
        f'    <position name="{n}_act" joint="{n}" kp="{ARM_KP}" kv="{ARM_KV}" '
        f'ctrlrange="{urdf.limits(u)[0]:.6f} {urdf.limits(u)[1]:.6f}"/>' for u, n in ARM_JOINTS)
    finger_acts = "\n".join(
        f'    <position name="{n}_act" joint="{n}" '
        f'kp="{FINGER_KP * (FINGER3_KP_SCALE if n.startswith("finger3") else 1):g}" '
        f'kv="{FINGER_KV * (FINGER3_KP_SCALE if n.startswith("finger3") else 1):g}" '
        f'ctrlrange="{urdf.limits(u)[0]:.6f} {urdf.limits(u)[1]:.6f}"/>' for u, n in FINGER_JOINTS)
    return build_assets(urdf), column, f"{arm_acts}\n{finger_acts}"


def write_mjcf(urdf):
    assets, column, actuators = robot_parts(urdf)
    cube_z = PEDESTAL_TOP_Z + CUBE_HALF
    px, py = PEDESTAL_POS_XY
    xml = f"""<mujoco model="upper_arm">
  <!-- GENERATED by arm/make_arm_urdf.py from URDF_1006/JAXV2_Assembly_description — do not hand-edit.
    Grasping scene. The 7 arm joints mirror arm/arm.urdf (same joint order, frames, limits) so
    angles solved by ikpy map 1:1 onto qpos[0:7] — see arm/ik.py. The hand adds 6 driven finger
    hinges (qpos[7:13]); finger 3's spacing joint is held fixed (ROTATION_BASE_ANGLE). The graspable
    cube is a freejoint (qpos[13:20]); 13 position actuators are driven by mujoco_receiver.py. -->
  <compiler angle="radian" meshdir="meshes" balanceinertia="true"/>
  <option timestep="0.002" integrator="implicitfast" gravity="0 0 -9.81"/>

  <default>
    <joint damping="0.2" frictionloss="0.05"/>
    <geom contype="0" conaffinity="0"/>
  </default>

  <asset>
{assets}
  </asset>

  <worldbody>
    <light pos="0 0 2" dir="0 0 -1" diffuse="1 1 1"/>
    <!-- Collision bitmasks (default contype=0 conaffinity=0 — arm links never collide):
         fingers contype=2 conaffinity=1; cube/pedestal/floor contype=1 conaffinity=3.
         finger↔finger is excluded, finger↔cube/pedestal and cube↔pedestal/floor are active. -->
    <geom name="floor" type="plane" size="1 1 0.1" contype="1" conaffinity="3" rgba="0.8 0.8 0.8 1"/>
    <geom name="pedestal" type="box" pos="{px} {py} {PEDESTAL_TOP_Z / 2}" size="{PEDESTAL_HALF_XY} {PEDESTAL_HALF_XY} {PEDESTAL_TOP_Z / 2}"
          contype="1" conaffinity="3" rgba="0.5 0.4 0.3 1"/>

{column}

    <!-- Graspable cube, defined AFTER the arm so the arm+finger hinges keep qpos[0:13]. Spawned on
         the pedestal face so the first mj_step starts penetration-free. -->
    <body name="object" pos="{px} {py} {cube_z}">
      <freejoint name="object_free"/>
      <geom name="object_geom" type="box" size="{CUBE_HALF} {CUBE_HALF} {CUBE_HALF}" mass="0.05"
            friction="1.0 0.02 0.001" condim="4" contype="1" conaffinity="3"
            solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}" rgba="0.9 0.2 0.2 1"/>
    </body>
  </worldbody>

  <actuator>
    <!-- Position servos: hand-tracking targets and mj_step contact physics coexist. -->
{actuators}
  </actuator>
</mujoco>
"""
    with open(OUT_XML, "w") as f:
        f.write(xml)


def main():
    urdf = Urdf(SRC)
    prepare_meshes(urdf)
    write_ik_urdf(urdf)
    write_mjcf(urdf)
    print(f"wrote {os.path.relpath(OUT_URDF)}, {os.path.relpath(OUT_XML)}, {os.path.relpath(MESH_DIR)}/")


if __name__ == "__main__":
    main()
