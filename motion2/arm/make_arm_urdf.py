"""Regenerate the IK chain and MuJoCo scene from the team's final URDF (URDF_1108/JAXV3_Assembly).

Reads ../../URDF_1108/JAXV3_Assembly_description/urdf/JAXV3_Assembly.urdf (the arm's masses and
the finger collision shapes from ../../URDF_1006, see load()) and writes
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
PKG = os.path.join(HERE, "..", "..", "URDF_1108", "JAXV3_Assembly_description")
SRC = os.path.join(PKG, "urdf", "JAXV3_Assembly.urdf")
SRC_MESH = os.path.join(PKG, "meshes", "JAXV3_Assembly")
# The previous export (URDF_1006, V2). The masses/inertias and the finger collision STLs still come
# from it (load()).
V2_PKG = os.path.join(HERE, "..", "..", "URDF_1006", "JAXV2_Assembly_description")
V2_SRC = os.path.join(V2_PKG, "urdf", "JAXV2_Assembly.urdf")
V2_MESH = os.path.join(V2_PKG, "meshes", "JAXV2_Assembly")
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

# --- What load() takes from the V3 export, and what it keeps from V2 ------------------------------
# V3 gives the joint origins, limits and visual meshes. Four things are adapted:
#  - Its exporter appends "_1" to every link name but base_link (Shoulder_Pitch_1, Finger_1_1_1):
#    stripped, so the MJCF keeps the body names motion1 and motion2 refer to.
LINK_SUFFIX = "_1"
#  - V3 turns these two joints' frames 180 deg about their x axis, so the axis points the other way
#    and the limits are negated (shoulder yaw -50..0 deg where V2 had 0..45). The child link is
#    re-expressed in V2's orientation instead: every joint still turns about its local +z with V2's
#    sign, which the IK seed, motion1's poses and reach maps and the teammates' Isaac policies
#    (JaxIsaacLab JAXV3_Assembly.urdf flips the same two) are written in. On the original V3 URDF
#    these two joints read the negative of the model's angle.
FLIPPED_JOINTS = ("shoulder_roll_to_shoulder_yaw", "under_arm_to_wrist_pitch")
#  - Masses and inertias stay V2's: V3's are CAD material values (about 1.8 kg for the arm where
#    V2 has 4.1 kg) that the team has not confirmed by weighing the arm.
#  - The finger collision STLs stay V2's: V3's are the exporter's primitives, bloated (a finger's
#    first link 5 cm thick for a 1.6 cm plate), and V3's finger links are V2's parts in V2's frames.

# --- Hand / grasp frame -------------------------------------------------------------------------
# The MG996R (hand) frame has +z pointing back up the forearm and the three fingers extending
# along -z. The end effector is the grasp center in that frame, with its z axis opposite to the
# fingers, so "palm down" means end-frame z == world +z (the convention mujoco_receiver.py's IK
# relies on). The finger plates are thin in the hand x axis, so the opening gap is along x.
# Grasp center: midway between the finger-1/2 plane (x~-0.02) and finger 3 once it is swung to
# the far side, at the height where the proximal links meet the cube. It moves with the fingers:
# V3 puts them 3.5 mm nearer the wrist than V2 did (z -0.17 -> -0.1665) and finger 2 3 mm further
# from finger 1, so the pair's middle moves 1.5 mm in y (-0.001 -> 0.0005). Left at V2's place,
# check_grasp.py lifted 0 of 9 cubes; with z alone 8 of 9; with both 9 of 9 (2026-11-09).
GRASP_CENTER_HAND = (0.002, 0.0005, -0.1665)
# Yaw of the end frame about the hand z axis, so a palm-down pose can put the finger gap along a
# convenient world direction without changing the arm's own joint range.
END_YAW = 0.0
# The URDF's hand_base_to_rotation_base joint only swings finger 3 around to the opposite side of
# the cube (range -pi..0). It is held at this angle in arm.xml (the real MG90S is positioned once).
ROTATION_BASE_ANGLE = -3.141593

# --- Scene --------------------------------------------------------------------------------------
# base_link (the stand) is placed at this world position; the shoulder frame sits at
# (-0.15, 0, 1.2125) from it. That puts the shoulder frame at (0.34, 0, 0.85), where this scene and
# arm/ik.py's BASE_POS_WORLD have it, so V3's 1.2 m stand reaches 0.3625 m below the floor (it is
# only drawn: the stand collides with nothing). The arm reaches toward local -x, so the pedestal is
# placed at -x.
BASE_LINK_WORLD = (0.49, 0.0, -0.3625)
SHOULDER_OFFSET = (-0.15, 0.0, 1.2125)   # base_link_to_shoulder_pitch in the URDF (load() checks it)
CUBE_HALF = 0.015
PEDESTAL_POS_XY = (0.12, 0.0)
PEDESTAL_HALF_XY = 0.014   # a post narrower than the cube, so the fingers can reach below its bottom face
PEDESTAL_TOP_Z = 0.25

# The finger links weigh 0.01-0.07 kg, which makes MuJoCo's default (0.02 s) contact far too soft for
# them: the servos pushed 10-19 mm into the cube. A stiffer, harder contact on the finger geoms fixes it.
CONTACT_SOLREF = "0.006 1"
CONTACT_SOLIMP = "0.95 0.99 0.001"
# The finger pads' friction (sliding, torsional, rolling). MuJoCo uses the larger of two touching
# geoms' values, so a scene with its own cube friction (motion1) overrides this to match.
FINGER_FRICTION = "1.0 0.02 0.001"

# What the finger links collide as. "mesh" is the URDF's collision STL, whose convex hull meets a
# flat cube face at a single point; "box" is the axis-aligned box around that STL in the link's
# own frame, which meets it along the whole pad, so a squeezed cube cannot roll about the pinch
# axis. The plates are close enough to boxes that the proxy costs no reach.
FINGER_COLLISION = os.environ.get("JAX_FINGER_COLLISION", "box").strip().lower()
if FINGER_COLLISION not in ("box", "mesh"):
    raise SystemExit(f"JAX_FINGER_COLLISION={FINGER_COLLISION!r}: use box (the default) or mesh")

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


def _set_origin(elem, xyz, rotation):
    o = elem.find("origin")
    if o is None:
        o = ET.SubElement(elem, "origin")
    o.set("xyz", _fmt(xyz))
    o.set("rpy", _fmt(rotation.as_euler("xyz")))


class Urdf:
    """A parsed URDF. `inertia_from` and `collision_from` (another Urdf, with the directory holding
    its collision meshes) supply the inertials and the finger collision meshes instead of this one."""

    def __init__(self, path, inertia_from=None, collision_from=None):
        self.root = ET.parse(path).getroot()
        self.inertia_from = inertia_from
        self.collision_from = collision_from or (self, SRC_MESH)
        self._index()

    def _index(self):
        self.links = {l.get("name"): l for l in self.root.findall("link")}
        self.joints = {j.get("name"): j for j in self.root.findall("joint")}
        self.children = {}
        for j in self.joints.values():
            self.children.setdefault(j.find("parent").get("link"), []).append(j)

    def strip_link_suffix(self, suffix):
        """Drop `suffix` from every link name that ends with it (and from the joints naming them)."""
        def stripped(name):
            return name[:-len(suffix)] if name.endswith(suffix) else name
        for link in self.root.findall("link"):
            link.set("name", stripped(link.get("name")))
        for joint in self.root.findall("joint"):
            for end in ("parent", "child"):
                joint.find(end).set("link", stripped(joint.find(end).get("link")))
        self._index()

    def flip_child_frame(self, joint):
        """Turn `joint`'s child link frame 180 deg about its x axis. The joint's +z axis then points
        the other way (so its limits negate), and whatever is given in the child frame (its inertial,
        visual and collision origins, the next joints' origins) is re-expressed so nothing moves."""
        flip = Rot.from_quat([1.0, 0.0, 0.0, 0.0])   # exactly 180 deg about x (scipy order x, y, z, w)
        negate_yz = np.array([1.0, -1.0, -1.0])        # the same turn on a point (+ 0.0 below: no "-0")
        j = self.joints[joint]
        xyz, rpy = _origin(j)
        _set_origin(j, xyz, Rot.from_euler("xyz", rpy) * flip)
        lower, upper = self.limits(joint)
        j.find("limit").set("lower", f"{-upper + 0.0:.9g}")   # + 0.0: no "-0"
        j.find("limit").set("upper", f"{-lower + 0.0:.9g}")
        child = j.find("child").get("link")
        for i in self.links[child].findall("inertial"):   # y and z negate: so do ixy and ixz
            xyz, rpy = _origin(i)
            assert not np.any(rpy), f"{child}: a rotated inertial frame is not handled"
            i.find("origin").set("xyz", _fmt(negate_yz * xyz + 0.0))
            for k in ("ixy", "ixz"):
                i.find("inertia").set(k, f"{-float(i.find('inertia').get(k)):.9g}")
        for kind in ("visual", "collision"):
            for g in self.links[child].findall(kind):
                xyz, rpy = _origin(g)
                _set_origin(g, negate_yz * xyz + 0.0, flip * Rot.from_euler("xyz", rpy))
        for k in self.children.get(child, []):
            xyz, rpy = _origin(k)
            _set_origin(k, negate_yz * xyz + 0.0, flip * Rot.from_euler("xyz", rpy))

    def inertial(self, link):
        src = self.inertia_from or self
        i = src.links[link].find("inertial")
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

    def finger_collision(self, link):
        """(pos, rpy, mesh file path, scale) of a finger link's collision mesh."""
        src, mesh_dir = self.collision_from
        pos, rpy, name, scale = src.geom(link, "collision")
        return pos, rpy, os.path.join(mesh_dir, name), scale


def load():
    """The robot as this generator models it: URDF_1108 (V3) with V2's masses and finger collision
    meshes, link names without the exporter's suffix and V2's joint signs (see FLIPPED_JOINTS)."""
    v2 = Urdf(V2_SRC)
    urdf = Urdf(SRC, inertia_from=v2, collision_from=(v2, V2_MESH))
    urdf.strip_link_suffix(LINK_SUFFIX)
    for joint in FLIPPED_JOINTS:
        urdf.flip_child_frame(joint)
    missing = [l for l in urdf.links if l not in v2.links and l in _moving_links(urdf)]
    assert not missing, f"no V2 inertial for {missing}"
    offset, _ = urdf.joint_frame("base_link_to_shoulder_pitch")
    assert np.allclose(offset, SHOULDER_OFFSET), f"SHOULDER_OFFSET {SHOULDER_OFFSET} != URDF {offset}"
    return urdf


def _moving_links(urdf):
    """Shoulder_Pitch and everything below it: the links the scene gives an inertial."""
    out, todo = [], ["Shoulder_Pitch"]
    while todo:
        link = todo.pop()
        out.append(link)
        todo += [j.find("child").get("link") for j in urdf.children.get(link, [])]
    return out


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
        path = urdf.finger_collision(link)[2]
        shutil.copy(path, os.path.join(MESH_DIR, os.path.basename(path)))


# ------------------------------------------------------------------------------------------------
# arm.urdf (ikpy chain)
# ------------------------------------------------------------------------------------------------
def write_ik_urdf(urdf):
    end_rpy = (0.0, 0.0, END_YAW)
    out = [f"""<?xml version="1.0"?>
<!-- GENERATED by arm/make_arm_urdf.py from URDF_1108/JAXV3_Assembly_description (masses from URDF_1006)
  — do not hand-edit. 7-DOF arm chain for ikpy. Every joint rotates about its local z; zero pose hangs straight down.
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
        _, _, path, scale = urdf.finger_collision(link)
        lines.append(f'    <mesh name="{_mesh_name(link, "collision")}" file="{os.path.basename(path)}" scale="{scale}"/>')
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
    pos, rpy, path, scale = urdf.finger_collision(link)
    points = _stl_points(path) * float(scale.split()[0])
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
        contact = (f'contype="2" conaffinity="1" friction="{FINGER_FRICTION}" condim="4" '
                   f'solref="{CONTACT_SOLREF}" solimp="{CONTACT_SOLIMP}" group="3"')
        if FINGER_COLLISION == "box":
            pos, quat, half = _collision_box(urdf, link)
            lines.append(f'{pad}<geom name="{link}_collision_geom" type="box" size="{_fmt(half)}" '
                         f'pos="{_fmt(pos)}" quat="{_fmt(quat)}" {contact}/>')
        else:
            pos, rpy, _, _ = urdf.finger_collision(link)
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


def robot_parts(urdf, base_link_world=BASE_LINK_WORLD, base_yaw=0.0):
    """The robot's MJCF pieces, for any scene to compose: (asset meshes, column body, actuators).

    The column body holds the whole arm and hand and carries the `end_effector` site (the grasp
    centre); the meshes live in MESH_DIR, so a scene needs `meshdir` pointing there. motion1's
    scene (a table with several cubes) is built from these too, so both motions share one robot.
    `base_yaw` (radians) turns the whole robot about the world z axis at base_link_world; this
    file's own scene leaves it at 0.
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
    # Other parts fixed to the stand (V3's Holder): drawn only, no inertial or collision. Last in
    # the column, so the arm's body and geom ids stay as they were.
    fixed_lines = []
    for j in urdf.children.get("base_link", []):
        if j.get("name") != "base_link_to_shoulder_pitch":
            link = j.find("child").get("link")
            xyz, rpy = urdf.joint_frame(j.get("name"))
            fixed_lines += [f'      <body name="{link}" pos="{_fmt(xyz)}" quat="{_fmt(_quat_wxyz(rpy))}">',
                            *_geoms(urdf, link, 8), "      </body>"]
    turn = f' quat="{_fmt(_quat_wxyz((0.0, 0.0, base_yaw)))}"' if base_yaw else ""
    column = f"""    <body name="column" pos="{_fmt(np.array(base_link_world))}"{turn}>
{base_geoms}
      <body name="base_link" pos="{_fmt(SHOULDER_OFFSET)}">
        <inertial pos="{_fmt(spos)}" mass="{smass:.6f}" fullinertia="{_fmt(sfull)}"/>
{sp_geoms}
{chr(10).join(shoulder_lines)}
      </body>
{chr(10).join(fixed_lines)}
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
  <!-- GENERATED by arm/make_arm_urdf.py from URDF_1108/JAXV3_Assembly_description (masses and finger
    collision meshes from URDF_1006) — do not hand-edit. Grasping scene. The 7 arm joints mirror arm/arm.urdf (same joint order, frames, limits) so
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
    urdf = load()
    prepare_meshes(urdf)
    write_ik_urdf(urdf)
    write_mjcf(urdf)
    print(f"wrote {os.path.relpath(OUT_URDF)}, {os.path.relpath(OUT_XML)}, {os.path.relpath(MESH_DIR)}/")


if __name__ == "__main__":
    main()
