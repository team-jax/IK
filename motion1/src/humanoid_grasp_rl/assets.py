"""The robot: the team's final URDF (URDF_1108/JAXV3_Assembly), through motion2's generator.

motion1 and motion2 drive the same arm and hand, so the model is written once, by
motion2/arm/make_arm_urdf.py, and each motion composes its own scene from the pieces it
exposes. This module loads that generator from the sibling directory (override with the
JAX_ARM_DIR environment variable) and makes sure the OBJ/STL meshes it writes exist.
"""
from __future__ import annotations

import importlib.util
import math
import os
from dataclasses import dataclass
from pathlib import Path


@dataclass(frozen=True)
class RobotParts:
    assets: str          # <mesh .../> lines for the <asset> block
    column: str          # the <body name="column"> subtree: column, arm, hand, end_effector site
    actuators: str       # 13 <position> actuators: 7 arm joints then 6 finger joints
    mesh_dir: str        # what the scene's <compiler meshdir> must point at
    base_link_world: tuple
    shoulder_world: tuple
    contact_solref: str
    contact_solimp: str
    arm_joints: tuple
    finger_joints: tuple


def _generator():
    root = Path(os.environ.get("JAX_ARM_DIR", Path(__file__).resolve().parents[3] / "motion2" / "arm"))
    path = root / "make_arm_urdf.py"
    if not path.is_file():
        raise FileNotFoundError(f"{path} not found: motion1 builds its robot with motion2's generator "
                                "(set JAX_ARM_DIR to the directory holding make_arm_urdf.py)")
    spec = importlib.util.spec_from_file_location("jax_arm_generator", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def robot_parts(finger_sliding_friction: float | None = None, shoulder_z: float | None = None,
                base_yaw_deg: float = 0.0) -> RobotParts:
    """The robot's pieces; `finger_sliding_friction` replaces the pads' sliding coefficient (the
    generator's torsional and rolling values are kept), and `shoulder_z` moves the whole robot,
    stand included, until its shoulder frame is that high. V3's stand puts the shoulder frame
    1.2125 m above its foot, so at the real setup's 1.2125 m the foot stands on the floor, and at
    the sim setup's height far below it: the stand is only drawn and collides with nothing.
    `base_yaw_deg` turns the robot about the vertical through its shoulder frame, which stays where
    the generator puts it (world x 0.34, y 0): -90 faces the stand's FRONT side towards world -x."""
    gen = _generator()
    if finger_sliding_friction is not None:
        gen.FINGER_FRICTION = " ".join([f"{finger_sliding_friction:g}"] + gen.FINGER_FRICTION.split()[1:])
    urdf = gen.load()
    if not (Path(gen.MESH_DIR) / "base_link.obj").is_file():
        gen.prepare_meshes(urdf)
    shoulder = [b + o for b, o in zip(gen.BASE_LINK_WORLD, gen.SHOULDER_OFFSET)]
    if shoulder_z is not None:
        shoulder[2] = shoulder_z
    yaw = math.radians(base_yaw_deg)
    c, s = round(math.cos(yaw), 12), round(math.sin(yaw), 12)   # exact for quarter turns
    ox, oy, oz = gen.SHOULDER_OFFSET
    # rounded so that the generator's own placement gives exactly its base
    base = tuple(round(v, 6) for v in (shoulder[0] - (c * ox - s * oy), shoulder[1] - (s * ox + c * oy),
                                       shoulder[2] - oz))
    assets, column, actuators = gen.robot_parts(urdf, base_link_world=base, base_yaw=yaw)
    shoulder = tuple(shoulder)
    return RobotParts(
        assets=assets, column=column, actuators=actuators, mesh_dir=str(Path(gen.MESH_DIR).resolve()),
        base_link_world=base, shoulder_world=shoulder,
        contact_solref=gen.CONTACT_SOLREF, contact_solimp=gen.CONTACT_SOLIMP,
        arm_joints=tuple(name for _, name in gen.ARM_JOINTS),
        finger_joints=tuple(name for _, name in gen.FINGER_JOINTS),
    )
