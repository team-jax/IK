"""The robot: the team's final URDF (URDF_1006/JAXV2_Assembly), through motion2's generator.

motion1 and motion2 drive the same arm and hand, so the model is written once, by
motion2/arm/make_arm_urdf.py, and each motion composes its own scene from the pieces it
exposes. This module loads that generator from the sibling directory (override with the
JAX_ARM_DIR environment variable) and makes sure the OBJ/STL meshes it writes exist.
"""
from __future__ import annotations

import importlib.util
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


def robot_parts() -> RobotParts:
    gen = _generator()
    urdf = gen.Urdf(gen.SRC)
    if not (Path(gen.MESH_DIR) / "base_link.obj").is_file():
        gen.prepare_meshes(urdf)
    assets, column, actuators = gen.robot_parts(urdf)
    base = tuple(gen.BASE_LINK_WORLD)
    shoulder = tuple(b + o for b, o in zip(base, gen.SHOULDER_OFFSET))
    return RobotParts(
        assets=assets, column=column, actuators=actuators, mesh_dir=str(Path(gen.MESH_DIR).resolve()),
        base_link_world=base, shoulder_world=shoulder,
        contact_solref=gen.CONTACT_SOLREF, contact_solimp=gen.CONTACT_SOLIMP,
        arm_joints=tuple(name for _, name in gen.ARM_JOINTS),
        finger_joints=tuple(name for _, name in gen.FINGER_JOINTS),
    )
