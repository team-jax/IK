from __future__ import annotations

import mujoco
import numpy as np


# How hard the spare joint freedom is pulled back toward the home posture, relative to the
# task error. A seven-jointed arm on a six-part task has one spare dimension, and left to
# itself it wanders towards a joint stop; pulling it towards a fixed posture makes the same
# commanded pose give the same arm, and keeps the elbow out of the way of the table.
NULLSPACE_PULL = 0.10


def rotation_error(current: np.ndarray, target: np.ndarray) -> np.ndarray:
    """The rotation vector, in the world frame, that turns `current` onto `target`.

    A real axis-angle, not the half-sum of column cross products: that expression is
    sin-scaled and vanishes at 180 degrees, so a hand facing exactly the wrong way would
    solve as a perfect match.
    """
    q_current, q_target, local = np.zeros(4), np.zeros(4), np.zeros(3)
    mujoco.mju_mat2Quat(q_current, np.asarray(current, dtype=np.float64).reshape(9))
    mujoco.mju_mat2Quat(q_target, np.asarray(target, dtype=np.float64).reshape(9))
    # mju_subQuat gives the turn in the current frame; the Jacobian is in the world's.
    mujoco.mju_subQuat(local, q_target, q_current)
    return np.asarray(current).reshape(3, 3) @ local


def _refresh_kinematics(model: mujoco.MjModel, data: mujoco.MjData) -> None:
    """Body and site poses plus the subtree COMs `mj_jacSite` reads, without the dynamics."""
    mujoco.mj_kinematics(model, data)
    mujoco.mj_comPos(model, data)


def solve_site_pose(
    model: mujoco.MjModel,
    data: mujoco.MjData,
    site_id: int,
    joint_ids: np.ndarray,
    target_position: np.ndarray,
    target_rotation: np.ndarray,
    home: np.ndarray,
    *,
    position_tolerance: float = 0.025,
    rotation_tolerance: float = 0.08,
    max_iterations: int = 600,
) -> tuple[bool, float, float]:
    """Move `site_id` onto a pose by damped least squares, within the joint limits.

    Starts from whatever `data.qpos` holds, so a caller that wants continuity seeds it with
    the previous solution. `home` is the posture the spare joint freedom is pulled towards.
    """
    qpos_ids = model.jnt_qposadr[joint_ids]
    dof_ids = model.jnt_dofadr[joint_ids]
    limits = model.jnt_range[joint_ids]
    jacobian_position = np.zeros((3, model.nv))
    jacobian_rotation = np.zeros((3, model.nv))
    damping = 1e-3 * np.eye(6)

    for _ in range(max_iterations):
        _refresh_kinematics(model, data)
        rotation = data.site_xmat[site_id].reshape(3, 3)
        position_error = np.asarray(target_position) - data.site_xpos[site_id]
        turn = rotation_error(rotation, target_rotation)
        if np.linalg.norm(position_error) <= position_tolerance and np.linalg.norm(turn) <= rotation_tolerance:
            return True, float(np.linalg.norm(position_error)), float(np.linalg.norm(turn))
        mujoco.mj_jacSite(model, data, jacobian_position, jacobian_rotation, site_id)
        jacobian = np.vstack((jacobian_position[:, dof_ids], 0.25 * jacobian_rotation[:, dof_ids]))
        error = np.concatenate((position_error, 0.25 * turn))
        pseudo = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + damping, error)
        toward_home = NULLSPACE_PULL * (home - data.qpos[qpos_ids])
        nullspace = toward_home - jacobian.T @ np.linalg.solve(
            jacobian @ jacobian.T + damping, jacobian @ toward_home)
        data.qpos[qpos_ids] = np.clip(
            data.qpos[qpos_ids] + 0.25 * (pseudo + nullspace), limits[:, 0], limits[:, 1])

    # The last update moved the joints; the error has to be read after the kinematics catch up.
    _refresh_kinematics(model, data)
    rotation = data.site_xmat[site_id].reshape(3, 3)
    position_error = float(np.linalg.norm(np.asarray(target_position) - data.site_xpos[site_id]))
    final_turn = float(np.linalg.norm(rotation_error(rotation, target_rotation)))
    return position_error <= position_tolerance and final_turn <= rotation_tolerance, position_error, final_turn
