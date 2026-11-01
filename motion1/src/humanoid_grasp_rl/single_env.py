"""One cube, one goal spot: the delivery task the scripted controller solves.

Two cubes ask for the grasp twice at once while the idle cube can be knocked around or shoved
onto the other; one cube is the task with everything but the grasp and the placement removed,
and it is the one every measurement in this project is made on.
"""
from __future__ import annotations

from humanoid_grasp_rl.env import JaxGraspEnv


class JaxDeliveryEnv(JaxGraspEnv):
    n_objects = 1
