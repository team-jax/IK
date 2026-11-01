"""Cubes to go on top of one another: the delivery task with the spot a cube higher each time.

The base cube stays where it spawned and counts as placed from the start; the others are
carried onto it in turn, each a cube's height above the last. Everything the delivery settled
carries over - the grasp, the carry, the lie-down before the hand lets go - because the top
of a cube is a flat spot too. What is new is the height of the spot, that the spot can itself
be knocked over, and that the open hand has to come down beside the base cube without
sweeping it.
"""
from __future__ import annotations

import numpy as np

from humanoid_grasp_rl.env import (
    OBJECT_SIZE,
    OBJECT_Z,
    REACH_CELLS,
    REACH_XS,
    REACH_YS,
    HAND_CLEAR,
    SPAWN_CLEARANCE,
    JaxGraspEnv,
    _snap_to_band,
)

# The open hand's footprint around its grasp centre: the plates stand 90 mm apart along x (58 mm
# to their outer faces) and are 73 mm wide along y. Another cube is in the way of a hand coming
# down on a cube when it overlaps that rectangle, with half a cube and a centimetre to spare.
# The hand never turns, so the footprint is axis-aligned.
CLEAR_DX = 0.058 + OBJECT_SIZE / 2 + 0.01
CLEAR_DY = 0.0365 + OBJECT_SIZE / 2 + 0.01

# Spots for the fixed layout, in the order they are used: the base first, snapped onto the band.
# Two cubes is as tall as a tower goes: to pick a third, the hand has to carry it over a tower
# of two, which puts the grasp centre at z = 0.39 - higher than the arm can hold the fingers
# straight down (a three-cube layout was searched for and none passed the IK check).
NOMINAL_SPOTS = np.array([[0.04, -0.10], [0.04, -0.24]])


def _far_enough(spots) -> bool:
    """No two cubes close enough that the hand coming down on one would touch the other."""
    spots = np.asarray(spots)
    return all(abs(a[0] - b[0]) >= CLEAR_DX or abs(a[1] - b[1]) >= CLEAR_DY
               for i, a in enumerate(spots) for b in spots[i + 1:])


class JaxStackEnv(JaxGraspEnv):
    def __init__(self, cubes: int = 2, **kwargs):
        if not 2 <= cubes <= len(NOMINAL_SPOTS):
            raise ValueError(f"a tower is 2 to {len(NOMINAL_SPOTS)} cubes: the arm cannot carry a cube "
                             "high enough to clear a taller one")
        self.n_objects = cubes
        self.layout_fallbacks = 0
        self.layout_fell_back = False
        super().__init__(**kwargs)
        self.moving = 1
        # One delivery's worth of steps per cube that has to move.
        if kwargs.get("max_steps") is None:
            self.max_steps = 700 * (cubes - 1)

    def _nominal_layout(self) -> tuple[np.ndarray, np.ndarray]:
        spots = np.array([_snap_to_band(s) for s in NOMINAL_SPOTS[: self.n_objects]])
        assert _far_enough(spots), "the fixed stacking spots snapped onto each other"
        return spots, np.repeat(spots[:1], self.n_objects, axis=0)

    def _target_heights_for(self, objects, targets) -> np.ndarray:
        del objects, targets
        return OBJECT_Z + np.arange(self.n_objects) * OBJECT_SIZE

    def _sample_layout(self) -> tuple[np.ndarray, np.ndarray]:
        """Cube spots, all far enough apart; the base is the goal.

        Chosen from the reach cells outright, one cube at a time with backtracking, not by
        blending a drawn spot towards the fixed one: for three spots that must stay apart the
        constraint is not preserved along a line between two valid layouts, so blending
        mostly failed and the episode quietly fell back to the fixed layout. Here the scale
        sets how far from its fixed spot each cube may be drawn, and a layout the search
        cannot find is counted, not hidden.
        """
        nominal_objects, nominal_targets = self._nominal_layout()
        if self.stage != "randomize":
            return nominal_objects, nominal_targets
        spots = np.column_stack((REACH_XS[REACH_CELLS[:, 0]], REACH_YS[REACH_CELLS[:, 1]]))
        spread = self.randomization_scale * np.linalg.norm(
            spots[None, :, :] - nominal_objects[:, None, :], axis=2).max(axis=1)

        def candidates(i, chosen):
            near = np.linalg.norm(spots - nominal_objects[i], axis=1)
            allowed = near <= max(spread[i], near.min()) + 1e-9
            for c in chosen:
                allowed &= (np.abs(spots[:, 0] - c[0]) >= CLEAR_DX) | (np.abs(spots[:, 1] - c[1]) >= CLEAR_DY)
            return spots[allowed]

        def search(chosen):
            if len(chosen) == self.n_objects:
                return chosen
            options = candidates(len(chosen), chosen)
            for k in self.np_random.permutation(len(options))[:24]:
                found = search(chosen + [options[k]])
                if found is not None:
                    return found
            return None

        self.layout_fell_back = False
        for _ in range(8):
            found = search([])
            if found is None:
                break
            objects = np.array(found)
            # Off the grid a little, so spots are continuous; kept only where the layout
            # still holds. The cells themselves pass the same whole-layout check.
            jitter = self.np_random.uniform([-0.01, -0.0125], [0.01, 0.0125], size=objects.shape)
            shaken = objects + jitter * self.randomization_scale
            for layout in (shaken, objects):
                targets = np.repeat(layout[:1], self.n_objects, axis=0)
                if _far_enough(layout) and self._clear_of_home(layout) and self._servable(layout, targets):
                    return layout, targets
        self.layout_fallbacks += 1
        self.layout_fell_back = True
        return nominal_objects, nominal_targets

    def _clear_of_home(self, objects) -> bool:
        # Cubes start clear of the hand, and the tower (where every cube ends up) clear of
        # where the hand ends up, so that the withdrawal home leaves it HAND_CLEAR away.
        home = self._rest_grasp_centre[:2]
        return bool(np.linalg.norm(objects - home, axis=1).min() >= SPAWN_CLEARANCE
                    and np.linalg.norm(objects[0] - home) >= HAND_CLEAR + 0.03)

    def _moving_cubes(self) -> list[int]:
        return list(range(1, self.n_objects))

    def _info(self, success: bool) -> dict:
        info = super()._info(success)
        info["layout_fell_back"] = bool(getattr(self, "layout_fell_back", False))
        return info

    def reset(self, *, seed=None, options=None):
        # The controller advances `moving` as the tower goes up; a new episode starts over
        # with the first cube to be carried. Left as it was, every episode after the first in
        # the same process skipped cube 1 and set cube 2 on the base a cube too low.
        self.moving = 1
        observation, info = super().reset(seed=seed, options=options)
        # The base cube was never carried anywhere; it is on its spot because it spawned
        # there, and the placed test wants to know it was lifted first.
        self._lifted_latch[0] = True
        return observation, info
