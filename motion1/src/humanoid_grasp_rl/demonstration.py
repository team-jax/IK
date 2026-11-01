"""The scripted pick-and-place, and the pose arithmetic it stands on.

Nothing here is learned. Every pose is measured on the hand and written down next to the place
it is used, and the controller is a state machine whose transitions are conditions on the
scene rather than timers: a phase has a deadline, but a deadline is a failure to arrive, not
an arrival.
"""
from __future__ import annotations

import numpy as np

from humanoid_grasp_rl.env import (
    CUBE_GRIP,
    GRIP_RATE,
    HAND_CLEAR,
    OBJECT_Z,
    POSITION_ACTION_SCALE,
    TARGET_TOLERANCE,
    JaxGraspEnv,
)

# How lightly the hand holds the cube while the cube lies down: the closure whose pads are a
# few millimetres wider than the cube, so it can turn flat inside the hand instead of being
# flicked out of it.
SETTLE_GRIP = CUBE_GRIP - 0.11
# How fast the hand may swing while carrying, and lift away from the table. 5 mm a step
# (0.25 m/s) pulls a 50 g cube out of the pads; at 2 mm a step 20 random layouts in 20 were
# delivered, at 3 mm 17.
CROSS_RATE = 0.002
UP_RATE = 0.002
# How far above the grasp pose the hand approaches from and carries at (at least).
TRAVEL_CLEAR = 0.06
# How many times a cube's delivery may recover from a phase that ran out of time before the
# hand gives up and parks.
MAX_RETRIES = 2
# How fast the hand comes down onto a cube. Driven at the full 10 mm a step it arrives hard
# enough to knock the cube out from under itself.
DOWN_RATE = 0.0015


def _recover(env: JaxGraspEnv, phase: str, moving: int, carrying: bool, hand, cube) -> str:
    """Where to go when a phase fails - by what the scene allows, not by the phase's name.

    A deadline is a failure, not an arrival: the failure is written down with what the hand
    was doing, and the next phase is chosen by the state of the cube: still in hand, go
    straight up and aim again; resting on its spot, finish letting go and withdraw; anywhere
    else on the table, start over from where it actually is. Retries are capped; after that
    the hand holds still where it is, holding whatever it holds.
    """
    env._palm_failures.append((env._step_count, moving, phase, "carrying" if carrying else "empty"))
    env._palm_retries += 1
    env._palm_since = env._step_count
    env._palm_settle = 0
    env.__dict__.pop("_carry_aim", None)
    on_spot = (env._resting_on_support(moving)
               and np.linalg.norm(cube[:2] - env._target_positions[moving]) < TARGET_TOLERANCE)
    if env._palm_retries > MAX_RETRIES:
        env._palm_phase = "park"
    elif carrying:
        # Straight up first, then aim again from carry height. Moving sideways from beside a
        # tower with a cube in hand is how a tower gets swept.
        env._rise_from = hand[:2].copy()
        env._palm_phase = "up"
    elif phase in ("settle", "let go") and on_spot:
        env._release_at = hand.copy()
        env._palm_phase = "let go" if phase == "settle" else "back"
    elif cube[2] < OBJECT_Z + 0.02:
        # On the table, wherever it got to: start over from there.
        env._pick_spot = cube[:2].copy()
        env._pick_height = 0.0
        env._palm_phase = "reach"
    else:
        # Lying on the tower, or on another cube. Coming down on it there means coming down
        # on the tower; nothing this controller can do from here is safe, so it stops.
        env._palm_failures.append((env._step_count, moving, "off-table", "parked"))
        env._palm_phase = "park"
    return env._palm_phase


def scripted_palm_place(env: JaxGraspEnv, step: int) -> np.ndarray:
    """Pick a cube up from above and set it on its spot.

    The cube is whichever one the env says is moving, and the spot is wherever and however
    high the env says it is - on the table for a delivery, on top of another cube for a stack.
    The phases do not care which.

    The hand comes down on the cube, closes, goes straight up, crosses at height, comes down
    until the table takes the cube, lets the cube lie down flat while still loosely holding
    it, opens, and withdraws. The cube does not move until the fingers do, and it does not
    move after the fingers let go.
    """
    del step
    moving = env.moving
    cube = env.data.xpos[env.object_body_ids[moving]]
    goal = env._target_positions[moving]
    hand = env.grasp_centre
    grip = env._grip
    cube_speed = float(np.linalg.norm(env.data.cvel[env.object_body_ids[moving], 3:]))
    cube_spin = float(np.linalg.norm(env.data.cvel[env.object_body_ids[moving], :3]))
    # How far the cube is from resting on one of its faces. Measured against its nearest face
    # normal, not its own z axis: a cube lies flat on any of its six faces.
    cube_frame = env.data.xmat[env.object_body_ids[moving]].reshape(3, 3)
    off_a_face = float(np.degrees(np.arccos(np.clip(np.abs(cube_frame[2, :]).max(), 0.0, 1.0))))
    _, touching, supported = env._contacts
    carrying = bool(touching[moving] and not supported[moving])

    phase = getattr(env, "_palm_phase", None)
    if phase is None or env._step_count <= 1:
        phase = env._palm_phase = "reach"
        env._palm_settle = 0
        env._palm_since = env._step_count
        env._palm_retries = 0
        env._palm_failures = []
        env._pick_spot = env._initial_object_positions[moving][:2].copy()
        env._pick_height = 0.0
        env._rise_from = env._pose_target_position[:2].copy()
        env._descend_from = env._pose_target_position.copy()
        env._release_at = env._pose_target_position.copy()
        env.__dict__.pop("_carry_aim", None)

    pick = env.pick_pose(env._pick_spot, env._pick_height)
    # The spot's own height: the table for a delivery, a cube higher for a stack.
    drop = env.drop_pose(goal, env._target_heights[moving])
    # Two heights. The hand approaches the cube from a fixed clearance above the grasp, and
    # carries at a clearance above whichever of grasp and spot is higher, and above the tallest
    # cube standing now: a tower would otherwise be swept over by the hand leaving for the next cube.
    standing = max((env.data.xpos[env.object_body_ids[j]][2] for j in range(env.n_objects) if j != moving),
                   default=OBJECT_Z)
    clear_z = env.clear_height(standing)
    approach_z = max(pick[2] + TRAVEL_CLEAR, clear_z)
    over_pick = np.array([pick[0], pick[1], approach_z])
    travel = max(pick[2] + TRAVEL_CLEAR, drop[2] + TRAVEL_CLEAR, clear_z)
    over_drop = np.array([drop[0], drop[1], travel])
    home = env._initial_pose_position
    retreat = home
    # The task counts a cube delivered only once the grasp centre is HAND_CLEAR from its spot,
    # so that distance is what the withdrawal drives.
    top = np.array([goal[0], goal[1], env._target_heights.max()])
    clear = float(np.linalg.norm(hand - top)) >= HAND_CLEAR + 0.02

    # Every phase gets a deadline. The arm settles a few millimetres short of the pose it is
    # asked for, so a test written against an exact arrival can wait forever. The descent's
    # deadline follows the descent: it starts wherever the clear-air height is.
    down_budget = int((over_pick[2] - pick[2]) / DOWN_RATE) + 30
    # The crossing's deadline follows its speed: the whole table is 0.4 m at CROSS_RATE.
    cross_budget = int(0.4 / CROSS_RATE) + 60
    budget = {"down": down_budget, "grip": 60, "settle": 120, "let go": 90, "lower": 90,
              "cross": cross_budget}.get(phase, 100)
    if env._step_count - env._palm_since > budget and phase not in ("back", "park"):
        phase = _recover(env, phase, moving, carrying, hand, cube)

    target, wanted = over_pick, 0.0
    if phase == "reach":
        # Straight up from where the hand is, then across: a diagonal from home would sweep
        # the open fingers through whatever stands between.
        if hand[2] < approach_z - 0.015 and np.linalg.norm(hand[:2] - over_pick[:2]) > 0.02:
            target = np.array([hand[0], hand[1], approach_z])
        else:
            target = over_pick
        wanted = 0.0
        if np.linalg.norm(hand - over_pick) < 0.01:
            env._palm_phase = "down"
            env._palm_since = env._step_count
    elif phase == "down":
        target, wanted = pick, 0.0
        if np.linalg.norm(hand - pick) < 0.01:
            env._palm_phase = "grip"
            env._palm_since = env._step_count
    elif phase == "grip":
        # Closed is closed; whether the cube is off the table cannot be the test here, because
        # it only leaves the table once the hand lifts.
        target, wanted = pick, min(grip + 0.025, 1.0)
        env._palm_settle = env._palm_settle + 1 if grip >= 1.0 - 1e-6 else 0
        if env._palm_settle > 8:
            env._palm_phase = "up"
            env._palm_since = env._step_count
            env._rise_from = hand[:2].copy()
    elif phase == "up":
        rise = getattr(env, "_rise_from", hand[:2])
        target, wanted = np.array([rise[0], rise[1], travel]), 1.0
        if hand[2] > travel - 0.012:
            env._palm_phase = "cross"
            env._palm_since = env._step_count
    elif phase == "cross":
        # Steer the cube, not the wrist. Where a cube sits in the hand varies by a few
        # millimetres from grasp to grasp, and placing open loop against an average costs
        # accuracy. The cube can be seen, so the hand moves by whatever the cube is off - but
        # only while the hand still has it: a cube dropped mid-carry sits on the table, and
        # aiming at *that* walks the arm off across the table until the leash saturates.
        if carrying:
            env._carry_aim = np.array([goal[0] - (cube[0] - hand[0]), goal[1] - (cube[1] - hand[1]), travel])
        target = getattr(env, "_carry_aim", over_drop)
        wanted = 1.0
        # Wait for the cube to stop, not just to pass over the spot - or, failing stillness,
        # for it to have stayed over the spot: an arm sitting against a joint limit jitters at
        # a few centimetres a second and never reads as still.
        aligned = np.linalg.norm(cube[:2] - goal) < 0.012
        env._palm_settle = env._palm_settle + 1 if (aligned and cube_speed < 0.30) else 0
        if not carrying and env._resting_on_support(moving) and not touching[moving]:
            # It is lying on something and the hand is not on it: dropped. Go and get it,
            # and do nothing else this step.
            phase = _recover(env, "cross", moving, False, hand, cube)
            target, wanted = env._pose_target_position.copy(), grip
        elif carrying and aligned and (cube_speed < 0.02 or env._palm_settle >= 30):
            env._palm_phase = "lower"
            env._palm_since = env._step_count
            env._palm_settle = 0
            # The hand, not the commanded pose: the pose runs up to its leash ahead of the
            # hand, and freezing that freezes in whatever of the leash was outstanding.
            env._descend_from = hand.copy()
    elif phase == "lower":
        # Straight down, no sideways correction, until the table takes the cube. Aiming at
        # the cube all the way down turns slip into a chase: the cube shifts, the controller
        # answers by moving the hand sideways, and that drags the cube further.
        aimed = env._descend_from
        floor = drop[2] - 0.05
        target = np.array([aimed[0], aimed[1], max(hand[2] - 0.03, floor)])
        wanted = 1.0
        landed = env._resting_on_support(moving)
        if landed and not touching[moving]:
            # Resting on something with no hand on it: it fell out on the way down.
            phase = _recover(env, "lower", moving, False, hand, cube)
            target, wanted = env._pose_target_position.copy(), grip
        elif hand[2] <= floor + 0.002 and not landed:
            # Down as far as it may go and nothing is under the cube: the spot is not where
            # the hand can reach. A failure, not a landing.
            phase = _recover(env, "lower", moving, carrying, hand, cube)
            target, wanted = env._pose_target_position.copy(), grip
        elif landed:
            env._palm_phase = "settle"
            env._palm_since = env._step_count
    elif phase == "settle":
        # Hand the cube's weight to the table before letting go of it, and let it lie down
        # while it is still loosely surrounded. The wrist stops, the grip comes off slowly,
        # and the cube is given room to turn flat under its own weight.
        # The commanded pose stays where it is. The hand lands pressed a few millimetres below
        # the height where the cube first touched; pulling the pose back up to that height (the
        # old target) raised the hand with the pads still closed and took the cube with it,
        # 4-9 mm, to be dropped when the grip let go. Held, the hand stays down until the grip
        # is open, and the pressing is released only in "let go", where nothing touches the cube.
        target = env._pose_target_position.copy()
        wanted = max(grip - 0.004, SETTLE_GRIP)
        flat = off_a_face < 4.0 and cube_speed < 0.01 and cube_spin < 0.05
        env._palm_settle = env._palm_settle + 1 if (flat and env._resting_on_support(moving)) else 0
        if np.linalg.norm(cube[:2] - goal) > TARGET_TOLERANCE:
            # It is no longer on its spot at all: it slid or fell out while the hand loosened.
            phase = _recover(env, "settle", moving, False, hand, cube)
            target, wanted = env._pose_target_position.copy(), grip
        elif env._palm_settle > 10 and grip <= SETTLE_GRIP + 0.01:
            env._palm_phase = "let go"
            env._palm_since = env._step_count
            env._palm_settle = 0
            env._release_at = hand.copy()
    elif phase == "let go":
        # Open the rest of the way, the hand holding still, and leave only once nothing on the
        # hand touches the cube: a timer here let the hand start rising with a finger still
        # against the cube.
        target = env._release_at
        wanted = max(grip - 0.015, 0.0)
        if grip <= 0.02 and not touching[moving]:
            env._palm_phase = "back"
            env._palm_since = env._step_count
    elif phase == "park":
        # Retries spent. Hold still where it is, holding whatever is in the hand: moving home
        # from here has no collision check, and an open hand here dropped a carried cube.
        target, wanted = env._pose_target_position.copy(), grip
    else:
        # Straight up for a moment, then across at that height until the hand is clear of the
        # spot, then home. Heading home diagonally while still beside a tower is how the open
        # hand swept one over: its plates stand 58 mm either side of the grasp centre.
        rising = env._step_count - getattr(env, "_palm_since", 0) < 40
        if rising:
            target = over_drop
        elif not clear:
            target = np.array([retreat[0], retreat[1], over_drop[2]])
        else:
            target = retreat
        wanted = 0.0
        # A tower is this, repeated. Once this cube counts as placed and the hand is clear of
        # it, the next cube becomes the moving one and the phases start over.
        if not rising and clear and env._placed_latch[moving] and moving + 1 < env.n_objects:
            env.moving = moving + 1
            env._palm_phase = "reach"
            env._palm_since = env._step_count
            env._palm_settle = 0
            env._palm_retries = 0
            env._pick_spot = env.data.xpos[env.object_body_ids[env.moving]][:2].copy()
            env._pick_height = float(env.data.xpos[env.object_body_ids[env.moving]][2] - OBJECT_Z)
            env.__dict__.pop("_carry_aim", None)

    # Rate limits per phase; the carry and the withdrawal happen in clear air.
    rate = {"down": DOWN_RATE, "grip": DOWN_RATE, "up": UP_RATE, "cross": CROSS_RATE, "lower": 0.003,
            "settle": 0.0005, "let go": 0.001}.get(phase, POSITION_ACTION_SCALE)
    towards = target - env._pose_target_position
    distance = float(np.linalg.norm(towards))
    if distance > rate:
        towards = towards * (rate / distance)
    action = np.zeros(4)
    action[:3] = np.clip(towards / POSITION_ACTION_SCALE, -1.0, 1.0)
    action[3] = np.clip((wanted - grip) / GRIP_RATE, -1.0, 1.0)
    return action
