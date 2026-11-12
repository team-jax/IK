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
    EMPTY_CLEAR,
    GRIP_RATE,
    HAND_CLEAR,
    OBJECT_HALF_SIZE,
    OBJECT_Z,
    POSITION_ACTION_SCALE,
    REACH_RESCUE,
    REACH_STALL_NEAR,
    REACH_STALL_PROGRESS,
    REACH_STALL_STEPS,
    SETUP,
    SETUPS,
    TARGET_TOLERANCE,
    WORKSPACE_HIGH,
    WORKSPACE_LOW,
    JaxGraspEnv,
)

# How lightly the hand holds the cube while the cube lies down: the closure whose pads are a
# few millimetres wider than the cube, so it can turn flat inside the hand instead of being
# flicked out of it.
SETTLE_GRIP = CUBE_GRIP - 0.11
# A cube that stays tilted inside the loosened hand, still on its spot and on its support, up to
# SETTLE_TILT degrees off a face for SETTLE_TILT_WAIT steps, is let go of as it is: the open hand
# lets it fall flat. Waiting for it to lie flat timed the settle out in 12 of 100 real-setup
# deliveries (2026-11-12, held 4-12 degrees tilted), and the retry that followed did just this.
SETTLE_TILT = 15.0
SETTLE_TILT_WAIT = 40
# How fast the hand may swing while carrying, and lift away from the table. With the pads'
# friction at 2.0, 5 mm a step (0.25 m/s) pulled a 50 g cube out of the pads and 2 mm a step
# delivered 20 random layouts in 20 (3 mm: 17). At the measured 0.4 (CUBE_FRICTION), 2 mm a step
# delivers 63 of seeds 0-99 and 1 mm (0.05 m/s) 68: the slips happen in the carry.
# The real setup carries at 0.4 mm a step (0.02 m/s): with the pads at 2.0 the slip out of the
# leaning hand is in the crossing, and at 1 mm a step it was the first failure in 30 of seeds 0-99
# (delivered 81), at 0.4 in 1 (delivered 95; 0.3 no better) (motion1/STATUS.md 2026-11-10 (5)).
CROSS_RATE = SETUPS[SETUP].get("carry_rate", 0.001)
UP_RATE = SETUPS[SETUP].get("carry_rate", 0.001)
# How close over its spot the carried cube must be before the hand lowers it. At 0.4 mm a step the
# cube is already below the 0.02 m/s stillness test while still closing in, so with 12 mm it went
# down at the edge: the real setup's placement error was 7.9 mm (median), at 6 mm 4.9 for the
# same deliveries (94 vs 95 of seeds 0-99, 97 and 97 of 100-199). Slowing the crossing near the
# spot instead made it worse (9.2 mm: slower is still sooner) (motion1/STATUS.md 2026-11-10 (6)).
PLACE_ALIGN = SETUPS[SETUP].get("place_align", 0.012)
# Held within 12 mm of its spot this many steps without getting within PLACE_ALIGN, the cube is set
# down where it is: the arm can do no better there. Over a tower on the real setup's table, wrist_yaw
# on its stop held cubes 6.6-7.1 mm off until the episode ran out (stacking seeds 11, 36; 2026-11-11).
PLACE_WAIT = 150
# How far above the grasp pose the hand approaches from and carries at (at least).
TRAVEL_CLEAR = 0.06
# How many times a cube's delivery may recover from a phase that ran out of time before the
# hand gives up and parks.
MAX_RETRIES = 2
# How fast the hand comes down onto a cube. Driven at the full 10 mm a step it arrives hard
# enough to knock the cube out from under itself.
DOWN_RATE = 0.0015
# How fast the hand lowers a held cube onto its spot, and moves with no cube (reach, back, park).
# The real setup lowers at 1 mm a step instead of 3: at 3 the cube, which hangs low in the leaning
# hand, tipped out as it touched down in 25 of seeds 0-49, at 1 in 1 (motion1/STATUS.md
# 2026-11-10 (4)). Its empty-hand moves go at 2 mm a step instead of 10, which keeps the arm
# joints' commanded speed down: with the half-turned grasp (env.HAND_FLIP) the arm leaves home with
# its shoulder roll at up to 86 degrees a second at 3 mm, and the robot replays the whole trajectory
# slowed by its fastest joint (motion1/STATUS.md 2026-11-10 (7)). The sim setup keeps 3 mm and the full 10 mm.
LOWER_RATE = SETUPS[SETUP].get("lower_rate", 0.003)
# The real setup lowers slower still, 0.5 mm a step with the lowering's deadline stretched to match, a
# turned cube going onto another cube and any cube gripped off its faces (env._misalign). Of seeds 0-99
# of turned two-cube stacks 86 stood instead of 82 and none fell from the hand on the way down (3 at
# 1 mm); turned deliveries without a retry went from 158 to 163 of seeds 0-199 (motion1/STATUS.md
# 2026-11-10 (7)). None: LOWER_RATE for every cube.
SLOW_LOWER_RATE = SETUPS[SETUP].get("slow_lower_rate")
TRAVEL_RATE = SETUPS[SETUP].get("travel_rate", POSITION_ACTION_SCALE)
# How close the hand must have turned to its grasp orientation before it comes down on a cube
# (env.hand_turn_lag, only ever above 0 where HAND_TURN_RATE limits the turn).
TURN_DONE = np.radians(2.0)
# How the open hand leaves a cube it has set down. It goes towards home at carry height until clear of
# the spot, as it always has, where the arm can hold the hand on that path. On the sim setup's table it
# cannot over a tower: V3 holds the hand palm-down at the carry height over a one-cube tower (z 0.389)
# only within x <= 0.08, so heading home the hand sagged and swung, and its trailing finger plate swept
# the top cube off (the fixed stacking layout: 24 of seeds 0-49). Where that path cannot be held, the
# hand first slides SLIDE_OUT along the gap between its plates - out past the cube's sides, never over
# its top - until it is clear of the spot, and then goes home. Checked every SLIDE_CHECK with
# env.holds_path, the pose control's own way of solving (env.reachable, solving each pose from home,
# passed paths the real setup's arm could not follow, 2026-11-11). 80 mm out (just past the 73 mm wide
# plates) and then home, coming down on the way, lost one more stacking of seeds 0-49 than 130 mm:
# heading home from there swung a plate back over the cube's corner (motion1/STATUS.md 2026-11-10 (8)).
SLIDE_OUT = 0.13
SLIDE_CHECK = 0.02
SLIDE_TURN = 15.0
SLIDE_FAR = 0.21
# How far from the spot the straight way home from the end of a slide must pass, horizontally: a slide
# whose way home passes nearer is taken only if no heading gives one that passes this far, and then
# the one passing farthest (seed 11: 99 mm at -y and 21 cm out, not 44 mm between the plates). The open
# leaning hand reaches 75 mm to the side, and the arm sags and lags up to 20 mm on the way home with
# the elbow at its stop: with the table in front of the stand's FRONT side (2026-11-12) the way home
# from a slide 11 cm along -y passed 73 mm from the tower, and finger 2 knocked the top cube off
# (stacking seed 11).
SLIDE_PASS = 0.10
RESCUE_ABOVE = 0.03


def _reset_reach(env: JaxGraspEnv) -> None:
    """A reach starts over: no joint-space move under way or tried, no progress measured yet."""
    env.cancel_joint_move()
    env._reach_rescued = False
    env._reach_checked = False
    env._reach_best, env._reach_stall = np.inf, 0


def _reach_stalled(env: JaxGraspEnv, hand, goal) -> bool:
    """Has the hand, still more than REACH_STALL_NEAR from goal, come less than REACH_STALL_PROGRESS
    closer to it in the last REACH_STALL_STEPS steps? Measured in windows of that many steps."""
    distance = float(np.linalg.norm(hand - goal))
    if not np.isfinite(env._reach_best) or distance < REACH_STALL_NEAR:
        env._reach_best, env._reach_stall = distance, 0
        return False
    env._reach_stall += 1
    if env._reach_stall < REACH_STALL_STEPS:
        return False
    stalled = env._reach_best - distance < REACH_STALL_PROGRESS
    env._reach_best, env._reach_stall = distance, 0
    return stalled


def _way_passes(frm, to, spot) -> float:
    """How near `spot` the straight way from `frm` to `to` passes, horizontally."""
    frm, to, spot = (np.asarray(p, dtype=float)[:2] for p in (frm, to, spot))
    way = to - frm
    s = np.clip(float((spot - frm) @ way) / max(float(way @ way), 1e-12), 0.0, 1.0)
    return float(np.linalg.norm(frm + s * way - spot))


def _slide_out(env: JaxGraspEnv, start, home) -> np.ndarray | None:
    """Where the open hand slides to from `start` (over the spot, at carry height) before it goes home:
    None to go home as always, which it does wherever the pose control can follow the way there for
    SLIDE_OUT. Otherwise SLIDE_OUT along the gap between the finger plates, the side nearer home first,
    then along the other horizontal directions, nearest home first, if it can follow that; the first
    whose way home then passes SLIDE_PASS from the spot, else the one whose way home passes farthest;
    None if along none."""
    def holdable(direction, length):
        points = [np.array([start[0] + direction[0] * r, start[1] + direction[1] * r, start[2]])
                  for r in np.arange(SLIDE_CHECK, length + 1e-9, SLIDE_CHECK)]
        if any(np.any(p < WORKSPACE_LOW) or np.any(p > WORKSPACE_HIGH) for p in points):
            return False
        return env.holds_path(points)

    to_home = np.asarray(home[:2]) - np.asarray(start[:2])
    distance = float(np.linalg.norm(to_home))
    if distance < 1e-6 or holdable(to_home / distance, min(SLIDE_OUT, distance)):
        return None
    # Fingers 1 and 2 close against finger 3: the gap between the plates runs across that.
    tips = [env.data.xpos[env.model.body(f"Finger_{k}_2").id][:2] for k in (1, 2, 3)]
    closing = tips[2] - (tips[0] + tips[1]) / 2
    gap = np.array([-closing[1], closing[0]]) / np.linalg.norm(closing)
    nearer_home = lambda d: -float(d @ to_home)
    # Then every SLIDE_TURN round: on the real setup's table a tower's carry height leaves the arm
    # only some headings, often none of the four along and across the plates (seed 1: 13 cm held only
    # between -y and -x). The hand is above the cube's top there, so any heading clears it.
    others = [np.array([np.cos(a), np.sin(a)]) for a in np.radians(np.arange(0.0, 360.0, SLIDE_TURN))]
    others = [d for d in others if min(abs(d @ gap), 1.0) < np.cos(np.radians(1.0))]
    passes = lambda end: _way_passes(end, home, start)
    fallback, widest = None, -1.0
    for direction in sorted((gap, -gap), key=nearer_home) + sorted(others, key=nearer_home):
        if holdable(direction, SLIDE_OUT):
            # As far as the arm follows, up to SLIDE_FAR: the open leaning hand reaches 75 mm to the
            # side, and heading home from 130 mm out the sagging arm curved back to 100 mm of the tower
            # and a finger knocked its top cube off (real setup, seed 5).
            length = SLIDE_OUT
            while length + SLIDE_CHECK <= SLIDE_FAR + 1e-9 and holdable(direction, length + SLIDE_CHECK):
                length += SLIDE_CHECK
            end = np.array([start[0] + direction[0] * length, start[1] + direction[1] * length, start[2]])
            if passes(end) >= SLIDE_PASS:
                return end
            if passes(end) > widest:
                fallback, widest = end, passes(end)
    return fallback


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
    _reset_reach(env)
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
    elif on_spot and np.abs(env.data.xmat[env.object_body_ids[moving]].reshape(3, 3)[2, :]).max() > np.cos(
            np.radians(4.0)):
        # Off the table but lying flat on its spot: it fell out of the hand onto the tower it was
        # going onto and lies where the lowering would have put it. Open the hand and withdraw, as
        # after a settle. At the sim setup's pad friction (0.4) the cube slides down the pads in the
        # lift and drops the last 2-3 cm as the lowering starts; parked here, the fixed stacking
        # layout failed with the cube flat, 4.5 mm from its spot (motion1/STATUS.md 2026-11-10 (8)).
        env._release_at = hand.copy()
        env._palm_phase = "let go"
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
        env._slide_key = env._slide_to = None   # a withdrawal of the last episode is not this one's
        _reset_reach(env)

    pick = env.pick_pose(env._pick_spot, env._pick_height)
    # The spot's own height: the table for a delivery, a cube higher for a stack.
    drop = env.drop_pose(goal, env._target_heights[moving])
    # Two heights. The hand approaches the cube from a fixed clearance above the grasp, and
    # carries at a clearance above whichever of grasp and spot is higher, and above the tallest
    # cube standing now: a tower would otherwise be swept over by the hand leaving for the next cube.
    standing = max((env.data.xpos[env.object_body_ids[j]][2] for j in range(env.n_objects) if j != moving),
                   default=OBJECT_Z)
    clear_z = env.clear_height(standing)
    # The empty hand goes higher than that over a cube standing in the real setup (EMPTY_CLEAR).
    approach_z = max(pick[2] + TRAVEL_CLEAR, clear_z, env.empty_hand_height(standing))
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
    # So does the lift's: up to the carry height at UP_RATE. A fixed 100 steps assumed 2 mm a
    # step; at 1 mm the 0.11 m lift above a one-cube tower ran out of time in every stacking.
    up_budget = int((travel - pick[2]) / UP_RATE) + 30
    # The crossing's deadline follows its speed: the whole table is 0.4 m at CROSS_RATE.
    cross_budget = int(0.4 / CROSS_RATE) + 60
    # So does the reach's, at TRAVEL_RATE: 100 steps at the sim setup's 10 mm a step, as it always was.
    # At the real setup's 3 mm a step 100 steps is 0.3 m, short of the far end of the band from home; it
    # was hidden while the hand's half-turn from home held the deadline off (TURN_DONE), and showed as
    # soon as a grasp needed no such turn.
    reach_budget = int(0.4 / TRAVEL_RATE) + 60
    # How fast this cube is lowered onto its spot, and the lowering's deadline with it.
    slow_lower = SLOW_LOWER_RATE is not None and (
        (env._cube_yaws[moving] != 0.0 and env._target_heights[moving] > OBJECT_Z + 1e-6) or env._misalign[moving] != 0.0)
    lower_rate = SLOW_LOWER_RATE if slow_lower else LOWER_RATE
    lower_budget = int(0.1 / lower_rate) + 30 if slow_lower else 90
    budget = {"down": down_budget, "up": up_budget, "grip": 60, "settle": 120, "let go": 90, "lower": lower_budget,
              "cross": cross_budget, "reach": reach_budget}.get(phase, 100)
    if phase == "reach" and ((env.hand_turn_lag > TURN_DONE and not env._reach_rescued) or env.joint_moving):
        # The hand is still turning towards its grasp orientation, or the arm is on a joint-space
        # move: the reach's deadline counts from when that is done. Once a joint-space move has
        # been tried, a turn that never settles no longer holds the deadline off.
        env._palm_since = env._step_count
    if env._step_count - env._palm_since > budget and phase not in ("back", "park"):
        phase = _recover(env, phase, moving, carrying, hand, cube)

    target, wanted = over_pick, 0.0
    if phase == "reach":
        # Straight up from where the hand is, then across: a diagonal from home would sweep
        # the open fingers through whatever stands between. In the real setup the hand goes across
        # EMPTY_CLEAR over the top of the cube it goes to as well, and comes down to the approach
        # height over it: turning from home's orientation (100 degrees off a grasp) on the way, the
        # leaning hand hung a finger 59 mm below the grasp centre, and at the approach height it
        # knocked the cube 13-49 cm away before grasping it in 11 of 100 turned deliveries (2026-11-12).
        across_z = approach_z if EMPTY_CLEAR is None else max(approach_z, cube[2] + OBJECT_HALF_SIZE + EMPTY_CLEAR)
        far = np.linalg.norm(hand[:2] - over_pick[:2]) > 0.02
        if hand[2] < across_z - 0.015 and far:
            target = np.array([hand[0], hand[1], across_z])
        else:
            target = np.array([over_pick[0], over_pick[1], across_z]) if far else over_pick
            # Not getting there pose by pose: once per reach, go the rest of the way in joint space -
            # at once where the pose control cannot follow the straight way (env.holds_path), else
            # when it stalls. On the real setup's table the hand sagged along the way at the elbow's
            # stop but kept creeping closer, too slowly to count as stalled, and 23 of 100
            # deliveries timed out on their first reach (2026-11-11).
            if REACH_RESCUE and not env._reach_rescued:
                if not env._reach_checked:
                    env._reach_checked = True
                    n = max(int(np.ceil(np.linalg.norm(over_pick - hand) / SLIDE_CHECK)), 1)
                    stuck = not env.holds_path([hand + (over_pick - hand) * k / n for k in range(1, n + 1)])
                else:
                    stuck = _reach_stalled(env, hand, over_pick)
                if stuck:
                    # To RESCUE_ABOVE over the approach point: on a joint-space move the hand turns as
                    # it goes, and at the approach height the low finger of the leaning hand pushed
                    # the cube 6 cm aside on the way in (seed 12); the pose control comes down from there.
                    env.start_joint_move(over_pick + np.array([0.0, 0.0, RESCUE_ABOVE]))
                    env._reach_rescued = True
        wanted = 0.0
        if np.linalg.norm(hand - over_pick) < 0.01 and env.hand_turn_lag <= TURN_DONE and not env.joint_moving:
            env._palm_phase = "down"
            env._palm_since = env._step_count
            _reset_reach(env)
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
        elif carrying and ((np.linalg.norm(cube[:2] - goal) < PLACE_ALIGN and (cube_speed < 0.02 or env._palm_settle >= 30))
                           or env._palm_settle >= PLACE_WAIT):
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
            env._settle_tilted = 0
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
        tilted = (off_a_face < SETTLE_TILT and cube_speed < 0.01 and cube_spin < 0.05
                  and env._resting_on_support(moving) and grip <= SETTLE_GRIP + 0.01)
        env._settle_tilted = getattr(env, "_settle_tilted", 0) + 1 if tilted else 0
        if np.linalg.norm(cube[:2] - goal) > TARGET_TOLERANCE:
            # It is no longer on its spot at all: it slid or fell out while the hand loosened.
            phase = _recover(env, "settle", moving, False, hand, cube)
            target, wanted = env._pose_target_position.copy(), grip
        elif (env._palm_settle > 10 and grip <= SETTLE_GRIP + 0.01) or env._settle_tilted > SETTLE_TILT_WAIT:
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
        if not rising and getattr(env, "_slide_key", None) != (moving, env._palm_since):
            # Once per withdrawal, from over the spot: home as always, or out between the plates first.
            env._slide_key = (moving, env._palm_since)
            env._slide_to = _slide_out(env, over_drop, retreat)
        slide = None if rising else env._slide_to
        if slide is not None and np.linalg.norm(hand - slide) < 0.01:   # all the way out, not just clear
            env._slide_to = slide = None
        if rising:
            target = over_drop
        elif slide is not None:
            target = slide
        elif not clear or _way_passes(hand, retreat, top) < SLIDE_PASS:
            # At carry height while the rest of the way home still passes the spot that near: home is
            # as low as the top of a one-cube tower in the real setup, and coming down on the way from
            # a slide the sagging hand passed 87 mm from one, 40 mm low, and a finger knocked its top
            # cube off (2026-11-12, stacking seed 11).
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
            _reset_reach(env)

    # Rate limits per phase; the carry and the withdrawal happen in clear air.
    rate = {"down": DOWN_RATE, "grip": DOWN_RATE, "up": UP_RATE, "cross": CROSS_RATE, "lower": lower_rate,
            "settle": 0.0005, "let go": 0.001}.get(phase, TRAVEL_RATE)
    towards = target - env._pose_target_position
    distance = float(np.linalg.norm(towards))
    if distance > rate:
        towards = towards * (rate / distance)
    action = np.zeros(4)
    action[:3] = np.clip(towards / POSITION_ACTION_SCALE, -1.0, 1.0)
    action[3] = np.clip((wanted - grip) / GRIP_RATE, -1.0, 1.0)
    return action
