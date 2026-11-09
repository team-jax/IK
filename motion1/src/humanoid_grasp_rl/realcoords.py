"""Real robot coordinates -> motion1's sim world, and whether the arm can serve a cube there.

numpy only, and nothing from this package (its __init__ loads MuJoCo), so a script on the real robot
loads the file on its own:

    sys.path.insert(0, "<repo>/motion1/src/humanoid_grasp_rl")
    from realcoords import real_to_sim
    real_to_sim(0.10, -0.38, 0.4725)   # cube 38 cm ahead, 10 cm to the left, table 472.5 mm down
    plan((0.10, -0.30, 20.0), (0.05, -0.27), 0.4725)   # that cube turned 20 deg, and where it goes

Real coordinates (user, 2026-11-05), in metres: origin on the axis of the shoulder roll motor (the
first arm joint, `shoulder_roll`, at its joint origin: in CAD the motor's corner is 1 mm from it),
x to the robot's left, y backwards (the front is -y), z up; right-handed.
The sim world is that frame turned by SIM_FROM_REAL_YAW about z. The spec's -90 degrees puts the
front on the side the arm reaches to, sim -x: x_real = -y_sim, y_real = x_sim. The user confirmed
this front on the real robot (2026-11-05); the stand's FRONT lettering (sim -y) is not the front.
A cube's yaw is in degrees, counter-clockwise seen from above, of one of its faces' normals from the
real +x axis. In the sim it is that plus SIM_FROM_REAL_YAW, folded to [-45, 45) since a cube looks
the same every 90 degrees (env.fold_yaw).
run_tests.sh checks the constants below against env.py and the model.
"""
from pathlib import Path

import numpy as np

# The shoulder roll joint's origin in the sim world: the stand at x 0.49, the shoulder frame -0.15,
# the joint a further -0.052 along the arm (motion2/arm/make_arm_urdf.py, URDF_1108).
ORIGIN_XY = (0.288, 0.0)
SIM_FROM_REAL_YAW = -90.0
# The real setup the reach map was solved for (env.py SETUPS["real"]): table top above the floor,
# D (shoulder axis down to the table top) and the cube.
TABLE_Z = 0.735
MAP_D = 0.4725
CUBE_SIZE = 0.05
# Where the grasp centre (the `end_effector` site, between the fingers) is sent before it goes down:
# this far above the cube's top face (the spec asks for 2-3 cm).
APPROACH = 0.025
# A D further than this from MAP_D is a table the map was not solved for.
D_TOLERANCE = 0.005
REACH_FILE = Path(__file__).with_name("reachmap_real.npz")
# Per cube yaw and hand turn (scripts/build_reachmap.py --yaws), for a turned cube.
YAW_REACH_FILE = Path(__file__).with_name("reachmap_real_yaw.npz")


def fold_yaw(degrees: float) -> float:
    """Same as env.fold_yaw (run_tests.sh checks): [-45, 45)."""
    return float((degrees + 45.0) % 90.0 - 45.0)


def hand_turns(cube_yaw_deg: float) -> list:
    """Same as env.hand_turns (run_tests.sh checks): the cube's yaw or a quarter more, least first."""
    y = fold_yaw(cube_yaw_deg)
    return [0.0] if y == 0.0 else sorted((y, y + 90.0, y - 90.0), key=abs)


def to_sim(point, origin=(ORIGIN_XY[0], ORIGIN_XY[1], TABLE_Z + MAP_D), yaw=SIM_FROM_REAL_YAW):
    """A real point (x, y, z about the origin) in the sim world, the origin at `origin` there."""
    c, s = np.cos(np.radians(yaw)), np.sin(np.radians(yaw))
    x, y, z = point
    return np.asarray(origin, dtype=float) + np.array([c * x - s * y, s * x + c * y, z])


def reachable_spot(xy, reach_file=REACH_FILE) -> tuple[bool, str]:
    """Is this sim-world cube spot inside the reach map: in the grid, its nearest cell feasible and
    the spot inside that cell (no interpolation between cells)?"""
    with np.load(reach_file) as reach:
        xs, ys, feasible = reach["xs"], reach["ys"], reach["feasible"]
    half_x, half_y = (xs[1] - xs[0]) / 2, (ys[1] - ys[0]) / 2
    if not (xs[0] - half_x <= xy[0] <= xs[-1] + half_x and ys[0] - half_y <= xy[1] <= ys[-1] + half_y):
        return False, "outside the reach map's grid"
    i, j = int(np.argmin(np.abs(xs - xy[0]))), int(np.argmin(np.abs(ys - xy[1])))
    return (True, "in the reach band") if feasible[i, j] else (False, "the arm cannot serve that cell")


def _cell(xs, ys, xy):
    """The nearest grid cell of a spot, or None outside the grid."""
    half_x, half_y = (xs[1] - xs[0]) / 2, (ys[1] - ys[0]) / 2
    if not (xs[0] - half_x <= xy[0] <= xs[-1] + half_x and ys[0] - half_y <= xy[1] <= ys[-1] + half_y):
        return None
    return int(np.argmin(np.abs(xs - xy[0]))), int(np.argmin(np.abs(ys - xy[1])))


def reachable_turned(spots, yaw_sim, reach_file=YAW_REACH_FILE):
    """Can the arm grip a cube turned yaw_sim (degrees, sim) and serve every spot in `spots` (sim
    xy) with one hand turn? The map's nearest yaw (90-degree period) and nearest cells are used;
    the env checks the exact poses again. Returns (ok, hand yaw in degrees or None, reason)."""
    with np.load(reach_file) as reach:
        yaws, offsets, xs, ys, feasible = (reach[k] for k in ("yaws", "offsets", "xs", "ys", "feasible"))
    k = int(np.argmin(np.abs([fold_yaw(v - yaw_sim) for v in yaws])))
    cells = [_cell(xs, ys, s) for s in spots]
    if any(c is None for c in cells):
        return False, None, "outside the reach map's grid"
    for turn in hand_turns(yaw_sim):
        o = int(np.argmin(np.abs(offsets - (turn - fold_yaw(yaw_sim)))))
        if all(feasible[k, o][c] for c in cells):
            return True, turn, f"in the reach band with the hand turned {turn:+.0f} deg"
    return False, None, f"no hand turn serves it at cube yaw {fold_yaw(yaw_sim):+.0f} deg"


def plan(cube, target, d: float) -> dict:
    """A delivery on the real table: cube = (x, y, yaw) and target = (x, y) in real coordinates (m,
    degrees), the table top d [m] below the shoulder axis. Returns the sim cube centre, target spot,
    cube yaw and hand yaw, the approach target, and whether the arm can do it, with the reason.
    env.reset(options={"objects": [[*cube_sim[:2], yaw_sim]], "targets": [target_sim]}) runs it."""
    x, y, yaw = cube
    first = real_to_sim(x, y, d, yaw=yaw)
    target_sim = to_sim((target[0], target[1], 0.0))[:2]
    out = {"cube": first["cube"], "approach": first["approach"], "target": target_sim,
           "yaw": first["yaw"], "hand_yaw": None}
    if abs(d - MAP_D) > D_TOLERANCE:
        return {**out, "reachable": False, "reason": first["reason"]}
    ok, turn, why = reachable_turned([first["cube"][:2], target_sim], first["yaw"])
    return {**out, "hand_yaw": turn, "reachable": ok, "reason": why}


def real_to_sim(x: float, y: float, d: float, reach_file=REACH_FILE, yaw=None) -> dict:
    """A cube on the real table at (x, y) [m], the table top `d` [m] below the shoulder axis.

    Returns the cube centre and the approach target (grasp centre APPROACH above the cube's top
    face) in the sim world, and whether the arm can serve the spot, with the reason. Only a table
    at the D the map was solved for can be judged (D_TOLERANCE). With `yaw` (real, degrees) the
    result also has the sim yaw, and the spot is judged for a cube turned that way (both picking
    and setting down there, YAW_REACH_FILE)."""
    cube = to_sim((x, y, -d + CUBE_SIZE / 2))
    approach = cube + np.array([0.0, 0.0, CUBE_SIZE / 2 + APPROACH])
    out = {"cube": cube, "approach": approach}
    if yaw is not None:
        out["yaw"] = fold_yaw(yaw + SIM_FROM_REAL_YAW)
    if abs(d - MAP_D) > D_TOLERANCE:
        ok, why = False, f"no reach map for D = {d:.4f} m (the map is for {MAP_D} m)"
    elif yaw is not None and out["yaw"] != 0.0:
        ok, _, why = reachable_turned([cube[:2]], out["yaw"])
    else:
        ok, why = reachable_spot(cube[:2], reach_file)
    return {**out, "reachable": ok, "reason": why}
