"""Real robot coordinates -> motion1's sim world, and whether the arm can serve a cube there.

numpy only, and nothing from this package (its __init__ loads MuJoCo), so a script on the real robot
loads the file on its own:

    sys.path.insert(0, "<repo>/motion1/src/humanoid_grasp_rl")
    from realcoords import real_to_sim
    real_to_sim(0.10, -0.38, 0.4725)   # cube 38 cm ahead, 10 cm to the left, table 472.5 mm down

Real coordinates (user, 2026-11-05), in metres: origin on the axis of the shoulder roll motor (the
first arm joint, `shoulder_roll`, at its joint origin: in CAD the motor's corner is 1 mm from it),
x to the robot's left, y backwards (the front is -y), z up; right-handed.
The sim world is that frame turned by SIM_FROM_REAL_YAW about z. The spec's -90 degrees puts the
front on the side the arm reaches to, sim -x: x_real = -y_sim, y_real = x_sim. If the real table
stands at the stand's FRONT lettering instead (sim -y; motion1/STATUS.md), the yaw is 0.
run_tests.sh checks the constants below against env.py and the model.
"""
from pathlib import Path

import numpy as np

# The shoulder roll joint's origin in the sim world: the stand at x 0.50, the shoulder frame -0.16,
# the joint a further -0.052 along the arm (motion2/arm/make_arm_urdf.py, URDF_1006).
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


def real_to_sim(x: float, y: float, d: float, reach_file=REACH_FILE) -> dict:
    """A cube on the real table at (x, y) [m], the table top `d` [m] below the shoulder axis.

    Returns the cube centre and the approach target (grasp centre APPROACH above the cube's top
    face) in the sim world, and whether the arm can serve the spot, with the reason. Only a table
    at the D the map was solved for can be judged (D_TOLERANCE)."""
    cube = to_sim((x, y, -d + CUBE_SIZE / 2))
    approach = cube + np.array([0.0, 0.0, CUBE_SIZE / 2 + APPROACH])
    if abs(d - MAP_D) > D_TOLERANCE:
        ok, why = False, f"no reach map for D = {d:.4f} m (the map is for {MAP_D} m)"
    else:
        ok, why = reachable_spot(cube[:2], reach_file)
    return {"cube": cube, "approach": approach, "reachable": ok, "reason": why}
