"""How much a pinched cube creeps in MuJoCo, per friction setting (no robot, no policy).

    venv/bin/python sim2sim/creep_test.py

A 50 mm, 40 g cube hangs between two 20 x 20 mm pads pressed together with F per pad, friction 0.4 (cube and robot
links in the deploy spec). F is twice what static friction needs (2 * 0.4 * F = 2 m g), so a rigid Coulomb contact
(PhysX) holds it still: any sliding is the soft constraint's creep. The scene's settings (build_scene.py) are the
row with the scene's contact time constant, step and impratio.
"""

import mujoco
import numpy as np

MASS, MU, G = 0.040, 0.4, 9.81
F = 2.0 * MASS * G / (2.0 * MU)          # per pad: friction capacity twice the weight

XML = """
<mujoco>
  <option timestep="{dt}" integrator="implicitfast" cone="{cone}" impratio="{imp}"/>
  <default><geom solref="{tc} 1" friction="{mu} 0.005 0.0001"/></default>
  <worldbody>
    <body name="padL" pos="0 -0.035 0.5"><joint name="sL" type="slide" axis="0 1 0"/>
      <geom type="box" size="0.01 0.01 0.01" mass="0.05"/></body>
    <body name="padR" pos="0 0.035 0.5"><joint name="sR" type="slide" axis="0 1 0"/>
      <geom type="box" size="0.01 0.01 0.01" mass="0.05"/></body>
    <body name="cube" pos="0 0 0.5"><freejoint/><geom type="box" size="0.025 0.025 0.025" mass="{mass}"/></body>
  </worldbody>
  <actuator>
    <motor joint="sL" gear="1"/><motor joint="sR" gear="-1"/>
  </actuator>
</mujoco>
"""


def creep(dt, tc, cone, imp, hold_s=3.0):
    m = mujoco.MjModel.from_xml_string(XML.format(dt=dt, tc=tc, cone=cone, imp=imp, mu=MU, mass=MASS))
    d = mujoco.MjData(m)
    d.ctrl[:] = F
    cube = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "cube")
    for _ in range(int(0.5 / dt)):                  # settle
        mujoco.mj_step(m, d)
    z0 = d.xpos[cube][2]
    for _ in range(int(hold_s / dt)):
        mujoco.mj_step(m, d)
    return (z0 - d.xpos[cube][2]) / hold_s * 1000.0  # mm/s downward


if __name__ == "__main__":
    print(f"pad force {F:.3f} N each, friction capacity {2 * MU * F:.3f} N vs weight {MASS * G:.3f} N")
    rows = [("MuJoCo default", 1 / 120, 0.02, "pyramidal", 1.0)]
    for imp in (1.0, 3.0, 10.0, 30.0, 100.0):
        rows.append((f"step 1/1200, tc 0.002, elliptic, impratio {imp:g}", 1 / 1200, 0.002, "elliptic", imp))
    rows.append(("step 1/1200, tc 0.002, pyramidal, impratio 1", 1 / 1200, 0.002, "pyramidal", 1.0))
    for name, dt, tc, cone, imp in rows:
        print(f"{name:52s} creep {creep(dt, tc, cone, imp):8.3f} mm/s")
