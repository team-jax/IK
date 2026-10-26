"""Headless grasp check: close the hand on the cube at +-8 mm offsets and lift it 84 mm.

    venv/bin/python motion2/scripts/check_grasp.py     # prints LIFTED per offset, then the count (expect 9/9)
"""
import _paths  # noqa: F401  (puts src/ on the path)
import numpy as np, mujoco
import mujoco_receiver as m
def run(dx,dy):
    model=mujoco.MjModel.from_xml_path(m.ARM_XML); data=mujoco.MjData(model)
    chain=m.build_chain(); q=np.array(m.SEED_POSE[1:-1])
    for j,a in zip(m.ARM_JOINTS,q): data.joint(j).qpos[0]=a
    for f in m.FINGER_ORDER: data.joint(f+"_joint1").qpos[0]=m.FINGER_OPEN
    m.apply_arm_ctrl(data,q); m.apply_grip(data,(0,0,0)); mujoco.mj_forward(model,data)
    st={'q':q}
    def go(t,curl,steps):
        a=m.solve_ik_palm_down(chain,t,seed=st['q']); st['q']=a
        m.apply_arm_ctrl(data,a); m.apply_grip(data,(curl,)*3)
        for _ in range(steps): mujoco.mj_step(model,data)
    x,y=0.12+dx,dy
    go([x,y,0.34],0,1200); go([x,y,m.BASE_Z],0,1500); go([x,y,m.BASE_Z],1,1500)
    for k in range(1,13): go([x,y,m.BASE_Z+0.007*k],1,300)
    return data.body('object').xpos.copy()
ok=0
for dx in (-0.008,0,0.008):
    for dy in (-0.008,0,0.008):
        c=run(dx,dy); lifted=c[2]>0.30; ok+=lifted
        print(f'dx={dx:+.3f} dy={dy:+.3f} cube z={c[2]:.3f} {"LIFTED" if lifted else "no"}',flush=True)
print('lifted',ok,'/9')
