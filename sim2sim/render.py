"""Render a chain run to an MP4 (or one PNG frame) with MuJoCo's offscreen renderer.

    venv/bin/python sim2sim/render.py --spot 0 0 0 --out sim2sim/results/chain_x0_y0_yaw0.mp4
    venv/bin/python sim2sim/render.py --spot 0 0 0 --frame 9.4 --out frame.png      # one frame at t = 9.4 s
    venv/bin/mjpython sim2sim/render.py --spot 0 0 0 --view                           # replay in the viewer (macOS)

The run is simulated first (chain.run), then its recorded joint and cube states are replayed for the pictures, so
rendering never changes the physics.
"""

import argparse
import os
import sys
import time

import mujoco
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import chain as C  # noqa: E402


def poses(tr, model):
    d = mujoco.MjData(model)
    jid = lambda n: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, n)
    qadr = np.array([model.jnt_qposadr[jid(n)] for n in C.ISAAC])
    cq = model.jnt_qposadr[jid("cube")]
    for k in range(len(tr["t"])):
        d.qpos[qadr] = tr["q"][k]
        d.qpos[cq:cq + 3] = tr["cube_pos"][k]
        d.qpos[cq + 3:cq + 7] = tr["cube_quat"][k]
        mujoco.mj_kinematics(model, d)
        mujoco.mj_comPos(model, d)
        mujoco.mj_camlight(model, d)
        yield tr["t"][k], d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--spot", nargs=3, type=float, default=(0.0, 0.0, 0.0), metavar=("DX_CM", "DY_CM", "YAW_DEG"))
    ap.add_argument("--camera", default="front")
    ap.add_argument("--out", default=os.path.join(HERE, "results", "chain.mp4"))
    ap.add_argument("--frame", type=float, default=None, help="write one PNG at this time (s)")
    ap.add_argument("--view", action="store_true", help="replay in the passive viewer instead of writing a file")
    args = ap.parse_args()
    dx, dy, yaw = args.spot[0] / 100, args.spot[1] / 100, args.spot[2]
    summary, tr = C.run(dx, dy, yaw)
    print({k: summary[k] for k in ("phase_t_s", "done", "fail_reason")})
    model = mujoco.MjModel.from_xml_path(C.SCENE)
    if args.view:
        from mujoco import viewer
        shown = mujoco.MjData(model)
        with viewer.launch_passive(model, shown) as v:
            for _, d in poses(tr, model):
                with v.lock():
                    shown.qpos[:] = d.qpos
                    mujoco.mj_kinematics(model, shown)
                v.sync()
                time.sleep(C.DT)
                if not v.is_running():
                    break
        return
    r = mujoco.Renderer(model, 720, 960)
    if args.frame is not None:
        import imageio.v2 as imageio
        k = int(np.argmin(np.abs(tr["t"] - args.frame)))
        for i, (t, d) in enumerate(poses(tr, model)):
            if i == k:
                r.update_scene(d, camera=args.camera)
                imageio.imwrite(args.out, r.render())
                print(f"wrote {args.out} (t = {t:.2f} s)")
                return
    import imageio.v2 as imageio
    os.makedirs(os.path.dirname(os.path.abspath(args.out)), exist_ok=True)
    with imageio.get_writer(args.out, fps=30, macro_block_size=8) as w:
        for i, (t, d) in enumerate(poses(tr, model)):
            if i % 2:
                continue
            r.update_scene(d, camera=args.camera)
            w.append_data(r.render())
    print(f"wrote {args.out}")


if __name__ == "__main__":
    main()
