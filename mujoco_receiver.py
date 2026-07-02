"""MuJoCo UDP receiver: drive the 7DOF arm with real-time wrist coordinates.

Usage:
    venv/bin/mjpython mujoco_receiver.py   # macOS
    venv/bin/python mujoco_receiver.py      # Linux

Receives right wrist position from vision_sender.py via UDP, converts to
arm workspace coordinates, solves IK, and applies joint angles to arm/arm.xml.

Lateral centering: MediaPipe's x=0.5 (frame center) is not necessarily where the arm should
consider "centered" — a person's right wrist naturally rests off to one side of the frame, which
otherwise shows up as the arm being permanently biased to one side. The receiver auto-calibrates
its lateral center to wherever the wrist is on the very first packet received; press 'C' in the
viewer window at any time to recalibrate to the wrist's current position instead.
"""

import os
import socket
import sys
import time

import mujoco
import mujoco.viewer
import numpy as np

_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _DIR)
from arm.ik import SEED_POSE, apply_pose, build_chain, solve_ik  # noqa: E402

UDP_IP = "127.0.0.1"
UDP_PORT = 5005

ARM_XML = os.path.join(_DIR, "arm", "arm.xml")

# MediaPipe → arm world-space coordinate mapping. Tune these at runtime.
LATERAL_SCALE = 0.6  # x∈[0,1] → arm y-axis: (x-0.5)*LATERAL_SCALE
HEIGHT_SCALE = 0.5   # y∈[0,1] → arm z-axis: (1-y)*HEIGHT_SCALE + BASE_Z
DEPTH_SCALE = 0.3    # z (depth) → arm x-axis
BASE_Z = 0.55        # world-z when wrist is at image center height


def main():
    model = mujoco.MjModel.from_xml_path(ARM_XML)
    data = mujoco.MjData(model)

    chain = build_chain()
    # Start from the same non-singular seed arm/ik.py uses, not zeros — the all-zero pose is a
    # kinematic singularity (see arm/ik.py's SEED_POSE comment) that IK can fail to escape once
    # it's used as a warm-start seed.
    last_good_qpos = np.array(SEED_POSE[1:-1])

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    sock.bind((UDP_IP, UDP_PORT))
    print(f"Listening on {UDP_IP}:{UDP_PORT} ...")

    # None triggers auto-calibration on the first packet; pressing 'C' re-arms it so the next
    # received x_norm becomes the new lateral center.
    calib = {"x_center": None}

    def key_callback(keycode):
        if keycode == ord("C"):
            calib["x_center"] = None
            print("[Calibration] next wrist position will become the new center")

    with mujoco.viewer.launch_passive(model, data, key_callback=key_callback) as viewer:
        while viewer.is_running():
            try:
                raw, _ = sock.recvfrom(1024)
                x_norm, y_norm, z_m = map(float, raw.decode().strip().split())

                if calib["x_center"] is None:
                    calib["x_center"] = x_norm
                    print(f"[Calibration] center x = {x_norm:.3f}")

                target_world = [
                    z_m * DEPTH_SCALE,
                    (x_norm - calib["x_center"]) * LATERAL_SCALE,
                    (1.0 - y_norm) * HEIGHT_SCALE + BASE_Z,
                ]

                try:
                    joint_angles = solve_ik(chain, target_world, seed=last_good_qpos)
                    if not np.any(np.isnan(joint_angles)):
                        last_good_qpos = joint_angles
                    else:
                        joint_angles = last_good_qpos
                except Exception:
                    joint_angles = last_good_qpos

                with viewer.lock():
                    apply_pose(model, data, joint_angles)

            except BlockingIOError:
                pass
            except ValueError:
                pass
            except Exception as e:
                print(f"Error: {e}")

            viewer.sync()
            time.sleep(0.01)


if __name__ == "__main__":
    main()
