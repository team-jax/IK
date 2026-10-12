"""Headless check of vision_sender.py - no webcam is opened.

    venv/bin/python motion2/check_sender.py

1. compute_finger_curls on synthetic hands
2. the two real .task models load and run on a blank frame (mediapipe Tasks API)
3. main() end to end with a fake camera and fake landmarkers: packet format, visibility gating,
   EMA, curl hold, fan-out to both UDP ports, and compatibility with the receiver's parser
"""
import os
import socket
import sys
from types import SimpleNamespace as NS

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

import vision_sender as vs


def lm(x, y, z=0.0, v=1.0):
    return NS(x=x, y=y, z=z, visibility=v)


def hand(poses):
    """21 landmarks; each tracked finger is 'straight', 'right' (90 deg at the PIP) or 'fold'."""
    h = [lm(0.5, 1.0) for _ in range(21)]
    for (_, mcp, pip, tip), pose in zip(vs.FINGERS, poses):
        h[mcp], h[pip] = lm(0, 0), lm(0, -1)
        h[tip] = {"straight": lm(0, -2), "right": lm(1, -1), "fold": lm(0, 0)}[pose]
    return h


# ---- 1. curl maths --------------------------------------------------------------------------
c = vs.compute_finger_curls(hand(["straight"] * 3))
assert max(c) < 1e-6, c
c = vs.compute_finger_curls(hand(["right", "straight", "straight"]))
assert abs(c[0] - (vs.CURL_EXTENDED_DEG - 90) / (vs.CURL_EXTENDED_DEG - vs.CURL_CLOSED_DEG)) < 1e-6, c
assert c[1] < 1e-6 and c[2] < 1e-6, c
assert vs.compute_finger_curls(hand(["straight", "fold", "straight"])) == [0.0, 1.0, 0.0]
print("1. curl maths ok (straight 0, 90 deg 0.833, folded 1; fingers independent)")

# ---- 2. the real models --------------------------------------------------------------------
assert vs.MODEL_PATH.is_file() and vs.HAND_MODEL_PATH.is_file(), "model files missing"
pose_options = mp_vision.PoseLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path=str(vs.MODEL_PATH)),
    running_mode=mp_vision.RunningMode.VIDEO, num_poses=1)
hand_options = mp_vision.HandLandmarkerOptions(
    base_options=mp_python.BaseOptions(model_asset_path=str(vs.HAND_MODEL_PATH)),
    running_mode=mp_vision.RunningMode.VIDEO, num_hands=1)
blank = np.zeros((480, 640, 3), np.uint8)
with mp_vision.PoseLandmarker.create_from_options(pose_options) as pose_lm, \
        mp_vision.HandLandmarker.create_from_options(hand_options) as hand_lm:
    for i in range(3):
        image = mp.Image(image_format=mp.ImageFormat.SRGB, data=blank)
        pose_result = pose_lm.detect_for_video(image, i * 33)
        hand_result = hand_lm.detect_for_video(image, i * 33)
        assert not pose_result.pose_landmarks and not hand_result.hand_landmarks
print("2. pose_landmarker_full.task and hand_landmarker.task load and run (blank frame -> nothing detected)")

# ---- 3. main() with a fake camera and fake landmarkers -------------------------------------
FRAMES = 40
state = {"frame": 0, "waits": 0}


class FakeCap:
    def __init__(self, *args):
        pass

    def isOpened(self):
        return True

    def read(self):
        state["frame"] += 1
        return True, np.zeros((480, 640, 3), np.uint8)

    def release(self):
        pass


def fake_wait_key(_ms):
    state["waits"] += 1
    return ord("q") if state["waits"] >= FRAMES else -1


cv2.VideoCapture = FakeCap
cv2.imshow = lambda *a, **k: None
cv2.destroyAllWindows = lambda *a, **k: None
cv2.waitKey = fake_wait_key


class FakePose:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def detect_for_video(self, _image, _ts):
        f = state["frame"] - 1
        wrist_v = 0.2 if 20 <= f < 30 else 0.9    # frames 20-29: wrist not visible -> nothing sent
        elbow_v = 0.2 if 10 <= f < 20 else 0.9     # frames 10-19: elbow unreliable -> 6-field packet
        lms = [lm(0.5, 0.5, 0.0, 1.0) for _ in range(33)]
        lms[vs.RIGHT_WRIST] = lm(0.55, 0.40, -0.30, wrist_v)
        lms[vs.RIGHT_ELBOW] = lm(0.60, 0.60, -0.20, elbow_v)
        lms[vs.RIGHT_SHOULDER] = lm(0.55, 0.80, -0.10, 0.9)
        return NS(pose_landmarks=[lms])


class FakeHand:
    def __enter__(self):
        return self

    def __exit__(self, *exc):
        return False

    def detect_for_video(self, _image, _ts):
        f = state["frame"] - 1
        # frames 30-39: hand lost -> the sender must hold the last curls, not snap to 0
        return NS(hand_landmarks=[hand(["right", "straight", "straight"])] if f < 30 else [])


mp_vision.PoseLandmarker.create_from_options = staticmethod(lambda options: FakePose())
mp_vision.HandLandmarker.create_from_options = staticmethod(lambda options: FakeHand())

sockets = []
for port in (vs.UDP_PORT, vs.WEB_UDP_PORT):
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind((vs.UDP_IP, port))
    s.setblocking(False)
    sockets.append(s)

vs.main()


def drain(sock):
    out = []
    while True:
        try:
            out.append(sock.recv(1024).decode())
        except BlockingIOError:
            return out


to_receiver, to_dashboard = (drain(s) for s in sockets)
assert len(to_receiver) == len(to_dashboard) == 30, (len(to_receiver), len(to_dashboard))
assert to_receiver == to_dashboard, "the two ports must get the same packets"
fields = [len(p.split()) for p in to_receiver]
assert fields == [12] * 10 + [6] * 10 + [12] * 10, fields
vals = [[float(v) for v in p.split()] for p in to_receiver]
for v in vals:   # the wrist is constant, so the EMA must sit exactly on it
    assert abs(v[0] - 0.55) < 1e-3 and abs(v[1] - 0.40) < 1e-3 and abs(v[2] + 0.30) < 1e-3, v
index = [v[3] for v in vals]
assert abs(index[0] - 0.3 * 0.8333) < 1e-3, index[0]               # first EMA step: 0.3 * raw curl
assert all(b >= a for a, b in zip(index, index[1:20])), "curl should rise towards 0.833"
assert all(v[4] == 0.0 and v[5] == 0.0 for v in vals), "only the index finger is bent"
assert len(set(index[20:])) == 1 and index[20] > 0.8, "curls must be held while no hand is detected"
for v in vals[:10]:      # elbow and shoulder ride along in the 12-field packets
    assert v[6:] == [0.6, 0.6, -0.2, 0.55, 0.8, -0.1], v[6:]

import mujoco_receiver as receiver   # noqa: E402  (its parser must accept what the sender sends)
for p in to_receiver:
    receiver.parse_packet(p.encode())
print("3. main(): 30 packets on both ports (12 / 6 / 12 fields), low-visibility frames dropped, "
      "curls smoothed and held, receiver parser accepts all")
print("sender: ok")
