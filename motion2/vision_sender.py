"""OpenCV + MediaPipe Tasks PoseLandmarker + HandLandmarker: extract right wrist (X,Y,Z) and
per-finger curl values for the index/middle/ring fingers (0=extended, 1=fully curled), send
them via UDP as "x y z curl_index curl_middle curl_ring ex ey ez sx sy sz" (elbow + shoulder landmarks are
appended whenever both are visible, so the simulated arm can follow the whole arm, not just the wrist).

mediapipe>=0.10.30 dropped the legacy `mp.solutions` API (see pip package contents —
`mediapipe.solutions` no longer ships), so this uses the replacement Tasks API
(`mediapipe.tasks.python.vision.PoseLandmarker` / `HandLandmarker`) instead.

Usage:
    venv/bin/python vision_sender.py
    (press 'q' in the video window to quit)

Requires pose_landmarker_full.task and hand_landmarker.task in this directory (downloaded from
https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task
and
https://storage.googleapis.com/mediapipe-models/hand_landmarker/hand_landmarker/float16/latest/hand_landmarker.task).
"""

import math
import socket
import sys
import time
from pathlib import Path

import cv2
import mediapipe as mp
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

UDP_IP = "127.0.0.1"
UDP_PORT = 5005       # mujoco_receiver.py
WEB_UDP_PORT = 5006   # web_dashboard.py

RIGHT_SHOULDER = 12
RIGHT_ELBOW = 14
RIGHT_WRIST = 16  # PoseLandmark index, same 33-point topology as the legacy Solutions API
MODEL_PATH = Path(__file__).parent / "pose_landmarker_full.task"
HAND_MODEL_PATH = Path(__file__).parent / "hand_landmarker.task"

# HandLandmarker's 21-point topology: 0=wrist; per finger, (MCP knuckle, PIP joint, fingertip)
# landmark indices. Only index/middle/ring are tracked — the MuJoCo gripper has exactly 3
# fingers, mapped 1:1 to these (thumb and pinky are ignored on purpose).
HAND_WRIST = 0
FINGERS = (
    ("index", 5, 6, 8),
    ("middle", 9, 10, 12),
    ("ring", 13, 14, 16),
)

# Curl from the 3D bend angle at each finger's PIP joint (angle MCP–PIP–TIP): a straight
# finger measures ~180°, a fist ~60–90°. Angles are orientation- and distance-invariant, unlike
# the previous 2D tip-to-wrist distance ratio which compressed to a narrow mid-band (~0.4–0.65
# measured live) whenever the palm wasn't square to the camera — fingers then "didn't move".
CURL_EXTENDED_DEG = 165.0  # PIP angle mapped to curl=0 (fully extended)
CURL_CLOSED_DEG = 75.0     # PIP angle mapped to curl=1 (fully curled)
GRIP_EMA_ALPHA = 0.3

# Measured live: when the wrist isn't actually visible in frame, MediaPipe still returns a
# landmark (visibility ~0.1) with wildly extrapolated x/y/z that jump far more between frames
# than real tracked motion ever does — this is the actual cause of the arm "flapping", not
# camera noise on a genuinely tracked wrist. Drop frames below this confidence instead of
# sending them.
MIN_VISIBILITY = 0.5

# Exponential moving average on the (already visibility-filtered) coordinate so a single noisy
# — but still "visible enough" — frame can't snap the sent position all the way to itself in one
# step. new_smoothed = EMA_ALPHA*raw + (1-EMA_ALPHA)*old_smoothed. Lower = smoother but laggier.
EMA_ALPHA = 0.3


def compute_finger_curls(hand_landmarks):
    """[index, middle, ring] curls, each 0.0 (extended) .. 1.0 (fully curled), from one
    HandLandmarker detection. Uses the 3D bend angle at each finger's PIP joint, so each
    finger is independent of the others and of hand orientation/distance to the camera."""
    curls = []
    for _, mcp_i, pip_i, tip_i in FINGERS:
        mcp = hand_landmarks[mcp_i]
        pip = hand_landmarks[pip_i]
        tip = hand_landmarks[tip_i]
        v1 = (mcp.x - pip.x, mcp.y - pip.y, mcp.z - pip.z)
        v2 = (tip.x - pip.x, tip.y - pip.y, tip.z - pip.z)
        dot = sum(a * b for a, b in zip(v1, v2))
        norm = (math.sqrt(sum(a * a for a in v1)) * math.sqrt(sum(a * a for a in v2))) + 1e-9
        angle = math.degrees(math.acos(max(-1.0, min(1.0, dot / norm))))
        curl = (CURL_EXTENDED_DEG - angle) / (CURL_EXTENDED_DEG - CURL_CLOSED_DEG)
        curls.append(max(0.0, min(1.0, curl)))
    return curls


def draw_hand(frame, hand_landmarks):
    """Draw the three tracked fingers (wrist→MCP→PIP→TIP) so the user can SEE whether the
    hand is being recognized and which landmarks it locked onto."""
    h, w, _ = frame.shape
    wrist = hand_landmarks[HAND_WRIST]
    for _, mcp_i, pip_i, tip_i in FINGERS:
        pts = [(int(hand_landmarks[i].x * w), int(hand_landmarks[i].y * h))
               for i in (HAND_WRIST, mcp_i, pip_i, tip_i)]
        for a, b in zip(pts, pts[1:]):
            cv2.line(frame, a, b, (0, 255, 0), 2)
        cv2.circle(frame, pts[-1], 5, (0, 255, 255), -1)
    cv2.circle(frame, (int(wrist.x * w), int(wrist.y * h)), 5, (255, 0, 255), -1)


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    pose_options = mp_vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    hand_options = mp_vision.HandLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(HAND_MODEL_PATH)),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_hands=1,
        # 0.4 (down from 0.5): with the whole upper body in frame for pose tracking, the hand
        # is small in the image and 0.5 dropped it often enough to look like "no recognition".
        min_hand_detection_confidence=0.4,
        min_hand_presence_confidence=0.4,
        min_tracking_confidence=0.4,
    )

    print(f"Sending right wrist position + finger curls to {UDP_IP}:{UDP_PORT} and {UDP_IP}:{WEB_UDP_PORT} ...")

    smoothed = None                    # (x, y, z) EMA state; None until the first visible frame
    arm_smoothed = {}                  # landmark index → (x, y, z) EMA state for elbow/shoulder
    curls_smoothed = [0.0, 0.0, 0.0]   # index/middle/ring; holds steady when no hand is detected

    with mp_vision.PoseLandmarker.create_from_options(pose_options) as landmarker, \
         mp_vision.HandLandmarker.create_from_options(hand_options) as hand_landmarker:
        start_time = time.monotonic()
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Error: Failed to read frame")
                break

            rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb_frame)
            timestamp_ms = int((time.monotonic() - start_time) * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)
            hand_result = hand_landmarker.detect_for_video(mp_image, timestamp_ms)

            hand_detected = bool(hand_result.hand_landmarks)
            if hand_detected:
                raw_curls = compute_finger_curls(hand_result.hand_landmarks[0])
                curls_smoothed = [
                    GRIP_EMA_ALPHA * raw + (1 - GRIP_EMA_ALPHA) * old
                    for raw, old in zip(raw_curls, curls_smoothed)
                ]
                draw_hand(frame, hand_result.hand_landmarks[0])

            # Always-on status line: the user must be able to tell "hand not recognized" apart
            # from "recognized but fingers not moving" at a glance.
            if hand_detected:
                cv2.putText(frame, "HAND: OK", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                cv2.putText(frame, "HAND: NOT DETECTED", (10, 60),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

            if result.pose_landmarks:
                wrist = result.pose_landmarks[0][RIGHT_WRIST]
                is_visible = (wrist.visibility or 0.0) >= MIN_VISIBILITY

                if is_visible:
                    raw = (wrist.x, wrist.y, wrist.z)
                    if smoothed is None:
                        smoothed = raw
                    else:
                        smoothed = tuple(
                            EMA_ALPHA * r + (1 - EMA_ALPHA) * s for r, s in zip(raw, smoothed)
                        )
                    text = (
                        f"{smoothed[0]:.4f} {smoothed[1]:.4f} {smoothed[2]:.4f} "
                        f"{curls_smoothed[0]:.4f} {curls_smoothed[1]:.4f} {curls_smoothed[2]:.4f}"
                    )
                    # Elbow + shoulder, same visibility gate and EMA as the wrist. If either is
                    # unreliable this frame the packet stays 6-field and the receiver just
                    # tracks the wrist.
                    lms = result.pose_landmarks[0]
                    arm_pts = []
                    for idx in (RIGHT_ELBOW, RIGHT_SHOULDER):
                        lm = lms[idx]
                        if (lm.visibility or 0.0) < MIN_VISIBILITY:
                            break
                        raw_lm = (lm.x, lm.y, lm.z)
                        old = arm_smoothed.get(idx)
                        arm_smoothed[idx] = raw_lm if old is None else tuple(
                            EMA_ALPHA * r + (1 - EMA_ALPHA) * o for r, o in zip(raw_lm, old))
                        arm_pts.append(arm_smoothed[idx])
                    if len(arm_pts) == 2:
                        text += " " + " ".join(f"{v:.4f}" for pt in arm_pts for v in pt)
                        h, w, _ = frame.shape
                        pts = [(int(pt[0] * w), int(pt[1] * h)) for pt in (arm_pts[1], arm_pts[0], smoothed)]
                        cv2.line(frame, pts[0], pts[1], (255, 128, 0), 3)
                        cv2.line(frame, pts[1], pts[2], (255, 128, 0), 3)
                    packet = text.encode()
                    try:
                        sock.sendto(packet, (UDP_IP, UDP_PORT))
                        sock.sendto(packet, (UDP_IP, WEB_UDP_PORT))
                    except Exception as e:
                        print(f"UDP send error: {e}")

                draw_x, draw_y = smoothed[:2] if smoothed is not None else (wrist.x, wrist.y)
                h, w, _ = frame.shape
                cx, cy = int(draw_x * w), int(draw_y * h)
                color = (0, 255, 0) if is_visible else (0, 0, 255)  # red = not sent (low confidence)
                cv2.circle(frame, (cx, cy), 8, color, -1)
                curls_text = " ".join(
                    f"{name}:{c:.2f}" for (name, *_), c in zip(FINGERS, curls_smoothed)
                )
                cv2.putText(frame, curls_text, (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)

            cv2.imshow("Vision Sender — Right Wrist", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()
