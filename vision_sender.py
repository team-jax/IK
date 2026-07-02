"""OpenCV + MediaPipe Tasks PoseLandmarker: extract right wrist (X,Y,Z) and send via UDP.

mediapipe>=0.10.30 dropped the legacy `mp.solutions` API (see pip package contents —
`mediapipe.solutions` no longer ships), so this uses the replacement Tasks API
(`mediapipe.tasks.python.vision.PoseLandmarker`) instead.

Usage:
    venv/bin/python vision_sender.py
    (press 'q' in the video window to quit)

Requires pose_landmarker_full.task in this directory (downloaded from
https://storage.googleapis.com/mediapipe-models/pose_landmarker/pose_landmarker_full/float16/latest/pose_landmarker_full.task).
"""

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

RIGHT_WRIST = 16  # PoseLandmark index, same 33-point topology as the legacy Solutions API
MODEL_PATH = Path(__file__).parent / "pose_landmarker_full.task"

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


def main():
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open webcam")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    options = mp_vision.PoseLandmarkerOptions(
        base_options=mp_python.BaseOptions(model_asset_path=str(MODEL_PATH)),
        running_mode=mp_vision.RunningMode.VIDEO,
        num_poses=1,
        min_pose_detection_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    print(f"Sending right wrist position to {UDP_IP}:{UDP_PORT} and {UDP_IP}:{WEB_UDP_PORT} ...")

    smoothed = None  # (x, y, z) EMA state; None until the first visible frame

    with mp_vision.PoseLandmarker.create_from_options(options) as landmarker:
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
                    packet = f"{smoothed[0]:.4f} {smoothed[1]:.4f} {smoothed[2]:.4f}".encode()
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

            cv2.imshow("Vision Sender — Right Wrist", frame)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    sock.close()


if __name__ == "__main__":
    main()
