#!/usr/bin/env bash
# vision_sender.py 실행 스크립트 (어느 폴더에서 실행해도 된다)
# 필요: 웹캠, venv/bin/python, mediapipe, opencv-python

set -e

ROOT="$(cd "$(dirname "$0")/../.." && pwd)"   # 저장소 루트 (venv/가 있는 곳)

if [ ! -d "$ROOT/venv" ]; then
    echo "[ERROR] venv 없음. 저장소 루트($ROOT)에서 먼저 Python 가상환경을 만들고 패키지를 설치하세요:"
    echo "  python3 -m venv venv"
    echo "  venv/bin/python -m pip install mediapipe opencv-python"
    exit 1
fi

echo "[INFO] 웹캠에서 오른손 손목 좌표를 읽어 UDP 127.0.0.1:5005 로 전송합니다."
echo "[INFO] 종료: 비디오 창에서 'q' 키"
echo ""

"$ROOT/venv/bin/python" "$ROOT/motion2/src/vision_sender.py"
