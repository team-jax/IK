#!/usr/bin/env bash
# vision_sender.py 실행 스크립트
# 필요: 웹캠, venv/bin/python, mediapipe, opencv-python

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d "../venv" ]; then
    echo "[ERROR] venv 없음. 먼저 Python 가상환경을 만들고 패키지를 설치하세요:"
    echo "  python3 -m venv ../venv"
    echo "  ../venv/bin/python -m pip install mediapipe opencv-python"
    exit 1
fi

echo "[INFO] 웹캠에서 오른손 손목 좌표를 읽어 UDP 127.0.0.1:5005 로 전송합니다."
echo "[INFO] 종료: 비디오 창에서 'q' 키"
echo ""

../venv/bin/python vision_sender.py
