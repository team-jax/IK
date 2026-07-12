#!/usr/bin/env bash
# mujoco_receiver.py 실행 스크립트
# 필요: venv/bin/python (Linux) 또는 venv/bin/mjpython (macOS), mujoco

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d "venv" ]; then
    echo "[ERROR] venv 없음. 먼저 Python 가상환경을 만들고 패키지를 설치하세요:"
    echo "  python3 -m venv venv"
    echo "  venv/bin/pip install mujoco"
    exit 1
fi

echo "[INFO] UDP 127.0.0.1:5005 에서 손목 좌표 + 손가락 curl을 수신해 MuJoCo 7DOF 팔 + 그리퍼를 구동합니다."
echo "[INFO] 뷰어 키: C = 좌우 중심 재캘리브레이션, T = 그리퍼 추적 카메라 토글"
echo "[INFO] vision_sender.py 를 먼저 실행하세요 (별도 터미널)."
echo "[INFO] 종료: MuJoCo 뷰어 창 닫기"
echo ""

if [[ "$(uname)" == "Darwin" ]]; then
    if [ ! -f "venv/bin/mjpython" ]; then
        echo "[ERROR] venv/bin/mjpython 없음. macOS에서는 mjpython 필요:"
        echo "  venv/bin/pip install mujoco"
        exit 1
    fi
    venv/bin/mjpython mujoco_receiver.py
else
    venv/bin/python mujoco_receiver.py
fi
