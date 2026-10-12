#!/usr/bin/env bash
# web_dashboard.py 실행 스크립트
# 필요: venv/bin/python (표준 라이브러리만 사용, 추가 설치 불필요)

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
cd "$SCRIPT_DIR"

if [ ! -d "../venv" ]; then
    echo "[ERROR] venv 없음. 먼저 Python 가상환경을 만드세요:"
    echo "  python3 -m venv ../venv"
    exit 1
fi

echo "[INFO] UDP 127.0.0.1:5006 에서 손목 좌표를 받아 http://127.0.0.1:8000 에 띄웁니다."
echo "[INFO] vision_sender.py 를 먼저(또는 같이) 실행하세요."
echo "[INFO] 종료: Ctrl+C"
echo ""

../venv/bin/python web_dashboard.py
