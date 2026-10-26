"""Imported first by every script: puts src/ on the path so the scripts run from any directory
with the repo's venv, no PYTHONPATH needed. ROOT is the motion2/ folder (src/, models/, arm/ live
under it); mujoco_receiver.py adds ROOT itself to reach the arm/ package."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
