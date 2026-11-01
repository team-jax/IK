"""Imported first by every script: puts src/ on the path so the scripts run from any directory
with the repo's venv, no `pip install -e .` or PYTHONPATH needed. ROOT is the motion1/ folder;
default output paths are resolved against it."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))
