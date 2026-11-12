"""Imported first by every script: puts src/ on the path so the scripts run from any directory
with the repo's venv, no `pip install -e .` or PYTHONPATH needed. ROOT is the motion1/ folder;
default output paths are resolved against it."""
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))


def _setup_from_layouts(argv):
    """With --layouts FILE and no JAX_SETUP, take the setup the file was made in (its "setup" key; files from
    before the setups existed are the sim setup's), so the file opens in its own scene without the variable
    (user 2026-11-12). The setup is read when env.py is first imported, hence here, before any script imports
    it. An explicit JAX_SETUP still wins, and a mismatch is still refused by layouts.replay."""
    import json
    import os
    if "JAX_SETUP" in os.environ or "--layouts" not in argv[:-1]:
        return
    path = Path(argv[argv.index("--layouts") + 1])
    if path.is_file():
        os.environ["JAX_SETUP"] = json.loads(path.read_text()).get("setup", "sim")


_setup_from_layouts(sys.argv)
