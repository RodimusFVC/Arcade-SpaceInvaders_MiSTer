"""This core's engine sample rate for the fleet assembler (RodimusCoresMaster_MiSTer/tools/dsnd.py).

Every board module imports this before `from dsnd import ...`: board modules read dsnd.FS at import time, so the
rate is set before the first of them loads, whichever script imports them.
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "RodimusCoresMaster_MiSTer" / "tools"))
import dsnd  # noqa: E402

if dsnd.FS is None:
    dsnd.set_fs(39936000 / 832)                  # 48 kHz: 39.936 MHz engine clock / TICK 832
