"""Midway sound board parts shared by several board programs: one definition, imported by each board module."""
import math
import dsnd_rate  # noqa: F401  master assembler + this core's FS, before any dsnd import
from dsnd import FN_TRG0, FN_TRG1  # noqa: E402

K, M_, U = 1e3, 1e6, 1e-6

# tone / music trigger-VCA stage (DISCRETE_OP_AMP_TRIG_VCA) common to the Midway boards
MIDWAY_MUSIC_TVCA = dict(r1=3.3 * M_, r2=10 * K + 680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.001 * U,
                         v1=12, vP=12, f0=FN_TRG0, f2=FN_TRG1)


def logadj(rmin, rmax, pct):
    """DISCRETE_ADJUSTMENT with DISC_LOGADJ at a PORT_ADJUSTER default (percent)"""
    return 10 ** (math.log10(rmin) + pct / 100 * (math.log10(rmax) - math.log10(rmin)))
