#!/usr/bin/env python3
"""Zaccaria The Invaders sound board (SN76477 + 555 + PVI filter): fixed-point constants for rtl/zac_snd.sv and the
verilator/snd reference model.  Component values and switching from the schematic (zaccaria_theinvaders_schematics-1.pdf
p4), SN76477 behaviour from MAME devices/sound/sn76477.cpp (rate formulas, thresholds, output gain tables).

    zac_snd_consts.py  ->  rtl/zac_snd_consts.svh, verilator/snd/zac_snd_consts.h

Voltages are Q24 (1 V = 1 << 24).  The model updates at FS = 39.936 MHz / 208 = 192 kHz.
"""
import math
from pathlib import Path

FS = 39_936_000 / 208
Q = 1 << 24
K, M, U, N = 1e3, 1e6, 1e-6, 1e-9

# SN76477 thresholds (MAME, measured)
ONE_SHOT_MAX = 2.5
SLF_MIN, SLF_MAX = 0.33, 2.37
VCO_DIFF = 0.35
VCO_MIN, VCO_MAX = SLF_MIN, SLF_MAX + VCO_DIFF
NOISE_MAX, NOISE_HI, NOISE_LO = 5.0, 3.35, 0.74
AD_MAX = 4.44
OUT_CENTER, OUT_HI, OUT_LO = 2.57, 3.51, 0.715
OUT_POS = [0.00] * 9 + [0.01, 0.03, 0.11, 0.15, 0.19, 0.21, 0.23, 0.26, 0.29, 0.31, 0.33, 0.36, 0.38, 0.41, 0.43, 0.46,
           0.49, 0.52, 0.54, 0.57, 0.60, 0.62, 0.65, 0.68, 0.70, 0.73, 0.76, 0.80, 0.82, 0.84, 0.87, 0.90, 0.93, 0.96,
           0.98, 1.00]
OUT_NEG = [0.00] * 9 + [-0.01, -0.02, -0.09, -0.13, -0.15, -0.17, -0.19, -0.22, -0.24, -0.26, -0.28, -0.30, -0.32,
           -0.34, -0.37, -0.39, -0.41, -0.44, -0.46, -0.48, -0.51, -0.53, -0.56, -0.58, -0.60, -0.62, -0.65, -0.67,
           -0.69, -0.72, -0.74, -0.76, -0.78, -0.81, -0.84, -0.85]
assert len(OUT_POS) == len(OUT_NEG) == 45

# board components (fixed)
R_NOISE_CLK, R_ATTACK, R_AMP, R_VCO, R_SLF = 47 * K, 4.7 * K, 47 * K, 47 * K, 47 * K
C_AD = 1 * U
R_FEEDBACK = 12 * K      # NOT on the sheet (pin 12 drawn as OUT AUDIO): chosen for a ~1 V centre-to-peak output


def par(*rs):
    return 1 / sum(1 / r for r in rs)


def q(v):
    return int(round(v * Q))


def step(rate):
    return max(1, q(rate / FS))


# one-shot: R = 220K || (1M when DB2 | DB3); C = 0.1u + 1u (DB2) + 6.8u (DB1 | DB5)
def os_r(b23):
    return par(220 * K, 1 * M) if b23 else 220 * K


def os_c(b2, b15):
    return 0.1 * U + (1 * U if b2 else 0) + (6.8 * U if b15 else 0)


OS_CHG = [step(ONE_SHOT_MAX / (0.8024 * os_r(i >> 2 & 1) * os_c(i >> 1 & 1, i & 1) + 0.002079)) for i in range(8)]
OS_DIS = [step(ONE_SHOT_MAX / (854.7 * os_c(i >> 1 & 1, i & 1) + 0.00001795)) for i in range(4)]

# SLF: R 47K, C = 1u + 6.8u (DB1)
def slf_c(b1):
    return 1 * U + (6.8 * U if b1 else 0)


SLF_CHG = [step((SLF_MAX - SLF_MIN) / (0.5885 * R_SLF * slf_c(b) + 0.001300)) for b in range(2)]
SLF_DIS = [step((SLF_MAX - SLF_MIN) / (0.5413 * R_SLF * slf_c(b) + 0.001343)) for b in range(2)]

# VCO: R 47K, C = 22n + 0.1u (DB3); duty 50 % (pitch pin 19 not on the sheet)
VCO_STEP = [step(0.64 * 2 * (VCO_MAX - VCO_MIN) / (R_VCO * (22 * N + (100 * N if b else 0)))) for b in range(2)]

# noise filter: R = 330K || (10K when DB2 | DB3); C = C5 1n + C6 1n in series, C6 shorted by DB4
def nf_r(b23):
    return par(330 * K, 10 * K) if b23 else 330 * K


def nf_c(b4):
    return 1 * N if b4 else 0.5 * N


NF_CHG = [step(NOISE_MAX / (0.1571 * nf_r(i >> 1 & 1) * nf_c(i & 1) + 0.00001430)) for i in range(4)]
NF_DIS = [step(NOISE_MAX / (0.1331 * nf_r(i >> 1 & 1) * nf_c(i & 1) + 0.00001734)) for i in range(4)]
NOISE_FREQ = int(339100000 * R_NOISE_CLK ** -0.8849)

# attack / decay: attack 4.7K, decay = 2.2M || 180K (DB2) || 330K (DB3) || 220K (DB1 | DB5); C 1u
def dec_r(b2, b3, b15):
    rs = [2.2 * M] + ([180 * K] if b2 else []) + ([330 * K] if b3 else []) + ([220 * K] if b15 else [])
    return par(*rs)


AD_CHG = step(AD_MAX / (R_ATTACK * C_AD))
AD_DIS = [step(AD_MAX / (dec_r(i >> 2 & 1, i >> 1 & 1, i & 1) * C_AD)) for i in range(8)]

# output: centre-to-peak (MAME compute_center_to_peak_voltage_out), clipped, as a signed 16-bit sample
CTP = 3.818 * (R_FEEDBACK / R_AMP) + 0.03
SCALE = 1 / (OUT_CENTER - OUT_LO)
OUT_P = [int(round(min(CTP * g, OUT_HI - OUT_CENTER) * SCALE * 32767)) for g in OUT_POS]
OUT_N = [int(round(max(CTP * g, OUT_LO - OUT_CENTER) * SCALE * 32767)) for g in OUT_NEG]

# 555 (CI5) from DZ1 7.5 V: charge via R12 + R11 = 181K toward 7.5 V, discharge via R11 1K, C11 1u, thresholds 1/3, 2/3
V555 = 7.5
K555_CHG = int(round((1 - math.exp(-1 / (FS * 181 * K * 1 * U))) * (1 << 24)))   # Q24 fraction per update
K555_DIS = int(round((1 - math.exp(-1 / (FS * 1 * K * 1 * U))) * (1 << 24)))

# PVI path: 7406 open collector (pull-up R14) -> R15 100R -> C16 4.7u (R16 150R to Vcc) -> R17 100R -> C17 10u -> out
def kf(r, c):
    return int(round((1 - math.exp(-1 / (FS * r * c))) * (1 << 24)))


# node 1: 7406 on -> R15 100R to 0 V against R16 150R to 5 V (2.0 V through 60R); off -> R16 alone (5 V through 150R,
# the R14 pull-up path through R15 neglected).  Node 2: R17 100R / C17 10u.  Output: R19 + P1 into C12 1u (~20 ms HP).
# P1 (the PVI / SN76477 balance trimmer) has no value on the sheet: PVI_GAIN sets the balance.
K_PVI1_LO = kf(par(100, 150), 4.7 * U)
K_PVI1_HI = kf(150, 4.7 * U)
K_PVI2 = kf(100, 10 * U)
K_PVI_HP = kf(20 * K, 1 * U)
PVI_GAIN = 5650                          # 16-bit units per volt of filtered PVI swing (HW: 4000 a little low, +3 dB)


def emit():
    consts = {
        "ZS_ONE_SHOT_MAX": q(ONE_SHOT_MAX), "ZS_SLF_MIN": q(SLF_MIN), "ZS_SLF_MAX": q(SLF_MAX),
        "ZS_VCO_DIFF": q(VCO_DIFF), "ZS_VCO_MIN": q(VCO_MIN), "ZS_VCO_MAX": q(VCO_MAX),
        "ZS_NOISE_MAX": q(NOISE_MAX), "ZS_NOISE_HI": q(NOISE_HI), "ZS_NOISE_LO": q(NOISE_LO),
        "ZS_AD_MAX": q(AD_MAX), "ZS_AD_CHG": AD_CHG, "ZS_NOISE_FREQ": NOISE_FREQ, "ZS_FS": int(FS),
        "ZS_V555": q(V555), "ZS_V555_LO": q(V555 / 3), "ZS_V555_HI": q(2 * V555 / 3),
        "ZS_K555_CHG": K555_CHG, "ZS_K555_DIS": K555_DIS,
        "ZS_K_PVI1_LO": K_PVI1_LO, "ZS_K_PVI1_HI": K_PVI1_HI, "ZS_K_PVI2": K_PVI2, "ZS_K_PVI_HP": K_PVI_HP,
        "ZS_PVI_LO": q(5.0 * 100 / (100 + 150)), "ZS_PVI_HI": q(5.0), "ZS_PVI_GAIN": PVI_GAIN,
    }
    tables = {"ZS_OS_CHG": OS_CHG, "ZS_OS_DIS": OS_DIS, "ZS_SLF_CHG": SLF_CHG, "ZS_SLF_DIS": SLF_DIS,
              "ZS_VCO_STEP": VCO_STEP, "ZS_NF_CHG": NF_CHG, "ZS_NF_DIS": NF_DIS, "ZS_AD_DIS": AD_DIS,
              "ZS_OUT_P": OUT_P, "ZS_OUT_N": OUT_N}
    here = Path(__file__).resolve().parent.parent
    sv = ["// generated by tools/zac_snd_consts.py -- do not edit"]
    h = ["// generated by tools/zac_snd_consts.py -- do not edit", "#pragma once", "#include <cstdint>"]
    for n, v in consts.items():
        sv.append(f"localparam int {n} = {v};")
        h.append(f"static const int64_t {n} = {v};")
    for n, t in tables.items():
        sv.append(f"localparam int {n} [{len(t)}] = '{{{', '.join(str(x) for x in t)}}};")
        h.append(f"static const int32_t {n}[{len(t)}] = {{{', '.join(str(x) for x in t)}}};")
    (here / "rtl" / "zac_snd_consts.svh").write_text("\n".join(sv) + "\n")
    (here / "verilator" / "snd" / "zac_snd_consts.h").write_text("\n".join(h) + "\n")
    print("\n".join(sv))


if __name__ == "__main__":
    emit()
