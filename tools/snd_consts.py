#!/usr/bin/env python3
"""Space Invaders sound board constants: Midway A084-90700-B000, Taito L-shaped SVN00001B / SVN00003A, the two
Space Invaders II cocktail (invad2ct) boards and the Taito PV / CV game boards.

Circuit topology and algorithms follow MAME mw8080bw_a.cpp invaders_discrete (D. Renaud) + sn76477.cpp.
Midway values: Daughterboard.tif (+ parts catalog layouts B739 / C739). Every discrete value in MAME matches the
Midway sheet except the SN76477 (MAME has the Taito PV board's 8.2K / 0.1uF VCO and 56K amplitude resistor), C45,
the SN76477 mix (Midway: R125 47K into the summing amp) and the summing amp (MAME: rF 100K, no clipping; boards:
R105 100K || 50K volume pot, output limited to the LM3900 swing around its 5.73 V bias, C44 input coupling).
Taito values: taito_space_invader_l_shaped_board_schematics.pdf p2/p3 (clean redraws). Same topology; differences
are listed in BOARD_PARAMS. Taito mixes every voice through its own 1uF + 50K trimmer (VR1-7, taken at maximum) and
22K into the summing bus.
invad2ct values: Space Invaders 2 Cocktail.pdf, board 1 p17-18 (A082-90700-A000), board 2 p25 (M051-00851-A005);
both match MAME's invad2ct values. R41 is unclear on the print (10K assumed, as MAME).
Taito PV / CV values: tcsi-sp.pdf p5 (PVN00004, P.V game PC board) / p6 (CVR00002, C.V game PC board); scans, the
illegible values (taken from MAME or the Taito L-shaped set) are marked "illegible" in BOARD_PARAMS.

Fixed point: voltages Q20, coefficients Q28 (product >> 28). 48 kHz sample rate, oscillators 16 substeps/sample.

    snd_consts.py -> rtl/invaders_snd_consts.svh (RTL_BOARDS, selected by the `board` input),
                     verilator/snd/snd_consts.h (every BOARD_PARAMS entry)
"""
import math
from pathlib import Path

FS = 48000.0
SUB = 16
VBE = 0.5          # OP_AMP_NORTON_VBE
VP = 12.0
VOH = VP - VBE     # Norton output high = 11.5 V


def K(x):          # coefficient Q28
    v = round(x * (1 << 28))
    assert abs(v) < (1 << 35), x
    if x != 0:
        assert abs(v) >= 2000, f"coefficient {x} too small for Q28"
    return v


def V(x):          # voltage Q20
    return round(x * (1 << 20))


def rcexp(rc):
    return 1.0 - math.exp(-1.0 / FS / rc)


def par(*r):
    return 1.0 / sum(1.0 / x for x in r)


TAITO_TRIM = {}                                          # board -> Taito trimmer setting per voice (fraction of maximum)

# voice source resistance into the summing bus, in mixer order SH FL BN IH EX MS SN
MIDWAY_MIX = dict(SH=200e3, FL=10e3 + 200, BN=150e3, IH=200e3, EX=2e3 + 6.8e3 + 5.6e3, MS=150e3, SN=47e3)
INVAD2CT_MIX = dict(MIDWAY_MIX, SH=100e3, FL=15e3 + 200, IH=150e3, EX=10e3 + 6.8e3 + 5.6e3)   # board 1: R78 R134 R59 R86
TAITO_SRC = dict(SH=47e3, FL=10e3 + 200, BN=220e3, IH=47e3, EX=2e3 + 6.8e3 + 560, MS=150e3, SN=47e3)
PV_SRC = dict(TAITO_SRC, FL=68e3 + 200)                                                   # PV step output 68K

BOARD_PARAMS = {
    "MIDWAY": dict(trig=5.0, ms_vco_c=330e-12, ih_os_c1=0.1e-6, ms_os_c1=1e-6, ih_osc_c=0.1e-6, ex_r1=5.6e3,
                   sn_vco_r=39e3, sn_vco_c=0.047e-6, sn_amp_r=150e3, c45=1.0e-6, c37=0.0, rf=par(100e3, 50e3),
                   clamp=True, taito_mix=False, mix=MIDWAY_MIX, out=5200.0),
    # Taito L-shaped: trigger lines pulled to +12 V (1K), missile VCO 300pF, target-hit one-shot 1uF (redraw;
    # the PV scan shows 0.1uF - unverified), target-hit oscillator 0.22uF, flash filter 560R, SN76477 3.9K / 0.22uF
    "TAITO": dict(trig=12.0, ms_vco_c=300e-12, ih_os_c1=1e-6, ms_os_c1=1e-6, ih_osc_c=0.22e-6, ex_r1=560.0,
                  sn_vco_r=3.9e3, sn_vco_c=0.22e-6, sn_amp_r=150e3, c45=1.0e-6, c37=0.0, rf=par(100e3, 50e3),
                  clamp=True, taito_mix=True, mix=MIDWAY_MIX, out=9300.0),   # gameplay peak just under full scale
    "MAME": dict(trig=5.0, ms_vco_c=330e-12, ih_os_c1=0.1e-6, ms_os_c1=1e-6, ih_osc_c=0.1e-6, ex_r1=5.6e3,
                 sn_vco_r=8.2e3, sn_vco_c=0.1e-6, sn_amp_r=56e3, c45=0.1e-6, c37=0.0, rf=100e3,
                 clamp=False, taito_mix=False, mix=MIDWAY_MIX, out=1250.0),
}
# invad2ct board 1: SN76477 8.2K / 0.1uF / 56K, C18 + C16 0.22uF, missile one-shot C12 0.22uF, C45 0.1uF, own mixer R
BOARD_PARAMS["INVAD2CT1"] = dict(BOARD_PARAMS["MIDWAY"], sn_vco_r=8.2e3, sn_vco_c=0.1e-6, sn_amp_r=56e3,
                                 ih_os_c1=0.22e-6, ih_osc_c=0.22e-6, ms_os_c1=0.22e-6, c45=0.1e-6, mix=INVAD2CT_MIX)
# invad2ct board 2: C18 1uF, SN76477 amp R123 56K, C45 0.1uF, C37 1nF across the 50K volume pot (summer low-pass)
BOARD_PARAMS["INVAD2CT2"] = dict(BOARD_PARAMS["MIDWAY"], ih_os_c1=1e-6, sn_amp_r=56e3, c45=0.1e-6, c37=1e-9)
# Taito PV (invadpt2 family): SN76477 8.2K / 0.1uF (amplitude resistor illegible: MAME 56K), target-hit one-shot 0.1uF,
# trigger one-shot 0.22uF, flash one-shot C1 not drawn (2.2uF as CV / MAME), 47K per voice, step 68K, bus 1uF into
# the summer, MB3712 input 0.1uF / 47K, extend play gated by 960 Hz (8V)
BOARD_PARAMS["PV"] = dict(BOARD_PARAMS["TAITO"], sn_vco_r=8.2e3, sn_vco_c=0.1e-6, sn_amp_r=56e3, ih_os_c1=0.1e-6,
                          ms_os_c1=0.22e-6, tsrc=PV_SRC, tmix_r=47e3, c44=1e-6, c45=0.1e-6, amp_r=47e3, bn_8v=True,
                          out=64900.0)                     # gameplay peak = the Taito L-shaped set's
# Taito CV (sicv, sicv1): Taito L-shaped values except the target-hit one-shot 0.1uF and the LM377 bridge input
# 0.1uF / 10K; SN76477 pins 17 / 18 and the trigger one-shot C1 illegible (Taito L-shaped 3.9K / 0.22uF, 1uF);
# extend-play jumper A = 480 Hz
BOARD_PARAMS["CV"] = dict(BOARD_PARAMS["TAITO"], ih_os_c1=0.1e-6, c45=0.1e-6, amp_r=10e3,
                          out=18300.0)                     # gameplay peak = the Taito L-shaped set's
RTL_BOARDS = ["MIDWAY", "TAITO", "INVAD2CT1", "INVAD2CT2", "PV", "CV"]   # `board` input value = index (3 bits); append


def board(p, name=""):
    c = {}
    # ---- noise: 7515 Hz clock (MAME: breadboarded)
    c["NZ_INC"] = round(7515.0 / FS * (1 << 32))
    c["V12"] = V(12.0)
    c["VOH"] = V(VOH)
    c["VBE"] = V(VBE)
    c["OS_TRIG"] = V(p["trig"])

    # ---- Norton oscillator (OSC_1) thresholds, shared r3=100K r4=120K r5=1M
    osc_tl = (VOH / 1e6 + (0 - VBE) / 120e3) * 100e3 + VBE
    osc_th = (VOH / 1e6 + (VOH - VBE) / 120e3) * 100e3 + VBE
    c["OSC_TL"], c["OSC_TH"] = V(osc_tl), V(osc_th)
    # Norton VCO_1 thresholds, r3=680K r4=1M r5=1M
    i1 = (VP - VBE) / 1e6
    vco_tl = (i1 - (VP - 2 * VBE) / 1e6) * 680e3 + VBE
    vco_th = (i1 - (0 - VBE) / 1e6) * 680e3 + VBE
    c["VCO_TL"], c["VCO_TH"] = V(vco_tl), V(vco_th)

    # ---- saucer hit
    c["SH_ENV_UP"] = V(((12.0 - VBE) / 100e3) / (FS * 1e-6))        # R72 / C23
    c["SH_ENV_DN"] = V((VOH / 1e6) / (FS * 1e-6))                     # R71
    q0 = VOH / 1e6                                                   # R70
    c["SH_OSC_D0"] = V(q0 / (0.1e-6 * FS * SUB))                     # C21
    c["SH_OSC_D1"] = V(((VOH - VBE) / 470e3 - q0) / (0.1e-6 * FS * SUB))   # R64
    c["SH_VCO_KA"] = K(1.0 / (1e6 * 470e-12 * FS * SUB))            # R65, C22
    c["SH_VCO_KB"] = K((1.0 / 470e3 - 1.0 / 1e6) / (470e-12 * FS * SUB))  # R66
    c["SH_OUT_C"] = V(VOH * 680e3 / 2.7e6)                           # R74
    # ---- fleet: 555, R1 = parallel of the selected bits, 75K, 0.1uF
    rbits = [40e3, 68e3, 82e3, 100e3]
    for d in range(16):
        g = sum(1.0 / rbits[b] for b in range(4) if d >> b & 1)
        c[f"FL_KC{d}"] = K(rcexp((1.0 / g + 75e3) * 0.1e-6)) if g else K(rcexp(10e6 * 0.1e-6))
    c["FL_KD"] = K(rcexp(75e3 * 0.1e-6))
    c["FL_VCH"] = V(5.0 - 0.6)
    c["V555_TH"], c["V555_TR"] = V(10.0 / 3.0), V(5.0 / 3.0)
    c["VTTL"] = V(3.4)
    c["FL_RC1"] = K(rcexp(100 * 4.7e-6))
    c["FL_RC2"] = K(rcexp(200 * 10e-6))
    # ---- bonus missile base / extended play: 556, 100K, 47K, 1uF
    c["BN_KC"] = K(rcexp((100e3 + 47e3) * 1e-6))
    c["BN_KD"] = K(rcexp(47e3 * 1e-6))
    c["V5"] = V(5.0)
    # ---- one-shots (all three: r1 4.7M, r2 100K, r3 1M, r4 1M, r5 2.2M, c2 470pF)
    c["OS_K10"] = K(10.0)                                            # (trig - vc2) / r2 * r3
    c["OS_VOUT_R5"] = V(VOH * 1e6 / 2.2e6)                           # vout / r5 * r3
    c["OS_IFIX"] = V((VP - VBE) / 4.7e6 * 1e6)                       # i_fixed * r3
    c["OS_E2"] = K(rcexp(100e3 * 470e-12))
    c["OS_T1"], c["OS_T0"] = V((VOH - VBE) * 0.5 + VBE), V((0 - VBE) * 0.5 + VBE)
    c["V06"] = V(0.6)
    for n, c1 in (("IH", p["ih_os_c1"]), ("EX", 2.2e-6), ("MS", p["ms_os_c1"])):
        c[f"{n}_E1C"] = K(rcexp(par(1e6, 1e6) * c1))
        c[f"{n}_E1D"] = K(rcexp(1e6 * c1))
    # ---- invader hit / target hit
    c["IH_ENV_UP"] = V(((VOH - VBE) / 10e3) / (FS * 0.47e-6))        # 10K / 0.47uF
    c["IH_ENV_DN"] = V((VOH / 1e6) / (FS * 0.47e-6))                 # 1M
    ic = p["ih_osc_c"]
    c["IH_OSC_D0"] = V(q0 / (ic * FS * SUB))                         # 1M
    c["IH_OSC_D1"] = V(((VOH - VBE) / 10e3 - q0) / (ic * FS * SUB))  # 10K
    c["IH_VCO_KA"] = K(1.0 / (1e6 * 330e-12 * FS * SUB))            # 1M, 330pF
    c["IH_VCO_KB"] = K((1.0 / 470e3 - 1.0 / 1e6) / (330e-12 * FS * SUB))  # 470K
    c["IH_OUT_G"] = K(680e3 / 470e3)                                 # 680K : 470K
    c["IH_OUT_C"] = V(VOH * 680e3 / 2.7e6)                           # 2.7M
    # ---- explosion / flash TVCA: r1 2.7M, r2 680K, r4 680K, r5 10K, r7 680K, c1 1uF, v1 11.5
    c["EX_VT"] = V((VOH - 0.6 - VBE) * 680e3 / (10e3 + 680e3) + VBE)
    c["EX_EC"] = K(rcexp(par(10e3, 680e3) * 1e-6))
    c["EX_ED"] = K(rcexp(680e3 * 1e-6))
    c["EX_C"] = V(VOH * 680e3 / 2.7e6)
    c["EX_RC1"] = K(rcexp(p["ex_r1"] * 0.1e-6))
    c["EX_RC2"] = K(rcexp((p["ex_r1"] + 6.8e3) * 0.1e-6))
    # ---- missile / trigger sound
    c["MS_ENV_UP"] = V(((VOH - VBE) / 10e3) / (FS * 0.22e-6))        # 10K / 0.22uF
    c["MS_ENV_DN"] = V((VOH / 1.5e6) / (FS * 0.22e-6))               # 1.5M
    c["MS_CR"] = K(rcexp((1e6 + 330e3) * 0.1e-6))                    # 0.1uF, 1M + 330K
    c["MS_NZ_G"] = K(330e3 / (1e6 + 330e3))
    r1 = par(1e6, 330e3) + 1.5e6
    cs = p["ms_vco_c"] * FS * SUB
    c["MS_VCO_IF"] = V(((VP - VBE) / 3.3e6) / cs)                    # 3.3M
    c["MS_VCO_K1"] = K(1.0 / (r1 * cs))
    c["MS_VCO_K6"] = K(1.0 / (4.7e6 * cs))                           # 4.7M
    c["MS_VCO_T1"] = V(((VP - 2 * VBE) / 1e6) / cs)                  # 1M
    c["MS_VCO_TL"] = V((i1 + (0 - VBE) / 2.2e6) * 560e3 + VBE)       # 560K, 2.2M
    c["MS_VCO_TH"] = V((i1 + (VP - 2 * VBE) / 2.2e6) * 560e3 + VBE)
    c["MS_A3_G"] = K(560e3 / 470e3)
    c["MS_A3_C"] = V(VOH * 560e3 / 2.7e6)
    c["MS_VT"] = V((p["trig"] - 0.6 - VBE) * 560e3 / (1e3 + 560e3) + VBE)
    c["MS_EC"] = K(rcexp(par(1e3, 560e3) * 0.1e-6))                  # 0.1uF
    c["MS_ED"] = K(rcexp(560e3 * 0.1e-6))                            # 560K
    c["MS_C"] = V(VOH * 560e3 / 2.7e6)                               # 2.7M
    # ---- SN76477 saucer / UFO (VCO from SLF, one-shot envelope held at full level)
    c["SN_SLF_UP"] = V(2.04 / (0.5885 * 120e3 * 1e-6 + 0.0013) / FS)
    c["SN_SLF_DN"] = V(2.04 / (0.5413 * 120e3 * 1e-6 + 0.001343) / FS)
    c["SN_SLF_MIN"], c["SN_SLF_MAX"] = V(0.33), V(2.37)
    c["SN_VCO_DIFF"] = V(0.35)
    c["SN_VCO_STEP"] = V(0.64 * 2 * 2.39 / (p["sn_vco_r"] * p["sn_vco_c"]) / FS / SUB)
    a = 3.818 * (10e3 / p["sn_amp_r"]) + 0.03
    c["SN_HI"], c["SN_LO"], c["SN_MID"] = V(2.57 + a * 1.00), V(2.57 - a * 0.85), V(2.57)
    # ---- mixer: op-amp summer, rF = 100K || 50K volume pot (C42 blocks DC)
    rf = p["rf"]
    if p["taito_mix"]:
        # voice source -> 1uF -> 50K trimmer VR1-7 to ground -> 22K -> 1uF -> bus. No factory setting is documented:
        # the trimmers are set to give the Midway mix's balance (user: the all-at-maximum mix buried the march). The
        # march limits it - its trimmer stays at maximum, every other voice is turned down to keep Midway's ratios.
        rm = p.get("tmix_r", 22e3)                                     # voice resistor into the bus: L / CV 22K, PV 47K
        rp = par(50e3, rm)
        rsrc = {n: r for n, r in p.get("tsrc", TAITO_SRC).items()}
        gmax = {n: rf * (rp / (r + rp)) / rm for n, r in rsrc.items()}
        gmid = {n: rf / r for n, r in p["mix"].items()}
        scale = min(gmax[n] / gmid[n] for n in gmax)
        TAITO_TRIM[name] = {}
        for n in rsrc:
            c[f"G_{n}"] = K(scale * gmid[n])
            TAITO_TRIM[name][n] = scale * gmid[n] / gmax[n]
        rbus = par(*[rm + par(50e3, r) for r in rsrc.values()])       # each voice: 22K / 47K + its trimmer || source
        c["MX_C11"] = K(rcexp((150e3 + rp) * 0.001e-6))
    else:
        for n, r in p["mix"].items():
            c[f"G_{n}"] = K(rf / r)
        rbus = par(*p["mix"].values())
        c["MX_C11"] = K(rcexp(150e3 * 0.001e-6))
    c["MX_C44"] = K(rcexp(rbus * p.get("c44", 10e-6)))                  # bus coupling: C44 10uF (PV 1uF)
    c["MX_CAMP"] = K(rcexp(p.get("amp_r", 100e3) * p["c45"]))          # power amp input: C45 x 100K (PV 47K, CV 10K)
    c["MX_LP"] = K(rcexp(rf * p["c37"]) if p["c37"] else 1.0)          # C37 across the volume pot: rF x C37 low-pass
    # first LM3900 stage: DC bias (12 - VBE) / 220K x 100K + VBE, swing 0 .. VOH around it
    bias = (VP - VBE) / 220e3 * 100e3 + VBE
    c["MX_VLO"], c["MX_VHI"] = V(0 - bias), V(VOH - bias)
    c["OUT_G"] = K(p["out"] / (1 << 20))
    return c


BOARDS = {n: board(p, n) for n, p in BOARD_PARAMS.items()}


def main():
    root = Path(__file__).resolve().parent.parent
    bs = [BOARDS[b] for b in RTL_BOARDS]
    mid = bs[0]
    assert all(b.keys() == mid.keys() for b in bs) and len(bs) <= 8

    def lit(v):
        return f"{'-' if v < 0 else ''}36'sd{abs(v)}"

    sv = ["// Generated by tools/snd_consts.py - do not edit",
          "// `board`: " + ", ".join(f"{i} {b}" for i, b in enumerate(RTL_BOARDS)) + "; constants that differ follow it"]
    for k in mid:
        if all(b[k] == mid[k] for b in bs):
            sv.append(f"localparam signed [35:0] {k:<12} = {lit(mid[k])};")
        else:
            sel = "".join(f"board == 3'd{i} ? {lit(b[k])} : " for i, b in enumerate(bs) if i and b[k] != mid[k])
            sv.append(f"wire       signed [35:0] {k:<12} = {sel}{lit(mid[k])};")
    b8 = " || ".join(f"board == 3'd{i}" for i, b in enumerate(RTL_BOARDS) if BOARD_PARAMS[b].get("bn_8v")) or "1'b0"
    sv.append(f"wire                     BN_8V        = {b8};   // extend play gated by 8V (960 Hz), else 16V")
    (root / "rtl" / "invaders_snd_consts.svh").write_text("\n".join(sv) + "\n")
    hd = ["// Generated by tools/snd_consts.py - do not edit", "#pragma once", "#include <cstdint>",
          "namespace snd {"]
    for b, cs in BOARDS.items():
        hd.append(f"struct {b} {{")
        hd.append(f"    static constexpr bool CLAMP = {'true' if BOARD_PARAMS[b]['clamp'] else 'false'};")
        hd.append(f"    static constexpr bool BN_8V = {'true' if BOARD_PARAMS[b].get('bn_8v') else 'false'};")
        for k, v in cs.items():
            hd.append(f"    static constexpr int64_t {k} = {v}LL;")
        hd.append("};")
    hd.append("}")
    out = root / "verilator" / "snd"
    out.mkdir(parents=True, exist_ok=True)
    (out / "snd_consts.h").write_text("\n".join(hd) + "\n")
    for b in RTL_BOARDS[1:]:
        diff = [k for k in mid if mid[k] != BOARDS[b][k]]
        print(f"{len(mid)} constants per board, {len(diff)} differ Midway/{b}: {' '.join(diff)}")
    for b, t in TAITO_TRIM.items():
        print(f"{b} trimmer gain (fraction of maximum):", " ".join(f"{n}={v:.3f}" for n, v in t.items()))


if __name__ == "__main__":
    main()
