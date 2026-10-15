"""Double Play / Extra Inning (Midway 1977 / 1978) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Board: Extra Inning game logic P.C. A084-90700-A642 (Extra_Innings_Game_Logic_(A084-90700-A642).pdf, one page, sound =
right half). MAME runs both sets on one machine (dplay, DPLAY_AUDIO); no Double Play (619) sheet to compare. Started
from dsnd_boards.dplay() (MAME dplay_discrete, sim-checked; kept below as dplay_mame()). Latch H3 (port 3): D0 tone on
(7417 H2 + 10K to +12 V), D1 cheer (1K pull-up to +5 V), D2 siren (1K pull-up to +5 V), D3 whistle (7417 H2 + 10K to
+12 V), D4 game on (7404 J3 -> L1 mute diodes), D5 coin counter (TIP110) = MAME. Every part value = MAME's, except
what MAME leaves out. Schematic corrections (ngspice, verilator/snd/dplay/dp_spice.py):
- Siren / whistle VCO (M2-3 / K2-3 integrator + Schmitt, L4 4016): the vmod current is (vmod - VBE) / r1 (MAME's
  VCO_2 uses vmod / r1: the oscillator then starts at 0 V instead of ~1.35 V and runs 6 % (siren) / 14 % (whistle)
  high at the top), the Schmitt's rising edge slews at 0.5 V/us (LM3900) and closes the 4016 at ~6 V, 12 us late: the
  cap overshoots the low threshold by d x 12 us (whistle -12 % pitch). Crossings carry the overshoot (no clamp).
- Cheer gate P2 (8 / 13 / 9) is a Norton subtractor: out = integrator - noise - 1.76 V (680K / 680K, 3.3M from +12),
  so the cheer starts 0.22 s after D1 and stops ~1 s before the integrator empties; MAME switches the integrator
  voltage in whole.
- Cheer band pass P2 (11 / 12 / 10) sits on a 4.38 V bias (430K from the 20K / 10 uF node into +): it swings
  -4.38 / +7.1 V around it. MAME's version has no bias and clips everything below 0 V (half-wave).
- L1 mixer feedback is 100K || (50K volume pot + 1 uF); output bias 5.68 V (220K from the 2K / 10 uF node), swing
  0 .. 11.5 V. Modelled at the pot's maximum (the loudest setting): the output keeps MAME's level and scale, the L1
  window becomes 3 x (-5.68 / +5.82 V) in MAME's 100K units. MAME has no pot and no clipping there.
"""
from dsnd import Board, LATCH1, LATCH2, LATCH3, FN_TRG0, FN_TRG1, q, ONE, FS, VBE, rc_exp

K, M_, U = 1e3, 1e6, 1e-6
VBN = 0.55                    # LM3900 inverting input (gain transistor Vbe), as the fleet's ngspice model
VOH = 11.5                    # LM3900 output swing at V+ 12 V (datasheet V+ - 0.5 V = MAME's vP - VBE)
SLEW_TD = 6.0 / 0.5e6         # Schmitt output 0 -> 6 V (4016 control threshold at Vdd 12 V) at 0.5 V/us
MIDWAY_MUSIC_TVCA = dict(r1=3.3 * M_, r2=10 * K + 680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.001 * U,
                         v1=12, vP=12, f0=FN_TRG0, f2=FN_TRG1)


def logadj(rmin, rmax, pct):
    """DISC_LOGADJ as MAME computes it"""
    import math
    return 10 ** (math.log10(rmin) + pct / 100 * (math.log10(rmax) - math.log10(rmin)))


def vco2_hw(b, out, vmod, r1, r2, r3, r4, r5, r6, c, vP=12.0):
    """Norton VCO (VCO_2 topology: vmod -> r1 -> integrator -, r2 + 4016-switched r6 from vP -> +, inverting
    Schmitt r3 / r4 / r5) as the circuit runs it: discharge d0 = (max(0, vmod - VBN) / r1 - I2) / (FS C) per sample,
    charge d1 = (I2 + I6) / (FS C) - that current; I2 = (vP - VBE) / r2, I6 = (vP - VBE) / r6. Thresholds from the
    Schmitt at VBN / VBE with the output swinging 0 .. VOH. The low turn comes SLEW_TD after the threshold (the cap
    keeps discharging until the slewing Schmitt closes the 4016). Overshoot carried into the next phase (one RCP).
    d0 <= 0 (vmod < ~1.35 V): the cap drifts up to VOH and the output stays low (no oscillation). out = VOH while
    charging (Schmitt high), else 0."""
    i2, i6 = (vP - VBE) / r2, (vP - VBE) / r6
    i5 = (vP - VBE) / r5
    tl = VBN + (i5 + (0.0 - VBE) / r4) * r3
    th = VBN + (i5 + (VOH - VBE) / r4) * r3
    td = SLEW_TD * FS                                 # samples
    sc = 16.0                                         # RCP range: rate x 16 within 1 / 128 .. 64 (whistle d1 <= 4 V)
    b.kreg(out + "_ka", 1.0 / (r1 * FS * c), "O", clamp=False)
    b.kreg(out + "_k2", i2 / (FS * c), "O", clamp=False)
    b.kreg(out + "_k26", (i2 + i6) / (FS * c), "O", clamp=False)
    b.load(vmod)
    b.op("ADDI", imm=q(-VBN))
    b.op("MAXI", imm=0)
    b.op("MUL", out + "_ka")
    b.op("ST", out + "_iv")
    b.op("SUB", out + "_k2")
    b.op("ST", out + "_d0")                           # discharge step (<= 0: stopped)
    b.op("LD", out + "_k26")
    b.op("SUB", out + "_iv")
    b.op("ST", out + "_d1")                           # charge step (> 0 always)
    b.op("LD", out + "_lo")
    b.op("CMPI", imm=q(0.5))
    at_dis = b.skip_if(True)
    # charging (Schmitt high, 4016 closed)
    b.op("LD", out + "_v")
    b.op("ADD", out + "_d1")
    b.op("ST", out + "_v")
    b.op("CMPI", imm=q(th))
    at = b.skip_if(False)
    b.op("ADDI", imm=q(-th))                          # crossed: rest of the sample discharging
    b.op("MULI", imm=q(sc))
    b.op("ST", out + "_ov")
    b.op("LD", out + "_d1")
    b.op("MULI", imm=q(sc))
    b.op("RCP")
    b.op("MUL", out + "_ov")
    b.op("MINI", imm=ONE)
    b.op("MUL", out + "_d0")
    b.op("ST", out + "_ov")
    b.op("LDI", imm=q(th))
    b.op("SUB", out + "_ov")
    b.op("MINI", imm=q(VOH))
    b.op("ST", out + "_v")
    b.op("LDI", imm=ONE)
    b.op("ST", out + "_lo")
    b.land(at)
    end = b.jump()
    b.land(at_dis)
    # discharging (Schmitt low, 4016 open); turn at tl - d0 x td
    b.op("LD", out + "_d0")
    b.op("MAXI", imm=0)
    b.op("MULI", imm=q(-td))
    b.op("ADDI", imm=q(tl))
    b.op("ST", out + "_tl")
    b.op("LD", out + "_v")
    b.op("SUB", out + "_d0")
    b.op("MINI", imm=q(VOH))
    b.op("ST", out + "_v")
    b.op("CMP", out + "_tl")
    at = b.skip_if(True)
    b.op("LD", out + "_tl")                           # crossed: rest of the sample charging
    b.op("SUB", out + "_v")
    b.op("MULI", imm=q(sc))
    b.op("ST", out + "_ov")
    b.op("LD", out + "_d0")
    b.op("MULI", imm=q(sc))
    b.op("RCP")
    b.op("MUL", out + "_ov")
    b.op("MINI", imm=ONE)
    b.op("MUL", out + "_d1")
    b.op("ADD", out + "_tl")
    b.op("ST", out + "_v")
    b.op("LDI", imm=0)
    b.op("ST", out + "_lo")
    b.land(at)
    b.land(end)
    b.op("LD", out + "_lo")
    b.op("CMPI", imm=q(0.5))
    b.op("LDI", imm=q(VOH))
    b.op("LDIF", imm=0)
    b.op("ST", out)


# cheer gate P2 8 / 13 / 9: out = (io - VBE) - 680K x (12 - VBN) / 3.3M - (noise - VBN) + VBN
CHEER_OFS = -VBE - 680 * K * (12 - VBN) / (3.3 * M_) + 2 * VBN
# cheer band pass P2 11 / 12 / 10 output bias: + input 430K from the 20K / 10 uF node off +12 V
_vb = (12 + VBE * 20 / 430) / (1 + 20 / 430)
CHEER_BIAS = VBN + 150 * K * (_vb - VBE) / (430 * K)
# L1 mixer: bias 5.68 V (+ 220K from the 2K / 10 uF node; both L1 halves load it), swing 0 .. VOH; feedback
# 100K || 50K pot (max) = 33.3K -> MAME's 100K-units window = (bias - 0, VOH - bias) x 100K / 33.3K
_vl = 12 - 2 * K * 2 * (12 - VBE) / (220 * K)
L1_BIAS = VBN + 100 * K * (_vl - VBE) / (220 * K)
L1_K = 100 * K / (1 / (1 / (100 * K) + 1 / (50 * K)))


def dplay():
    """Double Play / Extra Inning from the A084-90700-A642 schematic. S1 = port 3 (D0 tone on, D1 cheer, D2 siren,
    D3 whistle, D4 game on, D5 coin), S2 / S3 = Midway tone generator (ports 5 / 6). Route 0.8 x gain 2000 as MAME,
    music pot 60 % of 1M..1k log (MAME's default)."""
    b = Board("dplay")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("music", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH1, 0), 0), inp=(12.0, 0.0))
    b.integrate_norton1("siren_i", (LATCH1, 2), 5.0, 1 * M_, 100 * K, 3.3 * U, 12, 12)
    vco2_hw(b, "siren", "siren_i", 390 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 56 * K, 0.0022 * U)
    b.integrate_norton1("wh_i", (LATCH1, 3), 12.0, 1 * M_, 230 * K, 3.3 * U, 12, 12)
    vco2_hw(b, "whistle", "wh_i", 510 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 300 * K, 220e-12)
    b.lfsr_noise("noise", 7700, 12.0, 6.0)            # 4006 / 4030 clock: MAME's breadboard measurement
    b.integrate_norton1("ch_i", (LATCH1, 1), 5.0, 1.5 * M_, 100 * K, 4.7 * U, 12, 12)
    b.op("LD", "ch_i")                                # P2 gate: noise high (12 V) holds it at 0 V
    b.op("ADDI", imm=q(CHEER_OFS))
    b.op("MAXI", imm=0)
    b.op("ST", "ch_sw")
    b.op("LD", "noise")
    b.op("CMPI", imm=1 << 24)
    b.op("LDI", imm=0)
    b.op("STF", "ch_sw")
    b.op_amp_filt_bp1m("cheer", "ch_sw", 100 * K, 150 * K, 0.0047 * U, 0.0047 * U, r3=100 * K, vref=CHEER_BIAS,
                       vp=VOH + 1.5, vn=0.0)
    music = 68 * K + logadj(1e6, 1000, 60)
    b.mixer_op_amp("mix", [("tone", 68 * K, 0.1 * U), ("siren", 68 * K, 0.1 * U), ("whistle", 68 * K, 0.1 * U),
                           ("cheer", 18 * K, 0.1 * U), ("music", music, 0.1 * U, True)], rf=100 * K)
    b.op("LD", "mix")                                 # L1 output window (pot at max), then the 0.1 uF / 100K cAmp
    b.op("MAXI", imm=q(-L1_K * L1_BIAS))             # mix = (L1 out - bias) x L1_K, L1 out 0 .. VOH
    b.op("MINI", imm=q(L1_K * (VOH - L1_BIAS)))
    b.op("ST", "mix_s")
    b.kreg("mix_ka", rc_exp(100 * K * 0.1 * U), "F")
    b.op("RCM", "mix_amp", b.r("mix_ka"))
    b.op("LD", "mix_s")
    b.op("SUB", "mix_amp")
    b.op("ST", "mix_o")
    b.op("BIT", 0, b.src(LATCH1, 4))                  # D4 game on (low: J3 drives both L1 - inputs: mute)
    b.op("NOT")
    b.op("LD", "mix_o")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(2000 * 0.8 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def dplay_mame():
    """dsnd_boards.dplay() as it was (MAME dplay_discrete), for A / B checks"""
    b = Board("dplay")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("music", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH1, 0), 0), inp=(12.0, 0.0))
    b.integrate_norton1("siren_i", (LATCH1, 2), 5.0, 1 * M_, 100 * K, 3.3 * U, 12, 12)
    b.op_amp_vco2_norton("siren", "siren_i", 390 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 56 * K, 0.0022 * U, 12)
    b.integrate_norton1("wh_i", (LATCH1, 3), 12.0, 1 * M_, 230 * K, 3.3 * U, 12, 12)
    b.op_amp_vco2_norton("whistle", "wh_i", 510 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 300 * K, 220e-12, 12)
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    b.integrate_norton1("ch_i", (LATCH1, 1), 5.0, 1.5 * M_, 100 * K, 4.7 * U, 12, 12)
    b.op("LD", "noise")
    b.op("CMPI", imm=1 << 24)
    b.op("LDI", imm=0)
    b.op("LDF", "ch_i")
    b.op("ST", "ch_sw")
    b.op_amp_filt_bp1m("cheer", "ch_sw", 100 * K, 150 * K, 0.0047 * U, 0.0047 * U, r3=100 * K)
    music = 68 * K + logadj(1e6, 1000, 60)
    b.mixer_op_amp("mix", [("tone", 68 * K, 0.1 * U), ("siren", 68 * K, 0.1 * U), ("whistle", 68 * K, 0.1 * U),
                           ("cheer", 18 * K, 0.1 * U), ("music", music, 0.1 * U, True)], rf=100 * K, c_amp=0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 4))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(2000 * 0.8 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    for name, f in (("dplay", dplay), ("dplay_mame", dplay_mame)):
        p = f()
        path = (sys.argv[1] + "/" if len(sys.argv) > 1 else "") + name + ".bin"
        open(path, "wb").write(p.binary())
        print(f"{name}: {len(p.code)} words, worst sample {p.worst_path()}, {len(p.regs)} registers -> {path}")
