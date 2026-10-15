"""Space Encounters (Midway 1980) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

From the schematic (Sound Generator A084-91324-F645, M051-00645-A004, Useful Information/schematics/
Space_Encounters_Wiring_and_Schematics.pdf p3); MAME mw8080bw_a.cpp (D. Renaud) where it agrees, and its node models
for the parts MAME has. ngspice decks: verilator/snd/spcenctr/sc_spice.py.
Latches: P1 = S1 (D0 sound on via 7404 + 7417 -> /GAME ON, D2 crash), P2 = S2 (D0-D3 wind DAC, D4 explosion, D5 player
shot), P3 = S3 (D0 screech, D1 enemy ship shot, D2 strobe, D3 lamp, D4 bonus, D5 SN76477 enable, active low).
Mixer: every sound feeds the summing bus through its resistor (and coupling cap); the bus reaches the D0 Norton
summing input through C900 1 uF (no resistor), so C900 sees the bus resistance (1.35k): a 118 Hz high pass. The
SN76477 bypasses C900 (R706 15k + C987 1 uF straight into D0 pin 6). Output = the summing current, scaled so a
current gives MAME's level for it (MAME: Millman volts x 20000 x route 0.45; Millman rt 1589 ohm).
Rates (engine budget): per sample = noise, enemy / player shot, VCA outputs, bonus tone, SN76477 VCO, bus.
Alternate samples (24 kHz, held): A = wind filter + phaser, explosion ladder; B = screech, crash filter.
4-sample round robin (12 kHz, 4x steps): wind LFO / DAC / duty, VCA envelope caps, SN76477 SLF, bonus gate 555,
coupling caps >= 2 ms, enemy-shot VCO reciprocals (vmod moves over 60 ms+).
"""
import math
from dsnd import Board, LATCH1, LATCH2, LATCH3, q, ONE, FS, rc_exp, VBE, par

K, M_, U = 1e3, 1e6, 1e-6
VN = 0.55                                            # LM3900 input voltage at the currents here (ngspice: 0.55-0.62)
RS = dict(es=15 * K, ps=15 * K, sc=22 * K, cr=3.6 * K, ex=4.7 * K, bn=27 * K, wn=27 * K)    # bus resistors
RREF = 1.0 / sum(1.0 / r for r in RS.values())       # bus resistance (1345 ohm)
OUT_A = 1589.0 * 20000 * 0.45 / 32768                # output per (summing current x 1 ohm): MAME's level
DT = 1.0 / FS
N_RR = 4                                             # round robin length
DT_RR, DT_2 = N_RR * DT, 2 * DT
PH_FAC, PH_GAIN = 0.9, 1.1        # phaser step / level trims: 24 kHz Euler all-pass stages vs the ngspice deck (sweep)


def lin(b, dst, const, terms):
    """dst = const + sum(k x reg): LD / MULI for the first term, MACI after"""
    first = True
    for reg, k in terms:
        if first:
            b.op("LD", reg); b.op("MULI", imm=q(k)); first = False
        else:
            b.op("MACI", reg, imm=q(k))
    if const:
        b.op("ADDI", imm=q(const))
    if dst:
        b.op("ST", dst)


def rc4(b, reg, tau):
    """acc = target: reg relaxes toward it, one round-robin step (4 samples)"""
    b.op("RC", reg, imm=q(1.0 - math.exp(-DT_RR / tau)))


def round_robin(b, blocks):
    """blocks[k] runs on samples k, k + n, ...: each block's cost counts once in the worst path"""
    n = len(blocks)
    b.op("LD", "_rr"); b.op("ADDI", imm=ONE); b.op("CMPI", imm=n * ONE); b.op("LDIF", imm=0); b.op("ST", "_rr")
    ends = []
    for k, f in enumerate(blocks):
        at = None
        if k < n - 1:
            b.op("LD", "_rr"); b.op("CMPI", imm=(k + 1) * ONE)
            at = b.skip_if(True)
        f()
        if k < n - 1:
            ends.append(b.jump())
            b.land(at)
    for e in ends:
        b.land(e)


def alternate(b, block_a, block_b):
    """block_a on even samples, block_b on odd ones"""
    b.op("LD", "_ph"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", "_ph")
    at = b.skip_if(True)
    block_a()
    at2 = b.jump()
    b.land(at)
    block_b()
    b.land(at2)


# ---------------------------------------------------------------- triggered op-amp VCAs (MAME DST_TVCA_OP_AMP, F2 =
# TRG0, r6 0): C (c1) charges from the 7417 pull-up through the diode / r5 1k toward vt1, discharges through r7 into
# the + input; out = max(c - VBE, 0) r4 / r7 - r4 (vP - VBE) / r1 - max(in - VBE, 0) r4 / r2, clamped 0 .. vP - VBE
def vca_env(b, name, trig, v1, r5, r7, c1):
    vt1 = (v1 - 0.6 - VBE) * r7 / (r5 + r7) + VBE
    b.kreg(name + "_kd", 1.0 - math.exp(-DT_RR / (r7 * c1)), "T")             # Game Audio: timing caps
    b.kreg(name + "_kc", 1.0 - math.exp(-DT_RR / (par(r5, r7) * c1)), "T")
    b.op("BIT", 0, b.src(*trig))
    b.op("LD", name + "_kd")
    b.op("LDF", name + "_kc")
    b.op("ST", name + "_k")
    b.op("LDI", imm=q(VBE))
    b.op("LDIF", imm=q(vt1))
    b.op("RCM", name + "_c", b.r(name + "_k"))


def vca_out(b, name, inp, r1, r2, r4, r7, vP=12.0):
    """inp = "nz" (the noise as max(noise - VBE, 0): 0 / 11.5) or a register (Norton input, clamped at VBE)"""
    vmax = vP - VBE
    assert r4 == r7
    if inp != "nz":
        b.op("LD", inp); b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("ST", name + "_i")
    b.op("LD", name + "_c")
    b.op("ADDI", imm=q(-VBE - r4 * vmax / r1))       # c below VBE: negative anyway (clamped below)
    if r4 == r2:
        b.op("SUB", inp if inp == "nz" else name + "_i")
    else:
        b.op("MACI", inp if inp == "nz" else name + "_i", imm=q(-r4 / r2))
    b.op("MAXI", imm=0)
    b.op("MINI", imm=q(vmax))
    b.op("ST", name)


def norton_1sht(b, out, trig, vmax):
    """Player shot one-shot E1 pins 2 / 3 / 4 (trigger C501 470p / R502 100k, feedback R504 2.2M): the 47 us trigger
    pulse (120 uA) always wins, so a rising trigger edge sets the output, a falling one clears it (-120 uA) and while
    the trigger is held the R504 feedback (5.2 uA) holds it until C500 at the - input wins (norton_1sht_c1)"""
    b.op("BIT", 0, b.src(*trig))
    b.op("LDI", imm=0)
    b.op("STNF", out)
    b.op("LD", out + "_t")
    b.op("CMPI", imm=q(0.5))
    b.op("NOT")
    b.op("FAND", 0, b.src(*trig))
    b.op("LDI", imm=q(vmax))
    b.op("STF", out)
    b.op("BIT", 0, b.src(*trig))
    b.op("LDI", imm=0)
    b.op("LDIF", imm=ONE)
    b.op("ST", out + "_t")


def norton_1sht_c1(b, out, r1, r3, r4, r5, c1, vmax):
    """C500 relaxes toward the R503 (from the output) / R501 (to the - input) divider, tau (R501 || R503) C500, in
    both directions (MAME: charge only; discharge through R503 alone); D501 clamps it to out + 0.6 V. The output
    drops once the R501 + R500 current reaches the R504 feedback: C500 >= VBE + R501 (vmax / R504 - vmax / R500)
    (3.28 V, 0.40 s from rest) (round robin)"""
    b.op("LD", out)
    b.op("MULI", imm=q(r3 / (r3 + r4)))
    b.op("ADDI", imm=q(VBE * r4 / (r3 + r4)))
    rc4(b, out + "_c1", par(r3, r4) * c1)
    b.op("LD", out)
    b.op("ADDI", imm=q(0.6))
    b.op("CMP", out + "_c1")
    b.op("STNF", out + "_c1")
    b.op("LD", out + "_c1")
    b.op("CMPI", imm=q(VBE + r3 * (vmax / r5 - vmax / r1)))
    b.op("LDI", imm=0)
    b.op("STF", out)


def crash_filter(b, out, src, dc, fs):
    """C2 pins 5 / 6 / 1: C300 .022u + R305 56k into the R308 56k / C302 .47u / R306 56k / C301 1n multiple-feedback
    low pass (Norton, + input R307 200k): 3rd order, zero at DC; peak 129 Hz, gain 6.2, Q 8.8 (exact AC; MAME: 130 Hz
    band pass d 1/8 x 6, whose skirts pass 1 kHz 15 dB louder). Bilinear at fs, prewarped at 129 Hz: a first order
    section (with the low pass DC gain) and a unity-DC-gain biquad. out = dc + filtered, clamped 0 .. 11.5"""
    import numpy as np
    Ca, R1, Cb, R2, R3, Cf = 22e-9, 56e3, 0.47e-6, 56e3, 56e3, 1e-9
    G = 1 / R2 + 1 / R3
    den = [Cf * R3 * Ca * Cb * R1, Cf * R3 * (Ca + Cb + G * Ca * R1), Cf * R3 * G + Ca * R1 / R2, 1 / R2]
    p = np.roots(den)
    pr = [x.real for x in p if abs(x.imag) < 1e-6][0]
    pc = [x for x in p if x.imag > 1e-6][0]
    k0 = -Ca / den[0]                                 # H = k0 s / ((s - pr)(s^2 - 2 Re(pc) s + |pc|^2))
    wp = 2 * math.pi * 129.4
    Kb = wp / math.tan(wp / (2 * fs))
    a0 = Kb - pr
    h_b0, h_b1, h_a1 = Kb / a0, -Kb / a0, -(Kb + pr) / a0
    c1, c0 = -2 * pc.real, abs(pc) ** 2
    d0 = Kb * Kb + c1 * Kb + c0
    l_a1, l_a2 = (2 * c0 - 2 * Kb * Kb) / d0, (Kb * Kb - c1 * Kb + c0) / d0
    gl = k0 / c0
    nb = (1 + l_a1 + l_a2) / 4
    b.load(src)                                       # transposed direct form II: x -> h -> y
    b.op("ST", out + "_x")
    b.op("MULI", imm=q(h_b0 * gl)); b.op("ADD", out + "_s0"); b.op("ST", out + "_h")
    b.op("LD", out + "_x"); b.op("MULI", imm=q(h_b1 * gl)); b.op("MACI", out + "_h", imm=q(-h_a1))
    b.op("ST", out + "_s0")
    b.op("LD", out + "_h"); b.op("MULI", imm=q(nb)); b.op("ST", out + "_hn")
    b.op("ADD", out + "_s1"); b.op("ST", out + "_y")
    b.op("LD", out + "_hn"); b.op("MULI", imm=q(2)); b.op("MACI", out + "_y", imm=q(-l_a1)); b.op("ADD", out + "_s2")
    b.op("ST", out + "_s1")
    b.op("LD", out + "_hn"); b.op("MACI", out + "_y", imm=q(-l_a2)); b.op("ST", out + "_s2")
    b.op("LD", out + "_y")
    b.op("ADDI", imm=q(dc))
    b.op("MAXI", imm=0)
    b.op("MINI", imm=q(11.5))
    b.op("ST", out)


def screech(b, out, dt):
    """Screech (P3 D0, 7417 + R44 1k to +12): D2 pin 4 gates the noise (R200 / R201 / R203 680k, R202 2.7M): 9.6 V
    while enabled and the noise bit is low (ngspice), else 0. D2 pins 5 / 9: two-integrator loop, R205 560k in,
    R206 / R209 100k bias, R207 2.7M || C200 3300p, R208 / R210 47k, C201 3300p: rings at 1.01 kHz, Q 57.
    Semi-implicit Euler at dt on the Norton outputs (linear input currents at VN), clamped 0 .. 11.5"""
    c200 = c201 = 3.3e-9
    b.op("LD", "nz")
    b.op("MULI", imm=q(-9.6 / 11.5))
    b.op("ADDI", imm=q(9.6))
    b.op("ST", out + "_g")
    b.op("BIT", 1, b.src(LATCH3, 0))
    b.op("LDI", imm=0)
    b.op("STF", out + "_g")
    k1, k2, k4 = dt / (c200 * 47 * K), dt / (c200 * 560 * K), dt / (c200 * 2.7 * M_)
    lin(b, None, -k1 * VN + k2 * VN - dt / c200 * (12 - VN) / (100 * K) + k4 * VN,
        [(out + "_5", 1 - k4), (out + "_9", k1), (out + "_g", -k2)])
    b.op("MAXI", imm=0); b.op("MINI", imm=q(11.5)); b.op("ST", out + "_5")
    k6 = dt / (c201 * 47 * K)
    lin(b, None, dt / c201 * (12 - VN) / (100 * K) + k6 * VN, [(out + "_5", -k6), (out + "_9", 1.0)])
    b.op("MAXI", imm=0); b.op("MINI", imm=q(11.5)); b.op("ST", out + "_9")
    v5 = VN + 47 * K * (12 - VN) / (100 * K)
    b.init(out + "_5", v5)
    b.init(out + "_9", VN + 47 * K * ((12 - VN) / (100 * K) + (v5 - VN) / (2.7 * M_) - VN / (560 * K)))


def spcenctr():
    b = Board("spcenctr")
    b.lfsr_noise("noise", 7515, 12.0, 6.0)        # 4006 / 4030 noise, clock breadboarded at 7515 Hz (MAME)
    b.op("LD", "noise"); b.op("MULI", imm=q(11.5 / 12)); b.op("ST", "nz")       # VCA noise input current x r

    # ---- round robin (12 kHz, 4 x dt)
    SL_LO, SL_HI, VC_LO, DV = 0.33, 2.37, 0.33, 0.35      # SN76477 (MAME's measured model)

    A_PH, B_PH = 1 + 10 / par(10, 13), 10 / 13      # phaser stage 1: R40 / (R38 || R50), R40 / R50

    def lfo_phaser():
        # wind LFO B0 10 / B0 4: flips at 7.462 / 1.490 V, +2.143 / -2.514 V/s, period 5.16 s (ngspice: C10 4.7u,
        # R27 1M, R28 510k, R29 1M, R30 82k, R31 150k); power-up C10 discharged (0.5 V)
        b.op("LD", "w_tri"); b.op("ADD", "w_tr"); b.op("ST", "w_tri")
        b.op("CMPI", imm=q(7.462)); b.op("LD", "w_dn"); b.op("STF", "w_tr")
        b.op("LD", "w_tri"); b.op("CMPI", imm=q(1.490) + 1); b.op("LD", "w_up"); b.op("STNF", "w_tr")
        # phaser 4016 duty: saw > LFO (B1 13 via R35): d2 = 0.9702 - 0.1112 tri (ngspice); all-pass step
        # al = d2 dt / RC at 24 kHz. Stages 2-4: out = c in + (1 - c) p (c = 2 al - 1); the R50 loop solved per
        # sample: out1 = inv (A p1 - in) - G E, inv = 1 / (1 + B c^3), G = B (1 - c) inv
        b.op("LD", "w_tri"); b.op("MULI", imm=q(-0.1112 * PH_FAC * DT_2 / (15.3 * K * 0.01 * U)))
        b.op("ADDI", imm=q(0.9702 * PH_FAC * DT_2 / (15.3 * K * 0.01 * U))); b.op("ST", "w_al")
        b.op("MULI", imm=q(2)); b.op("ADDI", imm=-ONE); b.op("ST", "w_c")
        b.op("MUL", "w_c"); b.op("MUL", "w_c"); b.op("MULI", imm=q(B_PH)); b.op("ADDI", imm=ONE); b.op("RCP")
        b.op("ST", "w_inv")
        b.op("LDI", imm=ONE); b.op("SUB", "w_c"); b.op("MUL", "w_inv"); b.op("MULI", imm=q(B_PH)); b.op("ST", "w_g")

    def dac_duty():
        # wind level target: C2 pins 8 / 9 / 13, R1-R4 from the 7417 outputs with 1k pull-ups, R52 100k, R5 6.2k:
        # -0.603 V + 0.737 / 1.305 / 2.657 / 5.314 V per bit (ngspice, incl. the input diodes and the low outputs)
        b.op("LDI", imm=q(-0.603)); b.op("ST", "w_lt")
        for k, w in enumerate((0.7366, 1.3053, 2.6571, 5.3143)):
            b.op("LDLM", 0, b.src(LATCH2, 0), imm=(1 << k) | ((24 - k) << 8))
            b.op("MULI", imm=q(w)); b.op("ADD", "w_lt"); b.op("ST", "w_lt")
        # noise filter switches: level > saw (B1 2): d = 0.1111 L + 0.0039 (ngspice, 43.5 us saw 0.01 .. 8.79 V);
        # loop step a = dt / C5 (1 / R25 + d / (R26 + 4016)): 15n, 150k, 33.3k (filter at 24 kHz)
        a0, a1 = DT_2 / (15e-9 * 150 * K), DT_2 / (15e-9 * 33.3 * K)
        b.op("LD", "w_l"); b.op("MULI", imm=q(0.1111)); b.op("ADDI", imm=q(0.0039))
        b.op("MAXI", imm=0); b.op("MINI", imm=ONE)
        b.op("MULI", imm=q(a1)); b.op("ADDI", imm=q(a0)); b.op("ST", "w_a")
        b.op("MULI", imm=-ONE); b.op("ST", "w_na")

    def envelopes():
        vca_env(b, "cr_v", (LATCH1, 2), 5.0, 1 * K, 470 * K, 2.2 * U)          # D300, R301, C304
        vca_env(b, "ex_v", (LATCH2, 4), 12.0, 1 * K, 680 * K, 2.2 * U)         # D401, R401, C400
        vca_env(b, "ps", (LATCH2, 5), 12.0, 1 * K, 560 * K, 1 * U)             # D500, R523, C506

    def ladder():
        # explosion: R405 1.8k / C401 .22u / R406 3.6k / C402 .22u ladder into R407 4.7k to the bus (MAME: two
        # separate RC filters + 10.1k); the output ramps to each new value over the next 4 samples (no 12 kHz steps)
        k1, k2, k3 = DT_RR / (0.22 * U * 1.8 * K), DT_RR / (0.22 * U * 3.6 * K), DT_RR / (0.22 * U * 4.7 * K)
        lin(b, "ex_1", 0, [("ex_1", 1 - k1 - k2), ("ex_v", k1), ("ex_2", k2)])
        lin(b, None, 0, [("ex_2", 1 - k2 - k3), ("ex_1", k2), ("bus", k3)])
        b.op("ST", "ex_2"); b.op("SUB", "ex_o"); b.op("MULI", imm=q(1.0 / N_RR)); b.op("ST", "ex_s")

    def slf():
        # SN76477 SLF (C701 1u, R705 150k)
        b.op("LD", "sn_s"); b.op("ADD", "sn_sr"); b.op("MINI", imm=q(SL_HI)); b.op("MAXI", imm=q(SL_LO))
        b.op("ST", "sn_s")
        b.op("CMPI", imm=q(SL_HI)); b.op("LD", "sn_dn")
        b.op("STF", "sn_sr")
        b.op("LD", "sn_s"); b.op("CMPI", imm=q(SL_LO) + 1)
        b.op("LD", "sn_up"); b.op("STNF", "sn_sr")

    def bonus_b():
        # bonus 556 B (R713 100k, R714 47k, C713 1u): 7.4 Hz gate, phase timer, held in reset with 556 A
        th, tl, t1 = 147 * K * U * math.log(2) / 0.01, 47 * K * U * math.log(2) / 0.01, 147 * K * U * math.log(3) / 0.01
        b.op("BIT", 0, b.src(LATCH3, 4))
        at = b.skip_if(True)
        b.op("LDI", imm=q(th - t1)); b.op("ST", "bn_pb")
        at2 = b.jump()
        b.land(at)
        b.op("LD", "bn_pb"); b.op("ADD", "bn_dtb"); b.op("CMPI", imm=q(th + tl))
        b.op("ADDI", imm=q(-(th + tl))); b.op("STF", "bn_pb"); b.op("ADDI", imm=q(th + tl)); b.op("STNF", "bn_pb")
        b.land(at2)
        b.op("LD", "bn_pb"); b.op("CMPI", imm=q(th)); b.op("LDI", imm=q(3.4)); b.op("LDIF", imm=0); b.op("ST", "bn_gb")

    def cap(name, src, r, c):                         # coupling caps (>= 2 ms) relax toward source - bus
        b.op("LD", src); b.op("SUB", "bus"); rc4(b, name + "_cc", r * c)

    def rr0():
        lfo_phaser()
        b.op("LD", "noise"); rc4(b, "ps_nc", (1 * M_ + 330 * K) * 0.1 * U)          # C503 (R507 + R509)
        b.op("LD", "sn"); rc4(b, "sn_c", 15 * K * 1 * U)                            # C987 / R706
        cap("cr", "cr", 3.6 * K, 1 * U)

    def rr1():
        envelopes()
        ladder()
        cap("sc", "sc_9", 22 * K, 0.1 * U)

    def rr2():
        slf()
        bonus_b()
        b.op("LD", "es_o_d0"); b.op("RCP"); b.op("ST", "es_o_r0")     # enemy shot VCO (vmod E1 moves over 60 ms+)
        cap("wn", "w_out", 27 * K, 10 * U)

    def rr3():
        dac_duty()
        norton_1sht_c1(b, "ps_1", 4.7 * M_, 1 * M_, 1 * M_, 2.2 * M_, 1 * U, 11.5)    # R500, R501, R503, R504, C500

    # rates scaled by the Game Audio oscillator-cap setting (others above / below are fixed constants)
    rng = SL_HI - SL_LO
    st = 0.64 * 2 * (SL_HI + DV - VC_LO) / (56 * K * 0.047 * U) * DT
    for reg, v in (("w_up", 2.143 * DT_RR), ("w_dn", -2.514 * DT_RR),
                   ("sn_up", rng / (0.5885 * 150 * K * U + 0.001300) * DT_RR),
                   ("sn_dn", -rng / (0.5413 * 150 * K * U + 0.001343) * DT_RR), ("sn_vp", st), ("sn_vn", -st),
                   ("bn_dta", DT / 0.01), ("bn_dtb", DT_RR / 0.01)):
        b.kreg(reg, v, "O", clamp=False)
    round_robin(b, [rr0, rr1, rr2, rr3])
    b.init("w_tri", 0.5)
    b.init("sn_s", SL_LO)
    b.init("sn_v", VC_LO - DV)
    b.op("LD", "w_lt"); b.op("RC", "w_l", imm=q(rc_exp(6.2 * K * 1 * U))); b.op("MAXI", imm=0); b.op("ST", "w_l")

    # ---- enemy ship shot (P3 D1, 7417 + R47 1k to +12): E1 R100 510k in, R102 2.2M || C100 0.1u, R101 22M bias
    # (schematic; MAME 2.2M); D1 4 / 5 VCO, D1 9 amp, D1 10 band pass
    b.op_amp_norton_cap("es_e", (LATCH3, 1), 510 * K, 22 * M_, 2.2 * M_, 0.1 * U, 12)
    b.op_amp_vco3_norton_exact("es_o", "es_e", 560 * K, 7.5 * K, 22 * K, 47 * K, 100 * K, 0.0022 * U, 12, rcp=True)
    body, b.code = b.code, b.kcode                   # charge-step reciprocal: 1 / kt (d0 <= 1.4 % of kt), per setting
    b.op("LD", "es_o_kt"); b.op("RCP"); b.op("ST", "es_o_r1")
    b.kcode, b.code = b.code, body
    b.op_amp_norton_sqw("es_d", "es_o", 11.5, "es_e", 100 * K, 100 * K, 2.7 * M_, 100 * K, 12)
    b.op_amp_filt_bp1m_norton("es_f", "es_d", 100 * K, 4.3 * M_, 2.2 * M_, 0.001 * U, 0.001 * U, 12, r2=10 * K)

    # ---- player shot (P2 D5, 7417 + 1k to +12): one-shot, E1 5 ramp (C502), D502, VCO C1 5 / 4, C1 9 amp, C1 10 VCA
    norton_1sht(b, "ps_1", (LATCH2, 5), 11.5)
    b.op_amp_norton_cap("ps_e", "ps_1", 10 * K, 1.5 * M_, 0, 0.22 * U, 12)
    b.op("LD", "ps_e"); b.op("ADDI", imm=q(-0.5)); b.op("MAXI", imm=0); b.op("ST", "ps_r")      # D502
    b.op("LD", "noise"); b.op("SUB", "ps_nc"); b.op("ST", "ps_n")                              # C503
    b.op_amp_vco3_norton_exact("ps_o", "ps_n", 1 / (1 / (1 * M_) + 1 / (330 * K)) + 1.5 * M_, 1 * M_, 560 * K,
                               2.7 * M_, 1 * M_, 330e-12, 12, vmod2="ps_r", r6=4.7 * M_, r7=3.3 * M_,
                               vm1_gain=330 / 1330, vm1_max=12, vm2_max=11.5)
    b.op_amp_norton_sqw("ps_a", "ps_o", 11.5, "ps_r", 560 * K, 470 * K, 2.7 * M_, 560 * K, 12)
    vca_out(b, "ps", "ps_a", 2.7 * M_, 560 * K, 560 * K, 560 * K)            # R522, R521, R524, R523

    # ---- crash (P1 D2, 7417 + 1k to +5) / explosion (P2 D4, 7417 + 1k to +12): noise VCAs C2 4 / D2 10
    vca_out(b, "cr_v", "nz", 2.7 * M_, 470 * K, 470 * K, 470 * K)            # R302, R300, R303, R301
    vca_out(b, "ex_v", "nz", 2.7 * M_, 680 * K, 680 * K, 680 * K)            # R402, R400, R403, R401
    b.op("LD", "ex_o"); b.op("ADD", "ex_s"); b.op("ST", "ex_o")                # ladder output ramp (round robin)

    # ---- bonus (P3 D4): 556 A (R710 1k, R711 27k, C710 .047u) AND 556 B AND enable (7411), TTL into R715 27k
    U_ = 0.01
    th, tl, t1 = 28 * K * 0.047 * U * math.log(2) / U_, 27 * K * 0.047 * U * math.log(2) / U_, \
        28 * K * 0.047 * U * math.log(3) / U_
    b.op("BIT", 0, b.src(LATCH3, 4))
    at = b.skip_if(True)
    b.op("LDI", imm=q(th - t1)); b.op("ST", "bn_pa"); b.op("LDI", imm=0); b.op("ST", "bn")
    at2 = b.jump()
    b.land(at)
    b.op("LD", "bn_pa"); b.op("ADD", "bn_dta"); b.op("CMPI", imm=q(th + tl))
    b.op("ADDI", imm=q(-(th + tl))); b.op("STF", "bn_pa"); b.op("ADDI", imm=q(th + tl)); b.op("STNF", "bn_pa")
    b.op("LD", "bn_pa"); b.op("CMPI", imm=q(th)); b.op("LD", "bn_gb"); b.op("LDIF", imm=0); b.op("ST", "bn")
    b.land(at2)

    # ---- 24 kHz blocks
    def block_a():
        # wind: C4 .001u / R19 1M noise into B0 6 (crfilter on the 24 kHz samples); two-integrator loop B0 5 / 9
        # (C5 = C8 = .015u, R21 330k damping) with R25 / R23 150k + switched R26 / R22 33k (averaged: conductance x
        # duty; checked against the switched ngspice deck); phaser LM324 A2 x 4 all-pass (10k / 10k, 15k + 4016 +
        # .01u), R50 13k from the last stage into the first (solved per sample: lfo_phaser)
        b.op("LD", "noise"); b.op("SUB", "w_c4"); b.op("ST", "w_hp")
        b.op("MULI", imm=q(1.0 - math.exp(-DT_2 / (1 * M_ * 0.001 * U)))); b.op("ADD", "w_c4"); b.op("ST", "w_c4")
        kb_, kn_ = DT_2 / (15e-9 * 330 * K), DT_2 / (15e-9 * 1 * M_)
        b.op("LD", "w_y"); b.op("MUL", "w_a"); b.op("MACI", "w_x", imm=q(1 - kb_)); b.op("MACI", "w_hp", imm=q(-kn_))
        b.op("ST", "w_x")
        b.op("MUL", "w_na"); b.op("ADD", "w_y"); b.op("ST", "w_y")
        b.op("LD", "w_p2"); b.op("MUL", "w_c"); b.op("ADD", "w_p3"); b.op("MUL", "w_c"); b.op("ADD", "w_p4")
        b.op("MUL", "w_g"); b.op("ST", "w_e")
        b.op("LD", "w_y"); b.op("RCM", "w_p1", b.r("w_al"))
        b.op("MULI", imm=q(A_PH)); b.op("SUB", "w_y"); b.op("MUL", "w_inv"); b.op("SUB", "w_e"); b.op("ST", "w_o1")
        for k in (2, 3, 4):
            src = f"w_o{k - 1}"
            b.op("LD", src); b.op("RCM", f"w_p{k}", b.r("w_al")); b.op("MULI", imm=q(2)); b.op("SUB", src)
            b.op("ST", f"w_o{k}")
        b.op("MULI", imm=q(PH_GAIN)); b.op("ST", "w_out")

    def block_b():
        screech(b, "sc", DT_2)
        crash_filter(b, "cr", "cr_v", VBE + 112 * K * (12 - VBE) / (200 * K), FS / 2)

    alternate(b, block_a, block_b)

    # ---- SN76477 VCO (C700 .047u, R704 56k; VCO top = SLF + 0.35 V, kept 0.35 V down), output pin 13 swing
    cp = 3.818 * (10 * K / (56 * K)) + 0.03
    hi, lo = min(cp * 1.00, 3.51 - 2.57), max(cp * -0.85, 0.715 - 2.57)          # gain tables at 4.44 V A/D
    b.op("LD", "sn_v"); b.op("ADD", "sn_vr"); b.op("CMP", "sn_s"); b.op("LDF", "sn_s")
    b.op("MAXI", imm=q(VC_LO - DV)); b.op("ST", "sn_v")
    b.op("CMP", "sn_s"); b.op("LD", "sn_vn"); b.op("STF", "sn_vr")
    b.op("LD", "sn_v"); b.op("CMPI", imm=q(VC_LO - DV) + 1); b.op("LD", "sn_vp"); b.op("STNF", "sn_vr")
    b.op("LD", "sn_vr"); b.op("CMPI", imm=0); b.op("LDI", imm=q(hi)); b.op("LDIF", imm=q(lo)); b.op("ST", "sn")
    b.op("BIT", 0, b.src(LATCH3, 5)); b.op("LDI", imm=0); b.op("STF", "sn")

    # ---- summing bus (C900 1u into D0 pin 6): branch currents x RREF
    w = {k: RREF / r for k, r in RS.items()}
    g116 = 1.8 / (1.8 + 15)                           # enemy shot: C104 0.1u, R116 1.8k to ground, R117 15k
    b.op("LD", "bus"); b.op("MULI", imm=q(-g116)); b.op("ADD", "es_f")
    b.op("RC", "es_c", imm=q(rc_exp(0.1 * U * par(1.8 * K, 15 * K))))
    b.op("LD", "es_f"); b.op("SUB", "es_c"); b.op("SUB", "bus"); b.op("ST", "es")
    b.init("es_c", (12 - VBE) / (4.3 * M_) * 2.2 * M_)
    b.op("LD", "ps"); b.op("SUB", "bus"); b.op("RC", "ps_cc", imm=q(rc_exp(15 * K * 0.001 * U)))   # C505 15 us
    b.op("LD", "ps"); b.op("SUB", "bus"); b.op("SUB", "ps_cc")
    b.op("MULI", imm=q(w["ps"]))
    b.op("MACI", "es", imm=q(w["es"]))
    for name, src in (("cr", "cr"), ("sc", "sc_9"), ("wn", "w_out")):
        b.op("MACI", src, imm=q(w[name])); b.op("MACI", name + "_cc", imm=q(-w[name]))
    b.op("MACI", "ex_o", imm=q(w["ex"]))
    b.op("MACI", "bn", imm=q(w["bn"]))
    b.op("MACI", "bus", imm=q(-(w["cr"] + w["sc"] + w["wn"] + w["ex"] + w["bn"])))
    b.op("ST", "_i")
    b.op("MULI", imm=q(DT / (1 * U * RREF))); b.op("ADD", "bus"); b.op("ST", "bus")
    b.op("LD", "sn"); b.op("SUB", "sn_c"); b.op("MULI", imm=q(RREF / (15 * K)))  # R706 / C987 to the summing input
    b.op("ADD", "_i")
    b.op("MULI", imm=q(OUT_A / RREF))
    b.op("BIT", 1, b.src(LATCH1, 0))                  # /GAME ON high (D0 low) mutes the summer (D900 / D901)
    b.op("LDIF", imm=0)
    b.op("OUT")
    return b.finish()
