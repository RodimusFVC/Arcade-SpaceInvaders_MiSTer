"""Phantom II (Midway 1979, game no. 652) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

From the schematic (Useful Information/schematics/Phantom_II_Parts_and_Operating_Manual_(Oct_1979).pdf, PDF p19 =
manual p17 "GAME BOARD SCHEMATIC", right half), settled with ngspice (verilator/snd/phantom2/ph_spice.py). MAME plays
two samples (shot, explosion) and leaves enemy shot, radar and jet out.
Latches (74174, 1k pull-ups to +5 V): S1 = port 5 (H5): D0 player shot, D1 enemy shot, D2 sound enable (P52 low ->
7404 -> diodes hold the summer low), D3 coin counter, D4 radar; S2 = port 6 (H4): D3 explosion, D4 explosion lamp,
D5-D7 jet (P65 82k / P66 39k / P67 22k).
LM3900 Norton stages: inputs at VBE = 0.5 V, output 0 .. VOH = 10.5 V; an output can sink only its 1.3 mA class-A
pull-down (explosion 560 ohm / 1 uF ladder and radar 0.1 uF / 1.5k loads).
Noise: 4006 18-stage register (taps 5 / 17, out bit 13) clocked by a 4030 RC oscillator (56k / 1 nF, 7.8 kHz by
ngspice); the engine's LFSR stands in for the 4006 (same clocking, different sequence).
"""
from dsnd import Board, LATCH1, LATCH2, q, ONE, FS, rc_exp
import numpy as np

K, M_, U = 1e3, 1e6, 1e-6
VBE, VOH, VP = 0.5, 10.5, 12.0
VNSAT = 0.11                                         # - input pulled down by the mirror while I+ > I- (saturated high)
IB = VP - VBE                                        # current into an input from a resistor to +12 V, x R
TTL_HI, TTL_LO = 5.0, 0.2 + 20 * 4.8 / 1020          # latch output with the 1k pull-up (high: light load)
NOISE_HZ = 7798.5
U_HZ = 15600.0 / 4                                   # mother board pin U "3840 Hz" = 2V video counter bit
OUT_SCALE = 1.0 / 16.0                               # speaker volts -> full scale (explosion boom peak ~14 V)


def ktau(tau):
    return rc_exp(tau)


def lin(b, dst, const, terms):
    """dst = const + sum(k x reg); terms [(reg, k)]"""
    r0, k0 = terms[0]
    b.op("LD", r0)
    if k0 != 1.0:
        b.op("MULI", imm=q(k0))
    for reg, k in terms[1:]:
        b.op("MACI", reg, imm=q(k))
    if const:
        b.op("ADDI", imm=q(const))
    if dst:
        b.op("ST", dst)


def clamp(b, dst, lo=0.0, hi=VOH):
    b.op("MAXI", imm=q(lo))
    b.op("MINI", imm=q(hi))
    if dst:
        b.op("ST", dst)


def oneshot(b, p, trig, r1, r3, r4, r5, c1):
    """Norton one-shot (C coupled trigger from a latch bit, c1 timing cap): out = VOH from the trigger's rising
    edge until c1's current through r3 (+ the r1 bias) beats the r5 feedback; c1 charges through r4 from the output
    toward the r3 / r4 divider, the diode empties it when the output drops. The trigger pulse (1 nF / 470 pF into
    100k: 0.1 ms) is reduced to the edge."""
    src, bit = trig
    b.op("LDLM", 0, b.src(src, 0), imm=(1 << bit) | ((24 - bit) << 8))      # latch bit as 0 / 1.0
    b.op("SUB", p + "pv"); b.op("ST", "_e")                                  # +1.0 on the rising edge
    b.op("ADD", p + "pv"); b.op("ST", p + "pv")
    # d = I- - I+ (uA): bias + c1 current - feedback - edge
    b.op("LD", p + "c1"); b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(1e6 / r3)); b.op("ADDI", imm=q(IB * 1e6 / r1))
    b.op("MACI", p + "o", imm=q(-(VOH - VBE) / VOH * 1e6 / r5))
    b.op("MACI", "_e", imm=q(-100.0))
    b.op("CMPI", imm=0)
    b.op("LDI", imm=q(VOH)); b.op("LDIF", imm=0); b.op("ST", p + "o")
    # c1: toward (out r3 + v(-) r4) / (r3 + r4), tau r3 || r4 x c1. Output high: the mirror sinks more than the
    # - input gets and pulls it to ~0.11 V (ngspice); low: the diode / r4 / r3 settle c1 near 0.2 V
    b.kreg(p + "kc", ktau(r3 * r4 / (r3 + r4) * c1), "T")
    b.op("NOT")                                       # F = output high
    b.op("LDI", imm=q(0.2))
    b.op("LDIF", imm=q((VOH * r3 + VNSAT * r4) / (r3 + r4)))
    b.op("RCM", p + "c1", b.r(p + "kc"))
    b.op("MINI", imm=q(0.6))
    b.op("STNF", p + "c1")


def integ(b, out, gate, i_on, i_off, c, g="T", lim=True):
    """Norton integrator, output toward VOH at (i_on) / toward 0 at (i_off) A (signed) while gate reg is high /
    low; clamped 0 .. VOH"""
    b.kreg(out + "_ku", i_on / (c * FS), g, clamp=False)
    b.kreg(out + "_kd", i_off / (c * FS), g, clamp=False)
    b.op("LD", gate); b.op("CMPI", imm=q(VBE))
    b.op("LD", out + "_kd"); b.op("LDF", out + "_ku")
    b.op("ADD", out)
    if lim:
        clamp(b, out)
    else:
        b.op("ST", out)


def noise(b):
    """4030 RC clock -> 4006: nz = 0 / 12 V"""
    b.kreg("nz_inc", NOISE_HZ / FS, "O", clamp=False)
    b.op("LD", "nz_ph"); b.op("ADD", "nz_inc"); b.op("ST", "nz_ph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "nz_ph")
    b.op("NOISE", 0, 0)
    b.op("LDI", imm=0); b.op("LDIF", imm=q(VP)); b.op("ST", "nz")
    b.land(at)


def jet(b):
    """K3: P65 / P66 / P67 into the + input, R2004 200k bias, R2005 150k || C2001 1 uF -> jlag; K3 noise mixer
    (680k / 680k / 680k, 2.7M bias); K3 multiple-feedback low pass (150k, 0.47 uF, 22k, 62k, 1 nF, + bias 200k):
    resonant 199 Hz, Q 8.6"""
    R2005, C2001 = 150 * K, 1 * U
    # target = VBE + R2005 (sum of bit currents - bias); a low bit (0.29 V) takes current back out of the input
    base = VBE - R2005 * IB / (200 * K)
    lo, ws = 0.0, []
    for r in (82 * K, 39 * K, 22 * K):
        vh = (TTL_HI * r + VBE * 1 * K) / (r + 1 * K)                     # the 1k pull-up loaded by r
        ih, il = (vh - VBE) / r, (TTL_LO - VBE) / r
        lo += il
        ws.append(R2005 * (ih - il))
    for n, (bit, w) in enumerate(zip((5, 6, 7), ws)):
        b.op("LDLM", 0, b.src(LATCH2, 0), imm=(1 << bit) | ((24 - bit) << 8))
        b.op("MULI", imm=q(w))
        if n:
            b.op("ADD", "_j")
        else:
            b.op("ADDI", imm=q(base + R2005 * lo))
        b.op("ST", "_j")
    b.kreg("jl_k", ktau(R2005 * C2001), "T")
    b.op("LD", "_j"); b.op("RCM", "jlag", b.r("jl_k"))
    clamp(b, "jlag")
    # mixer: VBE + max(0, jlag - VBE) - 680k IB / 2.7M - (nz - VBE)
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("ADDI", imm=q(2 * VBE - 680 * K * IB / (2.7 * M_)))
    b.op("SUB", "nz")
    clamp(b, "jmix")
    # MFB: C1 dVm = (u - Vm) / R1 + (y - Vm) / R2 - (Vm - VBE) / R3 ; C2 dy = I+ - (Vm - VBE) / R3, I+ = IB / R2014
    R1, R2, R3, C1, C2 = 150 * K, 22 * K, 62 * K, 0.47 * U, 1e-9
    ip = IB / (200 * K)
    A = np.array([[-(1 / R1 + 1 / R2 + 1 / R3) / C1, 1 / (R2 * C1)], [-1 / (R3 * C2), 0.0]])
    Bm = np.array([[1 / (R1 * C1), VBE / (R3 * C1)], [0.0, (ip + VBE / R3) / C2]])      # inputs u, 1
    Ad, Bd = mexp_zoh(A, Bm, 1.0 / FS)
    lin(b, "_t1", Bd[0][1], [("jm", Ad[0][0]), ("jflt", Ad[0][1]), ("jmix", Bd[0][0])])
    lin(b, "jflt", Bd[1][1], [("jm", Ad[1][0]), ("jflt", Ad[1][1]), ("jmix", Bd[1][0])])
    b.op("LD", "_t1"); b.op("ST", "jm")


def mexp_zoh(A, B, h):
    n, m = A.shape[0], B.shape[1]
    Mx = np.zeros((n + m, n + m))
    Mx[:n, :n], Mx[:n, n:] = A * h, B * h
    E = np.eye(n + m)
    term = np.eye(n + m)
    for k in range(1, 40):
        term = term @ Mx / k
        E = E + term
    return E[:n, :n], E[:n, n:]


def explosion(b):
    """P63 -> D701 -> C704 2.2 uF (charged through the 1k pull-up), R709 300k into K5 +; K5 noise mixer
    (R703 680k noise, R702 2.7M bias, R707 680k); output (R701 27k pull-up, 1.3 mA sink) -> R706 560 -> C701 1 uF
    -> R705 6.8k -> C700 1 uF -> R704 5.6k -> summing node"""
    b.kreg("e7_kc", ktau(1 * K * 2.2 * U), "T")
    b.kreg("e7_kd", ktau(300 * K * 2.2 * U), "T")
    b.op("BIT", 0, b.src(LATCH2, 3))
    b.op("LD", "e7_kd"); b.op("LDF", "e7_kc"); b.op("ST", "_k")
    b.op("LDI", imm=q(VBE)); b.op("LDIF", imm=q(TTL_HI - 0.6))
    b.op("RCM", "e704", b.r("_k"))
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(680 / 300))
    b.op("ADDI", imm=q(2 * VBE - 680 * K * IB / (2.7 * M_)))
    b.op("SUB", "nz")
    clamp(b, "_vt")
    # sink limit: the pin can't go below the level where (e701 - vo) / 560 + (12 - vo) / 27k = 1.3 mA (k, mA)
    g1, g2 = 1 / 0.56, 1 / 27.0
    lin(b, "_vl", (VP * g2 - 1.3) / (g1 + g2), [("e701", g1 / (g1 + g2))])
    b.op("LD", "_vt"); b.op("CMP", "_vl"); b.op("NOT"); b.op("LDF", "_vl"); b.op("ST", "e_vo")


def radar(b):
    """P54 edge -> L4 one-shot (R3002 4.7M, R3001 1M, R3005 1M, R3004 2.2M, C3001 0.22 uF); L4 mixer: one-shot
    (R3006 680k) - U square (R3007 300k) - bias (R3008 2.7M), R3010 680k; output (R3009 27k pull-up, 1.3 mA sink)
    -> C3003 0.1 uF -> R3012 1.5k to ground, R3011 100k to the summing node"""
    oneshot(b, "ro_", (LATCH1, 4), 4.7 * M_, 1 * M_, 1 * M_, 2.2 * M_, 0.22 * U)
    b.squarewfix("u_ph", U_HZ)
    kr = 680 / 300
    b.op("LD", "u_ph"); b.op("CMPI", imm=q(0.5))
    b.op("LDI", imm=q(-kr * (TTL_LO - VBE))); b.op("LDIF", imm=q(-kr * (3.4 - VBE)))
    b.op("MACI", "ro_o", imm=q((VOH - VBE) / VOH))
    b.op("ADDI", imm=q(VBE - 680 * K * IB / (2.7 * M_)))
    clamp(b, "_vt")
    # C3003 + 1.5k (k, mA): voltage drive i = (vt - vc) / 1.5 unless the pin would sink more than 1.3 mA
    b.op("SUB", "r_vc"); b.op("MULI", imm=q(1 / 1.5)); b.op("ST", "_i")
    b.op("LD", "_vt"); b.op("MULI", imm=q(1 / 27.0)); b.op("ADD", "_i")
    b.op("CMPI", imm=q(VP / 27.0 - 1.3))                # i_op = i - (12 - vt) / 27 >= -1.3 mA
    lin(b, None, (-1.3 + VP / 27.0) / (1 + 1.5 / 27.0), [("r_vc", -1 / 27.0 / (1 + 1.5 / 27.0))])
    b.op("STNF", "_i")
    b.op("LD", "_i"); b.op("MULI", imm=q(1.5)); b.op("ST", "r3012")
    b.kreg("r_kc", ktau(1.5 * K * 0.1 * U), "F")
    b.op("MUL", "r_kc"); b.op("ADD", "r_vc"); b.op("ST", "r_vc")


def enemy(b):
    """N4: free-running sweep (integrator R37 1M / C2 1 uF, Schmitt R38 100k / R39 1M / R40 120k, R41 10k) ->
    triangle VCO (R42 1M -, R43 470k +, C17 330 pF, 4016 shorts + while the R44 / R45 / R46 Schmitt is high);
    P51 edge -> N3 one-shot (C18 0.1 uF) -> integrator (R53 10k, R137 1M, C19 2.2 uF); N3 mixer: envelope (R54)
    - triangle (R55 470k) - bias (R56 2.7M), R57 680k -> R59 200k"""
    # sweep integrator + Schmitt (new state high iff sweep < the old state's threshold)
    integ(b, "swp", "sch", (VOH - VBE) / (10 * K) - IB / (1 * M_), -IB / (1 * M_), 1 * U, lim=False)   # 1.23 .. 10
    th_hi = VBE + 100 * K * (IB / M_ + (VOH - VBE) / (120 * K))
    th_lo = VBE + 100 * K * (IB / M_ - VBE / (120 * K))
    b.op("LD", "sch"); b.op("CMPI", imm=q(VBE))
    b.op("LDI", imm=q(th_lo)); b.op("LDIF", imm=q(th_hi)); b.op("ST", "_t")
    b.op("LD", "swp"); b.op("CMP", "_t")
    b.op("LDI", imm=q(VOH)); b.op("LDIF", imm=0); b.op("ST", "sch")
    # VCO: down (switch on) (swp - VBE) / R42 / C, up (swp - VBE) (1 / R43 - 1 / R42) / C
    ratio = (1 * M_) / (470 * K) - 1.0                   # up / down
    t_hi = VBE + 680 * K * (IB / M_ + VBE / M_)          # Schmitt R44 680k, R45 1M bias, R46 1M feedback
    t_lo = VBE + 680 * K * (IB / M_ - (VOH - VBE) / M_)
    b.kreg("et_k", 1.0 / (1 * M_ * 330e-12 * FS), "O", clamp=False)
    b.op("LD", "swp"); b.op("ADDI", imm=q(-VBE)); b.op("MUL", "et_k"); b.op("ST", "_rd")
    b.op("LD", "vsq"); b.op("CMPI", imm=q(VBE))
    j_dn = b.skip_if(True)
    b.op("LD", "_rd"); b.op("MULI", imm=q(ratio)); b.op("ADD", "tri"); b.op("ST", "tri")
    b.op("CMPI", imm=q(t_hi))
    j1 = b.skip_if(False)
    b.op("ADDI", imm=q(-t_hi)); b.op("MULI", imm=q(-1 / ratio)); b.op("ADDI", imm=q(t_hi)); b.op("ST", "tri")
    b.op("LDI", imm=q(VOH)); b.op("ST", "vsq")
    b.land(j1)
    j_end = b.jump()
    b.land(j_dn)
    b.op("LD", "tri"); b.op("SUB", "_rd"); b.op("ST", "tri")
    b.op("CMPI", imm=q(t_lo))
    j2 = b.skip_if(True)
    b.op("ADDI", imm=q(-t_lo)); b.op("MULI", imm=q(-ratio)); b.op("ADDI", imm=q(t_lo)); b.op("ST", "tri")
    b.op("LDI", imm=0); b.op("ST", "vsq")
    b.land(j2)
    b.land(j_end)
    # shot envelope
    oneshot(b, "eo_", (LATCH1, 1), 4.7 * M_, 1 * M_, 1 * M_, 2.2 * M_, 0.1 * U)
    integ(b, "env", "eo_o", (VOH - VBE) / (10 * K) - IB / (1 * M_), -IB / (1 * M_), 2.2 * U)
    # mixer N3: VBE + max(0, env - VBE) - 680k IB / 2.7M - 680 / 470 (tri - VBE)
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("ADDI", imm=q(VBE - 680 * K * IB / (2.7 * M_) + 680 / 470 * VBE))
    b.op("MACI", "tri", imm=q(-680 / 470))
    clamp(b, "eout")


def player(b):
    """P50 edge -> L4 one-shot (R4001 4.7M, R4003 1M, R4002 1M, R4004 2.2M, C4040 0.047 uF) -> M5 integrator
    (R4035 10k, R4036 1.5M, C4013 0.22 uF) -> D4001 -> R4014 4.7M (VCO) / R4015 470k (mixer); M5 VCO: C4010 330 pF,
    R4013 3.3M bias, R4012 1.5M noise (C4002 0.1 uF, R4010 1M, R4011 330k), R4016 1M from the R4017 560k / R4018 1M
    / R4019 2.2M Schmitt; M5 mixer (R4022 560k square, R4020 2.7M, R4021 560k) -> K3 gate (C4014 0.1 uF charged by
    P50 through D4003, R4000 330k: read as 330k, 330 ohm would pin K3 high) (R4023 / R4025 / R4026) -> C4011 2.2 nF
    -> R4028 220k"""
    oneshot(b, "po_", (LATCH1, 0), 4.7 * M_, 1 * M_, 1 * M_, 2.2 * M_, 0.047 * U)
    integ(b, "pint", "po_o", (VOH - VBE) / (10 * K) - IB / (1.5 * M_), -IB / (1.5 * M_), 0.22 * U)
    VD = 1.752 * 0.025852 * np.log(20e-6 / 2.52e-9)    # D4001 at ~20 uA (1N4148 model)
    b.op("ADDI", imm=q(-VD - VBE)); b.op("MAXI", imm=0); b.op("ST", "_vb")
    # noise network: p1 = nz - qc; p2 = a p1 + bb (node R4010 / R4011 / R4012); qc += (p1 - p2) / 1M / (C FS)
    gsum = 1 / M_ + 1 / (330 * K) + 1 / (1.5 * M_)
    a, bb = (1 / M_) / gsum, (VBE / (1.5 * M_)) / gsum
    kq = 1.0 / (1 * M_ * 0.1 * U * FS)
    b.op("LD", "nz"); b.op("SUB", "qc"); b.op("ST", "_p1")
    b.op("MULI", imm=q(kq * (1 - a))); b.op("ADDI", imm=q(-kq * bb)); b.op("ADD", "qc"); b.op("ST", "qc")
    # VCO rates (V per sample): down = I- / C, up = (VOH - VBE) / R4016 / C - down
    C = 330e-12
    for reg, k in (("pv_k1", 1 / (4.7 * M_) / (C * FS)), ("pv_k2", a / (1.5 * M_) / (C * FS)),
                   ("pv_k0", (IB / (3.3 * M_) + (bb - VBE) / (1.5 * M_)) / (C * FS)),
                   ("pv_kt", (VOH - VBE) / (1 * M_) / (C * FS))):
        b.kreg(reg, k, "O", clamp=False)
    b.op("LD", "_vb"); b.op("MUL", "pv_k1"); b.op("ST", "_t")
    b.op("LD", "_p1"); b.op("MUL", "pv_k2"); b.op("ADD", "_t"); b.op("ADD", "pv_k0"); b.op("ST", "_rd")
    b.op("LD", "pv_kt"); b.op("SUB", "_rd"); b.op("ST", "_ru")
    t_hi = VBE + 560 * K * (IB / M_ + (VOH - VBE) / (2.2 * M_))
    t_lo = VBE + 560 * K * (IB / M_ - VBE / (2.2 * M_))
    b.op("LD", "psq"); b.op("CMPI", imm=q(VBE))
    j_dn = b.skip_if(False)
    # square high: + input fed, ramps up
    b.op("LD", "ptri"); b.op("ADD", "_ru"); b.op("ST", "ptri")
    b.op("CMPI", imm=q(t_hi))
    j1 = b.skip_if(False)
    b.op("ADDI", imm=q(-t_hi)); b.op("ST", "_x")       # overshoot; back down for (x / ru) of a sample
    b.op("LD", "_ru"); b.op("RCP"); b.op("MUL", "_rd"); b.op("MUL", "_x")
    b.op("MULI", imm=-ONE); b.op("ADDI", imm=q(t_hi)); b.op("ST", "ptri")
    b.op("LDI", imm=0); b.op("ST", "psq")
    b.land(j1)
    j_end = b.jump()
    b.land(j_dn)
    b.op("LD", "ptri"); b.op("SUB", "_rd"); b.op("ST", "ptri")
    b.op("CMPI", imm=q(t_lo))
    j2 = b.skip_if(True)
    b.op("ADDI", imm=q(-t_lo)); b.op("ST", "_x")
    b.op("LD", "_rd"); b.op("RCP"); b.op("MUL", "_ru"); b.op("MUL", "_x")
    b.op("MULI", imm=-ONE); b.op("ADDI", imm=q(t_lo)); b.op("ST", "ptri")
    b.op("LDI", imm=q(VOH)); b.op("ST", "psq")
    b.land(j2)
    b.land(j_end)
    # mixer M5: VBE + 560 / 470 vb - 560k IB / 2.7M - (psq - VBE)
    lin(b, None, 2 * VBE - 560 * K * IB / (2.7 * M_), [("_vb", 560 / 470), ("psq", -1.0)])
    clamp(b, "pmix")
    # K3 gate: C4014 charged by P50 (1k pull-up, 0.1 ms), else discharged through R4000 330k into the + input
    b.kreg("pg_kd", ktau(330 * K * 0.1 * U), "T")
    b.op("BIT", 0, b.src(LATCH1, 0))
    b.op("LDI", imm=q(VBE)); b.op("RCM", "c4014", b.r("pg_kd"))
    b.op("LDIF", imm=q(TTL_HI - 0.6)); b.op("ST", "c4014")
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(560 / 330))
    b.op("ADDI", imm=q(2 * VBE - 560 * K * IB / (2.7 * M_)))
    b.op("SUB", "pmix")
    clamp(b, "pout")


def summer(b):
    """Summing node X: explosion ladder (e701, e700), jet (C2004 1 uF + R2015 62k), enemy (R59 200k), radar
    (R3011 100k from R3012), player (C4011 2.2 nF + R4028 220k); X -> C1102 1 uF -> L2 (R1105 220k + bias,
    R1101 100k || R22 50k pot at 50 % + C1101 1 uF). Explicit node updates (every node has a large cap). k, mA."""
    b.op("LD", "e700"); b.op("SUB", "X"); b.op("ST", "_i3")
    b.op("LD", "jflt"); b.op("SUB", "qj"); b.op("SUB", "X"); b.op("ST", "_i4")
    b.op("LD", "pout"); b.op("SUB", "qp"); b.op("SUB", "X"); b.op("ST", "_i5")
    lin(b, "_is", 0.0, [("_i3", 1 / 5.6), ("_i4", 1 / 62.0), ("_i5", 1 / 220.0), ("eout", 1 / 200.0),
                        ("r3012", 1 / 100.0), ("X", -(1 / 200.0 + 1 / 100.0))])
    kc = 1e-3 / (1 * U * FS)                            # V per mA-sample on 1 uF
    b.op("MULI", imm=q(kc)); b.op("ADD", "X"); b.op("ST", "X")
    k1, k2, k3 = kc / 0.56, kc / 6.8, kc / 5.6
    lin(b, "_t", 0.0, [("e701", 1 - k1 - k2), ("e_vo", k1), ("e700", k2)])
    lin(b, "e700", 0.0, [("e700", 1 - k2), ("e701", k2), ("_i3", -k3)])
    b.op("LD", "_t"); b.op("ST", "e701")
    lin(b, None, 0.0, [("_i4", kc / 62.0)]); b.op("ADD", "qj"); b.op("ST", "qj")
    lin(b, None, 0.0, [("_i5", 1e-3 / (2.2e-9 * FS) / 220.0)]); b.op("ADD", "qp"); b.op("ST", "qp")
    # L2: - input at VBE takes I+ = IB / 220k; s1 = VBE + (I+ - i_C1102 + vc22 / R22) / (1 / R1101 + 1 / R22)
    R22 = 25.0
    gf = 1 / 100.0 + 1 / R22
    lin(b, None, VBE + IB / 220.0 / gf, [("vc22", 1 / R22 / gf), ("_is", -1 / gf)])
    clamp(b, "s1")
    b.op("BIT", 1, b.src(LATCH1, 2))                    # P52 low: 7404 / D1100 / R1102 pull the input up: s1 = 0
    b.op("LDI", imm=0); b.op("STF", "s1")
    b.op("LD", "s1"); b.op("ADDI", imm=q(-VBE)); b.op("SUB", "vc22")
    b.op("MULI", imm=q(1.0 / (R22 * K * 1 * U * FS))); b.op("ADD", "vc22"); b.op("ST", "vc22")
    # speaker (bridge LM377): R1 = 2 x HP(HP(s1)) (C1100 / R1100, C1105 / R1111), N1 = -2 x HP(s1) (C1104 / R1110),
    # all 16 Hz: taken as 4 x HP(s1)
    b.crfilter("h1", "s1", 100 * K, 0.1 * U)
    b.op("LD", "h1"); b.op("MULI", imm=q(4 * OUT_SCALE)); b.op("OUT")


# power-up operating point; H5 resets with P52 low (board muted: s1 held at 0)
DC = {"jflt": 5.926, "jm": 4.065, "qj": 5.926, "vc22": -VBE, "qc": 5.91,
      "e704": VBE, "c4014": VBE, "ro_c1": 0.2, "eo_c1": 0.2, "po_c1": 0.2, "tri": 5.0, "ptri": 8.0}


def phantom2():
    b = Board("phantom2")
    for reg, v in DC.items():
        b.init(reg, v)
    noise(b)
    jet(b)
    explosion(b)
    radar(b)
    enemy(b)
    player(b)
    summer(b)
    return b.finish()


if __name__ == "__main__":
    import sys
    p = phantom2()
    out = sys.argv[1] if len(sys.argv) > 1 else "phantom2.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"phantom2: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
