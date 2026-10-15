"""Sea Wolf (Midway 1976) sound board P.C. 596-909 for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Traced from the Kush N' Stuff Sea Wolf service manual (PDF p23 sound schematic; theory p22-24, scope photos p26-30).
Reference = verilator/snd/seawolf/sw_spice.py (ngspice deck of the whole board). MAME plays samples only; the samples
are measurement targets, never played.

Five sounds, port 5 = S1 (LATCH1), level triggers (bit set = 7406 output low = sound input active):
  D0 ship hit, D1 torpedo, D2 dive, D3 sonar, D4 mine hit (D5 coin counter).
Circuits (2N3394 NPN: Ebers-Moll IS 1e-14, BF 80 (spec hFE 55-110), VT at 300.15 K; units V, mA, kOhm):
  mine / ship: 2N4403 switch charges C1 22u / C2 33u via 47 ohm; Q2 / Q4 amplify the Q27 zener noise through a
    two-stage RC low pass; gain follows the supply cap (Gun Fight's generator: per-transistor Thevenin + one Newton
    step on Vbe with EXP / RCP), run at 12 kHz in alternate slots.
  torpedo: Q8 b-e breakdown noise -> Q7 (emitter 56k || .015u: rising response) -> .01u -> Q6 (collector feedback
    330k / 47k, emitter 100 ohm) on a 22u supply charged by Q5 via 470 ohm.
  sonar, dive: see their blocks.
  summing: Q29 (150k / 30k, 1k || 50u, 5.1k), 0.1u -> 30k -> 1n + 47k master pot (50 %) -> LM354 (gain ignored).
"""
import math
from dsnd import Board, LATCH1, q, ONE

FS = 48000.0
VT = 1.380649e-23 * 300.15 / 1.602176634e-19
IS, BF, NF = 1e-14, 80.0, 1.0
AF = BF / (1 + BF)
NVT = NF * VT
K1 = 1.0 / (NVT * math.log(2.0))                     # Ie (mA) = 2^(K1 Vbe + K0)
K0 = math.log2(IS / AF * 1e3)
VSAT = 0.1
V16 = 16.0
NEVER = -(1 << 31)                                   # CMPI NEVER: F = 1 (unconditional skip)

# noise generator Q27 (= Gun Fight's): collector DC and open-circuit noise (ngspice: 2 mV RMS Gaussian at the zener,
# 0.117 V RMS at the unloaded collector, 0.40 V p-p on a scope = TP35 "400 mV range"); Thevenin 6.8k || 6.8k seen
# by one generator with the other noise loads (the other generator's 20k + 0.1u, dive 220k, sonar tone pot 250k)
X0 = 9.6283
R_NZ = 1.0 / (1 / 3.4e3 + 1 / 20e3 + 1 / 220e3 + 1 / 250e3)
SIG_NZ = 0.117 * R_NZ / 3.4e3
# generator noise at 12 kHz: the 48 kHz density (half amplitude), x 1.41 = full-board ngspice (2026-10-14): the Q27
# node's noise density in the 100-300 Hz explosion band is ~1.6x its wideband level (zener feedback + the loads)
SIG_GEN = SIG_NZ / 2 * 1.41

# generator: supply C, mid filter C (after 20k), base C, switch bit
GEN = {"mine": (22e-6, 0.033e-6, 0.047e-6, 4), "ship": (33e-6, 0.047e-6, 0.1e-6, 0)}
SLOT = 4                                             # generators run every 4th sample (12 kHz), one per slot
# Q29 inputs: (name, series R, coupling C); mine / ship drive from their pot wiper (node w), the others from the
# Thevenin of their output network (u, r_th)
R_TH_T = 12.5e3                                                     # torpedo pot wiper (pot top node solved)
R_TH_N = 1.0 / (1 / 5e3 + 1 / 105e3)                                # sonar / dive: 100k into a 10k pot at 50 %
BR = {"mine": (30e3, 0.1e-6), "ship": (30e3, 0.1e-6), "torp": (22e3 + R_TH_T, 0.01e-6),
      "sonar": (22e3 + R_TH_N, 0.1e-6), "dive": (22e3 + R_TH_N, 1e-6)}


def inv(a):
    n = len(a)
    m = [[float(x) for x in r] + [1.0 if i == j else 0.0 for j in range(n)] for i, r in enumerate(a)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        m[i], m[p] = m[p], m[i]
        d = m[i][i]
        m[i] = [x / d for x in m[i]]
        for r in range(n):
            if r != i:
                f = m[r][i]
                m[r] = [x - f * y for x, y in zip(m[r], m[i])]
    return [r[n:] for r in m]


def lin(b, dst, const, terms):
    """dst = const + sum(k x reg)"""
    if not const and terms and terms[0][1] == 1.0:
        b.op("LD", terms[0][0])
        terms = terms[1:]
    else:
        b.op("LDI", imm=q(const))
    for reg, k in terms:
        b.op("MACI", reg, imm=q(k))
    if dst:
        b.op("ST", dst)


def jump(b):
    """unconditional forward skip (F forced to 1; tools/dsnd.py worst_path() takes it as always taken)"""
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def newton(b, p, rt):
    """one Newton step on Vbe = _dv - rt Ie(Vbe) from the previous sample's Vbe (rt kOhm), written as
    NVT (dv - v - rt Ie) / (NVT + rt Ie); acc = _ie (mA) = (_dv - Vbe) / rt, clamped to 0 .. min(100 mA, 64 V / rt)"""
    b.op("LD", p + "vbe"); b.op("MULI", imm=q(K1)); b.op("ADDI", imm=q(K0)); b.op("EXP")
    if rt * 127.0 > 100.0:
        b.op("MINI", imm=q(100.0 / rt))
    b.op("MULI", imm=q(rt)); b.op("ST", "_re")
    b.op("ADDI", imm=q(NVT)); b.op("RCP"); b.op("ST", "_rd")
    b.op("LD", "_dv"); b.op("SUB", p + "vbe"); b.op("SUB", "_re"); b.op("MULI", imm=q(NVT)); b.op("MUL", "_rd")
    b.op("ADD", p + "vbe"); b.op("ST", p + "vbe")
    b.op("LD", "_dv"); b.op("SUB", p + "vbe"); b.op("MAXI", imm=0); b.op("MINI", imm=q(min(100.0 * rt, 64.0)))
    k = 1.0 / rt
    while k >= 127.0:
        b.op("MULI", imm=q(1.0 / 8)); k *= 8.0
    b.op("MULI", imm=q(k)); b.op("ST", "_ie")


def bjt(b, p, zb, ze, zcc, ve0, vb="_vb0", vev="_ve0", vc="_vc0"):
    """transistor p against Thevenin _vb0 / zb, _ve0 (if ve0) / ze, _vc0 / zcc (kOhm): sets p+e, p+c, p+sat, _ic, _ib
    (Gun Fight's solver: active / saturated modes)"""
    b.op("LD", p + "sat"); b.op("CMPI", imm=q(0.5))
    j_sat = b.skip_if(True)
    b.op("LD", vb)
    if ve0:
        b.op("SUB", vev)
    b.op("ST", "_dv")
    newton(b, p, zb / (BF + 1) + ze)
    b.op("MULI", imm=q(AF)); b.op("ST", "_ic"); b.op("MULI", imm=q(-zcc)); b.op("ADD", vc); b.op("ST", p + "c")
    lin(b, p + "e", 0.0, ([(vev, 1.0)] if ve0 else []) + [("_ie", ze)])
    b.op("LD", p + "c"); b.op("SUB", p + "e"); b.op("CMPI", imm=q(VSAT))
    j = b.skip_if(True)
    b.op("LDI", imm=ONE); b.op("ST", p + "sat")
    b.op("LD", p + "e"); b.op("ADDI", imm=q(VSAT)); b.op("ST", p + "c")
    b.op("LD", vc); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("ST", "_ic")
    b.land(j)
    b.op("LD", "_ie"); b.op("SUB", "_ic"); b.op("ST", "_ib")
    j_end = jump(b)
    b.land(j_sat)
    lin(b, "_dv", -zb * VSAT / zcc, [(vb, 1.0), (vc, zb / zcc)] + ([(vev, -(1 + zb / zcc))] if ve0 else []))
    newton(b, p, zb + ze + zb * ze / zcc)
    b.op("MULI", imm=q(ze))
    if ve0:
        b.op("ADD", vev)
    b.op("ST", p + "e"); b.op("ADDI", imm=q(VSAT)); b.op("ST", p + "c")
    b.op("LD", vc); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("MAXI", imm=0); b.op("ST", "_ic")
    b.op("MULI", imm=q(-zcc)); b.op("ADD", vc); b.op("ST", p + "c")
    b.op("LD", "_ie"); b.op("SUB", "_ic"); b.op("ST", "_ib")
    b.op("LD", "_ic"); b.op("MULI", imm=q(1.0 / BF)); b.op("SUB", "_ib"); b.op("CMPI", imm=1)
    b.op("LDI", imm=0); b.op("STF", p + "sat")
    b.land(j_end)


def bjt_active(b, p, zb, ze, zcc, ve0, cmin=None, vb="_vb0", vev="_ve0", vc="_vc0"):
    """active-mode-only transistor (never saturates in this circuit): sets p+c, p+e, _ic, _ib; cmin clamps p+c"""
    b.op("LD", vb)
    if ve0:
        b.op("SUB", vev)
    b.op("ST", "_dv")
    newton(b, p, zb / (BF + 1) + ze)
    b.op("MULI", imm=q(AF)); b.op("ST", "_ic"); b.op("MULI", imm=q(-zcc)); b.op("ADD", vc)
    if cmin is not None:                             # collector floor: the current the collector network can give
        b.op("MAXI", imm=q(cmin)); b.op("ST", p + "c")
        b.op("LD", vc); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("ST", "_ic")
        b.op("LD", p + "c")
    b.op("ST", p + "c")
    lin(b, p + "e", 0.0, ([(vev, 1.0)] if ve0 else []) + [("_ie", ze)])
    b.op("LD", "_ie"); b.op("MULI", imm=q(1.0 / (BF + 1))); b.op("ST", "_ib")


def bjt_lin(b, p, zb, ze, zcc, ie0, vbe0, cmin):
    """transistor linearised at its DC point (ie0 mA, vbe0): Vbe = vbe0 + re0 (Ie - ie0) against _vb0 - _ve0 - rt Ie;
    Ie clamped >= 0, collector floor cmin (the current the collector network can then give); sets p+c, p+e, _ic, _ib"""
    rt = zb / (BF + 1) + ze
    re0 = NVT / ie0
    b.op("LD", "_vb0"); b.op("SUB", "_ve0"); b.op("ADDI", imm=q(re0 * ie0 - vbe0)); b.op("MULI", imm=q(1.0 / (rt + re0)))
    b.op("MAXI", imm=0); b.op("ST", "_ie")
    b.op("MULI", imm=q(AF)); b.op("ST", "_ic"); b.op("MULI", imm=q(-zcc)); b.op("ADD", "_vc0")
    b.op("MAXI", imm=q(cmin)); b.op("ST", p + "c")
    b.op("LD", "_vc0"); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("ST", "_ic")
    lin(b, p + "e", 0.0, [("_ve0", 1.0), ("_ie", ze)])
    b.op("LD", "_ie"); b.op("MULI", imm=q(1.0 / (BF + 1))); b.op("ST", "_ib")


# ---------------------------------------------------------------------------------------------------- mine / ship
def gen(b, n, part):
    """explosion generator n at FS / SLOT (Gun Fight's gen() with Sea Wolf's switch, noise source and values),
    split over two samples: part A = supply, noise filter and Thevenin networks, part B = transistor and outputs"""
    csup, cnf, cb, bit = GEN[n]
    p = n + "_"
    H = SLOT / FS
    if part == "B":                                  # build the A code into a scratch board (same constants)
        a = Board("scratch"); a.regs = b.regs; gen(a, n, "A"); real, b = b, a
    # supply cap vs the bias divider and the collector resistor (previous b, c); switch on: 47 ohm from 15.97 V
    g0 = csup / H
    y = g0 + 1 / 330e3 + 1 / 5.1e3
    lin(b, p + "s", 0.0, [(p + "s", g0 / y), (p + "b", 1 / 330e3 / y), (p + "c", 1 / 5.1e3 / y)])
    b.op("BIT", 0, b.src(LATCH1, bit))
    j = b.skip_if(False)
    b.op("LDI", imm=q(15.97)); b.op("RC", p + "s", imm=q((1 / 47.0) / (y + 1 / 47.0)))
    b.land(j)
    # noise: x (Thevenin R_NZ) -> 0.1 uF (qcp) -> 20k -> nf (C_nf) -> 30k -> b
    gcp, gnf = 0.1e-6 / H, cnf / H
    ai = inv([[1 / R_NZ + gcp, -gcp, 0.0], [-gcp, gcp + 1 / 20e3, -1 / 20e3],
              [0.0, -1 / 20e3, 1 / 20e3 + gnf + 1 / 30e3]])

    def row(r):                                      # coefficients on x, qcp, nf, b (previous sample)
        return [r[0] / R_NZ, gcp * (r[0] - r[1]), r[2] * gnf, r[2] / 30e3]
    rn, rq = row(ai[2]), row([ai[0][k] - ai[1][k] for k in range(3)])
    rq = [rq[k] - rq[2] * rn[k] / rn[2] for k in range(2)] + [rq[2] / rn[2], rq[3] - rq[2] * rn[3] / rn[2]]
    for reg, r in ((p + "nf", rn), (p + "qcp", rq)):
        lin(b, reg, r[0] * X0, [("_gx", r[0]), (p + "qcp", r[1]), (p + "nf", r[2]), (p + "b", r[3])])
    # base Thevenin: C_b, 330k from s, 30k down, 30k from nf
    gb = cb / H
    zb = 1 / (gb + 1 / 330e3 + 2 / 30e3)
    lin(b, p + "vb0", 0.0, [(p + "b", gb * zb), (p + "s", zb / 330e3), (p + "nf", zb / 30e3)])
    ge = 50e-6 / H
    ze = 1 / (ge + 1 / 1e3)
    lin(b, p + "ve0", 0.0, [(p + "e", ge * ze)])
    # collector: 5.1k from s, 0.1u (qcc) to o; o: .033u, 25k to w; w: 25k down, 30k + 0.1u to Q29's base
    gc, go = 0.1e-6 / H, 0.033e-6 / H
    gbr = 1.0 / (BR[n][0] + H / BR[n][1])
    yw = 2 / 25e3 + gbr
    aw, bw = (1 / 25e3) / yw, gbr / yw
    ai = inv([[1 / 5.1e3 + gc, -gc], [-gc, gc + go + (1 - aw) / 25e3]])
    b.op("LD", "mx_b"); b.op("ADD", p + "qm"); b.op("ST", p + "mbq")

    def col(r):
        return [(p + "s", r[0] / 5.1e3), (p + "qcc", gc * (r[0] - r[1])), (p + "o", r[1] * go),
                (p + "mbq", r[1] * bw / 25e3)]
    lin(b, p + "vc0", 0.0, col(ai[0]))
    if part == "A":
        return
    b = real
    # never saturates here (ngspice: Vce >= 0.33 V): active mode, collector floor 0.4 V
    bjt_active(b, p, zb / 1e3, ze / 1e3, ai[0][0] / 1e3, True, 0.4, p + "vb0", p + "ve0", p + "vc0")
    lin(b, p + "b", 0.0, [(p + "vb0", 1.0), ("_ib", -zb / 1e3)])
    lin(b, p + "o", 0.0, col(ai[1]) + [("_ic", -ai[1][0] / 1e3)])
    b.op("SUB", p + "c"); b.op("MULI", imm=-ONE); b.op("ST", p + "qcc")
    b.op("LD", p + "o"); b.op("MULI", imm=q(aw)); b.op("MACI", p + "mbq", imm=q(bw)); b.op("ST", p + "w")
    # Q29 sees the wiper ramped linearly over the next slot (no 12 kHz steps): wi += dw every sample
    b.op("SUB", p + "wi"); b.op("MULI", imm=q(1.0 / SLOT)); b.op("ST", p + "dw")


# ---------------------------------------------------------------------------------------------------- torpedo
# Q7 DC point (deck models: Q8 b-e breakdown 7.0 V, IBV 10 uA): Ie 0.1382 mA, Q8 at 27.2 uA (952 ohm dynamic)
IE7 = 0.1382e-3
RS7 = 10e3 + 952.0
SIG_T = 0.00157                                      # Q8 breakdown noise, V RMS at 48 kHz (TP6: ~20 mV p-p at Q7's collector)


def torp(b):
    """torpedo at FS: Q8 noise -> Q7 (small signal at its fixed operating point) -> Q6 (Newton) on the 22u supply"""
    H = 1 / FS
    p = "t_"
    # Q7 small signal: base (Rs from the noise, 330k / Q7 r_pi), emitter (.015u || 56k to the bypassed 2.2k)
    gm = AF * IE7 / VT
    rpi = BF / gm
    ge = 0.015e-6 / H
    ai = inv([[1 / RS7 + 1 / 330e3 + 1 / rpi, -1 / rpi], [-(1 / rpi + gm), 1 / rpi + gm + 1 / 56e3 + ge]])
    kb = (ai[0][0] / RS7, ai[0][1] * ge)             # b7 = kb0 vn + kb1 e7p
    ke = (ai[1][0] / RS7, ai[1][1] * ge)
    b.op("NOISE", 0, 1)                              # vn = +/- SIG_T
    b.op("LDI", imm=q(-SIG_T * ke[0])); b.op("LDIF", imm=q(SIG_T * ke[0])); b.op("ST", "_t")
    b.op("LDI", imm=q(-SIG_T * (kb[0] - ke[0]) * gm * 1e3)); b.op("LDIF", imm=q(SIG_T * (kb[0] - ke[0]) * gm * 1e3))
    b.op("ADDI", imm=q(AF * IE7 * 1e3)); b.op("MACI", p + "e7", imm=q((kb[1] - ke[1]) * gm * 1e3)); b.op("ST", p + "ic7")
    b.op("LD", "_t"); b.op("MACI", p + "e7", imm=q(ke[1])); b.op("ST", p + "e7")
    # 22u supply: Q5 (47... 470 ohm from 15.97 V when D1 is set) vs Q6's 5.1k collector resistor (previous c6)
    g0 = 22e-6 / H
    y = g0 + 1 / 5.1e3
    lin(b, p + "s", 0.0, [(p + "s", g0 / y), (p + "c6", 1 / 5.1e3 / y)])
    b.op("BIT", 0, b.src(LATCH1, 1))
    j = b.skip_if(False)
    b.op("LDI", imm=q(15.97)); b.op("RC", p + "s", imm=q((1 / 470.0) / (y + 1 / 470.0)))
    b.land(j)
    # network c7, b6, c6, to: Thevenin seen by Q6's base / collector currents (330k feedback solved implicitly)
    gc, ga = 0.01e-6 / H, 0.01e-6 / H
    z = inv([[1 / 10e3 + gc, -gc, 0, 0], [-gc, gc + 1 / 47e3 + 1 / 330e3, -1 / 330e3, 0],
             [0, -1 / 330e3, 1 / 5.1e3 + 1 / 330e3 + ga, -ga], [0, 0, -ga, ga + 1 / 25e3]])

    def node(i):                                     # const, terms on the states, ie coefficient (V per mA)
        r = z[i]
        terms = [(p + "ic7", -r[0] / 1e3), (p + "qc", gc * (r[0] - r[1])), (p + "s", r[2] / 5.1e3),
                 (p + "qa", ga * (r[2] - r[3])), (p + "w", r[3] / 25e3)]
        return r[0] * V16 / 10e3, terms, -(r[1] / (BF + 1) + r[2] * AF) / 1e3
    c_b, t_b, k_b = node(1)
    lin(b, "_dv", c_b, t_b)
    b.op("ST", "_vb0")
    newton(b, p, -k_b + 0.1)
    for i, reg in ((0, "_c7"), (1, p + "b6"), (2, p + "c6"), (3, "_to")):
        c0, terms, k = node(i)
        lin(b, None, c0, terms + [("_ie", k)])
        if reg == p + "c6":
            b.op("MAXI", imm=q(0.05))
        b.op("ST", reg)
    b.op("LD", "_c7"); b.op("SUB", p + "b6"); b.op("ST", p + "qc")
    b.op("LD", p + "c6"); b.op("SUB", "_to"); b.op("ST", p + "qa")
    b.op("LD", "_to"); b.op("MULI", imm=q(0.5)); b.op("ST", "torp_u")


# ---------------------------------------------------------------------------------------------------- dive
# Q23 phase-shift oscillator (10k + 50k dive tone pot at 20 %, 3 x 2200p, 39k, 820k / 82k) and Q26 (1M feedback)
# as one linear network with both transistors linearised at their DC points; the transistors' cutoff / saturation
# are current clamps (an extra emitter current that holds Ie at the limit). Exact (matrix exponential) discretisation
# at FS / 2 with the clamp currents held over the step. Q22 (astable) drives node A through .047u + 47k; the Q27
# noise injection (1u + 220k into A, -40 dB re the oscillation) is left out. Q25 (gate) = Q26 held cut off.
DTONE = 0.2
DIVE_RC = 1 / (1 / (100e3 + 1 / (1 / 5e3 + 1 / 27e3)) + 1 / 1e6)  # Q26 collector load with the gate closed
DSTEP = 2                                            # dive network every 2nd sample (24 kHz)
DN = ["c27", "A", "B", "b23", "e23", "j", "q", "d28", "c26"]
DCAPS = [("c27", "A", 2200e-12), ("A", "B", 2200e-12), ("B", "b23", 2200e-12), ("d26", "j", 0.047e-6),
         ("c27", "q", 0.047e-6)]
DU = ["d26", "one", "D23", "D26"]                    # inputs: Q22 collector, 1 (supply / offsets), clamp currents (A)
DR = [("c27", "vss", 10e3), ("b23", "vss", 820e3), ("b23", None, 82e3), ("e23", None, 100.0),
      ("A", None, 10e3 + 50e3 * DTONE), ("B", None, 39e3), ("j", "A", 47e3), ("q", "d28", 220e3),
      ("c26", "vss", 10e3), ("c26", "d28", 1e6), ("c26", None, 100e3 + 1 / (1 / 5e3 + 1 / 27e3))]


def mmul(a, b):
    return [[sum(a[i][k] * b[k][j] for k in range(len(b))) for j in range(len(b[0]))] for i in range(len(a))]


def expm(a):
    n = len(a)
    nrm = max(sum(abs(x) for x in r) for r in a)
    sq = max(0, int(math.ceil(math.log2(nrm))) + 1) if nrm > 0.5 else 0
    a = [[x / 2 ** sq for x in r] for r in a]
    res = [[float(i == j) for j in range(n)] for i in range(n)]
    term = [r[:] for r in res]
    for k in range(1, 30):
        term = [[x / k for x in r] for r in mmul(term, a)]
        res = [[x + y for x, y in zip(r1, r2)] for r1, r2 in zip(res, term)]
    for _ in range(sq):
        res = mmul(res, res)
    return res


def dive_dc():
    """DC points: Q23 (base Thevenin of 820k / 82k, 100 ohm emitter), Q26 (1M feedback, 10k, ~104k load)"""
    rb, vth = 1 / (1 / 820e3 + 1 / 82e3), V16 * 82 / 902
    ie = 1e-3
    for _ in range(200):
        vbe23 = NVT * math.log(ie * AF / IS)
        ie = (vth - vbe23) / (rb / (BF + 1) + 100)
    ie23 = ie
    rl = 100e3 + 1 / (1 / 5e3 + 1 / 27e3)
    ie = 0.5e-3
    for _ in range(500):
        vbe26 = NVT * math.log(ie * AF / IS)
        # c26 = (16/10k - ic - (c26 - vbe)/1M) / (1/10k + 1/rl);  ib = (c26 - vbe) / 1M
        c26 = (V16 / 10e3 - AF * ie + vbe26 / 1e6) / (1 / 10e3 + 1 / rl + 1 / 1e6)
        ie = 0.5 * ie + 0.5 * (c26 - vbe26) / 1e6 * (BF + 1)
    return ie23, vbe23, ie, vbe26, c26


def network(nodes, caps, inputs, res, trs, h):
    """linear network, capacitor voltages as states x, inputs u ("one" = 1 for the 16 V supply and offsets), the
    transistors linearised: ie = g (vb - ve) + ie0 - g vbe0 + D (D = an input: clamp current). Exact discretisation
    over h: x' = Phi x + Gam u. Returns Phi, Gam, vrow(node) -> (x row, u row), ierow(k) -> (x row, u row) in A"""
    n, m, nu = len(nodes), len(caps), len(inputs)
    idx = {k: i for i, k in enumerate(nodes)}
    N = n + m
    G = [[0.0] * N for _ in range(N)]
    Rx = [[0.0] * m for _ in range(N)]
    Ru = [[0.0] * nu for _ in range(N)]
    one = inputs.index("one")
    for a, bnode, r in res:
        g = 1 / r
        ia = idx[a]
        G[ia][ia] += g
        if bnode in idx:
            G[ia][idx[bnode]] -= g; G[idx[bnode]][idx[bnode]] += g; G[idx[bnode]][ia] -= g
        elif bnode == "vss":
            Ru[ia][one] += V16 * g
        elif bnode is not None:                      # an input voltage
            Ru[ia][inputs.index(bnode)] += g
    for k, (pn, nn, c) in enumerate(caps):           # cap k: current from pn to nn; v_pn - v_nn = x_k
        row = n + k
        for node, sgn in ((pn, 1), (nn, -1)):
            if node in idx:
                G[idx[node]][row] += sgn
                G[row][idx[node]] += sgn
            else:
                Ru[row][inputs.index(node)] -= sgn
        Rx[row][k] = 1.0
    for bn, en, cn, ie0, vbe0, dname in trs:         # b -> e: ie / (BF + 1), c -> e: AF ie
        g = ie0 / NVT
        for node, frac in ((bn, 1 / (BF + 1)), (cn, AF), (en, -1.0)):
            if node is None:
                continue
            i = idx[node]
            G[i][idx[bn]] += frac * g
            if en is not None:
                G[i][idx[en]] -= frac * g
            Ru[i][one] -= frac * (ie0 - g * vbe0)
            Ru[i][inputs.index(dname)] -= frac
    Mi = inv(G)
    Px = mmul(Mi, Rx)
    Pu = mmul(Mi, Ru)
    cv = [c for _, _, c in caps]
    A = [[Px[n + k][j] / cv[k] for j in range(m)] for k in range(m)]
    B = [[Pu[n + k][j] / cv[k] for j in range(nu)] for k in range(m)]
    Phi = expm([[x * h for x in r] for r in A])
    PmI = [[Phi[i][j] - (i == j) for j in range(m)] for i in range(m)]
    Gam = mmul(mmul(inv(A), PmI), B)

    def vrow(node):
        return Px[idx[node]], Pu[idx[node]]

    def ierow(k):
        bn, en, _, ie0, vbe0, dname = trs[k]
        g = ie0 / NVT
        xb, ub = vrow(bn)
        xe, ue = vrow(en) if en else ([0.0] * m, [0.0] * nu)
        rx = [g * (p_ - q_) for p_, q_ in zip(xb, xe)]
        ru = [g * (p_ - q_) for p_, q_ in zip(ub, ue)]
        ru[one] += ie0 - g * vbe0
        ru[inputs.index(dname)] += 1.0
        return rx, ru
    return Phi, Gam, vrow, ierow


def dive_system():
    ie23, vbe23, ie26, vbe26, _ = dive_dc()
    tr = [("b23", "e23", "c27", ie23, vbe23, "D23"), ("d28", None, "c26", ie26, vbe26, "D26")]
    Phi, Gam, vrow, ierow = network(DN, DCAPS, DU, DR, tr, DSTEP / FS)
    return Phi, Gam, vrow, ierow(0), ierow(1)


def dive_net(b, src, dst):
    """one dive network step (every DSTEP-th sample) from state bank src to bank dst: sets dive_c26, dive_u"""
    Phi, Gam, vrow, (r23x, r23u), (r26x, r26u) = dive_system()
    m = len(DCAPS)
    xs = [src + "%d" % k for k in range(m)]
    one, d23, d26 = DU.index("one"), DU.index("D23"), DU.index("D26")
    # Q23: free emitter current (mA), clamp 0 .. 1.55 mA (saturation: (16 - 0.1) / 10.1k), D = (clamped - free) / s
    def free(rx, ru, extra):
        lin(b, "_if", ru[one] * 1e3, [(x, k * 1e3) for x, k in zip(xs, rx)] + [("dv26", ru[0] * 1e3)] + extra)
    free(r23x, r23u, [])
    b.op("MAXI", imm=0); b.op("MINI", imm=q(1.55)); b.op("SUB", "_if")
    b.op("MULI", imm=q(1e-3 / r23u[d23])); b.op("ST", "_d23")         # clamp current in A
    # Q26: cut off while the gate (Q25) is closed; saturation (16 - 0.1) / 10k
    free(r26x, r26u, [("_d23", r26u[d23] * 1e3)])
    b.op("ST", "_t")
    b.op("LD", "dive_g"); b.op("MULI", imm=q(1.59)); b.op("ST", "_imax")             # 0 when the gate is closed
    b.op("LD", "_t"); b.op("MAXI", imm=0)
    b.op("SUB", "_imax"); b.op("MINI", imm=0); b.op("ADD", "_imax")                  # min(acc, imax)
    b.op("SUB", "_t"); b.op("MULI", imm=q(1e-3 / r26u[d26])); b.op("ST", "_d26")
    # Q26 collector (before the step), for the output network
    vx, vu = vrow("c26")
    lin(b, "dive_c26", vu[one], [(x, k) for x, k in zip(xs, vx)] + [("dv26", vu[0]), ("_d23", vu[d23]), ("_d26", vu[d26])])
    b.op("LD", "dive_g"); b.op("CMPI", imm=q(0.5))     # gate closed: Q25 grounds Q26's base, collector at DC
    b.op("LDI", imm=q(V16 * DIVE_RC / (10e3 + DIVE_RC))); b.op("LDF", "dive_c26"); b.op("ST", "dive_c26")
    b.op("MULI", imm=q(5e3 / 110e3)); b.op("ST", "dive_u")       # 100k into the 10k pot: wiper Thevenin (R_TH_N)
    # state step
    for k in range(m):
        lin(b, dst + "%d" % k, Gam[k][one], [(x, Phi[k][j]) for j, x in enumerate(xs)] +
            [("dv26", Gam[k][0]), ("_d23", Gam[k][d23]), ("_d26", Gam[k][d26])])


def dive_slow(b, part):
    """at FS / 4 (part "caps" and part "astable" on alternate dive slots): Q19 switch, C6 (2u: 220k + diode in, 180k to Q20's base) and C7 (1u: 120k + diode
    in, 68k to Q24's base), gate, Q21 / Q22 astable (period and low time vs C6 from ngspice)"""
    h = 2 * DSTEP / FS
    if part == "astable":
        dive_astable(b, h)
        return
    b.op("BIT", 0, b.src(LATCH1, 2))
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "_sw")
    # C6: dV = h / 2u [ sw (15.37 - V) / 220k - (V - 0.65) / 180k ]   (Q20 base held at ~0.65 V while it conducts)
    b.op("LD", "dv6"); b.op("ADDI", imm=q(-0.65)); b.op("MAXI", imm=0); b.op("MULI", imm=q(-h / 2e-6 / 180e3))
    b.op("ST", "_t")
    b.op("LDI", imm=q(15.37)); b.op("SUB", "dv6"); b.op("MUL", "_sw"); b.op("MULI", imm=q(h / 2e-6 / 220e3))
    b.op("ADD", "_t"); b.op("ADD", "dv6"); b.op("ST", "dv6")
    # C7: 120k in, 68k out into Q24's base (0.65 V)
    b.op("LD", "dv7"); b.op("ADDI", imm=q(-0.65)); b.op("MAXI", imm=0); b.op("MULI", imm=q(-h / 1e-6 / 68e3))
    b.op("ST", "_t")
    b.op("LDI", imm=q(15.37)); b.op("SUB", "dv7"); b.op("MUL", "_sw"); b.op("MULI", imm=q(h / 1e-6 / 120e3))
    b.op("ADD", "_t"); b.op("ADD", "dv7"); b.op("ST", "dv7")
    # gate open (Q24 on, Q25 off) while C7 > 0.9 V (Q24 sinks Q25's 47k base current through 68k)
    b.op("CMPI", imm=q(0.9)); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "dive_g")



def dive_astable(b, h):
    # astable: period 14.9 ms (C6 < 4.45 V) .. 9.35 ms (C6 > 5.68 V), low time 2.96 .. 0.98 ms (ngspice)
    def interp(v0, v1, y0, y1):
        k = (y1 - y0) / (v1 - v0)
        b.op("LD", "dv6"); b.op("MAXI", imm=q(v0)); b.op("MINI", imm=q(v1)); b.op("ADDI", imm=q(-v0))
        b.op("MULI", imm=q(k)); b.op("ADDI", imm=q(y0))
    interp(4.45, 5.68, h / 14.9e-3, h / 9.35e-3)
    b.op("ADD", "dph"); b.op("CMPI", imm=ONE); b.op("ADDI", imm=-ONE); b.op("STF", "dph"); b.op("ADDI", imm=ONE)
    b.op("STNF", "dph")
    interp(4.45, 5.68, 2.96 / 14.9, 0.98 / 9.35)
    b.op("ST", "_t")
    b.op("LD", "dph"); b.op("CMP", "_t")             # F = phase >= low fraction: Q22 off (collector high)
    b.op("LDI", imm=q(0.018)); b.op("LDIF", imm=q(14.08)); b.op("ST", "dv26")


# ---------------------------------------------------------------------------------------------------- sonar
# Timer: Q17 charges 4.7u via 100 ohm; Q16 holds Q15 out of saturation (sound on) while 4.7u * 100k / 570k > 0.6 V.
# Q9 / Q10 slow astable (sonar rate pot at max = 250k: ngspice Q9 on 0.43 s of 1.093 s); its falling edge turns Q13
# off for ~6 ms (Q13's base kicked to -7 V b-e breakdown, back through 68k / 0.22u) charging C4 (1u) through 10k +
# diode, its rising edge adds ~1.4 V through 0.22u + D2 (ngspice); C4 discharges through 1M and, while Q12 is on,
# 100k + D5. Q11 / Q12 astable (tone pot grounded): 1.229 ms, Q12 on 38 % (ngspice). D5 gates node 18: Q12 on ->
# 0.63 V, off -> C4. Q15 (1M feedback, .033u / .0033u / 8.2k (+ 8.2k via Q14 while Q9 is off), 390k + .01u from node
# 18) as a linear network with a clamp (saturated = muted), exact discretisation at FS / 2 on the generator slots.
SON_RATE_T, SON_RATE_ON = 1.093, 0.43
SON_TONE_T, SON_TONE_ON = 1.1755e-3, 0.396                 # ngspice (b-e breakdown 7 V)
SON_RL = 100e3 + 1 / (1 / 5e3 + 1 / 27e3)
SN = ["b15", "c22", "M", "bot"]
SCAPS = [("c22", "M", 0.033e-6), ("M", "b15", 0.0033e-6), ("bot", "b15", 0.01e-6)]
SU = ["n18", "one", "D15"]


def sonar_dc():
    ie = 0.7e-3
    for _ in range(500):
        vbe = NVT * math.log(ie * AF / IS)
        c = (V16 / 10e3 - AF * ie + vbe / 1e6) / (1 / 10e3 + 1 / SON_RL + 1 / 1e6)
        ie = 0.5 * ie + 0.5 * (c - vbe) / 1e6 * (BF + 1)
    return ie, vbe, c


def sonar_system(q14):
    ie, vbe, _ = sonar_dc()
    res = [("c22", "vss", 10e3), ("c22", "b15", 1e6), ("c22", None, SON_RL), ("M", None, 8.2e3),
           ("bot", "n18", 390e3)] + ([("M", None, 8.2e3)] if q14 else [])
    Phi, Gam, vrow, ierow = network(SN, SCAPS, SU, res, [("b15", None, "c22", ie, vbe, "D15")], 2 / FS)
    return Phi, Gam, vrow, ierow(0)


def sonar_slow(b, part):
    """at FS / 4: part "timer" (timer, mute) and part "c4" (slow astable, C4) on alternate dive slots"""
    h = 4 / FS
    if part == "c4":
        sonar_c4(b, h)
        return
    b.op("BIT", 0, b.src(LATCH1, 3))
    j = b.skip_if(False)
    b.op("LDI", imm=q(15.97)); b.op("RC", "sn20", imm=q(1 - math.exp(-h / (100 * 4.7e-6))))
    j2 = jump(b)
    b.land(j)                                        # discharge through 470k into Q16's base (100k down, ~0.62 V)
    b.op("LD", "sn20"); b.op("MULI", imm=q(100 / 570)); b.op("MINI", imm=q(0.62)); b.op("SUB", "sn20")
    b.op("MULI", imm=q(h / 4.7e-6 / 470e3)); b.op("ADD", "sn20"); b.op("ST", "sn20")
    b.land(j2)
    b.op("CMPI", imm=q(4.0)); b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", "sonar_m")    # 1 = muted (ngspice: ~4 V)


def sonar_c4(b, h):
    # slow astable: phase 0 .. 1, Q9 on (node 11 low) while phase < 0.43 / 1.093
    b.op("LD", "sph"); b.op("ADDI", imm=q(h / SON_RATE_T)); b.op("CMPI", imm=ONE); b.op("ADDI", imm=-ONE)
    b.op("STF", "sph"); b.op("ADDI", imm=ONE); b.op("STNF", "sph")
    b.op("CMPI", imm=q(SON_RATE_ON / SON_RATE_T)); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "_t")
    # edges: _t - sq14 = -1: falling (Q9 on) -> Q13 off timer; +1: rising -> C4 + 1.4 V
    b.op("SUB", "sq14"); b.op("ST", "_e")
    b.op("LD", "_t"); b.op("ST", "sq14")             # Q14 on while Q9 is off (node 11 high)
    b.op("LD", "_e"); b.op("CMPI", imm=q(-0.5))
    b.op("LDI", imm=q(6.0e-3)); b.op("STNF", "s13")
    b.op("LD", "_e"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "sn17"); b.op("ADDI", imm=q(1.4)); b.op("STF", "sn17")
    # C4: discharge (1M, and 100k + D5 for the 38 % Q12 is on), charge from Q13's collector while its timer runs
    b.op("LD", "sn17"); b.op("ADDI", imm=q(-0.65)); b.op("MAXI", imm=0); b.op("MULI", imm=q(-SON_TONE_ON * h / 1e-6 / 100e3))
    b.op("MACI", "sn17", imm=q(1 - h / 1e-6 / 1e6)); b.op("ST", "sn17")
    b.op("LD", "s13"); b.op("CMPI", imm=1)
    j = b.skip_if(False)
    b.op("ADDI", imm=q(-h)); b.op("ST", "s13")
    b.op("LDI", imm=q(15.4)); b.op("RC", "sn17", imm=q(1 - math.exp(-h / 10e-3)))
    b.land(j)


def sonar_net(b, src, dst):
    """at FS / 2: fast astable, node 18, Q15 network (bank src -> dst); sets sonar_u"""
    h = 2 / FS
    # node 18 averaged over the step (no aliasing of the 850 Hz square): the fraction of the step Q12 is off
    inc = h / SON_TONE_T
    b.op("LD", "tph"); b.op("ADDI", imm=q(inc)); b.op("ST", "tph")              # unwrapped end of step
    b.op("ADDI", imm=q(-SON_TONE_ON)); b.op("MULI", imm=q(1 / inc)); b.op("MAXI", imm=0); b.op("MINI", imm=ONE)
    b.op("ST", "_t")
    b.op("LD", "tph"); b.op("MULI", imm=q(-1 / inc)); b.op("ADDI", imm=q(1 / inc + 1))       # after a wrap: 1 - over / inc
    b.op("SUB", "_t"); b.op("MINI", imm=0); b.op("ADD", "_t"); b.op("MULI", imm=-ONE); b.op("ST", "_t")   # -(off fraction)
    b.op("LD", "tph"); b.op("CMPI", imm=ONE); b.op("ADDI", imm=-ONE); b.op("STF", "tph")
    b.op("LDI", imm=q(0.63)); b.op("SUB", "sn17"); b.op("MUL", "_t"); b.op("ADDI", imm=q(0.63)); b.op("ST", "sn18")
    # muted (Q16 off: Q16's diode saturates Q15): collector 0.04 V, base 0.68 V, the caps follow
    kw = 5e3 / 110e3                                 # Q15 collector -> 100k into the 10k pot: wiper Thevenin
    b.op("LD", "sonar_m"); b.op("CMPI", imm=q(0.5))
    jm = b.skip_if(False)
    b.op("LDI", imm=q(0.04)); b.op("ST", dst + "0"); b.op("MULI", imm=q(kw)); b.op("ST", "sonar_u")
    b.op("LDI", imm=q(-0.68)); b.op("ST", dst + "1"); b.op("ADD", "sn18"); b.op("ST", dst + "2")
    ends = [jump(b)]
    b.land(jm)
    b.op("LD", "sq14"); b.op("CMPI", imm=q(0.5))
    jv = b.skip_if(True)
    for q14 in (0, 1):
        if q14:
            b.land(jv)
        Phi, Gam, vrow, (rx, ru) = sonar_system(q14)
        xs = [src + "%d" % k for k in range(len(SCAPS))]
        i18, one, d15 = 0, 1, 2
        lin(b, "_if", ru[one] * 1e3, [(x, k * 1e3) for x, k in zip(xs, rx)] + [("sn18", ru[i18] * 1e3)])
        b.op("MAXI", imm=0); b.op("MINI", imm=q(1.59))
        b.op("SUB", "_if"); b.op("MULI", imm=q(1e-3 / ru[d15])); b.op("ST", "_d15")
        vx, vu = vrow("c22")
        lin(b, "sonar_u", vu[one] * kw, [(x, k * kw) for x, k in zip(xs, vx)] + [("sn18", vu[i18] * kw),
                                                                              ("_d15", vu[d15] * kw)])
        for k in range(len(SCAPS)):
            lin(b, dst + "%d" % k, Gam[k][one], [(x, Phi[k][j]) for j, x in enumerate(xs)] +
                [("sn18", Gam[k][i18]), ("_d15", Gam[k][d15])])
        if not q14:
            ends.append(jump(b))
    for e in ends:
        b.land(e)


# ---------------------------------------------------------------------------------------------------- Q29
def q29_dc():
    """Q29 DC operating point (all coupling caps open): base, emitter, collector, Vbe"""
    zb = 1 / (1 / 150e3 + 1 / 30e3)
    vth = V16 * 30 / 180
    ie = 1.5e-3
    for _ in range(200):
        vbe = NVT * math.log(ie * AF / IS)
        ie = (vth - vbe) / (zb / (BF + 1) + 1e3)
    vb = vth - zb * ie / (BF + 1)
    return vb, ie * 1e3, V16 - 5.1e3 * AF * ie, vbe


def q29(b, srcs):
    """summing amplifier at FS. srcs: name -> register holding the branch source voltage"""
    H = 1 / FS
    y = 1 / 150e3 + 1 / 30e3
    g = {}
    for n, (r, c) in BR.items():
        g[n] = 1.0 / (r + H / c)
        y += g[n]
    zb = 1 / y
    terms = []
    for n in BR:
        b.op("LD", srcs[n]); b.op("SUB", n + "_qm"); b.op("ST", "_u" + n)
        terms.append(("_u" + n, g[n] * zb))
    lin(b, "_vb0", V16 / 150e3 * zb, terms)
    ge = 50e-6 / H
    ze = 1 / (ge + 1e-3)
    lin(b, "_ve0", 0.0, [("mx_e", ge * ze)])
    # collector: 5.1k from 16 V, 0.1u (q4) to n4, 30k to n7, 1n + 47k pot (23.5k + 23.5k) down
    g4, g7 = 0.1e-6 / H, 0.001e-6 / H
    ai = inv([[1 / 5.1e3 + g4, -g4, 0.0], [-g4, g4 + 1 / 30e3, -1 / 30e3], [0.0, -1 / 30e3, 1 / 30e3 + g7 + 1 / 47e3]])

    def node(r, k):
        return r[0] * V16 / 5.1e3, [("mx_q4", g4 * (r[0] - r[1])), ("mx_n7", r[2] * g7)] + ([("_ic", k)] if k else [])
    lin(b, "_vc0", *node(ai[0], 0))
    vb_dc, ie_dc, vc_dc, vbe_dc = q29_dc()
    bjt_lin(b, "mx_", zb / 1e3, ze / 1e3, ai[0][0] / 1e3, ie_dc, vbe_dc, 1.6)
    lin(b, "_t", *node(ai[1], -ai[1][0] / 1e3))
    lin(b, "mx_n7", *node(ai[2], -ai[2][0] / 1e3))
    b.op("LD", "mx_c"); b.op("SUB", "_t"); b.op("ST", "mx_q4")
    lin(b, "mx_b", 0.0, [("_vb0", 1.0), ("_ib", -zb / 1e3)])
    # torpedo pot wiper for the next sample: w = u - R_th i
    b.op("LD", "torp_u"); b.op("SUB", "torp_qm"); b.op("SUB", "mx_b"); b.op("MULI", imm=q(-R_TH_T * g["torp"]))
    b.op("ADD", "torp_u"); b.op("ST", "t_w")
    for n, (r, c) in BR.items():                     # coupling caps: q += (u - q - vb) g H / C
        b.op("LD", "_u" + n); b.op("SUB", "mx_b"); b.op("MULI", imm=q(g[n] * H / c)); b.op("ADD", n + "_qm")
        b.op("ST", n + "_qm")


# ---------------------------------------------------------------------------------------------------- program
def init(b):
    """first sample after reset (silent): DC operating point"""
    vb, ie, vc, vbe = q29_dc()
    vals = {"mx_b": vb, "mx_e": ie * 1e-3 * 1e3, "mx_c": vc, "mx_q4": vc, "mx_vbe": vbe,
            "mine_qm": 0.0 - vb, "ship_qm": 0.0 - vb, "torp_qm": 0.0 - vb, "sonar_qm": 0.0 - vb, "dive_qm": 0.0 - vb}
    vals["t_qc"] = V16 - 10e3 * AF * IE7
    vals["t_ic7"] = AF * IE7 * 1e3
    for n in GEN:
        vals[n + "_qcp"] = X0
    du = V16 * DIVE_RC / (10e3 + DIVE_RC) * 5e3 / 110e3           # dive output at rest (gate closed)
    vals.update({"dive_u": du, "dive_qm": du - vb, "sonar_m": 1.0})
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for k, v in vals.items():
        b.op("LDI", imm=q(v)); b.op("ST", k)
    b.op("LDI", imm=ONE); b.op("ST", "_ini")
    b.op("END")
    b.land(j)
    return vals


OUT_SCALE = 1.0 / 4.0                                # master pot wiper (n7 / 2) of 4 V = full scale


def seawolf():
    b = Board("seawolf")
    init(b)
    # slot counter 0, .25, .5, .75
    # slot bits: _a toggles every sample (0 generator slot, 1 dive slot), _b every second sample (mine / caps vs
    # ship / astable): generators 12 kHz each, sonar network and dive network 24 kHz, slow parts 12 kHz
    b.op("LDI", imm=ONE); b.op("SUB", "_a"); b.op("ST", "_a")
    b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=ONE); b.op("SUB", "_b"); b.op("STF", "_b")
    b.op("LD", "_a"); b.op("CMPI", imm=q(0.5))
    j_dive = b.skip_if(True)
    ends = []
    b.op("NOISE", 0, 0); b.op("LDI", imm=q(-SIG_GEN)); b.op("LDIF", imm=q(SIG_GEN)); b.op("ST", "_gx")
    b.op("LD", "_b"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for n in GEN:
        if n == "ship":
            b.land(j)
        gen(b, n, "A"); gen(b, n, "B")
        sonar_net(b, *(("sxa", "sxb") if n == "mine" else ("sxb", "sxa")))
        ends.append(jump(b))
    b.land(j_dive)
    b.op("LD", "_b"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)                              # C6 / C7 / gate, sonar timer, network bank a -> b
    dive_slow(b, "caps")
    sonar_slow(b, "timer")
    dive_net(b, "dxa", "dxb")
    ends.append(jump(b))
    b.land(j)                                        # astables, C4, network bank b -> a
    dive_slow(b, "astable")
    sonar_slow(b, "c4")
    dive_net(b, "dxb", "dxa")
    for e in ends:
        b.land(e)
    for n in GEN:
        b.op("LD", n + "_wi"); b.op("ADD", n + "_dw"); b.op("ST", n + "_wi")
    torp(b)
    q29(b, {"mine": "mine_wi", "ship": "ship_wi", "torp": "torp_u", "sonar": "sonar_u", "dive": "dive_u"})
    b.op("LD", "mx_n7"); b.op("MULI", imm=q(0.5 * OUT_SCALE)); b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    p = seawolf()
    print(f"{p.name}: {len(p.code)} words, worst {p.worst_path()} units, {len(p.regs)} registers")
    if len(sys.argv) > 1:
        open(sys.argv[1], "wb").write(p.binary())
