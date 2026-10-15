"""Gun Fight (Midway 1975) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Four transistor noise generators (left / right shot and hit), each switched onto its own supply cap, feeding one
transistor mixer per side. Per-sample solver as verilator/snd/gunfight/gf_proto.py (the spec; MAME nl_gunfight
netlist values, BC548C, VT at 300 K): Thevenin base / emitter / collector networks with constant coefficients, one
warm-started Newton step on Vbe (EXP + RCP), saturation as a Vce clamp mode, cross-network couplings one sample late.
The sides alternate on the engine's 48 kHz samples (24 kHz each); output = 0.5 (L + R), like the other stereo boards.
Units: V, mA, kOhm. S1 = the sound port (A0 AND gate): (~D7-D4) = 1 / 2 / 3 / 4 turns on left shot / right shot /
left hit / right hit, any other value turns all four off (MAME gunfight_audio_device::write).
"""
import math
from dsnd import Board, LATCH1, q, ONE

SR = 24000.0                                         # per side
H = 1.0 / SR
VT = 1.380649e-23 * 300 / 1.602176634e-19
IS, BF, NF = 1.95e-14, 466.0, 0.993
AF = BF / (1 + BF)
NVT = NF * VT
K1 = 1.0 / (NVT * math.log(2.0))                     # Ie (mA) = 2^(K1 Vbe + K0)
K0 = math.log2(IS / AF * 1e3)
VSAT = 0.1
V16 = 16.0
X0 = 9.505752                                        # noise node DC
NOISE_A = 0.002 * math.sqrt(SR / 48000) * 55.15      # +/- one LFSR bit per side sample: the prototype's density
NEVER = -(1 << 31)                                   # CMPI NEVER: F = 1 (unconditional skip)

# supply C, noise-filter C, base C, emitter R, emitter C, collector coupling C, out C, mixer coupling C, side, code
GEN = {
    "ls": (10e-6, 0.015e-6, 0.01e-6, 100.0, 0.0, 0.047e-6, 0.022e-6, 0.047e-6, "l", 1),
    "rs": (10e-6, 0.015e-6, 0.015e-6, 100.0, 0.0, 0.047e-6, 0.022e-6, 0.047e-6, "r", 2),
    "lh": (20e-6, 0.033e-6, 0.1e-6, 1000.0, 50e-6, 0.1e-6, 0.033e-6, 0.1e-6, "l", 3),
    "rh": (20e-6, 0.033e-6, 0.047e-6, 1000.0, 50e-6, 0.1e-6, 0.033e-6, 0.1e-6, "r", 4),
}
# DC operating point (prototype settled at 24 kHz, switches off; sub-mV generator nodes start at 0): generators cut
# off (saturation mode), mixers active. State memory clears to 0 on reset.
DC = {"qcp": 9.505744, "qcm": -2.564178, "sat": 1.0}                    # every generator
DC_MIX = {"mb": 2.564177531, "e": 1.914497062, "c": 6.256972766, "q4": 6.256972766, "vbe": 0.649680468}


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
    if not const and terms[0][1] == 1.0:
        b.op("LD", terms[0][0])
        terms = terms[1:]
    else:
        b.op("LDI", imm=q(const))
    for reg, k in terms:
        b.op("MACI", reg, imm=q(k))
    b.op("ST", dst)


def jump(b):
    """unconditional forward skip (F forced to 1; tools/dsnd.py worst_path() takes it as always taken)"""
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def newton(b, p, rt):
    """one Newton step on Vbe = _dv - rt Ie(Vbe) from the previous sample's Vbe (rt kOhm), the step written as
    NVT (dv - v - rt Ie) / (NVT + rt Ie) to stay inside Q24; acc = _ie (mA) = (_dv - Vbe) / rt, clamped to
    0 .. min(100 mA, 64 V / rt)"""
    b.op("LD", p + "vbe"); b.op("MULI", imm=q(K1)); b.op("ADDI", imm=q(K0)); b.op("EXP")
    if rt * 127.0 > 100.0:
        b.op("MINI", imm=q(100.0 / rt))                                # rt Ie <= 100 V
    b.op("MULI", imm=q(rt)); b.op("ST", "_re")
    b.op("ADDI", imm=q(NVT)); b.op("RCP"); b.op("ST", "_rd")
    b.op("LD", "_dv"); b.op("SUB", p + "vbe"); b.op("SUB", "_re"); b.op("MULI", imm=q(NVT)); b.op("MUL", "_rd")
    b.op("ADD", p + "vbe"); b.op("ST", p + "vbe")
    b.op("LD", "_dv"); b.op("SUB", p + "vbe"); b.op("MAXI", imm=0); b.op("MINI", imm=q(min(100.0 * rt, 64.0)))
    k = 1.0 / rt
    if k >= 127.0:
        b.op("MULI", imm=q(k / 8)); k = 8.0
    b.op("MULI", imm=q(k)); b.op("ST", "_ie")


def bjt(b, p, zb, ze, zcc, ve0):
    """transistor p against Thevenin _vb0 / zb, _ve0 (if ve0) / ze, _vc0 / zcc (kOhm): sets p+e, p+c, p+sat, _ic, _ib"""
    b.op("LD", p + "sat"); b.op("CMPI", imm=q(0.5))
    j_sat = b.skip_if(True)
    # active
    b.op("LD", "_vb0")
    if ve0:
        b.op("SUB", "_ve0")
    b.op("ST", "_dv")
    newton(b, p, zb / (BF + 1) + ze)
    b.op("MULI", imm=q(AF)); b.op("ST", "_ic"); b.op("MULI", imm=q(-zcc)); b.op("ADD", "_vc0"); b.op("ST", p + "c")
    lin(b, p + "e", 0.0, ([("_ve0", 1.0)] if ve0 else []) + [("_ie", ze)])
    b.op("LD", p + "c"); b.op("SUB", p + "e"); b.op("CMPI", imm=q(VSAT))
    j = b.skip_if(True)                                          # Vce below VSAT: saturates this sample
    b.op("LDI", imm=ONE); b.op("ST", p + "sat")
    b.op("LD", p + "e"); b.op("ADDI", imm=q(VSAT)); b.op("ST", p + "c")
    b.op("LD", "_vc0"); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("ST", "_ic")
    b.land(j)
    b.op("LD", "_ie"); b.op("SUB", "_ic"); b.op("ST", "_ib")
    j_end = jump(b)
    b.land(j_sat)
    # saturated: Vce = VSAT, the base network also sees the collector network
    lin(b, "_dv", -zb * VSAT / zcc, [("_vb0", 1.0), ("_vc0", zb / zcc)] + ([("_ve0", -(1 + zb / zcc))] if ve0 else []))
    newton(b, p, zb + ze + zb * ze / zcc)
    b.op("MULI", imm=q(ze))
    if ve0:
        b.op("ADD", "_ve0")
    b.op("ST", p + "e"); b.op("ADDI", imm=q(VSAT)); b.op("ST", p + "c")
    b.op("LD", "_vc0"); b.op("SUB", p + "c"); b.op("MULI", imm=q(1.0 / zcc)); b.op("MAXI", imm=0); b.op("ST", "_ic")
    b.op("MULI", imm=q(-zcc)); b.op("ADD", "_vc0"); b.op("ST", p + "c")         # = e + VSAT, or vc0 if ic was < 0
    b.op("LD", "_ie"); b.op("SUB", "_ic"); b.op("ST", "_ib")
    b.op("LD", "_ic"); b.op("MULI", imm=q(1.0 / BF)); b.op("SUB", "_ib"); b.op("CMPI", imm=1)
    b.op("LDI", imm=0); b.op("STF", p + "sat")                                    # ic > BF ib: active again
    b.land(j_end)


def gbr_of(cm):
    return 1.0 / (30e3 + H / cm)                     # 30k + mixer coupling cap (companion) to the mixer base


def gen(b, n, mb):
    csup, cnf, cb, re, ce, cc, co, cm, side, code = GEN[n]
    p = n + "_"
    # supply cap vs the bias divider and the collector resistor (previous b, c); switch on: + 15.1 ohm from 16 V
    # (the switched solution = the open one pulled toward 16 V by gs / (y + gs))
    g0 = csup / H
    y = g0 + 1 / 330e3 + 1 / 5.1e3
    lin(b, p + "s", 0.0, [(p + "s", g0 / y), (p + "b", 1 / 330e3 / y), (p + "c", 1 / 5.1e3 / y)])
    b.op("LD", p + "sw"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(False)
    b.op("LDI", imm=q(V16)); b.op("RC", p + "s", imm=q((1 / 15.1) / (y + 1 / 15.1)))
    b.land(j)
    # noise filter: x -> 50 ohm -> 0.1 uF (qcp) -> 20k -> nf (C_nf) -> 30k -> b; nf first, qcp from the new nf
    gcp, gnf = 0.1e-6 / H, cnf / H
    ai = inv([[1 / 50 + gcp, -gcp, 0.0], [-gcp, gcp + 1 / 20e3, -1 / 20e3],
              [0.0, -1 / 20e3, 1 / 20e3 + gnf + 1 / 30e3]])

    def row(r):                                      # coefficients on x, qcp, nf, b (previous sample)
        return [r[0] / 50, gcp * (r[0] - r[1]), r[2] * gnf, r[2] / 30e3]
    rn, rq = row(ai[2]), row([ai[0][k] - ai[1][k] for k in range(3)])     # nf, qcp = xo - ni
    rq = [rq[k] - rq[2] * rn[k] / rn[2] for k in range(2)] + [rq[2] / rn[2], rq[3] - rq[2] * rn[3] / rn[2]]
    for reg, r in ((p + "nf", rn), (p + "qcp", rq)):
        lin(b, reg, r[0] * X0, [("_nx", r[0]), (p + "qcp", r[1]), (p + "nf", r[2]), (p + "b", r[3])])
    # base Thevenin: C_b, 330k from s, 30k down, 30k from nf
    gb = cb / H
    zb = 1 / (gb + 1 / 330e3 + 2 / 30e3)
    lin(b, "_vb0", 0.0, [(p + "b", gb * zb), (p + "s", zb / 330e3), (p + "nf", zb / 30e3)])
    if ce:                                           # emitter R_e || C_e
        ge = ce / H
        ze = 1 / (ge + 1 / re)
        lin(b, "_ve0", 0.0, [(p + "e", ge * ze)])
    else:
        ze = re
    # collector: 5.1k from s, C_c to o; o: C_o, 25k to w; w: 25k down, 30k + C_m to the mixer base (previous mb)
    gc, go, gbr = cc / H, co / H, gbr_of(cm)
    yw = 2 / 25e3 + gbr
    aw, bw = (1 / 25e3) / yw, gbr / yw
    ai = inv([[1 / 5.1e3 + gc, -gc], [-gc, gc + go + (1 - aw) / 25e3]])
    b.op("LD", mb); b.op("ADD", p + "qcm"); b.op("ST", "_mbq")

    def col(r):
        return [(p + "s", r[0] / 5.1e3), (p + "qcc", gc * (r[0] - r[1])), (p + "o", r[1] * go),
                ("_mbq", r[1] * bw / 25e3)]
    lin(b, "_vc0", 0.0, col(ai[0]))
    bjt(b, p, zb / 1e3, ze / 1e3, ai[0][0] / 1e3, bool(ce))
    lin(b, p + "b", 0.0, [("_vb0", 1.0), ("_ib", -zb / 1e3)])
    lin(b, p + "o", 0.0, col(ai[1]) + [("_ic", -ai[1][0] / 1e3)])                 # vo0 - zoc ic
    b.op("SUB", p + "c"); b.op("MULI", imm=-ONE); b.op("ST", p + "qcc")
    b.op("LD", p + "o"); b.op("MULI", imm=q(aw)); b.op("MACI", "_mbq", imm=q(bw)); b.op("ST", p + "w")


def mixer(b, sd, gens):
    p = "m" + sd + "_"
    # base: 150k from 16 V, 30k down, each generator's 30k + C_m from its wiper w (C_m charge from the last sample)
    y = 1 / 150e3 + 1 / 30e3 + sum(gbr_of(GEN[g][7]) for g in gens)
    zb = 1 / y
    terms = []
    for g in gens:
        k = gbr_of(GEN[g][7]) * zb
        terms += [(g + "_w", k), (g + "_qcm", -k)]
    lin(b, "_vb0", V16 / 150e3 * zb, terms)
    ge = 50e-6 / H
    ze = 1 / (ge + 1e-3)
    lin(b, "_ve0", 0.0, [(p + "e", ge * ze)])
    # collector: 5.1k from 16 V, 0.047 uF (q4) to n4, 30k to n7, 1 nF + the 47k master pot down
    g4, g7 = 0.047e-6 / H, 0.001e-6 / H
    ai = inv([[1 / 5.1e3 + g4, -g4, 0.0], [-g4, g4 + 1 / 30e3, -1 / 30e3], [0.0, -1 / 30e3, 1 / 30e3 + g7 + 1 / 47e3]])

    def node(r, k):                                  # v0 - z ic, z = the node's response to the collector current
        return r[0] * V16 / 5.1e3, [(p + "q4", g4 * (r[0] - r[1])), (p + "n7", r[2] * g7)] + ([("_ic", k)] if k else [])
    lin(b, "_vc0", *node(ai[0], 0))
    bjt(b, p, zb / 1e3, ze / 1e3, ai[0][0] / 1e3, True)
    lin(b, "_t", *node(ai[1], -ai[1][0] / 1e3))                                 # n4
    lin(b, p + "n7", *node(ai[2], -ai[2][0] / 1e3))
    b.op("LD", p + "c"); b.op("SUB", "_t"); b.op("ST", p + "q4")
    lin(b, p + "mb", 0.0, [("_vb0", 1.0), ("_ib", -zb / 1e3)])
    for g in gens:                                   # C_m charge from the branch current
        b.op("LD", g + "_w"); b.op("SUB", p + "mb")
        b.op("RC", g + "_qcm", imm=q(gbr_of(GEN[g][7]) * H / GEN[g][7]))


def switches(b, gens):
    """MAME: (~S1 >> 4) = a generator's code turns it on (others keep), any other value turns all four off"""
    b.op("LDLM", 0, b.src(LATCH1, 0), imm=0xF0 | (16 << 8))         # D7-D4 / 16: codes 1-4 = 0.875 .. 0.6875
    b.op("ADDI", imm=q(-0.78125)); b.op("ABS"); b.op("CMPI", imm=q(0.125))
    b.op("LDI", imm=0)
    for g in gens:
        b.op("STF", g + "_sw")
    for g in gens:
        d = ~GEN[g][9] & 15
        for k, bit in enumerate((7, 6, 5, 4)):
            b.op("BIT" if k == 0 else "FAND", 0 if (d >> (bit - 4)) & 1 else 1, b.src(LATCH1, bit))
        b.op("LDI", imm=ONE); b.op("STF", g + "_sw")


def init(b):
    """first sample after reset (silent): DC operating point"""
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for k, v in DC.items():
        b.op("LDI", imm=q(v))
        for g in GEN:
            b.op("ST", g + "_" + k)
    b.op("LDI", imm=ONE); b.op("ST", "_ini")
    for k, v in DC_MIX.items():
        b.op("LDI", imm=q(v))
        for sd in "lr":
            b.op("ST", "m" + sd + "_" + k)
    b.op("END")
    b.land(j)


def gunfight():
    b = Board("gunfight")
    init(b)
    b.op("NOISE", 0, 0); b.op("LDI", imm=q(-NOISE_A)); b.op("LDIF", imm=q(NOISE_A)); b.op("ST", "_nx")
    b.op("LDI", imm=ONE); b.op("SUB", "_side"); b.op("ST", "_side"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(False)
    for sd in "lr":                                  # left on odd samples, right on even
        gens = [g for g in GEN if GEN[g][8] == sd]
        switches(b, gens)
        for g in gens:
            gen(b, g, "m" + sd + "_mb")
        mixer(b, sd, gens)
        if sd == "l":
            j2 = jump(b)
            b.land(j)
    b.land(j2)
    # each side: master pot wiper (50 %) = n7 / 2; MAME stream 1/3, route 0.5; mono fold 0.5 (L + R)
    b.op("LD", "ml_n7"); b.op("ADD", "mr_n7"); b.op("MULI", imm=q(1.0 / 24)); b.op("OUT")
    return b.finish()
