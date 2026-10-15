"""M-4 (Midway 1977) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

The sound circuits are on the game logic board (schematic M-4 Game Logic A084-90700-D626, one sheet): 74174 F3
(port 3 = S1) / F4 (port 5 = S2) with 1k pull-ups, a 4006 / 4030 noise generator (MAME: measured 3760 Hz), and per
player (LM3900 Norton amps): shell whistle + shot bang (both on the shot bit), explosion (300k and 510k inputs), tank
motor (VCO idling / revving, gating filtered noise), "plane" drone (two shared LFOs), each side into its own summer
(volume pot R815 / R816 at 50 %) and LM377 bridge. MAME plays two samples instead.
Spec = verilator/snd/m4/m4_proto.py (float, same equations and order); circuit reference = m4_spice.py (ngspice,
National LM3900 model). Units: V, uA, Mohm, uF, s. The sides alternate on the 48 kHz samples (24 kHz each); the noise,
the tank noise filter and the LFOs run every sample. Output = 0.5 (left + right speaker) x OUT_SCALE.
Latch bits: S1 D3 sound on (7404 F2 -> summer mute diodes), D4 / D5 left / right shot + whistle, D6 / D7 left / right
explosion (300k); S2 D0 / D1 left / right explosion (510k), D2 / D3 left / right tank, D4 / D5 left / right plane.
"""
import math
from dsnd import Board, LATCH1, LATCH2, q, ONE

SR = 48000.0
DT = 1.0 / SR
D = 2.0 / SR                                     # per side
NEVER = -(1 << 31)
# LM3900 (National model, measured in ngspice: see m4_proto.py)
VN, VP, VPL, MIR, G = 0.515, 0.54, 0.43, 0.989, 315.0
VOH, VOL = 11.36, 0.58
VCH = 4.6
VNS, VPS = 0.61, 0.52
POT = 0.5
H1_0 = 5.67
OUT_SCALE = 1.0 / 32.0                           # speaker volts -> full scale
DETUNE = 0.0
ALL = ("wh", "sh", "ex", "tk", "pl")
BLOCKS = ALL


def kx(tau, dt=D):
    return 1.0 - math.exp(-dt / tau)


def zoh(A, B, dt):
    """exact one-sample discretisation (scaling and squaring), plain lists"""
    n = len(A)
    M = [[A[i][j] * dt for j in range(n)] + [B[i] * dt] for i in range(n)] + [[0.0] * (n + 1)]
    nrm = max(sum(abs(x) for x in r) for r in M)
    s = max(0, int(math.ceil(math.log2(max(nrm, 1e-30)))) + 4)
    Ms = [[x / 2 ** s for x in r] for r in M]

    def mm(a, b):
        return [[sum(a[i][k] * b[k][j] for k in range(n + 1)) for j in range(n + 1)] for i in range(n + 1)]
    E = [[1.0 if i == j else 0.0 for j in range(n + 1)] for i in range(n + 1)]
    T = [r[:] for r in E]
    for k in range(1, 20):
        T = [[x / k for x in r] for r in mm(T, Ms)]
        E = [[E[i][j] + T[i][j] for j in range(n + 1)] for i in range(n + 1)]
    for _ in range(s):
        E = mm(E, E)
    return [r[:n] for r in E[:n]], [r[n] for r in E[:n]]


# whistle
WH_C = 330e-6
WH_TH_HI = VN + 0.51 * MIR * ((12 - VP) / 1.0 + (VOH - VP) / 2.2)
WH_TH_LO = VN + 0.51 * MIR * ((12 - VP) / 1.0 + (VOL - VPL) / 2.2)
WH_IP_HI = MIR * (VOH - VP)
WH_IP_LO = MIR * (VOL - VPL)
WH_TDS = 8.7e-6 / D                              # stage-2 turn-round delay in side samples
WH_R211 = {"r": 1.5, "l": 3.3}
# shot
SH_RF = {"l": 0.15, "r": 0.12}
SH_RS = 0.68 / G
SH_RX, SH_VLO = 43.0, 1.9


def shot_mats(rf):
    R4, R2, C6, C5, C4 = 0.01, 0.015, 0.047, 0.47, 100e-6
    Ip = MIR * (12 - VP) / 0.33
    A = [[-1 / R4 / C6, -1 / R4 / C6, 0.0],
         [-1 / R4 / C5, (-1 / R4 - 1 / rf - 1 / R2) / C5, 1 / rf / C5],
         [0.0, -1 / R2 / C4, -1 / SH_RX / C4]]
    vq = R2 * Ip + rf * Ip
    Bu = [1 / R4 / C6, 1 / R4 / C5, 0.0]
    Bc = [0.0, (VN / rf + VN / R2) / C5, (Ip + VN / R2 + vq / SH_RX) / C4]
    Ad, bu = zoh(A, Bu, D)
    _, bc = zoh(A, Bc, D)
    return Ad, bu, bc


# explosion
EX_MIR = 0.96
# tank
TK_TH_HI, TK_TH_LO = 11.07, 4.20
TK_IP_HI, TK_IP_LO = MIR * (VOH - VP), 0.153
TK_RS = 1.0 / 778.0                # sink gain at mA currents (explosion-stage DC sweep: 0.874k behind 680k)
ISRC = 7300.0
_TA = [[(-1 / 0.012 - 1 / 0.068) / 0.015, 1 / 0.068 / 0.015], [1 / 0.068 / 0.015, (-1 / 0.068 - 2 / 0.33) / 0.015]]
TN_A, TN_BU = zoh(_TA, [1 / 0.012 / 0.015, 0.0], DT)
_, TN_BC = zoh(_TA, [0.0, 2 * VN / 0.33 / 0.015], DT)
# LFOs
LF_VN, LF_VP, LF_G, LF_VNS = 0.598, 0.48, 277.0, 0.55
LF_TH = LF_VNS + 3.0 * MIR * ((12 - LF_VP) + (VOH - LF_VP)) / 10
_den = 1 / 0.03 - LF_G * MIR / 10
LF_A = (1 / 0.03 - LF_G / 3.0) / _den
LF_B = (LF_G * LF_VN / 3.0 + LF_G * MIR * (12 - 2 * LF_VP) / 10) / _den
# summer
RP = POT * 0.05
SM_K = 1 + 0.1 / RP
IB_S = 0.01                                      # bus current scale in the program (uA x 0.01)


def lin(b, dst, const, terms):
    """dst = const + sum(k x reg); acc keeps the value"""
    if const == 0.0 and terms and terms[0][1] == 1.0:
        b.op("LD", terms[0][0])
        terms = terms[1:]
    else:
        b.op("LDI", imm=q(const))
    for reg, k in terms:
        b.op("MACI", reg, imm=q(k))
    if dst:
        b.op("ST", dst)


def jump(b):
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def amax(b, other):
    """acc = max(acc, M[other]) (uses _mx)"""
    b.op("ST", "_mx"); b.op("CMP", other); b.op("LD", other); b.op("LDF", "_mx")


def amin(b, other):
    """acc = min(acc, M[other])"""
    b.op("ST", "_mx"); b.op("CMP", other); b.op("LDF", other)


def shared(b):
    # noise: 4006 / 4030 replaced by the engine LFSR clocked at 3760 Hz, 0 / 12 V
    b.op("LD", "nph"); b.op("ADDI", imm=int(round(3760.0 / SR * ONE))); b.op("ST", "nph"); b.op("CMPI", imm=ONE)
    j = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "nph"); b.op("NOISE", 0, 0)
    b.op("LDI", imm=0); b.op("LDIF", imm=q(12.0)); b.op("ST", "nz")
    b.land(j)
    # tank noise filter R308 / C302 / R307 / C301 (both 330k loads)
    lin(b, "_t", TN_BC[0], [("tn1", TN_A[0][0]), ("tnx", TN_A[0][1]), ("nz", TN_BU[0])])
    lin(b, "tnx", TN_BC[1], [("tn1", TN_A[1][0]), ("tnx", TN_A[1][1]), ("nz", TN_BU[1])])
    b.op("LD", "_t"); b.op("ST", "tn1")
    # plane LFOs (R111 / C101, R101 / C100)
    for j, c in ((1, 0.33), (2, 0.33 * (1 + DETUNE))):
        lc, lh, lf = "lc%d" % j, "lh%d" % j, "lf%d" % j
        b.op("LD", lh); b.op("CMPI", imm=q(0.5))
        s1 = b.skip_if(False)
        b.op("LD", lc); b.op("CMPI", imm=q(LF_TH)); b.op("LDI", imm=0); b.op("STF", lh)
        b.land(s1)
        b.op("LD", lh); b.op("CMPI", imm=q(0.5))
        s2 = b.skip_if(True)
        b.op("LD", lc); b.op("MULI", imm=q(LF_A)); b.op("ADDI", imm=q(LF_B))
        b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", lf)
        b.op("CMP", lc); b.op("LDI", imm=ONE); b.op("STF", lh); b.op("LDI", imm=q(VOH)); b.op("STF", lf)
        s3 = jump(b)
        b.land(s2)
        b.op("LDI", imm=q(VOH)); b.op("ST", lf)
        b.land(s3)
        k = DT / c
        lin(b, lc, k * VN / 3.0, [(lc, 1 - k * (1 / 0.03 + 1 / 3.0)), (lf, k / 0.03)])


def side(b, sd):
    p = sd + "_"
    L = lambda n: p + n                                          # noqa: E731
    bsh = 4 if sd == "l" else 5
    # ---------------- whistle + shot caps (same bit)
    b.op("BIT", 0, b.src(LATCH1, bsh))
    b.op("LDI", imm=q(VP)); b.op("LDIF", imm=q(VCH)); b.op("ST", "_tg")
    b.op("LDI", imm=q(kx(2.2 * (3.3 * 0.27 / 3.57)))); b.op("LDIF", imm=q(kx(2.2e-3))); b.op("ST", "_k")
    b.op("LD", "_tg"); b.op("RCM", L("wc"), b.r("_k"))
    b.op("LDI", imm=q(kx(0.68))); b.op("LDIF", imm=q(kx(1e-3))); b.op("ST", "_k")
    b.op("LD", "_tg"); b.op("RCM", L("sc"), b.r("_k"))
    # whistle VCO (H0 / K0): noise FM through C202 / R212 / R213 / R211
    r211 = WH_R211[sd]
    al = 1 / (6 + 1 / r211)
    b.op("LD", "nz"); b.op("ADDI", imm=q(-al * VN / r211 / (1 - al))); b.op("RC", L("vc2"), imm=q(kx(0.1 / (1 - al))))
    g = D / WH_C
    c0 = (12 - VN) / 3.0 - VN / 3.3 + (al * VN / r211 - VN) / r211
    lin(b, "_su", g * (WH_IP_HI - c0), [(L("wc"), -g / 3.3), ("nz", -g * al / r211), (L("vc2"), g * al / r211)])
    b.op("ADDI", imm=q(g * (WH_IP_LO - WH_IP_HI))); b.op("ST", "_sd")
    lin(b, "_th", WH_TH_HI, [("_su", WH_TDS)])
    lin(b, "_tl", WH_TH_LO, [("_sd", WH_TDS)])
    o1, wh = L("wo1"), L("wh")
    b.op("LD", wh); b.op("CMPI", imm=q(0.5))
    jl = b.skip_if(False)
    b.op("LD", o1); b.op("ADD", "_su"); b.op("ST", o1); b.op("LDI", imm=ONE); b.op("ST", "_fh")
    b.op("LD", o1); b.op("CMP", "_th")
    jn = b.skip_if(False)
    b.op("LD", "_su"); b.op("RCP"); b.op("ST", "_r")
    b.op("LD", o1); b.op("SUB", "_th"); b.op("MUL", "_r"); b.op("ST", "_tau")
    b.op("MUL", "_sd"); b.op("ADD", "_th"); b.op("ST", o1)
    b.op("LDI", imm=ONE); b.op("SUB", "_tau"); b.op("ST", "_fh"); b.op("LDI", imm=0); b.op("ST", wh)
    b.land(jn)
    je = jump(b)
    b.land(jl)
    b.op("LD", o1); b.op("ADD", "_sd"); b.op("ST", o1); b.op("LDI", imm=0); b.op("ST", "_fh")
    b.op("LD", o1); b.op("CMP", "_tl")
    jn = b.skip_if(True)
    b.op("LD", "_sd"); b.op("RCP"); b.op("ST", "_r")
    b.op("LD", o1); b.op("SUB", "_tl"); b.op("MUL", "_r"); b.op("ST", "_tau"); b.op("ST", "_fh")
    b.op("MUL", "_su"); b.op("ADD", "_tl"); b.op("ST", o1)
    b.op("LDI", imm=ONE); b.op("ST", wh)
    b.land(jn)
    b.land(je)
    # whistle output stage (o3 for stage-2 high / low, blended by the time stage 2 was high)
    base = VN + 0.51 * (-MIR * VP / 0.27 - (12 - VN) / 2.7)
    kb = 0.51 * MIR / 0.27
    lin(b, None, base - (VOH - VN), [(L("wc"), kb)]); b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", "_oh")
    lin(b, None, base - (VOL - VN), [(L("wc"), kb)]); b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", "_ol")
    b.op("LD", "_oh"); b.op("SUB", "_ol"); b.op("MUL", "_fh"); b.op("ADD", "_ol"); b.op("ST", L("o3w"))
    # ---------------- shot bang: gated noise (stage 1, 27k pull-up) -> MFB (C406 / R414 / C405 / R411 / R412 / C404)
    x0, x1, x2 = L("x0"), L("x1"), L("x2")
    lin(b, "_st", VNS - MIR * VPS + VNS - 0.68 * (12 - VNS) / 2.7, [(L("sc"), MIR), ("nz", -1.0)])
    lin(b, None, 0.12 / 0.027, [("_st", -(0.01 / 0.027 + 1.0)), (x0, 1.0), (x1, 1.0)])
    b.op("CMPI", imm=0)
    j = b.skip_if(False)
    den = 1 + SH_RS / 0.027 + SH_RS / 0.01
    lin(b, "_st", SH_RS * 12 / 0.027 / den, [("_st", 1 / den), (x0, SH_RS / 0.01 / den), (x1, SH_RS / 0.01 / den)])
    b.land(j)
    lin(b, None, (12 - VNS) / 2.7 + (0.7 - VNS) / 0.68 - VNS / 0.68 + MIR * VPS / 0.68,
        [("nz", 1 / 0.68), (L("sc"), -MIR / 0.68)])
    b.op("MAXI", imm=q(0.6)); b.op("RCP"); b.op("MULI", imm=q(0.157)); b.op("ADDI", imm=q(0.69))
    amax(b, "_st"); b.op("MINI", imm=q(VOH)); b.op("ST", "_so1")
    Ad, bu, bc = shot_mats(SH_RF[sd])
    for i, t in enumerate(("_x0", "_x1")):
        lin(b, t, bc[i], [(x0, Ad[i][0]), (x1, Ad[i][1]), (x2, Ad[i][2]), ("_so1", bu[i])])
    lin(b, None, bc[2], [(x0, Ad[2][0]), (x1, Ad[2][1]), (x2, Ad[2][2]), ("_so1", bu[2])])
    b.op("MAXI", imm=q(SH_VLO - VN)); b.op("MINI", imm=q(VOH - VN)); b.op("ST", x2)
    b.op("LD", "_x0"); b.op("ST", x0); b.op("LD", "_x1"); b.op("ST", x1)
    # ---------------- explosion (300k C505 / 510k C506 onto one + input) -> R516 / C504 / R515 / C503 / R514
    c3, c5, f1, f2 = L("c3"), L("c5"), L("f1"), L("f2")
    lin(b, "_i3", -VPS / 0.3, [(c3, 1 / 0.3)])
    lin(b, "_i5", -VPS / 0.51, [(c5, 1 / 0.51)])
    b.op("ADD", "_i3"); b.op("ST", "_is"); b.op("CMPI", imm=1)
    j = b.skip_if(True)
    sg = 1 / 0.3 + 1 / 0.51
    w3, w5 = (1 / 0.3) / sg, (1 / 0.51) / sg
    lin(b, "_i3", 0.0, [(c3, (1 - w3) / 0.3), (c5, -w5 / 0.3)])
    b.op("MULI", imm=-ONE); b.op("ST", "_i5"); b.op("LDI", imm=0); b.op("ST", "_is")
    b.land(j)
    for c, ib_, bit in ((c3, "_i3", (LATCH1, 6 if sd == "l" else 7)), (c5, "_i5", (LATCH2, 0 if sd == "l" else 1))):
        b.op("BIT", 0, b.src(*bit))
        j = b.skip_if(False)
        b.op("LDI", imm=q(VCH)); b.op("RC", c, imm=q(kx(1e-3)))
        j2 = jump(b)
        b.land(j)
        lin(b, c, 0.0, [(c, 1.0), (ib_, -D)])
        b.land(j2)
    lin(b, "_t", VNS + VNS - 0.68 * (12 - VNS) / 2.7, [("_is", 0.68 * EX_MIR), ("nz", -1.0)])
    lin(b, "_a", 0.6, [(f1, 0.09)])
    lin(b, None, -0.942, [(f1, 0.602)])
    amax(b, "_a"); amax(b, "_t"); b.op("ST", "_t")
    lin(b, "_a", 4.09, [(f1, 1.0)]); b.op("MINI", imm=q(VOH)); b.op("ST", "_a")
    b.op("LD", "_t"); amin(b, "_a"); b.op("ST", "_eo")
    lin(b, f1, 0.0, [(f1, 1 - D / 0.00056 - D / 0.0068), ("_eo", D / 0.00056), (f2, D / 0.0068)])
    lin(b, "_i14", -IB_S * VN / 0.002, [(f2, IB_S / 0.002), (L("c801"), -IB_S / 0.002)])
    lin(b, f2, 0.0, [(f2, 1 - D / 0.0068), (f1, D / 0.0068), ("_i14", -D / IB_S)])
    # ---------------- tank motor: C304 (R316 / R314 + D300), VCO H1 8-9 / 6-5, stage 3 into C309, R301 / C300 / R300
    tc, to1, th, o3, c300 = L("tc"), L("to1"), L("th"), L("t3"), L("t300")
    b.op("BIT", 0, b.src(LATCH2, 2 if sd == "l" else 3))
    j = b.skip_if(False)
    b.op("LDI", imm=q(5.0)); b.op("RC", tc, imm=q(kx(7.8e-3 * 2.2)))
    ja = jump(b)
    b.land(j)
    b.op("LD", tc); b.op("CMPI", imm=q(0.65))
    j = b.skip_if(False)
    b.op("LDI", imm=q(0.6)); b.op("RC", tc, imm=q(kx(0.25e-3)))
    jb = jump(b)
    b.land(j)
    b.op("LDI", imm=q(0.1)); b.op("RC", tc, imm=q(kx(6.8e-3 * 2.2)))
    b.land(jb)
    b.land(ja)
    gk = D / 0.022
    b.op("LD", th); b.op("MULI", imm=q(gk * (TK_IP_HI - TK_IP_LO))); b.op("ST", "_t")
    lin(b, to1, gk * (TK_IP_LO - (12 - VN) / 10 + VN / 2.4), [(to1, 1.0), (tc, -gk / 2.4), ("_t", 1.0)])
    b.op("LD", th); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(False)
    b.op("LD", to1); b.op("CMPI", imm=q(TK_TH_HI)); b.op("LDI", imm=0); b.op("STF", th)
    jc = jump(b)
    b.land(j)
    b.op("LD", to1); b.op("CMPI", imm=q(TK_TH_LO + 1e-6)); b.op("LDI", imm=ONE); b.op("STNF", th)
    b.land(jc)
    lin(b, "_t", VN + VN / 0.33 - (12 - VN) / 2.2, [(th, MIR * (VOH - VP) / 0.56), ("tnx", -1 / 0.33)])
    lin(b, "_io", -IB_S * VN / 0.069, [(o3, IB_S / 0.069), (c300, -IB_S / 0.069), (L("c801"), -IB_S / 0.069)])
    b.op("LD", "_t"); b.op("CMP", o3)
    j = b.skip_if(False)
    lin(b, "_a", D * (ISRC + 12 / 0.027) / 2.2, [(o3, 1 - D / 0.027 / 2.2), ("_io", -D / IB_S / 2.2)])
    b.op("MINI", imm=q(VOH)); b.op("ST", "_a"); b.op("LD", "_t"); amin(b, "_a"); b.op("ST", o3)
    jd = jump(b)
    b.land(j)
    lin(b, None, D * 12 / 0.027 / 2.2, [(o3, 1 - D / 2.2 * (1 / 0.027 + 1 / TK_RS)), ("_t", D / TK_RS / 2.2),
                                        ("_io", -D / IB_S / 2.2)])
    b.op("MAXI", imm=q(VOL)); b.op("ST", o3)
    b.land(jd)
    lin(b, c300, 0.0, [(c300, 1.0), ("_io", D / IB_S)])
    # ---------------- plane drone amp (F0 4 / 10): LFO1 + LFO2 + enable through R118 / R108 100k
    b.op("BIT", 0, b.src(LATCH2, 4 if sd == "l" else 5))
    b.op("LDI", imm=0); b.op("LDIF", imm=q(0.22 * MIR * (5.0 - VP) / 0.101)); b.op("ST", "_t")
    lin(b, None, VN - 0.22 * (12 - VN) / 1.5 + 2 * 0.22 * VN / 0.47, [("_t", 1.0), ("lf1", -0.22 / 0.47), ("lf2", -0.22 / 0.47)])
    b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("RC", L("ao"), imm=q(kx(0.22e-3)))
    # ---------------- summer (bus -> C801 -> K1, R800 || R815 + C800), LM377 bridge = 4 x HP(o1)
    c801 = L("c801")
    ks = IB_S
    terms = [(L("o3w"), ks / 0.15), (x2, ks / 0.2), ("_i14", 1.0), ("_io", 1.0), (L("ao"), ks / 0.068),
             (c801, -ks * (1 / 0.15 + 1 / 0.2 + 1 / 0.068))]
    const = ks * (-VN / 0.15 + VN / 0.2 - VN / 0.2 - VN / 0.068)
    if BLOCKS != ALL:                                            # test builds only: isolate sounds
        keep = {"wh": [0], "sh": [1], "ex": [2], "tk": [3], "pl": [4]}
        idx = [i for k in BLOCKS for i in keep[k]]
        g = [ks / 0.15, ks / 0.2, 0.0, 0.0, ks / 0.068]
        terms = [terms[i] for i in idx] + [(c801, -sum(g[i] for i in idx))]
        const = -VN * sum(g[i] for i in idx if i in (0, 4))
    lin(b, "_ib", const, terms)
    lin(b, None, VN + 0.1 * MIR * (12 - VP) / 0.22 / SM_K, [("_ib", -0.1 / ks / SM_K), (L("c800"), 0.1 / RP / SM_K)])
    b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", "_o1")
    lin(b, L("c800"), -D * VN / RP, [(L("c800"), 1 - D / RP), ("_o1", D / RP)])
    lin(b, c801, 0.0, [(c801, 1.0), ("_ib", D / ks)])
    b.op("LD", "_o1"); b.op("RC", L("h1"), imm=q(kx(0.1)))
    b.op("LD", "_o1"); b.op("SUB", L("h1")); b.op("MULI", imm=q(4.0 * 0.5 * OUT_SCALE))
    b.op("BIT", 1, b.src(LATCH1, 3)); b.op("LDIF", imm=0)              # S1 D3 low: muted
    b.op("ST", L("spk"))


def m4():
    b = Board("m4")
    # first sample after reset (silent): power-up state. Own block ending in END, so it is not on the body's path
    init = [("tn1", 6.0), ("tnx", 6.0), ("lc1", 5.5), ("lc2", 5.5), ("lh1", 1.0), ("lh2", 1.0)]
    for sd in "lr":
        init += [(sd + "_" + reg, v) for reg, v in (
            ("wc", VP), ("sc", VP), ("vc2", 6.0), ("wo1", WH_TH_LO), ("wh", 1.0), ("x0", 0.0), ("x1", VN),
            ("x2", 5.7 - VN), ("c3", 0.1), ("c5", 0.1), ("f1", VOL), ("f2", VOL), ("tc", 0.1), ("to1", TK_TH_LO),
            ("th", 1.0), ("t3", VOL), ("ao", VOL), ("c800", 5.2), ("h1", H1_0))]
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for reg, v in init:
        b.op("LDI", imm=q(v)); b.op("ST", reg)
    b.op("LDI", imm=ONE); b.op("ST", "_ini")
    b.op("END")
    b.land(j)
    shared(b)
    b.op("LDI", imm=ONE); b.op("SUB", "_side"); b.op("ST", "_side"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(False)
    side(b, "l")
    j2 = jump(b)
    b.land(j)
    side(b, "r")
    b.land(j2)
    b.op("LD", "l_spk"); b.op("ADD", "r_spk"); b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    pr = m4()
    print(f"m4: {len(pr.code)} words, worst {pr.worst_path()} units, {len(pr.regs)} registers")
    if len(sys.argv) > 2:                        # test build: only the listed sounds reach the summers
        BLOCKS = tuple(sys.argv[2].split(","))
        pr = m4()
    if len(sys.argv) > 1:
        open(sys.argv[1], "wb").write(pr.binary())
