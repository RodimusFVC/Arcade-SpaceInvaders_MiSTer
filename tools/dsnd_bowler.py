"""Bowling Alley (Midway 1978) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

From the schematic (Useful Information/schematics/Bowling_Alley_Table_Game_Logic_(A084-90700-A749).pdf, one sheet,
lower third), settled with ngspice (verilator/snd/bowler/bw_spice.py). MAME models the foul only (as a 180 Hz square).
Latches / strobes (S3 / S4 need the shared RTL routing, see ROUTING):
  S1 = port 5 (K5): D2 GAME ON (P52; high = sound on, see below), D3 foul (P53), D5 strike chime envelope (P55)
  S2 = port 6 (K4): D0-D2 rolling integrator currents (P60 22k / P61 62k / P62 33k), D3 ball-return rumble (P63),
       D4 foul-line burst (P64, ~3 ms pulse per roll), D5 spare chime envelope (P65)
  S3 = port 15 (M5): D1 / D2 chime 1 pitch (68k / diode + 56k), D4 / D5 chime 2 pitch (91k / diode + 47k)
  S4 = strobe toggles: bit 0 port 8 (L2 one-shot, ball hits pin), bit 1 port 9 (L2, pin hits pin), bit 2 port 10
       (L1 one-shot, noise burst); each write toggles its bit
Sounds: foul (64V "120 Hz" TTL tone through a Norton VCA, edge pin V of the mother board), ball rolling (integrator +
peak hold gating 4006 noise, 199 Hz resonant MFB), ball / pin hits pin (two-integrator rings 1252 / 719 Hz, ~10 ms),
noise burst (CA3080 T3: noise 2 x 60 Hz low pass, differenced), chimes (two triangle VCOs + Schmitts -> mixer ->
3.9 kHz MFB -> CA3080 R4 gated by the strike / spare envelopes), summer N2 (R209 100k || R215 50k pot at 50 % + C205)
-> LM377 bridge (4 x s1, 16 Hz high passes); muted while D2 is low (common mode in the bridge: no thump).
GAME ON: the sheet draws K5 pin 10 straight to the summer's D201 / D200 (which would mute while P52 is high); the game
holds D2 = 1 in play (MAME capture), as MAME's mute reads it, and Phantom II's identical summer has a 7404 there.
"""
from dsnd import Board, LATCH1, LATCH2, LATCH3, LATCH4, q, ONE, FS, rc_exp
import numpy as np

K, M_, U = 1e3, 1e6, 1e-6
VBE, VOH, VP = 0.5, 10.5, 12.0
IB = VP - VBE                                        # current into an input from a resistor to +12 V, x R
VNSAT = 0.11                                         # - input pulled down by the mirror while I+ > I- (saturated high)
TTL_LO = 0.2 + 20 * 4.8 / 1020                       # latch low with the 1k pull-up
NOISE_HZ = 1183.0                                    # 4030 R56 56k / C30 6800 pF (ngspice: verilator/snd/bowler/bw_osc.cir)
FRAME = 15600.0 / 262                                # 64V: high on lines [32, 96) and [160, 262) of 262
TW_L2 = 0.25 * 47 * 100000 * (1 + 0.7 / 47) * 1e-9  # 74123 + diode: 0.25 Rt C (1 + 0.7 / Rt) = 1.193 ms
TW_L1 = 0.25 * 47 * 4.7e6 * (1 + 0.7 / 47) * 1e-9   # C1001 4.7 uF (label faded) = 56.05 ms
VD = 0.463                                           # 1N4148 at ~70 uA (model IS 2.52n, N 1.752)
VPIN = 0.555                                         # Norton + input at ~70 uA
OUT_SCALE = 1.0 / 24.0                               # speaker volts -> full scale (4 x the summer's 5.7 V swing)
R215 = 25 * K                                        # volume pot R215 50k at 50 %
RAC = 1.0 / (1 / (100 * K) + 1 / R215)               # summer AC transresistance (C205 a short above 6 Hz)
S1DC = VBE + 100 * K * IB / (220 * K)                # summer output at rest (R207 bias)
ROUTING = """port 15 write -> sound latch 3 (data); ports 8 / 9 / 10 write -> toggle sound latch 4 bit 0 / 1 / 2"""


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


def mexp_zoh(A, B, h):
    n, m = A.shape[0], B.shape[1]
    Mx = np.zeros((n + m, n + m))
    Mx[:n, :n], Mx[:n, n:] = A * h, B * h
    E, term = np.eye(n + m), np.eye(n + m)
    for k in range(1, 60):
        term = term @ Mx / k
        E = E + term
    return E[:n, :n], E[:n, n:]


def bitv(b, src, bit):
    """acc = source bit as 0 / 1.0"""
    b.op("LDLM", 0, b.src(src, 0), imm=(1 << bit) | ((24 - bit) << 8))


def coupled(b, name, v, r, c, vsrc_off=0.0):
    """series R + C into the summer's virtual ground (VBE): _i<name> = volts across R (x 1 / r = current);
    the cap follows"""
    b.kreg(name + "_k", 1.0 / (r * c * FS), "F", clamp=False)
    b.op("LD", v)
    b.op("SUB", name + "_c")
    b.op("ADDI", imm=q(-VBE - vsrc_off))
    b.op("ST", name + "_v")
    b.op("MUL", name + "_k")
    b.op("ADD", name + "_c")
    b.op("ST", name + "_c")


def oneshot(b, name, bit, tw, level=False):
    """74123 retriggered by a toggle of S4 bit: F = Q, or (level) name_q = Q as 0 / 1.0 (ramps down over the last 10 us)"""
    b.kreg("os_dt", 1000.0 / FS, "T", clamp=False)          # timer in ms
    bitv(b, LATCH4, bit)
    b.op("ST", "_b")
    b.op("SUB", name + "_pv")
    b.op("ABS")
    b.op("CMPI", imm=q(0.5))                       # F = toggled
    b.op("LD", "_b"); b.op("ST", name + "_pv")
    b.op("LD", name + "_t"); b.op("SUB", "os_dt")
    b.op("LDIF", imm=q(tw * 1000.0))
    b.op("MAXI", imm=0)
    b.op("ST", name + "_t")
    if level:
        b.op("MULI", imm=q(100.0)); b.op("MINI", imm=ONE); b.op("ST", name + "_q")
    else:
        b.op("CMPI", imm=1)                         # F = Q


def noise(b):
    """4030 RC clock -> 4006 (engine LFSR 0 stands in): nz = 0 / 12 V"""
    b.kreg("nz_inc", NOISE_HZ / FS, "O", clamp=False)
    b.op("LD", "nz_ph"); b.op("ADD", "nz_inc"); b.op("ST", "nz_ph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "nz_ph")
    b.op("NOISE", 0, 0)
    b.op("LDI", imm=0); b.op("LDIF", imm=q(VP)); b.op("ST", "nz")
    b.land(at)


def foul(b):
    """P3 13/8 -> 9: P53 -> D1118 -> C1050 0.1 uF (charged through the 1k pull-up) -> R1101 300k (+);
    64V (TTL) via R1102 680k, R1103 2.7M bias, R1104 680k feedback (-); out (R1105 27k pull-up) -> R1120 68k / C1048"""
    # 64V phase (frame fraction) and level: high on [32, 96) and [160, 262) / 262; fv = fout constant part
    b.op("LD", "v_ph"); b.op("ADDI", imm=q(FRAME / FS))
    b.op("CMPI", imm=ONE); b.op("ADDI", imm=-ONE); b.op("STF", "v_ph"); b.op("ADDI", imm=ONE); b.op("STNF", "v_ph")
    b.op("LD", "v_ph")
    base = VBE - 680 * K * IB / (2.7 * M_)
    hi, lo = q(base - (3.4 - VBE)), q(base - (0.2 - VBE))
    b.op("CMPI", imm=q(160 / 262)); j1 = b.skip_if(True)
    b.op("CMPI", imm=q(96 / 262)); j2 = b.skip_if(True)
    b.op("CMPI", imm=q(32 / 262)); j3 = b.skip_if(True)
    b.land(j2)
    b.op("LDI", imm=lo); j4 = b.jump()
    b.land(j1); b.land(j3)
    b.op("LDI", imm=hi)
    b.land(j4)
    b.op("ST", "_fv")
    # C1050: P53 high -> charged (1k, 0.1 ms) to 4.6 V; low -> into the + input through R1101 (30 ms)
    b.kreg("fc_kc", ktau(1 * K * 0.1 * U), "T")
    b.kreg("fc_kd", ktau(300 * K * 0.1 * U), "T")
    b.op("BIT", 0, b.src(LATCH1, 3))
    b.op("LD", "fc_kd"); b.op("LDF", "fc_kc"); b.op("ST", "_k")
    b.op("LDI", imm=q(VBE)); b.op("LDIF", imm=q(4.6))
    b.op("RCM", "c1050", b.r("_k"))
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(680 / 300)); b.op("ADD", "_fv")
    b.op("MAXI", imm=0); b.op("ST", "fout")                                # active: 4.0 .. 7.2 V
    coupled(b, "fb", "fout", 68 * K, 0.1 * U)


def roll(b):
    """P3 12/11 -> 10 integrator (P60 22k, P61 62k, P62 33k into +, R1029 1M bias, C1042 1 uF) -> D1014 -> C1044 1 uF
    -> R1022 680k; P63 -> R1016 330k / C1043 1 uF -> R1021 330k; P3 1/6 -> 5 noise mixer (R1017 680k noise, R1018 2.7M,
    R1020 680k, R1019 27k pull-up) -> MFB low pass (R1023 82k, C1040 0.47 uF, R1024 22k, R1025 62k, C1045 1 nF,
    R1026 200k): resonant 199 Hz -> R1027 56k / C1041 0.1 uF"""
    ws, lo = [], 0.0
    for r in (22.0, 62.0, 33.0):                       # k, mA
        vh = (5.0 * r + VBE * 1.0) / (r + 1.0)         # 1k pull-up loaded by r
        ws.append((vh - VBE) / r - (TTL_LO - VBE) / r)
        lo += (TTL_LO - VBE) / r
    for n, w in enumerate(ws):
        bitv(b, LATCH2, n)
        b.op("MULI", imm=q(w))
        if n:
            b.op("ADD", "_t")
        b.op("ST", "_t")
    b.op("ADDI", imm=q(lo)); b.op("MAXI", imm=0)       # I+ (mA)
    b.kreg("ri_k", 1e-3 / (1 * U * FS), "T", clamp=False)
    b.op("ADDI", imm=q(-IB / 1000.0)); b.op("MUL", "ri_k"); b.op("ADD", "rint")
    # at the rail the mirror pulls -IN down to ~0.11 V: C1042 takes VBE - 0.11 more, so the fall starts that late
    clamp(b, "rint", 0.0, VOH + VBE - VNSAT)
    # D1014 peak hold, C1044 into the mixer + input through R1022 680k
    b.kreg("r44_k", ktau(680 * K * 1 * U), "T")
    b.op("LDI", imm=q(VBE)); b.op("RCM", "c1044", b.r("r44_k"))
    b.op("LD", "rint"); b.op("MINI", imm=q(VOH)); b.op("ADDI", imm=q(-0.4)); b.op("CMP", "c1044"); b.op("STF", "c1044")
    # C1043: toward (P63 + VBE) / 2 through 330k || 330k
    b.kreg("r43_k", ktau(165 * K * 1 * U), "T")
    b.op("BIT", 0, b.src(LATCH2, 3))
    b.op("LDI", imm=q((TTL_LO + VBE) / 2)); b.op("LDIF", imm=q((4.99 + VBE) / 2))
    b.op("RCM", "c1043", b.r("r43_k"))
    # mixer: VBE + 680k (I+ - I-)
    b.op("LD", "c1044"); b.op("MACI", "c1043", imm=q(680 / 330))
    b.op("ADDI", imm=q(-VBE * (1 + 680 / 330))); b.op("MAXI", imm=0)
    b.op("ADDI", imm=q(2 * VBE - 680 * K * IB / (2.7 * M_)))
    b.op("SUB", "nz")
    clamp(b, "rmix")
    # MFB: C1 dVm = (u - Vm) / R1 + (y - Vm) / R2 - (Vm - VBE) / R3 ; C2 dy = I+ - (Vm - VBE) / R3, I+ = IB / R1026
    mfb(b, "rmix", "rm", "rflt", 82 * K, 22 * K, 62 * K, 0.47 * U, 1e-9, IB / (200 * K))
    coupled(b, "rb", "rflt", 56 * K, 0.1 * U)       # the onset step of rflt's mean is shaped by this 28 Hz HP


def mfb(b, u, m, y, R1, R2, R3, C1, C2, ip):
    A = np.array([[-(1 / R1 + 1 / R2 + 1 / R3) / C1, 1 / (R2 * C1)], [-1 / (R3 * C2), 0.0]])
    Bm = np.array([[1 / (R1 * C1), VBE / (R3 * C1)], [0.0, (ip + VBE / R3) / C2]])      # inputs u, 1
    Ad, Bd = mexp_zoh(A, Bm, 1.0 / FS)
    lin(b, "_t1", Bd[0][1], [(m, Ad[0][0]), (y, Ad[0][1]), (u, Bd[0][0])])
    lin(b, y, Bd[1][1], [(m, Ad[1][0]), (y, Ad[1][1]), (u, Bd[1][0])])
    b.op("LD", "_t1"); b.op("ST", m)


def ring(b, p, bit, r_in, r_b, r_f, c1, r_i, c2, r_bb, r_fb):
    """two-integrator ring: L2 one-shot (S4 bit) -> 7417 (1k to +12) -> r_in into stage A -; stage A: r_b bias,
    r_f || c1 feedback, + from stage B through r_fb; stage B: - from A through r_i, c2 feedback, + r_bb bias.
    States are the outputs (p1, p2); clamped 0 .. VOH"""
    oneshot(b, p, bit, TW_L2, level=True)
    A = np.array([[-1 / (r_f * c1), 1 / (r_fb * c1)], [-1 / (r_i * c2), 0.0]])
    # inputs: Q (7417 output 0.1 -> 11.95 V), 1
    uq = (11.95 - 0.1) / r_in
    c_a = (-VBE / r_fb - (0.1 - VBE) / r_in - IB / r_b + VBE / r_f) / c1
    c_b = (IB / r_bb + VBE / r_i) / c2
    Bm = np.array([[-uq / c1, c_a], [0.0, c_b]])
    Ad, Bd = mexp_zoh(A, Bm, 1.0 / FS)
    lin(b, "_t1", Bd[0][1], [(p + "1", Ad[0][0]), (p + "2", Ad[0][1]), (p + "_q", Bd[0][0])])
    lin(b, None, Bd[1][1], [(p + "1", Ad[1][0]), (p + "2", Ad[1][1])])
    b.op("MINI", imm=q(VOH)); b.op("ST", p + "2")
    b.op("LD", "_t1"); b.op("ST", p + "1")
    return Ad, Bd


def rings(b):
    """ball hits pin (N4 6/1 -> 5, 3/2 -> 4: R1035 220k, R1033 100k, R1034 1.5M, C1010 3.3 nF, R1036 47k, C1011 2.2 nF,
    R1037 100k, R1038 47k) and pin hits pin (N4 8/13 -> 9, 11/12 -> 10: R1040 220k, R1041 100k, R1042 560k, C1013
    10 nF, R1043 47k, C1014 2.2 nF, R1044 100k, R1045 47k); both -> 33k / 0.1 uF (identical: one coupling state)"""
    ring(b, "bh", 0, 220 * K, 100 * K, 1.5 * M_, 3.3e-9, 47 * K, 2.2e-9, 100 * K, 47 * K)
    ring(b, "ph", 1, 220 * K, 100 * K, 560 * K, 10e-9, 47 * K, 2.2e-9, 100 * K, 47 * K)
    b.op("LD", "bh2"); b.op("ADD", "ph2"); b.op("ST", "_pp")
    coupled(b, "pb", "_pp", 33 * K, 0.1 * U, vsrc_off=VBE)          # two branches: 2 VBE, sum of cap voltages


SOFT_A = 1.81                                         # minimax cubic for tanh (max error 0.060)


def softclip(b, scale):
    """acc = tanh(acc x scale) by the cubic 1.5 y - 0.5 y^3, y = x / SOFT_A clamped to +-1"""
    b.op("MULI", imm=q(scale / SOFT_A))
    b.op("MAXI", imm=-ONE); b.op("MINI", imm=ONE)
    b.op("ST", "_x")
    b.op("MUL", "_x"); b.op("MUL", "_x"); b.op("MULI", imm=q(-0.5)); b.op("MACI", "_x", imm=q(1.5))


def crash(b):
    """T3 CA3080: L1 one-shot (port 10, 56 ms) -> 7417 -> D1007 / C1008 2.2 uF -> R1014 68k; P64 -> D1008 / C1009 3.3 uF
    -> R1015 22k -> Iabc; noise -> R1005 5.6k / C1038 0.47 uF -> R1006 5.6k / C1039 0.47 uF -> R1009 1k (-),
    R1010 1k / C1006 0.1 uF (+); out: R1011 / R1013 1M bias, R1012 1k / C1007 1 uF into the summer: the OTA current
    goes into the virtual ground (500k bias vs 1k: a 0.3 Hz high pass, left out)"""
    oneshot(b, "l1", 2, TW_L1)
    b.kreg("c8_kc", ktau(1 * K * 2.2 * U), "T")
    b.kreg("c8_kd", ktau(68 * K * 2.2 * U), "T")
    b.op("LD", "c8_kd"); b.op("LDF", "c8_kc"); b.op("ST", "_k")
    b.op("LDI", imm=q(0.55)); b.op("LDIF", imm=q(11.34))
    b.op("RCM", "c1008", b.r("_k"))
    b.kreg("c9_kc", ktau(50 * 3.3 * U), "T")
    b.kreg("c9_kd", ktau(22 * K * 3.3 * U), "T")
    b.op("BIT", 0, b.src(LATCH2, 4))
    b.op("LD", "c9_kd"); b.op("LDF", "c9_kc"); b.op("ST", "_k")
    b.op("LDI", imm=q(0.55)); b.op("LDIF", imm=q(2.89))
    b.op("RCM", "c1009", b.r("_k"))
    b.op("ADDI", imm=q(-0.55)); b.op("MAXI", imm=0); b.op("MULI", imm=q(1 / 22.0)); b.op("ST", "_a")
    b.op("LD", "c1008"); b.op("ADDI", imm=q(-0.55)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(1 / 68.0)); b.op("ADD", "_a"); b.op("ST", "_a")       # Iabc (mA)
    # noise network, states n1, n2, t3p (exact ZOH, input nz)
    R5, R6, R10, C38, C39, C6 = 5.6 * K, 5.6 * K, 1 * K, 0.47 * U, 0.47 * U, 0.1 * U
    A = np.array([[-(1 / R5 + 1 / R6) / C38, 1 / (R6 * C38), 0.0],
                  [1 / (R6 * C39), -(1 / R6 + 1 / R10) / C39, 1 / (R10 * C39)],
                  [0.0, 1 / (R10 * C6), -1 / (R10 * C6)]])
    Bm = np.array([[1 / (R5 * C38)], [0.0], [0.0]])
    Ad, Bd = mexp_zoh(A, Bm, 1.0 / FS)
    st = ("n1", "n2", "t3p")
    for i, s in enumerate(st):
        terms = [(st[j], Ad[i][j]) for j in range(3) if abs(Ad[i][j]) > 1e-9] + \
                ([("nz", Bd[i][0])] if abs(Bd[i][0]) > 1e-9 else [])
        lin(b, "_n%d" % i if i < 2 else s, 0.0, terms)
    for i in range(2):
        b.op("LD", "_n%d" % i); b.op("ST", st[i])
    # OTA: Iout = Iabc tanh((t3p - n2) / 2 Vt) into the summer; the 1M / 1M bias pulls C1007 toward 5.5 V (501k)
    b.op("LD", "t3p"); b.op("SUB", "n2")
    softclip(b, 1 / 0.0517)
    b.op("MUL", "_a"); b.op("ST", "_i3")          # mA; the 1M / 1M bias + C1007 only add a 0.3 Hz high pass


def chime_cv(b, out, bits):
    """P5 Norton: + from M5 bits (k, mA weights), R1071 / R1073 1M bias, 27k || 0.1 uF feedback (2.7 ms):
    cv - VBE = 27k (I+ - IB / 1M) >= 0; out = the VCO ramp per sample (cv - VBE) / (220k C), lagged 2.7 ms"""
    lo = sum(il for _, _, il in bits)
    for n, (bit, ih, il) in enumerate(bits):
        bitv(b, LATCH3, bit)
        b.op("MULI", imm=q(ih - il))
        if n:
            b.op("ADD", "_t")
        b.op("ST", "_t")
    b.op("ADDI", imm=q(lo - IB / 1000.0)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(27.0))
    b.kreg("vc_k", 1.0 / (220 * K * 270e-12 * FS), "O", clamp=False)
    b.op("MUL", "vc_k")
    b.kreg("cv_k", ktau(27 * K * 0.1 * U), "T")
    b.op("RCM", out, b.r("cv_k"))


TH_HI = VPIN + 100 * K * (IB / (120 * K) + VPIN / (200 * K))           # Schmitt T4: R1089 100k, R1090 200k, R1091 120k
TH_LO = VPIN + 100 * K * (IB / (120 * K) - (VOH + 0.07 - VPIN) / (200 * K))
D_TURN = 1.6               # Schmitt slew + 4016 switching delay, samples (33 us): ngspice periods of the four chime tones
                           # give 1.35 .. 1.74 samples against this model; 1.6 puts all four within 1 %


def vco(b, p, cv):
    """P5 triangle VCO (R 220k -, 110k +, 270 pF, 4016 to ground while the Schmitt is high) + T4 Schmitt:
    the triangle ramps +-r = (cv - VBE) / (220k C) between TH_LO and TH_HI, turning down D_TURN after the TH_HI
    crossing while it sits at VOH (L = TH_HI + r D_TURN > VOH for any chime pitch): period
    P = (TH_HI - TH_LO + VOH - TH_LO) / r + D_TURN samples, sq = VOH for the (VOH - TH_LO) / r descent. Only the
    square is heard (mixer), so it runs as a phase accumulator: phase += r / (A + D_TURN r), high while
    phase < (VOH - TH_LO) / (A + D_TURN r)"""
    a = (TH_HI - TH_LO) + (VOH - TH_LO)
    b.op("LD", cv)
    b.op("MULI", imm=q(D_TURN)); b.op("ADDI", imm=q(a)); b.op("RCP"); b.op("ST", "_inv")
    b.op("MUL", cv); b.op("ADD", p + "ph")
    b.op("CMPI", imm=ONE); b.op("ADDI", imm=-ONE); b.op("STF", p + "ph"); b.op("ADDI", imm=ONE); b.op("STNF", p + "ph")
    b.op("LD", "_inv"); b.op("MULI", imm=q(VOH - TH_LO)); b.op("CMP", p + "ph")       # F = phase <= high part
    b.op("LDI", imm=0); b.op("LDIF", imm=q(VOH)); b.op("ST", p + "sq")


def chime(b):
    """M5 -> cv1 / cv2 -> VCO + Schmitt x 2 -> T4 10 mixer (R1074 1.5M, R1075 3M, R1076 680k, R1077 1.2M) -> MFB
    (R1078 560k, C1029 1 nF, R1079 82k, R1080 20k, C1028 1 nF, R1081 150k) -> R4 CA3080 (1k / 1k + C1031 2.2 uF);
    Iabc: P55 / P65 -> 7417 -> D1012 / D1013 -> C1036 / C1037 2.2 uF -> R1099 / R1100 100k; out R1084 / R1085 1M ->
    C1032 1 uF -> R1086 33k"""
    def hi(r, diode):
        vh = (5.0 * (r + 0) + VPIN * 1.0) / (r + 1.0)
        return (vh - (VD if diode else 0.0) - VPIN) / r
    chime_cv(b, "cv1", [(1, hi(68.0, False), (TTL_LO - VPIN) / 68.0), (2, hi(56.0, True), 0.0)])
    chime_cv(b, "cv2", [(4, hi(91.0, False), (TTL_LO - VPIN) / 91.0), (5, hi(47.0, True), 0.0)])
    vco(b, "c1", "cv1")
    vco(b, "c2", "cv2")
    lin(b, None, VBE + 680 * (IB / 1200.0 + VBE / 1500.0 + VBE / 3000.0),
        [("c1sq", -680 / 1500.0), ("c2sq", -680 / 3000.0)])
    clamp(b, "cmix")
    mfb(b, "cmix", "cm", "cflt", 560 * K, 82 * K, 20 * K, 1e-9, 1e-9, IB / (150 * K))
    # envelope: C1036 (P55 strike) and C1037 (P65 spare) are identical (1k / 2.2 uF / 100k) and never overlap in
    # play: one cap charged by either (an overlap would give the hardware twice the Iabc)
    b.kreg("ce_kc", ktau(1 * K * 2.2 * U), "T")
    b.kreg("ce_kd", ktau(100 * K * 2.2 * U), "T")
    b.op("BIT", 0, b.src(LATCH1, 5)); b.op("FOR", 0, b.src(LATCH2, 5))
    b.op("LD", "ce_kd"); b.op("LDF", "ce_kc"); b.op("ST", "_k")
    b.op("LDI", imm=q(0.55)); b.op("LDIF", imm=q(11.4))
    b.op("RCM", "c1036", b.r("_k"))
    b.op("ADDI", imm=q(-0.55)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(1 / 100.0)); b.op("ST", "_a")                     # Iabc (mA)
    # OTA R4: + = cflt low-passed (1k / 2.2 uF), - = cflt; mostly saturated (+-0.4 V vs 52 mV)
    b.kreg("r4_k", ktau(1 * K * 2.2 * U), "F")
    b.op("LD", "cflt"); b.op("RCM", "r4p", b.r("r4_k"))
    b.op("SUB", "cflt")
    softclip(b, 1 / 0.0517)
    b.op("MUL", "_a"); b.op("ST", "_io")
    # output: R1084 / R1085 1M bias (500k) || C1032 1 uF + R1086 33k into the virtual ground: 500 / 533 of the
    # current reaches the summer (the 0.3 Hz high pass is left out); swing 6 +- 3.3 V, inside the OTA compliance


def summer(b):
    """N2: bus currents into -IN (virtual ground at VBE); s1 = S1DC - RAC i_bus; P52 low: muted (s1 = 0);
    speaker = LM377 bridge, 4 x HP16(s1)"""
    import os
    dbg = os.environ.get("BW_DBG")                   # verification only: output a register / 16 instead
    if dbg:
        b.op("LD", dbg); b.op("MULI", imm=q(1 / 16.0)); b.op("OUT")
        return
    lin(b, None, S1DC, [("fb_v", -RAC / (68 * K)), ("rb_v", -RAC / (56 * K)), ("pb_v", -RAC / (33 * K)),
                        ("_i3", -RAC / K), ("_io", -RAC / K * 500.0 / 533.0)])
    clamp(b, "s1")
    # mute: GAME ON low pulls both N2 stages (D201 / D200) low together, a common-mode step for the LM377 bridge:
    # silent, no thump; s1 held at rest
    b.op("BIT", 1, b.src(LATCH1, 2))
    b.op("LDI", imm=q(S1DC)); b.op("STF", "s1")
    b.crfilter("h1", "s1", 100 * K, 0.1 * U)
    b.op("LD", "h1"); b.op("MULI", imm=q(4 * OUT_SCALE)); b.op("OUT")


# power-up operating point (states that rest away from 0)
DC = {"rm": None, "rflt": None, "bh1": None, "bh2": None, "ph1": None, "ph2": None, "cm": None, "cflt": None,
      "n1": 6.0, "n2": 6.0, "t3p": 6.0, "r4p": None, "h1_c": S1DC,
      "c1050": VBE, "c1044": VBE, "c1043": (TTL_LO + VBE) / 2, "c1008": 0.55, "c1009": 0.55}


def dc_points():
    """rest values of the linear blocks (solve A x + B = 0)"""
    d = {}
    # roll MFB, input rmix at rest: VBE - 680k IB / 2.7M - (nz - VBE) clamped: noise averages 6 V -> 0; rest input 0
    def mfb_dc(R1, R2, R3, ip, u):
        # Vm: (u - Vm) / R1 + (y - Vm) / R2 = (Vm - VBE) / R3 = ip
        vm = VBE + R3 * ip
        y = vm + R2 * (ip - (u - vm) / R1)
        return vm, y
    d["rm"], d["rflt"] = mfb_dc(82 * K, 22 * K, 62 * K, IB / (200 * K), 0.0)
    d["cm"], d["cflt"] = mfb_dc(560 * K, 82 * K, 20 * K, IB / (150 * K), VBE + 680 * (IB / 1200.0 + VBE / 1500.0 + VBE / 3000.0))
    d["r4p"] = d["cflt"]
    for p, rf in (("bh", 1.5 * M_), ("ph", 560 * K)):
        b1 = VBE + 47 * K * IB / (100 * K)
        b2 = VBE + 47 * K * (IB / (100 * K) + (b1 - VBE) / rf + (0.1 - VBE) / (220 * K))
        d[p + "1"], d[p + "2"] = b1, b2
    return d


def bowler():
    b = Board("bowler_hw")
    kreg0, seen = b.kreg, set()

    def kreg(reg, k, g, clamp=True):                  # shared rate registers: emit once
        if reg not in seen:
            seen.add(reg)
            kreg0(reg, k, g, clamp)
    b.kreg = kreg
    dc = dc_points()
    inits = [(reg, dc[reg] if v is None else v) for reg, v in DC.items()]
    inits += [("pb_c", (dc["bh2"] + dc["ph2"]) - 2 * VBE), ("fb_c", -VBE), ("rb_c", dc["rflt"] - VBE)]
    # power-up operating point: its own first sample (ends there), so it stays off the per-sample worst path
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    at = b.skip_if(True)
    for reg, v in inits:
        b.op("LDI", imm=q(v)); b.op("ST", reg)
    b.op("LDI", imm=ONE); b.op("ST", "_ini")
    b.op("END")
    b.land(at)
    noise(b)
    foul(b)
    roll(b)
    rings(b)
    crash(b)
    chime(b)
    summer(b)
    return b.finish()


if __name__ == "__main__":
    import sys
    p = bowler()
    out = sys.argv[1] if len(sys.argv) > 1 else "bowler.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"bowler: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
