"""Tornado Baseball / Ball Park (Midway 1976) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Board: Sound Generator 0605-00900 (L. Dekker, Mch 26 '76, change #1 7-20-76),
Useful Information/schematics/Tornado_Baseball_Schematic_Sound_Generator_(0605_00900).pdf (one sheet).
Started from dsnd_boards.tornbase() (= MAME tornbase_discrete: the three tones only, HW-confirmed 2026-10-02).
The sheet has four generators summed by an LM3900 (IC-1 10/11/12) and an LM380 power amp:
- strike / ball / hit: FREQ (pin 10) = the CPU board's 7403 wired-AND of the 240 / 960 / 120 Hz squares (D0-D2)
  -> R602 50K level pot -> R601 47K -> C601 .047u -> mixer. (MAME: the same 47K / .047u high pass, pot at max.)
- siren (pin 11, D3): LM3900 integrator (100K / 680K / C405 33 uF) -> R407 36K -> Q402 / Q401 2N3394 astable
  (C403 .02u, C402 .003u, R403 68K, R404 51K, R405 10K, R406 6.8K) -> R402 100K pot -> R401 100K -> C401 .02u.
  Silent below ~0.46 V control; 250 Hz -> 2.5 kHz as the 33 uF integrator ramps (0.47 V/s up, 0.38 V/s down).
- whistle (pin 3, D6): integrator (C501 6.8 uF) -> R503 200K -> Q501 / Q502 (C502 470 pF, R505 470K) -> R507 100K
  pot -> R508 100K -> C504 .02u. Oscillates only for a control between 1.05 and 2.78 V (3.2 -> 5.2 kHz); the game
  pulses D6 (0.27 s) so the integrator walks through that window: a trilled referee whistle.
- cheer (pin 2, D4): D201 1N5239 zener noise (4.7M from 9 V) -> IC-2 x22 -> C202 -> MFB low pass (R302 100K, C301
  470p, R304 270K, C303 68p, R303 1.5M; ~1.4 kHz) + R301 150K / C304 / R306 into IC-2 13 (x4.7) -> C306 / R318 ->
  D301 / D302 current steering, opened by the D4 integrator (C302 6.8 uF) through R319 6.8K / R314 1K -> IC-2 12
  (1.5M) -> R315 100K pot -> R320 100K -> C307 .047u. Swells in ~1.5 s, lingers ~2.5 s after D4 drops.
- mixer IC-1 11 (R705 100K, R706 200K bias) -> C705 .1u -> R704 75K -> R703 25K volume -> LM380 with the C702 .047u
  + R702 4.7K tone network on its input (treble shelf).
MAME models only the tones (siren / cheer / whistle are comments in tornbase_audio_w). Bits from a MAME capture of
play: D5 is held high while a game runs (no sound on this board), D6 = whistle on this set (MAME: D6 on the
default upright cabinet, D5 on the old one), D7 coin counter.
Spec: verilator/snd/tornbase/tb_proto.py (float model), ngspice deck tb_spice.py. Constants marked (spice) were
measured on that deck of the schematic (oscillator sweeps, integrator slopes, cheer gate), not fitted to recordings.
Assumptions (not on the sheet): FREQ pull-up 1K to +5 V; latch outputs 3.4 V high; level pots R402 / R507 / R315 /
R602 at mid travel, volume R703 at max; zener noise NZ.
"""
import math
from dsnd import Board, LATCH1, q, ONE, FS

DT = 1.0 / FS
LN2 = math.log(2.0)
# ---- spice-measured constants (tb_spice.py: LM3900 model with its 1.3 mA output sink, 2N3394 BF 80)
I_UP, I_DN = 15.52e-6, 12.44e-6         # Norton integrators: net charge current with the input high (3.4 V) / low
V_LO, V_HI = 0.114, 8.27                # integrator output clamps
S_LO, S_HI = 0.075, 8.73                # integrator state limits (cap keeps charging past the clamps: 17 ms / 0.25 s
                                        # recovery delays)
SA_TAU, SA_A, SA_ON = 782.2e-6, -1.940, 0.4575      # siren A (Q401 on): x -> Vs, tau; ends at x = SA_ON
SB_T0, SB_TK = 176.85e-6, 0.772e-6      # siren B (Q401 off) length = SB_T0 + SB_TK Vs
SB_VINF, SB_TAU = 7.15, 173.0e-6        # Q401 collector rise in B
SA_T0, SA_RT = 54e-6, 27e-6             # Q401 collector recovery in A (Q401 leaving saturation)
PL0, PLK = 2.063, 0.021                 # Q401 collector plateau = PL0 + PLK Vs
WL_TAU, WL_A, WL_ON = 84.1e-6, -9.43, 0.7125        # whistle L (Q502 on): x -> Vw; ends at x = WL_ON
WL_LO = 1.05                            # no oscillation below this control voltage
WH_C, WH_TAU, WH_TOP = 24.84e-6, 6.58e-6, 2.7785    # whistle H = WH_C + WH_TAU ln(1 / (WH_TOP - Vw)); latched above
WH_LOW, WH_HIGH, WH_RISE = 0.096, 8.05, 0.85e-6     # Q502 collector levels, rise-time area deficit
MO_DC, MO_LO, MO_HI = 4.67, 0.11, 8.31  # mixer output (IC-1 10) rest / range
N2O_DC = 4.51                           # noise amp (IC-2 5) rest
O9_FLOOR, I3_DC = 2.96, 0.98e-6         # IC-2 9 with no + current; rest bias current
O10_REST = 3.21                         # IC-2 10 with only the R313 bias
I317 = 1.72e-6                          # R317 4.7M into the steering node
K318 = 0.909                            # R318 AC current share reaching IC-2 12 (spice: o10 / HP(o9) = 2.90, not
                                        # 1.5M / 470K = 3.19)
RD3 = 0.02585 / I3_DC                  # IC-2 13 input diode small-signal resistance (VT / I, 26K)
GATE_V, GATE_S = 0.500, 0.0437          # D301 share = 1 / (1 + exp((GATE_V - VE) / GATE_S)), VE = cheer int / 7.8
                                        # (spice: cheer envelope through the swell / fade, 100 ms windows)
# ---- assumptions
VTTL = 3.4
VFREQ, RPU = 5.0, 1000.0
POTS = dict(p402=0.5, p507=0.5, p315=0.5, p602=0.5)
NZ = 0.0415                             # zener noise: in-band RMS at IC-2 pin 5 (not on the schematic; see report).
                                        # = tb_spice.py nz=0.05 (its TRNOISE is linearly interpolated: 0.83 in band)
TONE_NET = True                         # LM380 input tone network (C702 / R702); False = flat (MAME-like)


def thev(rsrc, rpot, p):
    """source at a pot's top (series rsrc), wiper at p (0 = ground .. 1 = top): (Thevenin gain, R)"""
    lo, hi = rpot * p, rpot * (1 - p)
    return lo / (rsrc + rpot), 1.0 / (1.0 / max(lo, 1e-3) + 1.0 / (rsrc + hi))


def paths(pots=None):
    """(gain, series R, coupling C) from each source node into the mixer's virtual ground"""
    p = dict(POTS, **(pots or {}))
    g = {}
    for name, pot, c in (("siren", "p402", 0.02e-6), ("whistle", "p507", 0.02e-6), ("cheer", "p315", 0.047e-6)):
        gain, rth = thev(0.0, 100e3, p[pot])
        g[name] = (gain, 100e3 + rth, c)
    gain, rth = thev(RPU, 50e3, p["p602"])
    g["tone"] = (gain, 47e3 + rth, 0.047e-6)
    return g


def expm(a):
    n = max(sum(abs(x) for x in col) for col in zip(*a))
    s = max(0, int(math.ceil(math.log2(n))) + 1) if n > 0 else 0
    m = len(a)
    a = [[x / (1 << s) for x in r] for r in a]
    mul = lambda x, y: [[sum(x[i][k] * y[k][j] for k in range(m)) for j in range(m)] for i in range(m)]
    r = [[float(i == j) for j in range(m)] for i in range(m)]
    term = [row[:] for row in r]
    for k in range(1, 20):
        term = [[v / k for v in row] for row in mul(term, a)]
        r = [[r[i][j] + term[i][j] for j in range(m)] for i in range(m)]
    for _ in range(s):
        r = mul(r, r)
    return r


def solve(a, b):
    m = len(a)
    aug = [list(map(float, a[i])) + [float(b[i])] for i in range(m)]
    for i in range(m):
        p = max(range(i, m), key=lambda r: abs(aug[r][i]))
        aug[i], aug[p] = aug[p], aug[i]
        for r in range(m):
            if r != i:
                f = aug[r][i] / aug[i][i]
                aug[r] = [x - f * y for x, y in zip(aug[r], aug[i])]
    return [aug[i][m] / aug[i][i] for i in range(m)]


def cheer_ss(rd=None):
    """cheer filter network: 4-state linear system driven by the IC-2 pin 5 AC voltage u. IC-2 3 (amp 2 -) is a
    virtual ground; IC-2 13 (amp 3 +) is the input diode of the current mirror: its small-signal resistance RD3 =
    VT / I3_DC (26K, the LM3900 model's input transistor) to ground. x = [v(C202), v(nb), v(IC-2 4) AC, v(C304)],
    nb = the C301 / R302 / R303 / R304 node. Returns ZOH-discrete (Ad, Bd) and the output row (Co, Do) = current
    into IC-2 13 (A)."""
    R302, R301, R303, R304, R306 = 100e3, 150e3, 1.5e6, 270e3, 100e3
    C202, C301, C303, C304 = 0.047e-6, 470e-12, 68e-12, 0.047e-6
    rd = RD3 if rd is None else rd
    g1, g2, g6 = 1 / R302, 1 / R301, 1 / R306
    k = 1 / (1 + rd * (g2 + g6))
    # i3 = k (g2 na + g6 y), na = u - x0, y = x2 - x3; v13 = rd i3
    # i301 = g2 (na - v13) = g2 na (1 - rd k g2) - g2 rd k g6 y
    # i306 = g6 (y - v13)  = g6 y (1 - rd k g6) - g6 rd k g2 na
    a_nn, a_ny = g2 * (1 - rd * k * g2), -g2 * rd * k * g6          # i301 = a_nn na + a_ny y
    b_yy, b_yn = g6 * (1 - rd * k * g6), -g6 * rd * k * g2          # i306 = b_yy y + b_yn na
    # C202 dx0 = (na - nb) g1 + i301 ; na = u - x0
    A = [[-(g1 + a_nn) / C202, -g1 / C202, a_ny / C202, -a_ny / C202],
         [-g1 / C301, -(g1 + 1 / R303 + 1 / R304) / C301, 1 / R303 / C301, 0],
         [0, -1 / R304 / C303, 0, 0],
         [-b_yn / C304, 0, b_yy / C304, -b_yy / C304]]
    B = [(g1 + a_nn) / C202, g1 / C301, 0, b_yn / C304]
    Ad = expm([[x * DT for x in r] for r in A])
    AdI = [[Ad[i][j] - (i == j) for j in range(4)] for i in range(4)]
    Bd = solve(A, [sum(AdI[i][j] * B[j] for j in range(4)) for i in range(4)])
    return Ad, Bd, [-k * g2, 0, k * g6, -k * g6], k * g2


def out_filter():
    """mixer -> LM380 input: C705 .1u, R704 75K, R703 25K (wiper at the top), C702 .047u + R702 4.7K, LM380 150K.
    Bilinear biquad (b, a) of H = Z / (R704 + Z + 1 / sC705), Z = Rp || (R702 + 1 / sC702)"""
    C705, R704, R703, C702, R702, RIN = 0.1e-6, 75e3, 25e3, 0.047e-6, 4.7e3, 150e3
    Rp = 1 / (1 / R703 + 1 / RIN)
    a, b = R702 * C702, (Rp + R702) * C702
    num = [C705 * Rp * a, C705 * Rp, 0.0]
    den = [b * C705 * R704 + C705 * Rp * a, b + C705 * R704 + C705 * Rp, 1.0]
    k = 2 * FS
    nz = [num[0] * k * k + num[1] * k + num[2], 2 * num[2] - 2 * num[0] * k * k, num[0] * k * k - num[1] * k + num[2]]
    dz = [den[0] * k * k + den[1] * k + den[2], 2 * den[2] - 2 * den[0] * k * k, den[0] * k * k - den[1] * k + den[2]]
    return [x / dz[0] for x in nz], [x / dz[0] for x in dz]


def out_flat_gain():
    """C705 / R704 into R703 || 150K with no tone network: the pass-band gain (flat variant)"""
    rp = 1 / (1 / 25e3 + 1 / 150e3)
    return rp / (75e3 + rp)


# OUT: full scale per volt at the LM380 input. tone_gain() would give the 120 Hz strike tone the RMS of the
# HW-confirmed MAME-derived program (1.495 with the tone network, 1.072 flat); a random-trigger stress run (all four
# generators overlapping, mixer at its rails) peaks at 0.707 V (0.85 x that gain fits under 0.9 full scale) / 1.14 V
# flat -> the strike tone sits at 0.85 / 0.74 of the old program's level.
OUT_GAIN = {True: 1.27, False: 0.79}


def kcode(b, fn):
    """emit code into the Game Audio on-change block"""
    body, b.code = b.code, b.kcode
    fn()
    b.kcode, b.code = b.code, body


def integrator(b, name, bit, c):
    """LM3900 integrator on a latch bit: state name_st (limits S_LO..S_HI), output name (clamped V_LO..V_HI)"""
    b.kreg(name + "_up", I_UP * DT / c, "T", clamp=False)
    b.kreg(name + "_dn", -I_DN * DT / c, "T", clamp=False)
    b.init(name + "_st", S_LO)
    b.op("BIT", 0, b.src(LATCH1, bit))
    b.op("LD", name + "_dn")
    b.op("LDF", name + "_up")
    b.op("ADD", name + "_st")
    b.op("MAXI", imm=q(S_LO))
    b.op("MINI", imm=q(S_HI))
    b.op("ST", name + "_st")
    b.op("MAXI", imm=q(V_LO))
    b.op("MINI", imm=q(V_HI))
    b.op("ST", name)


def jump(b):
    b.op("CMPI", imm=-(1 << 31))
    return b.skip_if(True)


def siren(b):
    """Q402 / Q401 astable: s_ph 0 = A (Q401 on), 1 = B; s_t = time in phase (nominal samples: +_fO per sample);
    s_x = the Q402 base proxy in A. Output s_c = Q401 collector (V), point-sampled."""
    integrator(b, "s_o", 3, 33e-6)
    b.kreg("s_ka", 1 - math.exp(-DT / SA_TAU), "O")
    vs0 = V_LO
    b.init("s_t", 64.0)
    b.init("s_x", vs0)
    b.op("LD", "s_ph")
    b.op("CMPI", imm=ONE // 2)
    at_b = b.skip_if(True)
    # ---- A
    b.op("LD", "s_x"); b.op("ST", "s_x0")
    b.op("LD", "s_o")
    b.op("RCM", "s_x", b.r("s_ka"))
    b.op("LD", "s_t"); b.op("ADD", "_fO"); b.op("MINI", imm=q(64.0)); b.op("ST", "s_t")
    b.op("LD", "s_x"); b.op("CMPI", imm=q(SA_ON))
    no = b.skip_if(False)
    b.op("SUB", "s_x0"); b.op("RCP"); b.op("ST", "_r")                   # 1 / (x - x0)
    b.op("LD", "s_x"); b.op("ADDI", imm=q(-SA_ON)); b.op("MUL", "_r")    # fraction of the sample spent in B
    b.op("MUL", "_fO"); b.op("ST", "s_t")
    b.op("LDI", imm=ONE); b.op("ST", "s_ph")
    b.land(no)
    done = jump(b)
    # ---- B
    b.land(at_b)
    b.op("LD", "s_t"); b.op("ADD", "_fO"); b.op("ST", "s_t")
    b.op("LDI", imm=q(SB_T0 * FS)); b.op("MACI", "s_o", imm=q(SB_TK * FS)); b.op("ST", "_tb")
    b.op("LD", "s_t"); b.op("CMP", "_tb")
    no = b.skip_if(False)
    b.op("SUB", "_tb"); b.op("ST", "s_t")                                # leftover (nominal samples)
    b.op("LD", "s_o"); b.op("ADDI", imm=q(-SA_A)); b.op("MUL", "s_t"); b.op("MULI", imm=q(1 / (SA_TAU * FS)))
    b.op("ADDI", imm=q(SA_A)); b.op("ST", "s_x")
    b.op("LDI", imm=0); b.op("ST", "s_ph")
    b.land(no)
    b.land(done)
    # ---- Q401 collector
    b.op("LDI", imm=q(PL0)); b.op("MACI", "s_o", imm=q(PLK)); b.op("ST", "_pl")
    b.op("LD", "s_ph"); b.op("CMPI", imm=ONE // 2)
    at_b = b.skip_if(True)
    b.op("LD", "s_t"); b.op("ADDI", imm=q(-SA_T0 * FS)); b.op("MULI", imm=q(-1 / (SA_RT * FS * LN2)))
    b.op("MINI", imm=q(6.9)); b.op("EXP"); b.op("MUL", "_pl"); b.op("ST", "_r")
    b.op("LD", "_pl"); b.op("SUB", "_r"); b.op("MAXI", imm=q(0.01)); b.op("ST", "s_c")
    done = jump(b)
    b.land(at_b)
    b.op("LD", "s_t"); b.op("MULI", imm=q(-1 / (SB_TAU * FS * LN2))); b.op("EXP"); b.op("ST", "_r")
    b.op("LDI", imm=q(SB_VINF)); b.op("SUB", "_pl"); b.op("MUL", "_r"); b.op("ST", "_r")
    b.op("LDI", imm=q(SB_VINF)); b.op("SUB", "_r"); b.op("ST", "s_c")
    b.land(done)


def whistle(b):
    """Q501 / Q502 oscillator: w_ph 0 = L (Q502 on), 1 = H; w_x = Q501 base proxy in L; w_t = time in H (nominal
    samples); w_z = log2(WH_TOP - Vw) tracked by one Newton step a sample (EXP). Output w_c = Q502 collector
    averaged over the sample."""
    integrator(b, "w_o", 6, 6.8e-6)
    b.kreg("w_kl", 1 - math.exp(-DT / WL_TAU), "O")
    b.init("w_x", V_LO)
    b.init("w_z", math.log2(WH_TOP - V_LO))
    b.init("w_c", WH_LOW)
    # log2(WH_TOP - Vw), H length (nominal samples); latched H (never ends) at / above the top
    b.op("LDI", imm=q(WH_TOP)); b.op("SUB", "w_o"); b.op("ST", "_u")
    b.op("LDI", imm=0); b.op("SUB", "w_z"); b.op("EXP"); b.op("MUL", "_u"); b.op("ADDI", imm=-ONE)
    b.op("MULI", imm=q(1 / LN2)); b.op("MAXI", imm=q(-1.5)); b.op("MINI", imm=q(1.5))
    b.op("ADD", "w_z"); b.op("MAXI", imm=q(-7.0)); b.op("MINI", imm=q(2.0)); b.op("ST", "w_z")   # EXP range
    b.op("MULI", imm=q(-WH_TAU * LN2 * FS)); b.op("ADDI", imm=q(WH_C * FS)); b.op("ST", "_th")
    b.op("LD", "_u"); b.op("CMPI", imm=q(1e-4))
    b.op("LDI", imm=q(120.0)); b.op("STNF", "_th")
    b.op("LD", "w_ph"); b.op("CMPI", imm=ONE // 2)
    at_h = b.skip_if(True)
    # ---- L: below the window the target is held under the threshold (never fires)
    b.op("LD", "w_x"); b.op("ST", "w_x0")
    b.op("LD", "w_o"); b.op("ST", "_g")
    b.op("CMPI", imm=q(WL_LO))
    b.op("LD", "w_o"); b.op("MINI", imm=q(WL_ON - 0.05)); b.op("STNF", "_g")
    b.op("LD", "_g")
    b.op("RCM", "w_x", b.r("w_kl"))
    b.op("CMPI", imm=q(WL_ON))
    b.op("LDI", imm=q(WH_LOW)); b.op("ST", "w_c")
    no = b.skip_if(False)
    b.op("LD", "w_x"); b.op("SUB", "w_x0"); b.op("RCP"); b.op("ST", "_r")
    b.op("LD", "w_x"); b.op("ADDI", imm=q(-WL_ON)); b.op("MUL", "_r"); b.op("ST", "_fr")   # fraction spent in H
    b.op("MUL", "_fO"); b.op("ST", "w_t")
    b.op("LDI", imm=ONE); b.op("ST", "w_ph")
    b.op("LD", "_fr"); b.op("ADDI", imm=q(-WH_RISE * FS)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(WH_HIGH - WH_LOW)); b.op("ADDI", imm=q(WH_LOW)); b.op("ST", "w_c")
    b.land(no)
    done = jump(b)
    # ---- H
    b.land(at_h)
    b.op("LDI", imm=q(WH_HIGH)); b.op("ST", "w_c")
    b.op("LD", "w_t"); b.op("ADD", "_fO"); b.op("MINI", imm=q(110.0)); b.op("ST", "w_t")
    b.op("CMP", "_th")
    no = b.skip_if(False)
    b.op("SUB", "_th"); b.op("ST", "_lf")                                 # leftover in L (nominal samples)
    b.op("MULI", imm=q(-1 / (WL_TAU * FS * LN2))); b.op("EXP"); b.op("ST", "_r")
    b.op("LDI", imm=ONE); b.op("SUB", "_r"); b.op("ST", "_r")
    b.op("LD", "w_o"); b.op("ADDI", imm=q(-WL_A)); b.op("MUL", "_r"); b.op("ADDI", imm=q(WL_A)); b.op("ST", "w_x")
    b.op("LD", "_lf"); b.op("MUL", "_sO"); b.op("MINI", imm=ONE)            # real fraction of the sample in L
    b.op("MULI", imm=q(-(WH_HIGH - WH_LOW))); b.op("ADDI", imm=q(WH_HIGH)); b.op("ST", "w_c")
    b.op("LDI", imm=0); b.op("ST", "w_ph")
    b.land(no)
    b.land(done)


def cheer(b, nz):
    """zener noise -> x22 -> 4-state filter network -> IC-2 9 (half wave, floor) -> C306 / R318 -> diode steering
    by the D4 integrator -> IC-2 10 (ch_o)"""
    integrator(b, "c_o", 4, 6.8e-6)
    b.noise(2, nz)
    b.op("ST", "_n")
    b.crfilter("n_hp", "_n", 100e3, 0.047e-6)                             # C201 into R203 (IC-2 6 virtual ground)
    b.op("LD", "n_hp")
    b.op("MAXI", imm=q(0.11 - N2O_DC)); b.op("MINI", imm=q(8.2 - N2O_DC)); b.op("ST", "_u")
    Ad, Bd, Co, Do = cheer_ss()
    for i in range(4):
        b.op("LDI", imm=0)
        for j in range(4):
            b.op("MACI", f"cx{j}", imm=q(Ad[i][j]))
        b.op("MACI", "_u", imm=q(Bd[i]))
        b.op("ST", f"_cn{i}")
    for i in range(4):
        b.op("LD", f"_cn{i}"); b.op("ST", f"cx{i}")
    # IC-2 9 = floor + 470K x max(0, I3_DC + i3), i3 = Co x + Do u
    b.op("LDI", imm=q(470e3 * I3_DC))
    for j in range(4):
        if Co[j]:
            b.op("MACI", f"cx{j}", imm=q(470e3 * Co[j]))
    b.op("MACI", "_u", imm=q(470e3 * Do))
    b.op("MAXI", imm=0); b.op("ADDI", imm=q(O9_FLOOR)); b.op("MINI", imm=q(8.31))
    # R318 current x 470K (volts), >= -I317 x 470K (the diodes cannot sink); C306 integrates it
    b.kreg("c6_k", DT / (470e3 * 0.047e-6), "F", clamp=False)
    b.init("c6", O9_FLOOR + 470e3 * I3_DC)
    b.op("SUB", "c6"); b.op("MULI", imm=q(K318)); b.op("MAXI", imm=q(-I317 * 470e3)); b.op("ST", "_w")
    b.op("MUL", "c6_k"); b.op("ADD", "c6"); b.op("ST", "c6")
    # D301 share: 1 / (1 + 2^((GATE_V - VE) / (GATE_S ln2))), VE = c_o x 1K / 7.8K; EXP saturates at 2^7 -> rescale
    b.op("LD", "c_o"); b.op("MULI", imm=q(-1 / (7.8 * GATE_S * LN2))); b.op("ADDI", imm=q(GATE_V / (GATE_S * LN2)))
    b.op("MINI", imm=q(6.99)); b.op("EXP"); b.op("ADDI", imm=ONE); b.op("RCP")
    b.op("ADDI", imm=q(-1 / (1 + 2 ** 6.99))); b.op("MULI", imm=q(1 / (1 - 1 / (1 + 2 ** 6.99)))); b.op("MAXI", imm=0)
    b.op("ST", "_gt")
    b.op("LD", "_w"); b.op("ADDI", imm=q(I317 * 470e3)); b.op("MUL", "_gt")
    b.op("MULI", imm=q(1.5e6 / 470e3)); b.op("ADDI", imm=q(O10_REST)); b.op("MINI", imm=q(8.31)); b.op("ST", "ch_o")


def tone_gain(pots=None, tone_net=True):
    """OUT gain (full scale per LM380-input volt) giving the strike tone (D2, 120 Hz) the RMS of the
    MAME-derived program; float simulation of both paths, 1 s after 0.2 s settling"""
    n = int(1.2 * FS)
    g = paths(pots)["tone"]
    ob, oa = out_filter()
    kc_old = 1 - math.exp(-DT / (47e3 * 0.047e-6))
    kc_new = 1 - math.exp(-DT / (g[1] * g[2]))
    ph = 0.0; c_old = 1.0; c_new = VFREQ * g[0]
    x1 = x2 = y1 = y2 = 0.0
    so = sn = 0.0
    for i in range(n):
        hi = ph >= 0.5
        ph += 120.0 / FS
        if ph >= 1.0:
            ph -= 1.0
        v = 0.0 if hi else 1.0
        c_old += (v - c_old) * kc_old
        old = v - c_old
        vt = (0.1 if hi else VFREQ) * g[0]
        c_new += (vt - c_new) * kc_new
        mo = min(MO_HI - MO_DC, max(MO_LO - MO_DC, -100e3 / g[1] * (vt - c_new)))
        if tone_net:
            y = ob[0] * mo + ob[1] * x1 + ob[2] * x2 - oa[1] * y1 - oa[2] * y2
            x2, x1, y2, y1 = x1, mo, y1, y
        else:
            y = mo * out_flat_gain()
        if i >= int(0.2 * FS):
            so += old * old; sn += y * y
    return math.sqrt(so / sn)


def tornbase(nz=None, pots=None, tone_net=None, out_gain=None):
    nz = NZ if nz is None else nz
    tone_net = TONE_NET if tone_net is None else tone_net
    gain = out_gain if out_gain is not None else OUT_GAIN[tone_net]
    g = paths(pots)
    b = Board("tornbase")
    b.groups |= {"O", "T", "F"}
    kcode(b, lambda: b.chain("_sO", 7, 0x07, 16, list(b.SCALES)))          # 1 / _fO (oscillator cap scale)
    # ---- strike / ball / hit: wired-AND of the enabled squares, through the R602 pot
    b.op("LDI", imm=q(VFREQ * g["tone"][0]))
    b.op("ST", "tl")
    for k, f in enumerate((240, 960, 120)):
        b.squarewfix(f"sq{k}", f)
        b.op("LD", f"sq{k}")
        b.op("CMPI", imm=1 << 23)
        b.op("FAND", 0, b.src(LATCH1, k))
        b.op("LDI", imm=q(0.1 * g["tone"][0]))
        b.op("STF", "tl")
    b.crfilter("tone", "tl", g["tone"][1], g["tone"][2])
    b.init("tone_c", VFREQ * g["tone"][0])
    siren(b)
    b.crfilter("siren", "s_c", g["siren"][1], g["siren"][2])
    b.init("siren_c", PL0 + PLK * V_LO)
    whistle(b)
    b.crfilter("whistle", "w_c", g["whistle"][1], g["whistle"][2])
    b.init("whistle_c", WH_LOW)
    cheer(b, nz)
    b.crfilter("cheer", "ch_o", g["cheer"][1], g["cheer"][2])
    b.init("cheer_c", O10_REST)
    b.init("ch_o", O10_REST)
    # ---- mixer: IC-1 10 AC = -100K x sum of the coupling currents (crfilter outputs are the source-side volts
    # before the pot gain), clamped to the LM3900 range
    b.op("LDI", imm=0)
    for name in ("tone", "siren", "whistle", "cheer"):
        gn, r, _ = g[name]
        k = gn if name != "tone" else 1.0                                 # tone: pot gain already in tl
        b.op("MACI", name, imm=q(-100e3 / r * k))
    b.op("MAXI", imm=q(MO_LO - MO_DC)); b.op("MINI", imm=q(MO_HI - MO_DC)); b.op("ST", "mo")
    # ---- C705 / R704 / R703 / tone network -> LM380 input
    if tone_net:
        ob, oa = out_filter()
        b.op("MULI", imm=q(ob[0] * gain))
        b.op("MACI", "of_x1", imm=q(ob[1] * gain))
        b.op("MACI", "of_x2", imm=q(ob[2] * gain))
        b.op("MACI", "of_y1", imm=q(-oa[1]))
        b.op("MACI", "of_y2", imm=q(-oa[2]))
        b.op("ST", "_y")
        b.op("LD", "of_x1"); b.op("ST", "of_x2")
        b.op("LD", "mo"); b.op("ST", "of_x1")
        b.op("LD", "of_y1"); b.op("ST", "of_y2")
        b.op("LD", "_y"); b.op("ST", "of_y1")
    else:
        # C705 into R704 + R703 || 150K: one-pole high pass (16 Hz) x the divider
        rp = 1 / (1 / 25e3 + 1 / 150e3)
        b.crfilter("of", "mo", 75e3 + rp, 0.1e-6)
        b.op("LD", "of")
        b.op("MULI", imm=q(out_flat_gain() * gain))
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    kw = dict(a.split("=") for a in sys.argv[2:])
    p = tornbase(nz=float(kw["nz"]) if "nz" in kw else None,
                 tone_net=(kw["tone_net"] == "1") if "tone_net" in kw else None)
    out = sys.argv[1] if len(sys.argv) > 1 else "tornbase.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"tornbase: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
