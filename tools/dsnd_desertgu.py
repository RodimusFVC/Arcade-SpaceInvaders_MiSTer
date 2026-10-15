"""Desert Gun / Road Runner (Midway 1977, desertgu / roadrunm) sound for the discrete sound engine (tools/dsnd.py,
rtl/dsnd_engine.sv), from the schematic "Road Runner Game Logic & Sound" P.C. 0618-00902E (6-1-77), one sheet.

Latches: S1 = port 3 (H4 74174 -> 7417 H3, 10k pull-ups to +12): D2 coin, D3 game on (7404 G2 -> 7417 J3 -> mute
diodes D800 / D801), D4 rifle (D603), D5 bottle (D601), D6 Road Runner (D701), D7 creature (R300). S2 / S3 = Midway
tone generator (ports 5 / 6). S4 = port 7 (J4 -> 7417 J3): D0 beep-beep (R209 10k -> R521), D1 trigger click
(R208 1k -> R207 2k), D2 recoil, D3 controller select.
Spec = verilator/snd/desertgu/dg_proto.py (float, same equations and order); circuit reference = dg_spice.py
(ngspice, National LM3900 model). The music (tone generator, TVCA, R503 pot at MAME's 60 %) and the summing amp are
the HW-confirmed MAME-derived parts of tools/dsnd_boards.py desertgu(), unchanged.
Voices on the schematic that MAME leaves at 0: bottle hit, Road Runner hit, creature hit, beep-beep (all here).
Rifle shot: schematic TVCA (2.7M / 680k / 680k, C605 0.47 uF, as MAME) into the R603 / C603 / R602 / C602 low-pass
ladder (MAME: RC + a CR high-pass), R601 / C601 to the bus. Click / bottle / beep: the L3-4 band-pass is an LM3900
biased at its low limit (output only swings up): two-state model with a clamp, beep through its small-signal path.
The Road Runner / creature voices run on even samples, bottle / beep / the band-pass on odd samples (24 kHz each);
the rest every sample.
"""
import math
from dsnd import Board, LATCH1, LATCH2, LATCH3, LATCH4, q, ONE
from dsnd_boards import MIDWAY_MUSIC_TVCA, logadj

SR = 48000.0
DT = 1.0 / SR
K, U = 1e3, 1e-6
NEVER = -(1 << 31)
# LM3900 (National model, ngspice): + / - input voltages, input mirror, output high; see dg_proto.py
VOH, VP, VM, VML, MIR = 11.35, 0.52, 0.49, 0.56, 0.989
PULL = 680e3 / 27e3 / 330.0               # TVCA with a 27k pull-up that must be sunk (sink gain ~330)


def kx(tau, dt=DT):
    return 1.0 - math.exp(-dt / tau)


def zoh(A, B, dt=DT):
    """exact one-step discretisation of dx/dt = A x + B u (u held), plain lists"""
    n = len(A)
    M = [[A[i][j] * dt for j in range(n)] + [B[i] * dt] for i in range(n)] + [[0.0] * (n + 1)]
    E = [[1.0 if i == j else 0.0 for j in range(n + 1)] for i in range(n + 1)]
    T = [r[:] for r in E]
    for k in range(1, 40):
        T = [[sum(T[i][m] * M[m][j] for m in range(n + 1)) / k for j in range(n + 1)] for i in range(n + 1)]
        E = [[E[i][j] + T[i][j] for j in range(n + 1)] for i in range(n + 1)]
    return [r[:n] for r in E[:n]], [r[n] for r in E[:n]]


def lin(b, dst, const, terms):
    """dst = const + sum(k x reg); acc keeps the value"""
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
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def ld(b, v):
    b.op("LD", v) if isinstance(v, str) else b.op("LDI", imm=q(v))


def tvca_c0(rfix):
    """Norton TVCA (680k input / feedback / cap resistor, rfix from +12) in its linear range:
    vo = MIR vc - vin + tvca_c0"""
    return VML - MIR * VP + VML - 680e3 / rfix * (12 - VML)


def env_cap(b, reg, src, t_on, kon, t_off, koff, g="T"):
    """reg relaxes toward t_on while the latch bit is set (diode charge), else toward t_off"""
    b.kreg(reg + "_kon", kon, g)
    b.kreg(reg + "_kof", koff, g)
    b.op("BIT", 0, b.src(*src))
    b.op("LDI", imm=q(t_off)); b.op("LDIF", imm=q(t_on)); b.op("ST", "_t")
    b.op("LD", reg + "_kof"); b.op("LDF", reg + "_kon"); b.op("ST", "_k")
    b.op("LD", "_t"); b.op("RCM", reg, b.r("_k"))


def muli_big(b, k):
    """acc x k for |k| up to 128^2 (two steps; the product itself stays small)"""
    if abs(k) < 64:
        b.op("MULI", imm=q(k))
    else:
        s = 2 ** math.ceil(math.log2(abs(k) / 32))
        b.op("MULI", imm=q(k / s)); b.op("MULI", imm=q(s))


def frac(b, v, inv, exact):
    """acc = cap - trip (past it): _f = the part of the step after the crossing, 0 .. 1.0"""
    if exact:
        b.op("ST", "_d"); b.op("LD", v); b.op("SUB", "_v0"); b.op("RCP"); b.op("MUL", "_d")
        b.op("MAXI", imm=0)
    else:
        muli_big(b, inv)
    b.op("MINI", imm=ONE); b.op("ST", "_f")


def osc(b, p, t_h, t_l, kh, kl, hi, lo, inv_h, inv_l, gate=None, exact=False):
    """two-phase exponential relaxation oscillator (dg_proto.Osc): cap p+'v', state p+'h' (0 / 1.0), p+'f' = time
    high in this sample (0 .. 1.0). t_h / t_l: register or constant targets; kh / kl: rate registers; trips hi / lo;
    inv_h / inv_l = 1 / per-sample slope at the trip (crossing placed by interpolation). gate: (source, bit) or a
    0 / 1.0 register; closed = output low. exact: place the crossing with the actual step (RCP) instead"""
    v, h, f = p + "v", p + "h", p + "f"
    ends = []
    if gate is not None:
        if isinstance(gate, tuple):
            b.op("BIT", 0, b.src(*gate))
        else:
            b.op("LD", gate); b.op("CMPI", imm=q(0.5))
        j = b.skip_if(True)
        ld(b, t_l); b.op("RCM", v, b.r(kl)); b.op("LDI", imm=0); b.op("ST", h); b.op("ST", f)
        ends.append(jump(b))
        b.land(j)
    if exact:
        b.op("LD", v); b.op("ST", "_v0")
    b.op("LD", h); b.op("CMPI", imm=q(0.5))
    jl = b.skip_if(False)
    # high phase
    ld(b, t_h); b.op("RCM", v, b.r(kh)); b.op("ADDI", imm=q(-hi)); b.op("CMPI", imm=0)
    jn = b.skip_if(False)
    frac(b, v, inv_h, exact)
    ld(b, t_l); b.op("ADDI", imm=q(-hi)); b.op("MUL", kl); b.op("MUL", "_f"); b.op("ADDI", imm=q(hi)); b.op("ST", v)
    b.op("LDI", imm=0); b.op("ST", h); b.op("LDI", imm=ONE); b.op("SUB", "_f"); b.op("ST", f)
    ends.append(jump(b))
    b.land(jn)
    b.op("LDI", imm=ONE); b.op("ST", f)
    ends.append(jump(b))
    # low phase
    b.land(jl)
    if gate is not None:                               # gate just opened with the cap below the low trip: go high
        b.op("LD", v); b.op("CMPI", imm=q(lo))
        jg = b.skip_if(True)
        ld(b, t_h); b.op("RCM", v, b.r(kh)); b.op("LDI", imm=ONE); b.op("ST", h); b.op("ST", f)
        ends.append(jump(b))
        b.land(jg)
    ld(b, t_l); b.op("RCM", v, b.r(kl)); b.op("ADDI", imm=q(-lo)); b.op("CMPI", imm=0)
    jn = b.skip_if(True)
    frac(b, v, -inv_l, exact)
    ld(b, t_h); b.op("ADDI", imm=q(-lo)); b.op("MUL", kh); b.op("MUL", "_f"); b.op("ADDI", imm=q(lo)); b.op("ST", v)
    b.op("LDI", imm=ONE); b.op("ST", h); b.op("LD", "_f"); b.op("ST", f)
    ends.append(jump(b))
    b.land(jn)
    b.op("LDI", imm=0); b.op("ST", f)
    for e in ends:
        b.land(e)


# ------------------------------------------------------------------------------------------------ rifle shot
def rifle(b):
    """N3-4 8/13/9 TVCA (R624 2.7M, R619 680k noise, R620 680k, R618 680k, C605 0.47 uF via D603 / 10k); output
    (sources into the ladder, low 0.73 V) -> R603 12k / C603 .01u / R602 68k / C602 .0022u / R601 30k / C601 0.1u"""
    env_cap(b, "sc", (LATCH1, 4), 11.44, kx(10e3 * 0.47 * U), VP, kx(680e3 * 0.47 * U))
    b.op("MULI", imm=q(MIR)); b.op("SUB", "nz"); b.op("ADDI", imm=q(tvca_c0(2.7e6)))
    b.op("MAXI", imm=q(0.73)); b.op("MINI", imm=q(VOH)); b.op("ST", "so")
    A = [[-(1 / 12e3 + 1 / 68e3) / 0.01e-6, 1 / 68e3 / 0.01e-6, 0.0],
         [1 / 68e3 / 0.0022e-6, -(1 / 68e3 + 1 / 30e3) / 0.0022e-6, 1 / 30e3 / 0.0022e-6],
         [0.0, 1 / 30e3 / 0.1e-6, -1 / 30e3 / 0.1e-6]]
    Ad, Bd = zoh(A, [1 / 12e3 / 0.01e-6, 0.0, 0.0])
    xs = ("s1", "s2", "s6")
    for i in range(3):
        lin(b, "_x%d" % i, 0.0, [(xs[j], Ad[i][j]) for j in range(3)] + [("so", Bd[i])])
    for i in range(3):
        b.op("LD", "_x%d" % i); b.op("ST", xs[i])
    b.op("LD", "s2"); b.op("SUB", "s6"); b.op("ST", "sh_v")      # volts across R601 (into C601 / the bus)


# ------------------------------------------------------------------------------------------------ bottle hit
BOTTLE_TRIPS = (6.7315, 4.9399)             # effective trips: ngspice high / low phases 216.6 / 205.5 us


def bottle(b, dtm):
    """N3-4 6/1/5 oscillator: C607 .01u, R617 62k to the output, R622 1M from the noise, R623 1M into the inverting
    input; N3-4 3/2/4 TVCA (C606 .22u via D601 / 10k, R621 680k), output into R604 2k"""
    dt = DT * dtm
    g = 1 / 62e3 + 2 / 1e6
    vol = 0.69
    th0 = (VOH / 62e3 + 0.5 / 1e6) / g
    tl0 = (vol / 62e3 + 0.5 / 1e6) / g
    gn = (1 / 1e6) / g
    k = kx(0.01e-6 / g, dt)
    b.kreg("b_k", k, "O")
    hi, lo = BOTTLE_TRIPS
    # 1 / slope at the trips (noise at its mean, 6 V): the interpolation only places the edge inside the sample
    inv_h = 1.0 / ((hi - (tl0 + 6 * gn)) * k)
    inv_l = 1.0 / (((th0 + 6 * gn) - lo) * k)
    lin(b, "b_th", th0, [("nz", gn)])
    b.op("ADDI", imm=q(tl0 - th0)); b.op("ST", "b_tl")
    osc(b, "bo", "b_th", "b_tl", "b_k", "b_k", hi, lo, inv_h, inv_l, exact=True)
    env_cap(b, "bc", (LATCH1, 5), 11.44, kx(10e3 * 0.22 * U, dt), VP, kx(680e3 * 0.22 * U, dt))
    b.op("MULI", imm=q(MIR)); b.op("ADDI", imm=q(tvca_c0(2.7e6) - vol))
    b.op("MACI", "bof", imm=q(-(VOH - vol)))
    b.op("MAXI", imm=q(0.63)); b.op("MINI", imm=q(VOH)); b.op("ST", "bo2")


# ------------------------------------------------------------------------------------------------ beep-beep
BEEP_TRIPS = (6.5339, 5.9529, 4.0462, 2.7354)  # effective trips: ngspice 697.4 / 592.8 us, 79 / 213 us


def beep(b, dtm):
    """L3-4 8/13/9 oscillator gated by P70 (C505 .15u, R516 39k); L3-4 6/1/5 oscillator gated by its output through
    R517 2.4M (C506 .01u, R513 47k), seen ~40 us late (osc1's slow output rise); eo2 -> R504 27k"""
    dt = DT * dtm
    v1l, v2l = 0.73, 0.58
    g1, g2 = 1 / 39e3 + 1 / 1e6, 1 / 47e3 + 1 / 1e6
    k1, k2 = kx(0.15e-6 / g1, dt), kx(0.01e-6 / g2, dt)
    th1, tl1 = (VOH / 39e3 + 0.5 / 1e6) / g1, (v1l / 39e3 + 0.5 / 1e6) / g1
    th2, tl2 = (VOH / 47e3 + 0.5 / 1e6) / g2, (v2l / 47e3 + 0.5 / 1e6) / g2
    hi1, lo1, hi2, lo2 = BEEP_TRIPS
    b.kreg("e1_k", k1, "O")
    b.kreg("e2_k", k2, "O")
    # osc2's gate = osc1's state 2 steps (83 us) back: in ngspice osc2 starts 41 us after osc1's output edge and its
    # first rise is ~10 us slower than this model's; with 41 us its 3rd pulse would just fit inside osc1's 697 us
    # high phase (ngspice: 2 pulses, 18 us to spare), so the delay carries both
    b.op("LD", "e_d2"); b.op("ST", "e_g2"); b.op("LD", "e_d1"); b.op("ST", "e_d2"); b.op("LD", "e1h"); b.op("ST", "e_d1")
    osc(b, "e1", th1, tl1, "e1_k", "e1_k", hi1, lo1, 1.0 / ((hi1 - tl1) * k1), 1.0 / ((th1 - lo1) * k1),
        gate=(LATCH4, 0))
    osc(b, "e2", th2, tl2, "e2_k", "e2_k", hi2, lo2, 1.0 / ((hi2 - tl2) * k2), 1.0 / ((th2 - lo2) * k2),
        gate="e_g2")


# ------------------------------------------------------------------------------------------------ Road Runner hit
RR_LFO_TRIPS = (8.852, 4.658)
RR_OSC_TRIPS = (8.88, 5.69)
RR_OSC_LO_T, RR_OSC_LO_TAU, RR_OSC_LO_LFO, RR_OSC_LO_LFO2, RR_OSC_LO_KNEE = 5.61, 165e-6, 0.04, 1.0, 8.72
RR_LFO_VOL = 0.784


def roadrunner(b, dtm):
    """R3-4 6/1/5 LFO (C702 4.7u, R713 22k, D702 + R715 10k), R3-4 3/2/4 oscillator (C700 .047u, R706 30k, D700 +
    R702 2k) with R703 100k between the caps, R3-4 11/12/10 TVCA (C701 3.3u via D701 / 10k, R708 680k, input R701
    from C700) with R704 27k pull-up -> R809 56k / C803. Low phases sink / diode limited: measured on ngspice"""
    dt = DT * dtm
    vd = 0.55
    gl_h = 1 / 22e3 + 1 / 1e6 + 1 / 100e3
    gl_l = gl_h + 1 / 10e3
    go_h = 1 / 30e3 + 1 / 1e6 + 1 / 100e3 + 1 / 680e3
    klh, kll = kx(4.7e-6 / gl_h, dt), kx(4.7e-6 / gl_l, dt)
    koh, kol = kx(0.047e-6 / go_h, dt), kx(RR_OSC_LO_TAU, dt)
    for r, kk in (("rl_kh", klh), ("rl_kl", kll), ("ro_kh", koh), ("ro_kl", kol)):
        b.kreg(r, kk, "O")
    # targets from the other cap's previous value
    lin(b, "rl_th", (VOH / 22e3 + 0.5 / 1e6) / gl_h, [("rov", 1 / 100e3 / gl_h)])
    lin(b, "rl_tl", (RR_LFO_VOL / 22e3 + (RR_LFO_VOL + vd) / 10e3 + 0.5 / 1e6) / gl_l, [("rov", 1 / 100e3 / gl_l)])
    lin(b, "ro_th", (VOH / 30e3 + 0.5 / 1e6 + VML / 680e3) / go_h, [("rlv", 1 / 100e3 / go_h)])
    b.op("LD", "rlv"); b.op("ADDI", imm=q(-RR_OSC_LO_KNEE)); b.op("MAXI", imm=0)
    b.op("MULI", imm=q(RR_OSC_LO_LFO2))
    b.op("MACI", "rlv", imm=q(RR_OSC_LO_LFO)); b.op("ADDI", imm=q(RR_OSC_LO_T - 7.0 * RR_OSC_LO_LFO))
    b.op("ST", "ro_tl")
    lhi, llo = RR_LFO_TRIPS
    ohi, olo = RR_OSC_TRIPS
    tlm = (RR_LFO_VOL / 22e3 + (RR_LFO_VOL + vd) / 10e3 + 0.5e-6 + 7.0 / 100e3) / gl_l
    thm = (VOH / 22e3 + 0.5e-6 + 7.0 / 100e3) / gl_h
    osc(b, "rl", "rl_th", "rl_tl", "rl_kh", "rl_kl", lhi, llo, 1.0 / ((lhi - tlm) * kll), 1.0 / ((thm - llo) * klh))
    oth = (VOH / 30e3 + 0.5e-6 + VML / 680e3 + 7.0 / 100e3) / go_h
    osc(b, "ro", "ro_th", "ro_tl", "ro_kh", "ro_kl", ohi, olo, 1.0 / ((ohi - RR_OSC_LO_T) * kol),
        1.0 / ((oth - olo) * koh))
    env_cap(b, "rc", (LATCH1, 6), 11.44, kx(10e3 * 3.3 * U, dt), VP, kx(680e3 * 3.3 * U, dt))
    b.op("MULI", imm=q(MIR)); b.op("SUB", "rov"); b.op("ADDI", imm=q(tvca_c0(2.7e6)))
    b.op("MULI", imm=q(1 / (1 + PULL))); b.op("ADDI", imm=q(PULL * 12 / (1 + PULL)))
    b.op("MAXI", imm=q(0.69)); b.op("MINI", imm=q(VOH)); b.op("ST", "ro3")


# ------------------------------------------------------------------------------------------------ creature hit
CR_VINT = 0.40                       # J1 6/1/5 inverting input as seen by R312 (ngspice sweep, see dg_proto.py)
CR_VC3_TRIP = (VOH - 0.58) - ((12 - 0.1) / 0.2 / MIR + (0.58 - 0.2) / 0.1) * 0.1   # one-shot ends (4.374 V)


def creature(b, dtm):
    """J1 11/12/10 one-shot (R300 100k trigger, C300 10u / R306 100k, R305 200k); D300 / R313 -> C305 1u (R312
    1.5M); noise R311 12k / C304 / R310 68k / C302 / C301 / R309 3.3M; J1 6/1/5 integrator (C303 390p, R304 /
    R308 2.2M) + J1 3/2/4 Schmitt (R307 510k, R303 1M, R302 510k, R301 2.2M); D600 / R605 56k -> C604 2.2u (R606
    680k); N3-4 11/12/10 TVCA (R615 680k, R611 2.2M, R612 680k, R610 27k pull-up) -> R600 180k / C600"""
    dt = DT * dtm
    t_src = (LATCH1, 7)
    b.kreg("c_k1", kx(1.0, dt), "T")
    # one-shot
    b.op("BIT", 0, b.src(*t_src)); b.op("LD", "ch"); b.op("LDIF", imm=ONE); b.op("ST", "ch")
    b.op("MULI", imm=q(VOH - VP - 0.53)); b.op("ADDI", imm=q(0.53)); b.op("RCM", "c3", b.r("c_k1"))
    b.op("CMPI", imm=q(CR_VC3_TRIP)); b.op("FAND", 1, b.src(*t_src)); b.op("LDI", imm=0); b.op("STF", "ch")
    # C305 / C604
    t6c = ((VOH - 0.5) / 56e3 + VP / 680e3) / (1 / 56e3 + 1 / 680e3)
    for reg, (ton, kon, toff, koff) in (("c5", (VOH - 0.36, kx(220 * 1e-6, dt), VML, kx(1.5, dt))),
                                        ("c6", (t6c, kx(2.2e-6 * 56e3 * 680e3 / 736e3, dt), VP, kx(680e3 * 2.2e-6, dt)))):
        b.kreg(reg + "_kon", kon, "T"); b.kreg(reg + "_kof", koff, "T")
        b.op("LD", "ch"); b.op("CMPI", imm=q(0.5))
        b.op("LD", reg + "_kof"); b.op("LDF", reg + "_kon"); b.op("ST", "_k")
        b.op("LD", "ch"); b.op("MULI", imm=q(ton - toff)); b.op("ADDI", imm=q(toff)); b.op("RCM", reg, b.r("_k"))
    # noise filter (2 states) and C301 / R309 (high-pass, the cap's voltage in cn)
    A = [[-(1 / 12e3 + 1 / 68e3) / 0.1e-6, 1 / 68e3 / 0.1e-6], [1 / 68e3 / 0.1e-6, -1 / 68e3 / 0.1e-6]]
    Ad, Bd = zoh(A, [1 / 12e3 / 0.1e-6, 0.0], dt)
    lin(b, "_n0", 0.0, [("cn1", Ad[0][0]), ("cn2", Ad[0][1]), ("nz", Bd[0])])
    lin(b, "cn2", 0.0, [("cn1", Ad[1][0]), ("cn2", Ad[1][1]), ("nz", Bd[1])])
    b.op("LD", "_n0"); b.op("ST", "cn1")
    b.kreg("c_khp", kx(3.3e6 * 0.1e-6, dt), "F")
    b.op("LD", "cn2"); b.op("RCM", "cnh", b.r("c_khp"))
    # integrator slopes (volts per sample) in the low / high Schmitt states
    sl = dt / 390e-12 * 1e-6
    ip_h = MIR * ((12 - VP) / 2.2 + (VOH - VP) / 2.2)
    ip_l = MIR * ((12 - VP) / 2.2 + 0.06 / 2.2)
    lin(b, "c_dl", sl * ip_l + sl * CR_VINT / 1.5, [("c5", -sl / 1.5), ("cn2", -sl / 3.3), ("cnh", sl / 3.3)])
    b.op("ADDI", imm=q(sl * (ip_h - ip_l))); b.op("ST", "c_dh")
    hi = VM + 0.51 * (MIR * ((12 - VP) / 0.51 + (VOH - VP) / 2.2) - (12 - VM) / 1.0)
    lo = VML + 0.51 * (MIR * ((12 - VP) / 0.51 + 0.06 / 2.2) - (12 - VML) / 1.0)
    ends = []
    b.op("LD", "cqh"); b.op("CMPI", imm=q(0.5))
    jl = b.skip_if(False)
    b.op("LD", "cio"); b.op("ADD", "c_dh"); b.op("MINI", imm=q(VOH)); b.op("ST", "cio")
    b.op("ADDI", imm=q(-hi)); b.op("CMPI", imm=0)
    jn = b.skip_if(False)
    b.op("ST", "_d"); b.op("LD", "c_dh"); b.op("CMPI", imm=1)
    jn2 = b.skip_if(False)                                 # rising (else: clipped at VOH, no oscillation)
    b.op("RCP"); b.op("MUL", "_d"); b.op("MINI", imm=ONE); b.op("ST", "_f")
    b.op("MUL", "c_dl"); b.op("ADDI", imm=q(hi)); b.op("ST", "cio")
    b.op("LDI", imm=0); b.op("ST", "cqh"); b.op("LDI", imm=ONE); b.op("SUB", "_f"); b.op("ST", "cqf")
    ends.append(jump(b))
    b.land(jn); b.land(jn2)
    b.op("LDI", imm=ONE); b.op("ST", "cqf")
    ends.append(jump(b))
    b.land(jl)
    b.op("LD", "cio"); b.op("ADD", "c_dl"); b.op("MINI", imm=q(VOH)); b.op("MAXI", imm=q(0.6)); b.op("ST", "cio")
    b.op("ADDI", imm=q(-lo)); b.op("CMPI", imm=0)
    jn = b.skip_if(True)
    b.op("ST", "_d"); b.op("LD", "c_dl"); b.op("CMPI", imm=0)
    jn2 = b.skip_if(True)                                  # falling
    b.op("RCP"); b.op("MUL", "_d"); b.op("MINI", imm=ONE); b.op("ST", "_f")
    b.op("MUL", "c_dh"); b.op("ADDI", imm=q(lo)); b.op("ST", "cio")
    b.op("LDI", imm=ONE); b.op("ST", "cqh"); b.op("LD", "_f"); b.op("ST", "cqf")
    ends.append(jump(b))
    b.land(jn); b.land(jn2)
    b.op("LDI", imm=0); b.op("ST", "cqf")
    for e in ends:
        b.land(e)
    # TVCA: sq = 0.58 + (VOH - 0.58) cqf
    b.op("LD", "c6"); b.op("MULI", imm=q(MIR)); b.op("ADDI", imm=q(tvca_c0(2.2e6) - 0.58))
    b.op("MACI", "cqf", imm=q(-(VOH - 0.58)))
    b.op("MULI", imm=q(1 / (1 + PULL))); b.op("ADDI", imm=q(PULL * 12 / (1 + PULL)))
    b.op("MAXI", imm=q(0.71)); b.op("MINI", imm=q(VOH)); b.op("ST", "cto")


# ------------------------------------------------------------------------------------------------ band-pass
BP_R1 = 1.0 / (1 / 2e3 + 1 / 27e3 + 1 / 3e3)
BP_R3, BP_RF, BP_C = 68.0, 39e3, 0.033e-6
BP_LOW, BP_AOL, BP_I0 = -0.07, 100.0, 2e-6
BEEP_BP = (math.sqrt(753 * 2571), math.sqrt(753 * 2571) / (2571 - 753), 0.13)   # ngspice AC at rest: f0, Q, gain


def bandpass(b, dtm):
    """L3-4 3/2/4 MFB band-pass (C502 / C503 .033u, R509 39k, R505 68, + input R510 68k to ground): output rests at
    ~0.6 V and swings up only; two capacitor states (va = V(C502), vb = V(C503)), finite gain BP_AOL while active,
    clamped at BP_LOW (inverting input absorbs the current) until the drive pulls BP_I0 out. Input: click (12 V via
    R208 + R207) and bottle (R604); output AC volts in bp_o (rest 0). Beep (R504 27k): small-signal path of the same
    circuit at rest (the drive never lifts the amp off its limit), a 2-pole band-pass added to bp_o"""
    R1, R3, Rf, C = BP_R1, BP_R3, BP_RF, BP_C
    a = 1.0 / (1 + 1.0 / BP_AOL)
    e = a / BP_AOL
    A = [[-1 / (Rf * C), 1 / (Rf * C)],
         [(-(1 / R1 + 1 / R3) * (1 - e) + 1 / Rf) / C, (-(1 / R1 + 1 / R3) * e - 1 / Rf) / C]]
    dt = DT * dtm
    Ad, Bd = zoh(A, [0.0, 1 / (R1 * C)], dt)
    gc = 1 / R1 + 1 / R3
    kc = 1 - math.exp(-dt * gc / (2 * C))
    gin = (1 / R1) / gc
    # input (Thevenin through R1): click 12 V / 3k + bottle / 2k
    b.op("BIT", 0, b.src(LATCH4, 1)); b.op("LDI", imm=0); b.op("LDIF", imm=q(R1 * 12 / 3e3))
    b.op("MACI", "bo2", imm=q(R1 / 2e3)); b.op("ST", "_x")
    # clamped: leave the clamp in this step if the input would pull more than BP_I0 out of the inverting input
    # (C (target - va) kc / dt + L / Rf < -I0), else relax va with the output held at BP_LOW
    b.op("LD", "bp_c"); b.op("CMPI", imm=q(0.5))
    ja = b.skip_if(False)
    b.op("LD", "_x"); b.op("MULI", imm=q(gin)); b.op("SUB", "bp_a")
    b.op("CMPI", imm=q((-BP_I0 - BP_LOW / Rf) * dt / C / kc))
    jg = b.skip_if(False)
    b.op("LD", "_x"); b.op("MULI", imm=q(gin)); b.op("RC", "bp_a", imm=q(kc))
    b.op("ADDI", imm=q(-BP_LOW / a)); b.op("ST", "bp_b")
    b.op("LDI", imm=q(BP_LOW)); b.op("ST", "bp_o")
    je = jump(b)
    b.land(jg)
    b.op("LDI", imm=0); b.op("ST", "bp_c")
    b.land(ja)
    lin(b, "_a", 0.0, [("bp_a", Ad[0][0]), ("bp_b", Ad[0][1]), ("_x", Bd[0])])
    lin(b, "bp_b", 0.0, [("bp_a", Ad[1][0]), ("bp_b", Ad[1][1]), ("_x", Bd[1])])
    b.op("LD", "_a"); b.op("ST", "bp_a"); b.op("SUB", "bp_b"); b.op("MULI", imm=q(a)); b.op("ST", "bp_o")
    b.op("CMPI", imm=q(BP_LOW))
    jk = b.skip_if(True)
    b.op("LDI", imm=ONE); b.op("ST", "bp_c"); b.op("LDI", imm=q(BP_LOW)); b.op("ST", "bp_o")
    b.op("LD", "bp_a"); b.op("ADDI", imm=q(-BP_LOW / a)); b.op("ST", "bp_b")
    b.land(jk)
    b.land(je)
    # beep small-signal path: input = R1 x eo2 / 27k (eo2 = 0.58 + (VOH - 0.58) e2f)
    f0, Q, G = BEEP_BP
    w0 = 2 * math.pi * f0
    Ad, Bd = zoh([[0.0, 1.0], [-w0 * w0, -w0 / Q]], [0.0, 1.0], dt)
    t = (w0 * w0, w0)                                      # states scaled to volts: y = diag(w0^2, w0) x
    Ad = [[Ad[i][j] * t[i] / t[j] for j in range(2)] for i in range(2)]
    Bd = [Bd[i] * t[i] for i in range(2)]
    s = R1 / 27e3 * (VOH - 0.58)                           # the DC part (0.58 V) is blocked
    lin(b, "_b0", 0.0, [("bb0", Ad[0][0]), ("bb1", Ad[0][1]), ("e2f", Bd[0] * s)])
    lin(b, "bb1", 0.0, [("bb0", Ad[1][0]), ("bb1", Ad[1][1]), ("e2f", Bd[1] * s)])
    b.op("LD", "_b0"); b.op("ST", "bb0")
    b.op("LD", "bb1"); b.op("MULI", imm=q(G / Q)); b.op("ADD", "bp_o"); b.op("ST", "bp_o")


# ------------------------------------------------------------------------------------------------ board
OUT_GAIN = 0.5        # x MAME's output scaling (6000 x 0.8): -6 dB headroom for the schematic click / bottle peaks
PROBE = None          # test builds: a register name -> OUT = that register x PROBE_SCALE (no mixer)
PROBE_SCALE = 1.0 / 16


def desertgu():
    b = Board("desertgu")
    for reg, v in (("bov", 5.0), ("e1v", 0.59), ("e2v", 0.49), ("rlv", 5.9), ("rlh", 1.0), ("rov", 6.8), ("roh", 1.0),
                   ("c3", 0.53), ("c5", 0.35), ("c6", 0.42), ("cio", VOH), ("cn1", 6.0), ("cn2", 6.0), ("cnh", 6.0),
                   ("bp_c", 1.0), ("bp_b", -BP_LOW), ("bo2", 0.63), ("cto", 0.71), ("ro3", 0.69), ("so", 0.73),
                   ("s1", 0.73), ("s2", 0.73), ("s6", 0.73), ("bp_o", BP_LOW),
                   ("mix_hp1", 0.69), ("mix_hp2", 0.71), ("mix_hp3", BP_LOW)):   # rest values: no power-up thump
        b.init(reg, v)
    # music (MAME-derived, HW-confirmed): tone generator + TVCA
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.lfsr_noise("nz", 7515, 12.0, 6.0)                   # 4006 / 4030 noise (MAME: breadboarded 7515 Hz), 0 / 12 V
    rifle(b)
    # alternate samples: Road Runner + creature / bottle + beep (24 kHz each)
    b.op("LDI", imm=ONE); b.op("SUB", "_ph"); b.op("ST", "_ph"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(False)
    roadrunner(b, 2)
    creature(b, 2)
    j2 = jump(b)
    b.land(j)
    bottle(b, 2)
    beep(b, 2)
    bandpass(b, 2)
    b.land(j2)
    if PROBE:
        b.op("LD", PROBE); b.op("MULI", imm=q(PROBE_SCALE)); b.op("OUT")
        return b.finish()
    music = 30 * K + logadj(1e6, 75e3, 60)
    b.mixer_op_amp("mix", [("sh_v", 30 * K, 0), ("ro3", 56 * K, 0.1 * U), ("cto", 180 * K, 0.1 * U),
                           ("bp_o", 47 * K, 0.1 * U), ("tone", music, 0.1 * U, True)], rf=100 * K, c_amp=0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 3))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(6000 * 0.8 * OUT_GAIN / 32768 * (1 << 24))))   # MAME gain 6000, route 0.8
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    if len(sys.argv) > 2:
        PROBE = sys.argv[2]
    p = desertgu()
    print(f"desertgu: {len(p.code)} words, worst {p.worst_path()} units, {len(p.regs)} registers")
    if len(sys.argv) > 1:
        open(sys.argv[1], "wb").write(p.binary())
