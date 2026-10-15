"""280-ZZZAP (Midway 1976) / Laguna Racer (Midway 1977) sound board for the discrete sound engine (tools/dsnd.py,
rtl/dsnd_engine.sv).

From the schematic "280 ZZZAP Game Logic P.C. 0610-00907A" (Useful Information/schematics/
280_ZZZap_Game_Logic_(0610-00907A).pdf, the only page: sound circuits top centre / right and lower right), with the
values MAME's nl_280zzzap.cpp (C. D. Howell) corrects from PCB evidence (R46 2.2M, R48 / R51 470k, R11 120k).
Laguna Racer differs only in C18 22 uF (engine inertia) and R30 100k (amplitude-modulation oscillator).
Spec = verilator/snd/zzzap/zz_proto.py (float, per sample); ngspice deck of the schematic = zz_spice.py; stage
constants measured on that deck = zz_char.py.
Latches: S1 = port 2 (74174 F5: D0-D3 accelerator, D4 HI SHIFT, D5 LO SHIFT), S2 = port 5 (74174 F4: D0 BOOM,
D1 ENGINE SOUND OFF, D2 NOISE CR 1 = tyre skid, D3 NOISE CR 2 = post-crash noise, D5 coin counter).
Model: LM3900 Norton stages with the input nodes at fixed voltages and the outputs clamped to the rails (noise
stages: open-loop gain 2250, piecewise linear normal / output-high (minus node free) / output-low modes, exact
zero-order-hold state-space updates); engine: pedal network -> C18 -> H5_4 with D5 (one Newton step on the diode per
sample: EXP + RCP) and the gear-shift cap C19 -> three triangle oscillators (slopes from the ngspice deck) -> diode
OR / C16 -> MC3340 (differential-pair steering a = 1 / (1 + exp(dV / VT)) from its internal schematic) with the
control pin driven by the top oscillator through C13 -> mixer J5_3 / J5_4 (coupling caps C20, C21, C23, C24).
Noise: binary noise (the saturating zener amplifier H4_2) at 24 kHz like MAME. Output = J5_4 at MAME's scale
((V - 2.5) / 1.25), route 1.0. Game Audio: Timing = C18 / C19 (engine response), Oscillator = engine oscillator caps.
"""
import math
from dsnd import Board, LATCH1, LATCH2, q, ONE, FS

H = 1.0 / FS
SECT = []                                            # (section, body index) for budget reports
NEVER = -(1 << 31)                                   # CMPI NEVER: F = 1 (unconditional skip)
# LM3900 at 5 V (ngspice macro, datasheet-corrected: output sink 1.3 mA, inverting bias 30 nA)
VOH, VOL = 4.312, 0.113
VM, VP = 0.516, 0.52
AOL, VR = 2250.0, 2.5
RON = 1000.0                                         # CD4016 on resistance
VT = 0.025864
NVT_D, D_IS_UA = 1.752 * VT, 2.52e-3                 # 1N914 (n, IS in uA)
# accelerator network -> H5_3 output: additive bit weights (ngspice, least squares over codes 0-12)
PED = (1.23300, 0.20924, 0.46089, 0.98037, 1.92104)
# oscillators: f = F (Vs - V0) Hz, triangle TL .. TH (ngspice); (F, Rminus, Rplus, C)
TL, TH, V0 = 1.312, 2.800, 0.50
OSC = {"low": (27.34, 560e3, 270e3, 0.022e-6), "mid": (106.4, 300e3, 150e3, 0.01e-6),
       "top_z": (144.0, 220e3, 110e3, 0.01e-6), "top_l": (157.6, 220e3, 100e3, 0.01e-6)}
IB = 30e-9                                           # LM3900 inverting-input bias: stalled integrators drift up
# MC3340 (ngspice of the Motorola internal schematic)
MC_KC, MC_VMID = 0.15775, 3.1213
MC_VINT, MC_RINT = 5.529, 4702.0
MC_GAIN = 6.04 * 0.99                                # V per mA of steered current
MC_VBE5, MC_RE5 = 0.565, 1.3306                      # Ie5 (mA) = (vin - VBE5) / RE5 (k)
MC_IE8 = 0.7687 / 0.99                               # mA
VTH_IN, RTH_IN = 1.600, 17.35e3
VDS_OFF = 4.998 - 0.585                              # engine-off line through R22 470 and D1
R_ = dict(R1=330e3, C1=10e-6, C2=6800e-12, R2=10e3, R3=820e3, R4=270e3, C3=220e-12,
          R5=39e3, R6=62.0, R7=82e3, R8=39e3, C7=0.1018e-6, C8=0.1018e-6, R9=39e3, R10=62e3, R11=120e3, R12=62e3,
          C4=0.022e-6, R16=1.5e3, C6=0.01e-6, R15=15e3, R13=150e3, C9=0.001e-6, R14=2.7e6,
          R17=10e3, C5=10e-6, R18=2.2e3, R19=1e3, C10=0.1e-6, C11=0.1e-6, R21=1e6, R20=3.3e6)
# C7 / C8 (0.1 uF) x 1.018: the skid resonance of the ngspice deck is 1.8 % below the finite-gain model's (op-amp
# phase lag); Q and peak gain already match with AOL


# ------------------------------------------------------------------ small matrix helpers (no numpy in tools/)
def mmul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(len(B))) for j in range(len(B[0]))] for i in range(len(A))]


def madd(A, B, s=1.0):
    return [[a + s * b for a, b in zip(ra, rb)] for ra, rb in zip(A, B)]


def eye(n):
    return [[1.0 if i == j else 0.0 for j in range(n)] for i in range(n)]


def expm(A):
    n = len(A)
    nrm = max(sum(abs(x) for x in r) for r in A)
    k = max(0, int(math.ceil(math.log2(nrm))) + 1) if nrm > 0 else 0
    As = [[x / 2 ** k for x in r] for r in A]
    E, T = eye(n), eye(n)
    for i in range(1, 22):
        T = [[x / i for x in r] for r in mmul(T, As)]
        E = madd(E, T)
    for _ in range(k):
        E = mmul(E, E)
    return E


def solve(A, b):
    n = len(A)
    m = [list(map(float, r)) + [float(v)] for r, v in zip(A, b)]
    for i in range(n):
        p = max(range(i, n), key=lambda r: abs(m[r][i]))
        m[i], m[p] = m[p], m[i]
        for r in range(n):
            if r != i:
                f = m[r][i] / m[i][i]
                m[r] = [x - f * y for x, y in zip(m[r], m[i])]
    return [m[i][n] / m[i][i] for i in range(n)]


def lin_sys(f, nx, nu):
    """A, B of an affine dx/dt = f(x, u) (u ends with the constant 1)"""
    z = [0.0] * nx
    zu = [0.0] * nu
    f0 = f(z, zu)
    A = [[0.0] * nx for _ in range(nx)]
    B = [[0.0] * nu for _ in range(nx)]
    for i in range(nx):
        e = [0.0] * nx; e[i] = 1.0
        c = f(e, zu)
        for r in range(nx):
            A[r][i] = c[r] - f0[r]
    for j in range(nu):
        e = [0.0] * nu; e[j] = 1.0
        c = f(z, e)
        for r in range(nx):
            B[r][j] = c[r] - f0[r]
    return A, B


def zoh(f, nx, nu):
    A, B = lin_sys(f, nx, nu)
    M = [[0.0] * (nx + nu) for _ in range(nx + nu)]
    for i in range(nx):
        for j in range(nx):
            M[i][j] = A[i][j] * H
        for j in range(nu):
            M[i][nx + j] = B[i][j] * H
    E = expm(M)
    return [r[:nx] for r in E[:nx]], [r[nx:] for r in E[:nx]]


def dc(f, nx, u):
    A, B = lin_sys(f, nx, len(u))
    return solve(A, [-sum(B[i][j] * u[j] for j in range(len(u))) for i in range(nx)])


# ------------------------------------------------------------------ noise stages (as zz_proto.py)
def iplus(r):
    return (5 - VP) / r


def lin_out(q_, one):
    vo = (q_ + (VM + VR / AOL) * one) / (1 + 1 / AOL)
    return vo, vo - q_


def noiselp_f(mode):
    r, ip = R_, iplus(R_["R3"])

    def f(x, u):
        n1, q3 = x; vin, one = u
        if mode == "n":
            vo, vm = lin_out(q3, one)
        elif mode == "h":
            vo = VOH * one; vm = vo - q3
        else:
            vo, vm = VOL * one, VM * one
        dn1 = ((vin - n1) / r["R1"] + (vo - n1) / r["R4"] - (n1 - vm) / r["R2"]) / r["C2"]
        dq3 = 0.0 * q3 if mode == "l" else (ip * one - (n1 - vm) / r["R2"]) / r["C3"]
        return [dn1, dq3]
    return f


def skid_parts():
    r = R_
    i43, i44 = iplus(r["R7"]), iplus(r["R10"])
    g9, g11 = 1 / r["R9"], 1 / r["R11"]

    def nodes(x, one):
        c7, c8 = x
        vn2 = ((VM + VR / AOL) * one + c7 + c8 / AOL) / (1 + 1 / AOL)
        v43, vm3 = vn2 - c8, vn2 - c7
        v44 = (i44 * one - v43 * g9 + (VM + VR / AOL) * one * (g9 + g11)) / (g11 + (g9 + g11) / AOL)
        return vn2, v43, vm3, v44

    def f(x, u):
        vn, one = u
        vn2, v43, vm3, v44 = nodes(x, one)
        dc7 = (i43 * one - (v43 - vm3) / r["R8"]) / r["C7"]
        dc8 = ((vn - vn2) / r["R5"] + (v44 - vn2) / r["R12"] - vn2 / r["R6"] - r["C7"] * dc7) / r["C8"]
        return [dc7, dc8]
    return f, nodes


def crash_f(mode):
    r, ip = R_, iplus(R_["R14"])

    def f(x, u):
        q4, n3, q9 = x; vn, one = u
        i16 = (vn - q4 - n3) / r["R16"]
        if mode == "n":
            vo, vm = lin_out(q9, one)
        elif mode == "h":
            vo = VOH * one; vm = vo - q9
        else:
            vo, vm = VOL * one, VM * one
        dn3 = (i16 + (vo - n3) / r["R13"] - (n3 - vm) / r["R15"]) / r["C6"]
        dq9 = 0.0 * q9 if mode == "l" else (ip * one - (n3 - vm) / r["R15"]) / r["C9"]
        return [i16 / r["C4"], dn3, dq9]
    return f


def boom_nodes(mode, x, one):
    q5, q10, q11 = x
    if mode == "h":
        vo = VOH * one; n4 = vo + q11; vm = n4 - q10
    elif mode == "l":
        vm = VM * one; n4 = vm + q10; vo = VOL * one
    else:
        vm = ((VM + VR / AOL) * one - (q10 - q11) / AOL) / (1 + 1 / AOL)
        n4 = vm + q10; vo = n4 - q11
    return vo, vm, n4


def boom_f(mode, sw):
    r, ip = R_, iplus(R_["R20"])
    rs = RON + r["R18"]

    def f(x, u):
        q5, q10, q11 = x; vn, one = u
        vo, vm, n4 = boom_nodes(mode, x, one)
        i_s = (vn - q5 - n4) / rs if sw else 0.0 * vn
        dq5 = (i_s - q5 / r["R17"]) / r["C5"]
        if mode == "l":
            dq10 = (i_s - n4 / r["R19"]) / (r["C10"] + r["C11"])
            dq11 = dq10
        else:
            dq10 = (ip * one - (vo - vm) / r["R21"]) / r["C10"]
            dq11 = (i_s - n4 / r["R19"] - r["C10"] * dq10) / r["C11"]
        return [dq5, dq10, dq11]
    return f


# ------------------------------------------------------------------ code generation helpers
def lin(b, dst, terms, const=0.0):
    """dst = const + sum(k x reg) (dst may be one of the regs: everything is read before the ST)"""
    terms = [(r, k) for r, k in terms if abs(q(k)) > 0]
    if terms and abs(terms[0][1] - 1.0) < 1e-12:
        b.op("LD", terms[0][0]); terms = terms[1:]
    else:
        b.op("LDI", imm=q(const)); const = 0.0
    for reg, k in terms:
        b.op("MACI", reg, imm=q(k))
    if const:
        b.op("ADDI", imm=q(const))
    if dst:
        b.op("ST", dst)


def jump(b):
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def ss(b, states, u_reg, Ad, Bd, out=None):
    """states <- Ad states + Bd [u_reg, 1]; the last state is written in place, the others through temps"""
    n = len(states)
    tmp = []
    for i in range(n):
        terms = [(states[j], Ad[i][j]) for j in range(n)] + [(u_reg, Bd[i][0])]
        dst = states[i] if i == n - 1 else "_ss%d" % i
        lin(b, dst, terms, Bd[i][1])
        tmp.append(dst)
    for i in range(n - 1):
        b.op("LD", "_ss%d" % i)
        b.op("ST", states[i])


def predict(b, states, u_reg, Ad, Bd, row):
    """acc = row . (Ad states + Bd [u, 1]) (the next-sample value of a linear output)"""
    n = len(states)
    terms = [(states[j], sum(row[i] * Ad[i][j] for i in range(n))) for j in range(n)]
    terms.append((u_reg, sum(row[i] * Bd[i][0] for i in range(n))))
    const = sum(row[i] * Bd[i][1] for i in range(n))
    lin(b, None, terms, const)


def norton_stage(b, name, states, u_reg, modes, row_out, qidx, out_reg, q_low, hi_ok=True):
    """piecewise-linear Norton stage: predict the normal-mode output (row_out . x_new + c); above VOH -> output-high
    update, below VOL -> output-low update (q pinned), else normal. out_reg = the op-amp output"""
    (An, Bn), (Ah, Bh), (Al, Bl) = modes["n"], modes["h"], modes["l"]
    row, c0 = row_out
    predict(b, states, u_reg, An, Bn, row)
    if c0:
        b.op("ADDI", imm=q(c0))
    b.op("ST", out_reg)
    b.op("CMPI", imm=q(VOH))
    j_hi = b.skip_if(True)
    b.op("CMPI", imm=q(VOL))
    j_lo = b.skip_if(False)
    ss(b, states, u_reg, An, Bn)                          # normal
    j1 = jump(b)
    b.land(j_hi)
    ss(b, states, u_reg, Ah, Bh)                          # output high, minus node free
    b.op("LDI", imm=q(VOH)); b.op("ST", out_reg)
    j2 = jump(b)
    b.land(j_lo)
    ss(b, states, u_reg, Al, Bl)                          # output low, feedback cap pinned
    if callable(q_low):
        q_low(b)
    else:
        b.op("LDI", imm=q(q_low)); b.op("ST", states[qidx])
    b.op("LDI", imm=q(VOL)); b.op("ST", out_reg)
    b.land(j1); b.land(j2)


# ------------------------------------------------------------------ the board
def board(variant):
    lag = variant == "lagunar"
    b = Board(variant)
    c18 = 22e-6 if lag else 47e-6
    oscs = ("low", "mid", "top_l" if lag else "top_z")
    ini = {}

    def init(reg, v):
        ini[reg] = v

    SECT.append(("Game Audio rates", len(b.code)))
    # ---------------- Game Audio rates
    b.kreg("k_t", 1.0, "T", clamp=False)                 # timing factor (C18 / C19)
    for i, nm in enumerate(oscs):
        F, rm, rp, c = OSC[nm]
        rho = (1 / (rp + RON) - 1 / rm) / (1 / rm)
        kd = F * (TH - TL) * (1 + 1 / rho)
        b.kreg("su%d" % i, rho * kd * H, "O", clamp=False)
        b.kreg("sd%d" % i, kd * H, "O", clamp=False)
        b.kreg("ib%d" % i, IB / c * H, "O", clamp=False)

    SECT.append(("accelerator -> H5_3 -> C18 (ch", len(b.code)))
    # ---------------- slow engine-speed path at 24 kHz: samples alternate (_ph) between the accelerator / C18 update
    # and the H5_4 / C19 update, each with a 2-sample step
    H2 = 2 * H
    b.op("LDI", imm=ONE); b.op("SUB", "_ph"); b.op("ST", "_ph"); b.op("CMPI", imm=q(0.5))
    j_alt = b.skip_if(False)
    # ---------------- accelerator -> H5_3 -> C18 (charge 47k, discharge also 10k + D4, H5_4 input, engine-off switch)
    b.op("LDI", imm=q(PED[0]))
    for i in range(4):
        b.op("BIT", 0, b.src(LATCH1, i))
        b.op("SKNF", imm=1)
        b.op("ADDI", imm=q(PED[1 + i]))
    b.op("MINI", imm=q(VOH))
    b.op("SUB", "vc"); b.op("ST", "_t")                 # v53 - vc
    b.op("LDI", imm=q(-0.37)); b.op("SUB", "_t"); b.op("MAXI", imm=0); b.op("ST", "_d4")
    b.op("LD", "vc"); b.op("ADDI", imm=q(-VP)); b.op("MAXI", imm=0); b.op("ST", "_d54")
    lin(b, None, [("_t", H2 / (47e3 * c18)), ("_d4", -H2 / (10.53e3 * c18)), ("_d54", -H2 / (560e3 * c18))])
    b.op("BIT", 0, b.src(LATCH2, 1))
    b.op("SKNF", imm=1)
    b.op("MACI", "vc", imm=q(-H2 / ((270.0 + RON) * c18)))
    b.op("MUL", "k_t"); b.op("ADD", "vc"); b.op("ST", "vc")
    j_alt2 = jump(b)
    b.land(j_alt)

    SECT.append(("H5_4 (uA, V)", len(b.code)))
    # ---------------- H5_4 (uA, V): i0 = I+ - I(2.7M); D5 (Newton on Vd) + 560k; hi gear adds 561k + C19
    # i0 = km (C18 - VP) / 560k + I0 (uA): km, I0 = the R54 / R55 / R57 currents and the mirror ratio, calibrated
    # on the ngspice H5_4 stage (zz_char.py speed: Vs within 4 mV for C18 = 1.0 .. 4.2 V)
    km, i0c = 0.973, -0.656
    r59 = 0.561                                          # Mohm (R59 + switch)
    lin(b, "_i0", [("_d54", km / 0.56)], i0c)
    b.op("BIT", 0, b.src(LATCH1, 5))                    # lo gear: C19 discharges through 10k + 2 switches
    b.op("SKNF", imm=3)
    b.op("LD", "vq"); b.op("MULI", imm=q(-H2 / ((10e3 + 2 * RON) * 2.2e-6))); b.op("MUL", "k_t")
    b.op("SKNF", imm=2)                                  # (F still = lo)
    b.op("ADD", "vq"); b.op("ST", "vq")
    # x = diode current: lo / none: i0; hi: (i0 - (vd5 - vq) / r59) / (1 + 0.56 / r59)
    b.op("LD", "_i0"); b.op("ST", "_x")
    b.op("BIT", 0, b.src(LATCH1, 4))
    j_nohi = b.skip_if(False)
    b.op("LD", "vd5"); b.op("SUB", "vq"); b.op("MULI", imm=q(-1 / r59)); b.op("ADD", "_i0")
    b.op("MULI", imm=q(1 / (1 + 0.56 / r59))); b.op("ST", "_x")
    b.land(j_nohi)
    b.op("LD", "_x"); b.op("CMPI", imm=q(0.002))
    j_off = b.skip_if(False)
    # Newton: vd += NVT (x / Id(vd) - 1), Id = IS 2^(vd / (NVT ln 2))
    b.op("LD", "vd5"); b.op("MULI", imm=q(1 / (NVT_D * math.log(2)))); b.op("ADDI", imm=q(math.log2(D_IS_UA)))
    b.op("EXP"); b.op("MAXI", imm=q(0.125)); b.op("RCP"); b.op("MUL", "_x"); b.op("ADDI", imm=-ONE)
    b.op("MINI", imm=q(8.0))
    b.op("MULI", imm=q(NVT_D)); b.op("ADD", "vd5"); b.op("MAXI", imm=q(0.05)); b.op("MINI", imm=q(0.6))
    b.op("ST", "vd5")
    lin(b, None, [("vd5", 1.0), ("_x", 0.56)], VM)
    j_on = jump(b)
    b.land(j_off)
    # diode off: hi gear -> feedback only through C19 (vs = VM + vq + r59 i0); otherwise the output sits low
    b.op("LDI", imm=q(VOL))
    b.op("BIT", 0, b.src(LATCH1, 4))
    b.op("SKNF", imm=4)
    b.op("LD", "_i0"); b.op("MULI", imm=q(r59)); b.op("ADD", "vq"); b.op("ADDI", imm=q(VM))
    b.land(j_on)
    b.op("MINI", imm=q(VOH)); b.op("MAXI", imm=q(VOL)); b.op("ST", "vs")
    # hi gear: C19 charges through 561k: vq += (vs - vq - VM) / 561k x h / C19
    b.op("BIT", 0, b.src(LATCH1, 4))
    j_c19 = b.skip_if(False)
    b.op("SUB", "vq"); b.op("ADDI", imm=q(-VM)); b.op("MULI", imm=q(H2 / (r59 * 1e6 * 2.2e-6))); b.op("MUL", "k_t")
    b.op("ADD", "vq"); b.op("ST", "vq")
    b.land(j_c19)
    b.land(j_alt2)

    SECT.append(("oscillators", len(b.code)))
    # ---------------- oscillators: triangle TL .. TH, rates (su g + ib) up / (sd g - ib) down, exact turn carry
    b.op("LD", "vs"); b.op("ADDI", imm=q(-V0)); b.op("MAXI", imm=0); b.op("ST", "_g")
    for i, nm in enumerate(oscs):
        F, rm, rp, c = OSC[nm]
        rho = (1 / (rp + RON) - 1 / rm) / (1 / rm)
        v, up = "o%d" % i, "up%d" % i
        b.op("LD", up); b.op("CMPI", imm=q(0.5))
        j_dn = b.skip_if(False)
        b.op("LD", "_g"); b.op("MUL", "su%d" % i); b.op("ADD", "ib%d" % i); b.op("ADD", v); b.op("ST", v)
        b.op("CMPI", imm=q(TH))
        j_e1 = b.skip_if(False)
        b.op("ADDI", imm=q(-TH)); b.op("MULI", imm=q(-1 / rho)); b.op("ADDI", imm=q(TH)); b.op("ST", v)
        b.op("LDI", imm=0); b.op("ST", up)
        j_e2 = jump(b)
        b.land(j_dn)
        b.op("LD", "_g"); b.op("MUL", "sd%d" % i); b.op("SUB", "ib%d" % i); b.op("ST", "_t")
        b.op("LD", v); b.op("SUB", "_t"); b.op("MINI", imm=q(VOH)); b.op("ST", v)
        b.op("CMPI", imm=q(TL))
        j_e3 = b.skip_if(True)
        b.op("ADDI", imm=q(-TL)); b.op("MULI", imm=q(-rho)); b.op("ADDI", imm=q(TL)); b.op("ST", v)
        b.op("LDI", imm=ONE); b.op("ST", up)
        b.land(j_e1); b.land(j_e2); b.land(j_e3)
        init(v, VOH)

    SECT.append(("diode OR -> C16 -> MC3340 inpu", len(b.code)))
    # ---------------- diode OR -> C16 -> MC3340 input
    b.op("LD", "o0"); b.op("CMP", "o1"); b.op("SKF", imm=1); b.op("LD", "o1")
    b.op("ADDI", imm=q(-0.443)); b.op("MULI", imm=q(1 / 1.032)); b.op("ST", "_vx")
    b.op("ADDI", imm=q(-VTH_IN))
    b.op("RC", "q16", imm=q(1 - math.exp(-H / (RTH_IN * 10e-6))))
    b.op("LD", "_vx"); b.op("SUB", "q16")
    b.op("ADDI", imm=q(-MC_VBE5)); b.op("MAXI", imm=0); b.op("MULI", imm=q(1 / MC_RE5))
    b.op("ADDI", imm=q(-MC_IE8)); b.op("ST", "_di")       # Ie5 - Ie8 (mA)
    SECT.append(("control pin", len(b.code)))
    # ---------------- control pin: 10k from 12 V, 3.3k, pin Thevenin, C13 from the top oscillator (backward
    # Euler), engine-off line through 470 + D1 when it would conduct
    g0 = 1 / 10e3 + 1 / 3.3e3 + 1 / MC_RINT
    s0 = 12 / 10e3 + MC_VINT / MC_RINT
    gc = 10e-6 / H
    gd = 1 / 495.0
    b.op("LD", "o2"); b.op("SUB", "q13"); b.op("ST", "_t")
    b.op("MULI", imm=q(gc / (g0 + gc))); b.op("ADDI", imm=q(s0 / (g0 + gc))); b.op("ST", "_v2")
    b.op("BIT", 0, b.src(LATCH2, 1))
    j_e = b.skip_if(False)
    b.op("CMPI", imm=q(VDS_OFF))
    j_e2 = b.skip_if(True)
    b.op("LD", "_t"); b.op("MULI", imm=q(gc / (g0 + gd + gc)))
    b.op("ADDI", imm=q((s0 + gd * VDS_OFF) / (g0 + gd + gc))); b.op("ST", "_v2")
    b.land(j_e); b.land(j_e2)
    b.op("LD", "o2"); b.op("SUB", "_v2"); b.op("ST", "q13")
    # a = e / (1 + e), e = exp(-KC (v2 - VMID) / VT)
    b.op("LD", "_v2"); b.op("ADDI", imm=q(-MC_VMID)); b.op("MULI", imm=q(-MC_KC / (VT * math.log(2))))
    b.op("EXP"); b.op("ST", "_t"); b.op("ADDI", imm=ONE); b.op("RCP"); b.op("MUL", "_t")
    b.op("MUL", "_di"); b.op("MULI", imm=q(-MC_GAIN * 0.982)); b.op("ST", "_ve")   # follower output, AC (V)

    SECT.append(("noise (24 kHz binary) -> H4_1 ", len(b.code)))
    # ---------------- noise (24 kHz binary) -> H4_1 low pass (mode-switched) -> C1
    b.op("LD", "_ph"); b.op("CMPI", imm=q(0.5))
    j_n = b.skip_if(False)
    b.op("NOISE", 0, 0); b.op("LDI", imm=q(VOL)); b.op("LDIF", imm=q(VOH)); b.op("ST", "_nz")
    b.land(j_n)
    b.op("LD", "_nz"); b.op("SUB", "q1"); b.op("ST", "_vin")
    md = {m: zoh(noiselp_f(m), 2, 2) for m in "nhl"}
    row = [0.0, 1 / (1 + 1 / AOL)]
    norton_stage(b, "lp", ["n1", "q3"], "_vin", md, (row, (VM + VR / AOL) / (1 + 1 / AOL)), 1, "vn", VOL - VM)
    b.op("LD", "_vin"); b.op("SUB", "n1"); b.op("MULI", imm=q(H / (R_["R1"] * R_["C1"]))); b.op("ADD", "q1")
    b.op("ST", "q1")
    ip = iplus(R_["R3"])
    x_lp = [VM + R_["R2"] * ip, VM + R_["R2"] * ip + R_["R4"] * ip - VM]
    init("n1", x_lp[0]); b.init("q3", x_lp[1]); b.init("q1", (VOH + VOL) / 2 - x_lp[0])
    init("vn", x_lp[1] + VM)
    vn_dc = x_lp[1] + VM

    SECT.append(("skid", len(b.code)))
    # ---------------- skid: linear (H4_3 / H4_4 inside the rails at these levels), output clamped
    fs_, nodes = skid_parts()
    Ad, Bd = zoh(fs_, 2, 2)
    ss(b, ["c7", "c8"], "vn", Ad, Bd)
    # v44 = affine in c7, c8 (nodes() is affine: read its coefficients)
    o0 = nodes([0.0, 0.0], 1.0)[3]
    o1 = nodes([1.0, 0.0], 1.0)[3] - o0
    o2 = nodes([0.0, 1.0], 1.0)[3] - o0
    lin(b, None, [("c7", o1), ("c8", o2)], o0)
    b.op("MINI", imm=q(VOH)); b.op("MAXI", imm=q(VOL)); b.op("ST", "v44")
    xs = dc(fs_, 2, [vn_dc, 1.0])
    init("c7", xs[0]); b.init("c8", xs[1]); b.init("v44", o0 + o1 * xs[0] + o2 * xs[1])

    SECT.append(("post-crash noise H5_2 (mode-sw", len(b.code)))
    # ---------------- post-crash noise H5_2 (mode-switched)
    md = {m: zoh(crash_f(m), 3, 2) for m in "nhl"}
    row = [0.0, 0.0, 1 / (1 + 1 / AOL)]
    norton_stage(b, "cr", ["q4", "n3", "q9"], "vn", md, (row, (VM + VR / AOL) / (1 + 1 / AOL)), 2, "v52", VOL - VM)
    xc = dc(crash_f("n"), 3, [vn_dc, 1.0])
    for k, v in zip(("q4", "n3", "q9"), xc):
        init(k, v)
    init("v52", lin_out(xc[2], 1.0)[0])

    SECT.append(("boom H5_1 (G4_A at its input", len(b.code)))
    # ---------------- boom H5_1 (G4_A at its input: two matrix sets)
    def boom_low(b):                                    # output low: q11 = n4 - VOL = q10 + VM - VOL
        b.op("LD", "q10"); b.op("ADDI", imm=q(VM - VOL)); b.op("ST", "q11")
    xb = dc(boom_f("n", 0), 3, [vn_dc, 1.0])
    vo0 = boom_nodes("n", [0.0, 0.0, 0.0], 1.0)[0]
    rown = [boom_nodes("n", e, 1.0)[0] - vo0 for e in ([1.0, 0, 0], [0, 1.0, 0], [0, 0, 1.0])]
    b.op("BIT", 0, b.src(LATCH2, 0))
    j_bon = b.skip_if(True)
    for sw in (0, 1):
        md = {m: zoh(boom_f(m, sw), 3, 2) for m in "nhl"}
        norton_stage(b, "bm", ["q5", "q10", "q11"], "vn", md, (rown, vo0), 2, "v51", boom_low)
        if sw == 0:
            j_bend = jump(b)
            b.land(j_bon)
    b.land(j_bend)
    for k, v in zip(("q5", "q10", "q11"), xb):
        init(k, v)
    init("v51", boom_nodes("n", xb, 1.0)[0])

    SECT.append(("mixer (mA, V, kohm)", len(b.code)))
    # ---------------- mixer (mA, V, kohm): noise sum node through C20, engine through 33.2k + C21, J5_3, J5_4
    # i20 = sum g (v - VM - q20) over the connected sources (C20 >> their conductances: forward Euler exact enough)
    # (q20 here = the sum-node side of C20 = its voltage + VM)
    b.op("LD", "v51"); b.op("MULI", imm=q(1 / 12.0)); b.op("MACI", "q20", imm=q(-1 / 12.0))
    for bit, reg, r in ((2, "v44", 13.0), (3, "v52", 151.0)):
        b.op("BIT", 0, b.src(LATCH2, bit))
        b.op("SKNF", imm=2)
        b.op("MACI", reg, imm=q(1 / r)); b.op("MACI", "q20", imm=q(-1 / r))
    b.op("ST", "_i20")
    b.op("MULI", imm=q(H / 10e-6 / 1e3)); b.op("ADD", "q20"); b.op("ST", "q20")
    # engine: i21 = (ve - q21) / (33.2k + h / C21) (backward Euler)
    r21 = 33.2 + H / 10e-6 / 1e3
    b.op("LD", "_ve"); b.op("SUB", "q21"); b.op("MULI", imm=q(1 / r21)); b.op("ST", "_i21")
    b.op("MULI", imm=q(H / 10e-6 / 1e3)); b.op("ADD", "q21"); b.op("ST", "q21")
    # J5_3 relative to its DC: vo' = (q23 / rp - i20 - i21) / (1 / 100k + 1 / rp), clamped to the rails
    rp = 5.0 + H / 10e-6 / 1e3
    gsum = 1 / 100.0 + 1 / rp
    vdc53 = VM + 100.0 * (4.92 - VP) / 220.0
    lin(b, None, [("q23", 1 / rp / gsum), ("_i20", -1 / gsum), ("_i21", -1 / gsum)])
    b.op("MINI", imm=q(VOH - vdc53)); b.op("MAXI", imm=q(VOL - vdc53)); b.op("ST", "_vo")
    b.op("SUB", "q23"); b.op("MULI", imm=q(H / 10e-6 / 1e3 / rp)); b.op("ADD", "q23"); b.op("ST", "q23")
    # J5_4: i24 = (vo' - q24) / (100k + h / C24); out = -100k i24 / 1.25
    r24 = 100.0 + H / 0.1e-6 / 1e3
    b.op("LD", "_vo"); b.op("SUB", "q24"); b.op("MULI", imm=q(1 / r24)); b.op("ST", "_t")
    b.op("MULI", imm=q(H / 0.1e-6 / 1e3)); b.op("ADD", "q24"); b.op("ST", "q24")
    b.op("LD", "_t"); b.op("MULI", imm=q(-100.0 / 1.25)); b.op("OUT")
    # DC operating point of the coupling caps: q20 = boom DC - VM, q21 = q23 = q24 = 0
    init("q20", boom_nodes("n", xb, 1.0)[0])
    # MC3340 side at power-up: oscillators stalled at VOH
    init("q13", VOH - s0 / g0)
    init("q16", (VOH - 0.443) / 1.032 - VTH_IN)
    init("vd5", 0.3)
    SECT.append(("end", len(b.code)))
    # own init block (first body sample, then END): DC operating point; costs 3 units on normal samples
    body, b.code = b.code, []
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for reg, v in ini.items():
        b.op("LDI", imm=q(v)); b.op("ST", reg)
    b.op("LDI", imm=ONE); b.op("ST", "_ini"); b.op("END")
    b.land(j)
    off = len(b.code)
    b.code += body
    SECT[:] = [(nm, i + off) for nm, i in SECT]
    SECT.insert(0, ("init guard", 0))
    return b.finish()


def zzzap():
    return board("zzzap")


def lagunar():
    return board("lagunar")


if __name__ == "__main__":
    import sys
    out = sys.argv[1] if len(sys.argv) > 1 else "."
    for f in (zzzap, lagunar):
        p = f()
        open(f"{out}/{p.name}.bin", "wb").write(p.binary())
        print(f"{p.name}: {len(p.code)} words, worst sample {p.worst_path()} units, {len(p.regs)} registers")
