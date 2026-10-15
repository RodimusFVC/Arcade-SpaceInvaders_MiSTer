"""Guided Missile (Midway 1977) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

From the game board schematic (Useful Information/schematics/Guided Missile - gameboard schematics.pdf), not MAME
(MAME plays two samples). Spec: verilator/snd/gmissile/gm_proto.py, ngspice reference gm_spice.py (same part names).
Port 3 = S1 (D2 coin counter, D3 sound on, D4 right missile, D5 left explosion, D6 left missile, D7 right explosion;
MAME's L / R naming), port 5 = S2 (D0-D2 airplane / copter / jet pan, D3-D5 tank pan: 4051 ladders), port 7 = S4
(D0 airplane, D1 tank, D2 copter, D5 jet). Every circuit is LM3900 based: integrator / Schmitt oscillators, noise gates
with sink-limited outputs (1.3 mA) into RC ladders, Norton VCOs with 4016 switches. Two summers (R / L) with their
volume pots at 50 %; the engine is mono: output = (R + L) around the summers' DC.
Rates: common work every 48 kHz sample (noise, jet VCO, pan nodes); each side's missile VCO / filter, bus and summer
at 24 kHz (alternating samples); envelopes / noise gates / vehicles at 12 kHz; slow terms at 6 kHz. A bit-reversed
counter picks the tiers so every tier is evenly spaced. Units: V, mA, kOhm, mS.
"""
import math
import sys
import numpy as np
from dsnd import Board, LATCH1, LATCH2, LATCH4, q, ONE

FS = 48000.0
DT = 1.0 / FS
K, M, U, N, P = 1.0, 1e3, 1e-3, 1e-6, 1e-9      # kOhm; capacitors so that R(kOhm) x C = seconds
VB = 0.55                                         # LM3900 - input
VOH = 10.82
V12, V5, RPU = 12.0, 5.0, 1.0                     # latch outputs: 1k pull-up to +5
ISNK, ISRC = 1.3, 10.0                            # mA
TD = 12e-6                                        # Schmitt 0 -> 6 V at 0.5 V/us: 4016 closes this long after the crossing
VD = 0.42                                         # 1N4148 charging a cap from a pull-up / LM3900 output
NEVER = -(1 << 31)


def vp(i_ma):
    """LM3900 + input diode at current i (mA)"""
    return 0.025852 * math.log(i_ma * 1e-3 / 1e-14)


def expm(a):
    a = np.array(a, float)
    s = max(0, int(math.ceil(math.log2(max(np.abs(a).sum(1).max(), 1e-30)))) + 4)
    b = a / 2 ** s
    r, t = np.eye(len(a)), np.eye(len(a))
    for k in range(1, 20):
        t = t @ b / k
        r = r + t
    for _ in range(s):
        r = r @ r
    return r


def zoh(a, b, h):
    n, m = len(a), np.array(b).shape[1]
    big = np.zeros((n + m, n + m))
    big[:n, :n], big[:n, n:] = a, b
    e = expm(big * h)
    return e[:n, :n], e[:n, n:]


def kexp(tau, h):
    return 1.0 - math.exp(-h / tau)


# ----------------------------------------------------------------------------------------------- code helpers
def lin(b, dst, const, terms):
    """dst = const + sum(k x reg) (zero terms dropped)"""
    terms = [(r, k) for r, k in terms if abs(k) > 1e-12]
    if not const and terms and abs(terms[0][1] - 1.0) < 1e-15:
        b.op("LD", terms[0][0])
        terms = terms[1:]
    else:
        b.op("LDI", imm=q(const))
    for r, k in terms:
        b.op("MACI", r, imm=q(k))
    b.op("ST", dst)


def jump(b):
    b.op("CMPI", imm=NEVER)
    return b.skip_if(True)


def gate_rc(b, t, vc, vo, r):
    """LM3900 noise gate with a 27k pull-up into r -> cap at vc: vo = t unless the 1.3 mA sink / ISRC source binds"""
    g = 1 / r + 1 / 27.0
    a = (1 / r) / g
    lo, hi = (V12 / 27.0 - ISNK) / g, (V12 / 27.0 + ISRC) / g
    b.op("LD", vc); b.op("MULI", imm=q(a)); b.op("ST", "_g")
    b.op("LD", t); b.op("SUB", "_g"); b.op("MAXI", imm=q(lo)); b.op("MINI", imm=q(hi)); b.op("ADD", "_g")
    b.op("MAXI", imm=0); b.op("ST", vo)


def ladder2(b, vo, c1, c2, out, r1, ca, r2, cb, r3, h):
    """vo -r1- c1 (ca) -r2- c2 (cb) -r3- out (a bus / coupling node): one-pole exact on each resistor, in sequence"""
    k1, k2 = kexp(r1 * ca, h), h / (r2 * ca)
    lin(b, c1, 0.0, [(c1, 1 - k1 - k2), (vo, k1), (c2, k2)])
    k2b, k3 = h / (r2 * cb), h / (r3 * cb)
    lin(b, c2, 0.0, [(c2, 1 - k2b - k3), (c1, k2b), (out, k3)])


def vco_core(b, p, th_lo):
    """Norton integrator + Schmitt + 4016 (closed while high): p+'v', p+'s'; rates _up / _dn (V per tick), upper
    threshold _th (incl. the switch delay), reciprocals p+'rup' / 'rdn' and ratios 'rud' (dn/up) / 'rdu' (up/dn) from
    the slow tier. Crossings are carried exactly; p+'sq' = the Schmitt output averaged over the tick"""
    v, s, sq = p + "v", p + "s", p + "sq"
    b.op("LD", s); b.op("CMPI", imm=q(0.5))
    jd = b.skip_if(True)
    b.op("LD", v); b.op("ADD", "_up"); b.op("ST", v); b.op("SUB", "_th"); b.op("ST", "_x"); b.op("CMPI", imm=0)
    jn = b.skip_if(False)
    b.op("LD", "_x"); b.op("MUL", p + "rup"); b.op("MINI", imm=ONE); b.op("ST", "_f")
    b.op("LD", "_x"); b.op("MUL", p + "rud"); b.op("ST", "_t"); b.op("LD", "_th"); b.op("SUB", "_t"); b.op("ST", v)
    b.op("LDI", imm=ONE); b.op("ST", s); b.op("LD", "_f"); b.op("MULI", imm=q(VOH)); b.op("ST", sq)
    e1 = jump(b)
    b.land(jn)
    b.op("LDI", imm=0); b.op("ST", sq)
    e2 = jump(b)
    b.land(jd)
    b.op("LD", v); b.op("SUB", "_dn"); b.op("ST", v)
    b.op("LDI", imm=q(th_lo)); b.op("SUB", v); b.op("ST", "_x"); b.op("CMPI", imm=1)
    jn2 = b.skip_if(False)
    b.op("LD", "_x"); b.op("MUL", p + "rdn"); b.op("MINI", imm=ONE); b.op("ST", "_f")
    b.op("LD", "_x"); b.op("MUL", p + "rdu"); b.op("ADDI", imm=q(th_lo)); b.op("ST", v)
    b.op("LDI", imm=0); b.op("ST", s); b.op("LD", "_f"); b.op("MULI", imm=q(-VOH)); b.op("ADDI", imm=q(VOH))
    b.op("ST", sq)
    e3 = jump(b)
    b.land(jn2)
    b.op("LDI", imm=q(VOH)); b.op("ST", sq)
    for e in (e1, e2, e3):
        b.land(e)


def recips(b, p, up, dn):
    """slow tier: p+'rup' = 1 / up, 'rdn' = 1 / dn, 'rud' = dn / up, 'rdu' = up / dn (up / dn registers)"""
    b.op("LD", up); b.op("RCP"); b.op("ST", p + "rup"); b.op("MUL", dn); b.op("ST", p + "rud")
    b.op("LD", dn); b.op("RCP"); b.op("ST", p + "rdn"); b.op("MUL", up); b.op("ST", p + "rdu")


# ----------------------------------------------------------------------------------------------- circuits
F_NOISE = 1.0 / (2.2 * 56e3 * 0.001e-6)          # 4030 RC clock (R403 56k, C401 .001 uF) into the 4006 LFSR


def noise(b, jet_jump):
    """nz = 0 / 12 V, a new LFSR bit per 4030 clock; on each edge the jet HP network takes its charge step"""
    b.op("LD", "nph"); b.op("ADDI", imm=int(round(F_NOISE / FS * ONE))); b.op("ST", "nph"); b.op("CMPI", imm=ONE)
    j = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "nph")
    b.op("LD", "nz"); b.op("ST", "_np")
    b.op("NOISE", 0, 0); b.op("LDI", imm=0); b.op("LDIF", imm=q(V12)); b.op("ST", "nz")
    b.op("SUB", "_np"); b.op("ST", "_dn")
    for reg, k in jet_jump:
        b.op("LD", reg); b.op("MACI", "_dn", imm=q(k)); b.op("ST", reg)
    b.land(j)


def nf_filter(b, h):
    """copter / tank noise filter: nz -12k- nf1 (C100 .015) -68k- nf2 (C101 .015), nf2 loaded by 2 x 330k to VB"""
    g1, g2, g3 = 1 / 12.0, 1 / 68.0, 2 / 330.0
    c = 0.015 * U
    a = [[-(g1 + g2) / c, g2 / c], [g2 / c, -(g2 + g3) / c]]
    bb = [[g1 / c, 0.0], [0.0, g3 / c]]
    ad, bd = zoh(a, bb, h)
    lin(b, "_t", bd[0][1] * VB, [("nf1", ad[0][0]), ("nf2", ad[0][1]), ("nz", bd[0][0])])
    lin(b, "nf2", bd[1][1] * VB, [("nf1", ad[1][0]), ("nf2", ad[1][1]), ("nz", bd[1][0])])
    b.op("LD", "_t"); b.op("ST", "nf1")


def copter_tank(b, p, bit, cint, h, pan):
    """R4 integrator (620k from the latch, 2.2M from 12 and the Schmitt) + inverting Schmitt (510k; 1.5M, 2.2M) ->
    P5 noise gate (560k from the Schmitt; 2.2M, 330k noise filter, 1M feedback) on 2.2 uF with a 27k pull-up ->
    56k + 1 uF -> pan node"""
    v, s, V, cc = p + "v", p + "s", p + "V", p + "cc"
    vps = vp(8e-3)
    ie_on, ie_off = (V5 - VB) / (620 + RPU), (0 - VB) / 620
    ip0 = (V12 - vps) / 2200 + (0 - vps) / 2200
    ip1 = (V12 - vps) / 2200 + (VOH - vps) / 2200
    kv = h / cint
    b.op("BIT", 0, b.src(LATCH4, bit))
    b.op("LDI", imm=q((ip0 - ie_off) * kv)); b.op("LDIF", imm=q((ip0 - ie_on) * kv))
    b.op("MACI", s, imm=q((ip1 - ip0) * kv)); b.op("ADD", v); b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH))
    b.op("ST", v)
    th0 = VB + 510 * ((V12 - vps) / 1500 + (0 - vps) / 2200)
    th1 = VB + 510 * ((V12 - vps) / 1500 + (VOH - vps) / 2200)
    b.op("LDI", imm=q(th0)); b.op("MACI", s, imm=q(th1 - th0)); b.op("ST", "_t")
    b.op("LD", v); b.op("CMP", "_t"); b.op("NOT"); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", s)
    ipg = (VOH - vp(19e-3)) / 560
    b.op("LDI", imm=q(VB - 1000 * (V12 - VB) / 2200 + 1000 * VB / 330)); b.op("MACI", s, imm=q(1000 * ipg))
    b.op("MACI", "nf2", imm=q(-1000 / 330)); b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "_T")
    c = 2.2 * U
    # free drift (pull-up, load) then the amplifier: target, rising at most ISRC, falling at most ISNK
    lin(b, "_vf", V12 * h / (27 * c), [(V, 1 - h / c * (1 / 27.0 + 1 / 56.0)), (cc, h / (56 * c)), (pan, h / (56 * c))])
    b.op("LD", "_T"); b.op("SUB", "_vf"); b.op("MAXI", imm=q(-ISNK * h / c)); b.op("MINI", imm=q(ISRC * h / c))
    b.op("ADD", "_vf"); b.op("MAXI", imm=0); b.op("MINI", imm=q(V12)); b.op("ST", V)
    k = h / (56 * 1 * U)
    lin(b, cc, 0.0, [(cc, 1 - k), (V, k), (pan, -k)])


def airplane(b, h):
    """L4 envelope integrator (39k from the latch, 150k from 12, 2.2 uF), L4 gate comparator (100k; 1.2M), VCO
    (L3 integrator 330k / 510k on -, 180k / 240k on + through the 4016, 0.1 uF; Schmitt 750k / 3M on +, gate 1M on -).
    The two VCOs have identical values: one VCO, outputs 150k || 56k -> 100k -> 1 uF -> pan 1"""
    vpe = vp(110e-3)
    ie, im = (V5 - vpe) / (39 + RPU), (V12 - VB) / 150
    kc = h / (2.2 * U)
    b.op("BIT", 0, b.src(LATCH4, 0))
    b.op("LDI", imm=q(-im * kc)); b.op("LDIF", imm=q((ie - im) * kc)); b.op("ADD", "aenv")
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "aenv")
    vpg = vp(10e-3)
    b.op("CMPI", imm=q(vpg + 100 * (V12 - VB) / 1200)); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "ag")
    vpv = vp(70e-3)
    kc = h / (0.1 * U)
    # dn = (env - VB)/330 + (12 - VB)/510 ; up = (12 - vpv)/240 + (env - vpv)/180 - dn ; v += s ? -dn : up
    d0, d1 = ((V12 - VB) / 510 - VB / 330) * kc, kc / 330
    u0, u1 = ((V12 - vpv) / 240 - vpv / 180) * kc - d0, kc / 180 - d1
    lin(b, "_up", u0 + d0, [("aenv", u1 + d1)])                  # up + dn
    b.op("LD", "_up"); b.op("MUL", "as"); b.op("ST", "_t")
    lin(b, "_dn", u0, [("aenv", u1)])                            # up
    b.op("LD", "av"); b.op("ADD", "_dn"); b.op("SUB", "_t"); b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH))
    b.op("ST", "av")
    vps = vp(8e-3)
    ig1, ig0 = (VOH - VB) / 1000, (0 - VB) / 1000
    th_hi0 = vps + 750 * (ig0 + vps / 3000)
    th_hi1 = vps + 750 * (ig1 + vps / 3000)
    ds = -750 * VOH / 3000
    b.op("LDI", imm=q(th_hi0)); b.op("MACI", "ag", imm=q(th_hi1 - th_hi0)); b.op("MACI", "as", imm=q(ds))
    b.op("ST", "_t"); b.op("LD", "av"); b.op("CMP", "_t"); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "as")
    k = h / ((100 + 1 / (1 / 150 + 1 / 56)) * 1 * U)
    lin(b, "c303", 0.0, [("c303", 1 - k), ("as", k * VOH), ("vp1", -k)])


JET_CS = 1 / (1 / 1e-9 + 1 / 1e-9 + 1 / 4.7e-9)


def jet_hp(h):
    """jet HP noise: nz -C206 1n- h1 (R219 68k) -C208 1n- h2 (R220 68k, C207 4.7n, R218 100k to VB); states q206, h2"""
    c206, c208, c207 = 1 * N, 1 * N, 4.7 * N
    mm = np.linalg.inv(np.array([[c206 + c208, c208], [-c208, -c208 - c207]]))
    rr = np.array([[-1 / 68.0, 0.0, 1 / 68.0, 0.0], [0.0, 1 / 68.0 + 1 / 100.0, 0.0, -1 / 100.0]])
    d = mm @ rr
    return zoh(d[:, :2], d[:, 2:], h)


def jet_fast(b):
    """every sample: HP noise network, VCO (P3 integrator .0022 uF, 330k / 510k / 100k-noise on -, 180k / 240k on +
    via the 4016; Schmitt 510k / 2M on +, 1.5M on -), ramp - square amp, 100k + 0.1 uF -> pan 1"""
    ad, bd = jet_hp(DT)
    lin(b, "_t", bd[0][1] * VB, [("jq", ad[0][0]), ("jh", ad[0][1]), ("nz", bd[0][0])])
    lin(b, "jh", bd[1][1] * VB, [("jq", ad[1][0]), ("jh", ad[1][1]), ("nz", bd[1][0])])
    b.op("LD", "_t"); b.op("ST", "jq")
    vpv = vp(100e-3)
    kc = DT / (2.2 * N)
    d0, de, dh = ((V12 - VB) / 510 - VB / 330 - VB / 100) * kc, kc / 330, kc / 100
    lin(b, "_dn", d0, [("jenv", de), ("jh", dh)])
    b.op("LD", "jenv"); b.op("ADDI", imm=q(-vpv)); b.op("MAXI", imm=0); b.op("MULI", imm=q(kc / 180))
    b.op("ADDI", imm=q((V12 - vpv) / 240 * kc)); b.op("SUB", "_dn"); b.op("ST", "_up")
    vps = vp(6e-3)
    ith = (V12 - VB) / 1500
    th_hi = vps + 510 * (ith + vps / 2000)
    th_lo = vps + 510 * (ith - (VOH - vps) / 2000)
    b.op("LD", "_up"); b.op("MULI", imm=q(TD / DT)); b.op("ADDI", imm=q(th_hi)); b.op("ST", "_th")
    vco_core(b, "j", th_lo)
    vpr = vp(15e-3)
    b.op("LD", "jenv"); b.op("SUB", "jsq"); b.op("ADDI", imm=q(VB - vpr + VB - 680 * (V12 - VB) / 2700))
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "jrs")
    k = DT / (100 * 0.1 * U)
    lin(b, "c204", 0.0, [("c204", 1 - k), ("jrs", k), ("vp1", -k)])
    return th_lo


def jet_slow(b, h):
    """P5 envelope integrator (22k from the latch, 100k from 12, 1 uF); R206 330k + D200 -> C200 10 uF -> 200k;
    VCO reciprocals from the envelope"""
    vpe = vp(190e-3)
    ie, im = (V5 - vpe) / (22 + RPU), (V12 - VB) / 100
    kc = h / (1 * U)
    b.op("BIT", 0, b.src(LATCH4, 5))
    b.op("LDI", imm=q(-im * kc)); b.op("LDIF", imm=q((ie - im) * kc)); b.op("ADD", "jenv")
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "jenv")
    vpn = vp(10e-3)
    k2 = h / (200 * 10 * U)
    lin(b, "_t", k2 * vpn, [("c200", 1 - k2)])
    k1 = h / (330 * 10 * U)
    b.op("LD", "jenv"); b.op("ADDI", imm=q(-VD)); b.op("SUB", "c200"); b.op("MAXI", imm=0); b.op("MULI", imm=q(k1))
    b.op("ADD", "_t"); b.op("ST", "c200")
    vpv = vp(100e-3)
    kc = DT / (2.2 * N)
    d0, de = ((V12 - VB) / 510 - VB / 330) * kc, kc / 330
    lin(b, "_sd", d0, [("jenv", de)])
    b.op("LD", "jenv"); b.op("ADDI", imm=q(-vpv)); b.op("MAXI", imm=0); b.op("MULI", imm=q(kc / 180))
    b.op("ADDI", imm=q((V12 - vpv) / 240 * kc)); b.op("SUB", "_sd"); b.op("ST", "_su")
    recips(b, "j", "_su", "_sd")


def jet_noise(b, h):
    """P5 noise gate: + 200k from C200, - 680k noise / 2.7M 12 V / 680k feedback, 27k pull-up; 560 / .47 uF,
    6.8k / .47 uF, 10k + 1 uF -> pan 1"""
    vpn = vp(10e-3)
    b.op("LDI", imm=q(VB - 680 * vpn / 200 + VB - 680 * (V12 - VB) / 2700)); b.op("MACI", "c200", imm=q(680 / 200))
    b.op("MACI", "nz", imm=q(-1.0)); b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "_T")
    gate_rc(b, "_T", "c202", "_vo", 0.56)
    k1, k2 = kexp(0.56 * 0.47 * U, h), h / (6.8 * 0.47 * U)
    lin(b, "c202", 0.0, [("c202", 1 - k1 - k2), ("_vo", k1), ("c201", k2)])
    k2b, k3 = h / (6.8 * 0.47 * U), h / (10 * 0.47 * U)
    lin(b, "c201", 0.0, [("c201", 1 - k2b - k3), ("c202", k2b), ("c203", k3), ("vp1", k3)])
    k4 = h / (10 * 1 * U)
    lin(b, "c203", 0.0, [("c203", 1 - k4), ("c201", k4), ("vp1", -k4)])


def mfb(h):
    r817, r823, r824, c804, c806, c807 = 330.0, 82.0, 150.0, 2.2 * N, 1 * N, 100 * P
    ip = (V12 - vp(22e-3)) / 510
    a = [[-1 / (r817 * c804), -1 / (r817 * c804), 0.0],
         [-1 / (r817 * c806), (-1 / r817 - 1 / r823 - 1 / r824) / c806, 1 / (r823 * c806)],
         [0.0, -1 / (r824 * c807), 0.0]]
    bb = [[1 / (r817 * c804), 0.0], [1 / (r817 * c806), VB / (r824 * c806)], [0.0, (ip + VB / r824) / c807]]
    return zoh(a, bb, h), ip


def mis_rates(b, p, h, up, dn):
    """VCO (H2 integrator 330 pF): + 1M from 12 and 1M from x (x grounded while the 4016 is closed); - 3M from 12,
    2.2M from x, 2.2M from the LF noise bus (R502 / R503: not modelled, about +/- 1 % slow pitch wobble; the
    program has no room left). x = C803 -1M- node"""
    vpv = vp(15e-3)
    gx = 1 / 1000 + 1 / 2200 + 1 / 1000
    kc = h / (330 * P)
    # closed: imc = (12-VB)/3000 + (vxc-VB)/2200 + (lfb-VB)/2200, vxc = (c803/1000 + VB/2200)/gx
    ax = 1 / (1000 * gx)
    xc0 = VB / 2200 / gx
    xo0 = (VB / 2200 + vpv / 1000) / gx
    d0 = ((V12 - VB) / 3000 + (xc0 - VB) / 2200) * kc
    lin(b, dn, d0, [(p + "c803", ax / 2200 * kc)])
    # open: up = (12-vpv)/1000 + (vxo-vpv)/1000 - [(12-VB)/3000 + (vxo-VB)/2200 + (lfb-VB)/2200]
    u0 = ((V12 - vpv) / 1000 + (xo0 - vpv) / 1000 - (V12 - VB) / 3000 - (xo0 - VB) / 2200) * kc
    lin(b, up, u0, [(p + "c803", ax * (1 / 1000 - 1 / 2200) * kc)])


def missile_fast(b, p, h, bus):
    mis_rates(b, p, h, "_up", "_dn")
    vps = vp(10e-3)
    ith = (V12 - VB) / 1000
    th_hi = vps + 510 * (ith + vps / 2200)
    th_lo = vps + 510 * (ith - (VOH - vps) / 2200)
    b.op("LD", "_up"); b.op("MULI", imm=q(TD / h)); b.op("ADDI", imm=q(th_hi)); b.op("ST", "_th")
    vco_core(b, p, th_lo)
    # VCA (H2 13/8/9): + 510k from C803; - 510k square, 2.7M 12 V, 510k feedback
    vpa = vp(19e-3)
    b.op("LD", p + "c803"); b.op("SUB", p + "sq"); b.op("ADDI", imm=q(2 * VB - vpa - 510 * (V12 - VB) / 2700))
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "_a")
    (ad, bd), _ = mfb(h)
    f = [p + "f0", p + "f1", p + "f2"]
    for i, dst in ((2, "_t2"), (1, "_t1"), (0, f[0])):
        lin(b, dst, bd[i][1], [(f[0], ad[i][0]), (f[1], ad[i][1]), (f[2], ad[i][2]), ("_a", bd[i][0])])
    b.op("LD", "_t1"); b.op("ST", f[1]); b.op("LD", "_t2"); b.op("ST", f[2])


def missile_slow(b, p, bit, h):
    """L4 envelope integrator (20k from the latch, 330k from 12, 4.7 uF); J1-2 comparator (1M; 1.2M) -> D800 -> C803
    2.2 uF, discharged by 1M (VCO control) and 510k (VCA)"""
    vpe = vp(210e-3)
    ie, im = (V5 - vpe) / (20 + RPU), (V12 - VB) / 330
    kc = h / (4.7 * U)
    b.op("BIT", 0, b.src(LATCH1, bit))
    b.op("LDI", imm=q(-im * kc)); b.op("LDIF", imm=q((ie - im) * kc)); b.op("ADD", p + "env")
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", p + "env")
    b.op("CMPI", imm=q(vp(10e-3) + 1000 * (V12 - VB) / 1200))
    j = b.skip_if(False)
    b.op("LDI", imm=q(VOH - VD)); b.op("RC", p + "c803", imm=q(kexp(1.0 * 2.2 * U, h)))
    b.land(j)
    vpv, vpa = vp(15e-3), vp(19e-3)
    gx = 1 / 1000 + 1 / 2200 + 1 / 1000
    # i = (c803 - x)/1000 + (c803 - vpa)/510, x = (c803/1000 + VB/2200 + vq/1000)/gx, vq = s ? 0 : vpv
    kx = 1 / (1000 * gx)
    a = (1 - kx) / 1000 + 1 / 510
    c0 = -(VB / 2200 + vpv / 1000) / gx / 1000 - vpa / 510
    cs = (vpv / 1000) / gx / 1000                                  # switch closed: x lower by vpv / 1000 / gx
    kc = h / (2.2 * U)
    lin(b, p + "c803", -c0 * kc, [(p + "c803", 1 - a * kc), (p + "s", cs * kc)])


def rocket(b, p, h, bus):
    """J1-2 2/3/4 noise gate: + 680k from the envelope, - 680k noise / 2.7M / 680k; 27k pull-up; 560 / 1 uF,
    6.8k / 1 uF, 10k -> bus"""
    vpr = vp(15e-3)
    b.op("LD", p + "env"); b.op("SUB", "nz"); b.op("ADDI", imm=q(2 * VB - vpr - 680 * (V12 - VB) / 2700))
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "_T")
    gate_rc(b, "_T", p + "c801", "_vo", 0.56)
    ladder2(b, "_vo", p + "c801", p + "c802", bus, 0.56, 1 * U, 6.8, 1 * U, 10.0, h)


def explosion(b, p, bit, h, bus):
    """D601 -> C603 1 uF -> 510k; D602 + 330k -> C604 3.3 uF -> 120k -> J1-2 comparator (2.7M) -> D600 -> C602 1 uF
    -> 1.2M; J1-2 noise gate (+ from both, - 680k noise / 2.7M / 680k; 27k pull-up); 560 / 1 uF, 6.8k / 1 uF,
    5.6k -> bus"""
    c603, c604, c602 = p + "c603", p + "c604", p + "c602"
    b.op("BIT", 0, b.src(LATCH1, bit))
    j = b.skip_if(False)
    b.op("LDI", imm=q(V5 - VD)); b.op("RC", c603, imm=q(kexp(RPU * 1 * U, h)))
    b.land(j)
    vpg = vp(10e-3)
    kd = h / (510 * 1 * U)
    lin(b, c603, kd * vpg, [(c603, 1 - kd)])
    vpk = vp(5e-3)
    kd = h / (120 * 3.3 * U)
    lin(b, c604, kd * vpk, [(c604, 1 - kd)])
    b.op("BIT", 0, b.src(LATCH1, bit))
    j = b.skip_if(False)
    kch = h / (330 * 3.3 * U)
    lin(b, c604, kch * (V5 - 0.38), [(c604, 1 - kch)])
    b.land(j)
    b.op("CMPI", imm=q(vpk + 120 * (V12 - VB) / 2700))            # acc = c604
    j = b.skip_if(False)
    b.op("LDI", imm=q(VOH - VD)); b.op("RC", c602, imm=q(kexp(1.0 * 1 * U, h)))
    b.land(j)
    kd = h / (1200 * 1 * U)
    lin(b, c602, kd * vpg, [(c602, 1 - kd)])
    lin(b, "_T", VB + 680 * (-vpg / 510 - vpg / 1200 + VB / 680 - (V12 - VB) / 2700),
        [(c603, 680 / 510), (c602, 680 / 1200), ("nz", -1.0)])
    b.op("LD", "_T"); b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "_T")
    gate_rc(b, "_T", p + "c601", "_vo", 0.56)
    ladder2(b, "_vo", p + "c601", p + "c600", bus, 0.56, 1 * U, 6.8, 1 * U, 5.6, h)


def lf_noise(b, h):
    """nz -680k- lf1 (C500 .1) -680k- lf2 (C501 .1) -C502 1u- lfb, lfb -2.2M- VB twice (both missile VCOs)"""
    g, gl = 1 / 680.0, 2 / 2200.0
    a = [[-2 * g / (0.1 * U), g / (0.1 * U), 0.0], [g / (0.1 * U), (-g - gl) / (0.1 * U), gl / (0.1 * U)],
         [0.0, gl / (1 * U), -gl / (1 * U)]]
    bb = [[g / (0.1 * U), 0.0], [0.0, -gl * VB / (0.1 * U) / VB], [0.0, gl * VB / (1 * U) / VB]]
    # inputs (nz, VB): lfb = lf2 - q, current (lfb - VB) gl
    bb = [[g / (0.1 * U), 0.0], [0.0, gl / (0.1 * U)], [0.0, -gl / (1 * U)]]
    ad, bd = zoh(a, bb, h)
    s = ["lf1", "lf2", "lq"]
    for i, dst in ((2, "_t2"), (1, "_t1"), (0, "lf1")):
        lin(b, dst, bd[i][1] * VB, [(s[0], ad[i][0]), (s[1], ad[i][1]), (s[2], ad[i][2]), ("nz", bd[i][0])])
    b.op("LD", "_t1"); b.op("ST", "lf2"); b.op("LD", "_t2"); b.op("ST", "lq")
    b.op("LD", "lf2"); b.op("SUB", "lq"); b.op("ST", "lfb")


def ladder(k, ron=0.18):
    """4051 ladder, tap k grounded: Zin from the source node, iR / i, iL / i (buses at 0 V)"""
    g = np.zeros((9, 9))

    def r(a, bb, R):
        for x in (a, bb):
            if x is not None:
                g[x, x] += 1 / R
        if a is not None and bb is not None:
            g[a, bb] -= 1 / R
            g[bb, a] -= 1 / R
    r(0, 1, 10.0); r(0, 8, 10.0); r(1, None, 10.0); r(8, None, 10.0)
    for i in range(7):
        r(1 + i, 2 + i, 1.0)
    r(1 + k, None, ron)
    i = np.zeros(9); i[0] = 1.0
    v = np.linalg.solve(g, i)
    return v[0], v[1] / 10.0, v[8] / 10.0


PAN1_G = {"c": 1 / 56.0, "a": 1 / (100 + 1 / (1 / 150 + 1 / 56)), "r": 1 / 100.0, "n": 1 / 10.0}
PAN2_G = 1 / 56.0
Y_LAD = 1 / 13.0                                    # one ladder seen from a bus (approx.: 10k + half the ladder)


def cubic(vals):
    """exact cubic through (0..3, vals): Horner coefficients a3, a2, a1, a0"""
    x = np.array([0, 1, 2, 3], float)
    return list(np.polyfit(x, np.array(vals, float), 3))


def pan_lookup(b, n, sh, gsum):
    """latch 2 bits sh..sh+2 = 4051 tap k: K = 1 / (Yin + sum g) (kOhm), gR / gL (mS per volt of the pan node).
    k and 7 - k mirror (R <-> L): j = min(k, 7 - k), exact cubics in j"""
    b.op("LDLM", 0, b.src(LATCH2, 0), imm=(7 << sh) | ((24 - sh) << 8)); b.op("ST", "_k")
    b.op("CMPI", imm=q(4.0)); b.op("LDI", imm=q(7.0)); b.op("SUB", "_k"); b.op("ST", "_t"); b.op("LD", "_k")
    b.op("LDF", "_t"); b.op("ST", "_j")                            # F = k >= 4 kept below (no compares)
    tab = [ladder(k) for k in range(4)]
    kk = [1 / (1 / z + gsum) for z, _, _ in tab]
    gr = [ir / z for z, ir, _ in tab]                            # k < 4: the tap is near the R end
    gl = [il / z for z, _, il in tab]
    for dst, vals in ((n + "k", kk), ("_gn", gr), ("_gf", gl)):
        a3, a2, a1, a0 = cubic(vals)
        b.op("LD", "_j"); b.op("MULI", imm=q(a3)); b.op("ADDI", imm=q(a2)); b.op("MUL", "_j"); b.op("ADDI", imm=q(a1))
        b.op("MUL", "_j"); b.op("ADDI", imm=q(a0)); b.op("ST", dst)
    b.op("LD", "_gn"); b.op("LDF", "_gf"); b.op("ST", n + "gr")
    b.op("LD", "_gf"); b.op("LDF", "_gn"); b.op("ST", n + "gl")


def pan_nodes(b):
    g = PAN1_G
    b.op("LD", "cV"); b.op("SUB", "ccc"); b.op("MULI", imm=q(g["c"])); b.op("ST", "_t")
    b.op("LD", "as"); b.op("MULI", imm=q(g["a"] * VOH)); b.op("MACI", "c303", imm=q(-g["a"])); b.op("ADD", "_t")
    b.op("ST", "_t")
    b.op("LD", "jrs"); b.op("SUB", "c204"); b.op("MULI", imm=q(g["r"])); b.op("ADD", "_t"); b.op("ST", "_t")
    b.op("LD", "c201"); b.op("SUB", "c203"); b.op("MULI", imm=q(g["n"])); b.op("ADD", "_t")
    b.op("MUL", "p1k"); b.op("ST", "vp1")
    b.op("LD", "tV"); b.op("SUB", "tcc"); b.op("MULI", imm=q(PAN2_G)); b.op("MUL", "p2k"); b.op("ST", "vp2")


def summer(h):
    """N1-2 summer: - held at VB, input ic (mA) through C1102; R1101 100k || (R22 || C16 + C1101); + 220k from 12 V.
    states q16, q11; out = VB + q16 + q11"""
    r1, r22, c16, c11 = 100.0, R22, 1 * N, 1 * U
    ip = (V12 - vp(52e-3)) / 220
    a = [[(-1 / r1 - 1 / r22) / c16, -1 / (r1 * c16)], [-1 / (r1 * c11), -1 / (r1 * c11)]]
    bb = [[-1 / c16, ip / c16], [-1 / c11, ip / c11]]
    return zoh(a, bb, h), ip


R22 = 25.0                                          # volume pot (50k rheostat) at 50 %
_, SU_IP = summer(DT)
SU_VDC = VB + SU_IP * 100


def side_out(b, x, h, mis, exp_):
    """bus X (C1102 1 uF) and summer X: bus current = ladders + whistle (150k) + rocket (10k) + explosion (5.6k)"""
    bus = "bus" + x
    gt = 1 / 150 + 1 / 10 + 1 / 5.6 + 2 * Y_LAD
    b.op("LD", "vp1"); b.op("MUL", "p1g" + x.lower()); b.op("ST", "_t")
    b.op("LD", "vp2"); b.op("MUL", "p2g" + x.lower()); b.op("ADD", "_t")
    b.op("MACI", mis + "f2", imm=q(1 / 150)); b.op("MACI", mis + "c802", imm=q(1 / 10))
    b.op("MACI", exp_ + "c600", imm=q(1 / 5.6)); b.op("MACI", bus, imm=q(-gt)); b.op("ST", "_ic")
    b.op("LD", bus); b.op("MACI", "_ic", imm=q(h / (1 * U))); b.op("ST", bus)
    (ad, bd), _ = summer(h)
    s = ["s16" + x, "s11" + x]
    lin(b, "_t", bd[0][1], [(s[0], ad[0][0]), (s[1], ad[0][1]), ("_ic", bd[0][0])])
    lin(b, s[1], bd[1][1], [(s[0], ad[1][0]), (s[1], ad[1][1]), ("_ic", bd[1][0])])
    b.op("LD", "_t"); b.op("ST", s[0]); b.op("ADD", s[1]); b.op("ADDI", imm=q(VB)); b.op("MAXI", imm=0)
    b.op("MINI", imm=q(VOH)); b.op("ST", "so" + x)


OUT_SCALE = 1 / 6.0                                 # summer volts (R + L about DC) -> full scale


def gmissile():
    b = Board("gmissile")
    # power-on state (state memory clears to 0): integrators at the top while their latch bit is low, summers /
    # filters at DC; the rest settles while the game keeps the sound muted (D3 low)
    inits = []
    for p in "ct":
        inits.append((p + "v", VOH))
    (_, _), ip = mfb(DT)
    m = VB + 150 * ip
    for p in "rl":
        inits += [(p + "f0", -m), (p + "f1", m), (p + "f2", m + 82 * ip)]
        inits += [("s11" + {"r": "R", "l": "L"}[p], SU_VDC - VB), ("so" + {"r": "R", "l": "L"}[p], SU_VDC)]
    inits += [("as", 1.0), ("p1k", 6.0), ("p2k", 6.0), ("_po", 2 * SU_VDC)]
    # first sample after reset: power-on state, then END (a silent sample; keeps it off the budget path)
    b.op("LD", "_ini"); b.op("CMPI", imm=q(0.5))
    j = b.skip_if(True)
    for reg, v in inits:
        b.op("LDI", imm=q(v)); b.op("ST", reg)
    b.op("LDI", imm=ONE); b.op("ST", "_ini"); b.op("END")
    b.land(j)
    c206, c207 = 1e-9, 4.7e-9
    noise(b, [("jq", JET_CS / c206), ("jh", JET_CS / c207)])
    nf_filter(b, DT)
    jet_fast(b)
    pan_nodes(b)
    H2, H4, H8 = 2 * DT, 4 * DT, 8 * DT
    R = 1 << 20                                     # tier counter step
    order = [0, 4, 2, 6, 1, 5, 3, 7]                # bit-reversed: side = r >= 4 alternates every sample
    nxt = {order[i]: order[(i + 1) % 8] for i in range(8)}

    def eighth(r):
        if r == 0:
            mis_rates(b, "r", H2, "_su", "_sd"); recips(b, "r", "_su", "_sd")
        elif r == 1:
            jet_slow(b, H8)
        elif r == 3:
            pan_lookup(b, "p1", 0, sum(PAN1_G.values()))
        elif r == 4:
            mis_rates(b, "l", H2, "_su", "_sd"); recips(b, "l", "_su", "_sd")
        elif r == 6:
            pan_lookup(b, "p2", 3, PAN2_G)
        b.op("LDI", imm=nxt[r] * R); b.op("ST", "_r")

    ends = []
    b.op("LD", "_r"); b.op("CMPI", imm=4 * R)
    jB = b.skip_if(True)
    for side, base in (("A", 0), ("B", 4)):
        if side == "B":
            b.land(jB)
        p, x, mbit, ebit, bus = ("r", "R", 4, 7, "busR") if side == "A" else ("l", "L", 6, 5, "busL")
        missile_fast(b, p, H2, bus)
        side_out(b, x, H2, p, "x" + p)
        b.op("LD", "_r"); b.op("CMPI", imm=(base + 2) * R)
        jq = b.skip_if(True)
        # quarter 0: explosion, rocket noise, missile envelope
        explosion(b, "x" + p, ebit, H4, bus)
        rocket(b, p, H4, bus)
        missile_slow(b, p, mbit, H4)
        b.op("LD", "_r"); b.op("CMPI", imm=(base + 1) * R)
        je = b.skip_if(True)
        eighth(base)
        ends.append(jump(b))
        b.land(je)
        eighth(base + 1)
        ends.append(jump(b))
        b.land(jq)
        # quarter 1
        if side == "A":
            copter_tank(b, "c", 2, 0.047 * U, H4, "vp1")
            jet_noise(b, H4)
        else:
            copter_tank(b, "t", 1, 0.022 * U, H4, "vp2")
            airplane(b, H4)
        b.op("LD", "_r"); b.op("CMPI", imm=(base + 3) * R)
        je = b.skip_if(True)
        eighth(base + 2)
        ends.append(jump(b))
        b.land(je)
        eighth(base + 3)
        if side == "A":
            ends.append(jump(b))
    for e in ends:
        b.land(e)
    # output: (R + L) around the summers' DC, muted while port 3 D3 is low
    # each summer is held for 2 samples (24 kHz): average with the previous sum (a 48 kHz 2-tap boxcar) to take the
    # 24 kHz - f images (missile whistle -> 20 kHz) down another ~12 dB
    b.op("LD", "soR"); b.op("ADD", "soL"); b.op("ST", "_t"); b.op("ADD", "_po"); b.op("ADDI", imm=q(-4 * SU_VDC))
    b.op("MULI", imm=q(0.5 * OUT_SCALE)); b.op("ST", "_o"); b.op("LD", "_t"); b.op("ST", "_po")
    b.op("BIT", 0, b.src(LATCH1, 3)); b.op("NOT"); b.op("LD", "_o"); b.op("LDIF", imm=0); b.op("OUT")
    global LAST
    LAST = b
    return b.finish()


if __name__ == "__main__":
    prog = gmissile()
    out = sys.argv[1] if len(sys.argv) > 1 else "gmissile.bin"
    open(out, "wb").write(prog.binary())
    print(f"gmissile: {len(prog.code)} words, worst sample {prog.worst_path()} units, {len(prog.regs)} registers")
