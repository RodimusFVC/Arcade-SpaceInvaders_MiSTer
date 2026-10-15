"""Blue Shark (Midway 1978) sound board for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

From the schematic M051-00742-A005 (Useful Information/schematics/Blue_Shark_Schematic_(M051-00742-A005).pdf), MAME
blueshrk_discrete as the cross-check; spec = verilator/snd/blueshrk/bs_proto.py, checked against ngspice decks of the
traced circuits (verilator/snd/blueshrk/bs_spice.py). S1 = port 3 latch E5: D0 game on (amplifier mute), D1 shot,
D2 hit, D3 shark, D4 not connected, D5 octopus.
- Octopus: LM3900 sawtooth LFO sweeps a Norton VCO (4016 switch), one-shot -> integrator envelope -> VCA amp.
- Shot (MAME: none): one-shot -> two diode peak envelopes + comparator-held third -> noise (N5 13) VCA -> amp gated
  by D1 -> C599 / R520 / R521 into the mixer.
- Hit: 556 (14 Hz) clocks a 74107 (cleared while D2 is low) that switches the 556 tone cap between C602 + C603 and
  C604 (MAME holds the flip-flop in reset: one tone); the floating cap keeps its charge (7417 substrate diode).
- Shark: three slow LM3900 oscillators ANDed (M3 10) gate a 13 Hz Norton VCO enabled by D3 (R718); the VCO keys the
  K3 amp on noise (N5 10) through the R724 / C704, R725 / C705 ladder (C706 sits on the Norton input); K3 drives
  C707 2.2 uF directly: fast rise, class-A sink (1.3 mA) fall.
- Mixer: each source through R / C into the bus, C900 into the L-M2 virtual ground; L-M2 / LM377 bridge high passes
  (16 Hz); level per source as MAME's mixer (DISCRETE_OUTPUT 90000, route 0.25).
LM3900 conventions as MAME (VBE 0.5 V, out 0 .. 11.5 V) with each input current less VBE (the node sits at VBE).
"""
import math
import sys
from dsnd import Board, LATCH1, q, ONE, FS, rc_exp, par

K, M_, U = 1e3, 1e6, 1e-6
VBE, VH = 0.5, 11.5
DT = 1.0 / FS


def vd(i):
    """1N4148 (IS 2.52 nA, N 1.752) forward drop at current i"""
    return 1.752 * 0.025852 * math.log(i / 2.52e-9 + 1)


VCO_TD = (8.5e-6, 13.8e-6)        # octopus VCO turn-around delay after th / tl (Schmitt + 4016 at 0.5 V/us; ngspice)
SH_OSC = [(68 * K, 470 * K, 22 * U, vd(63e-6)), (100 * K, 470 * K, 22 * U, vd(43.5e-6)),
          (470 * K, 39 * K, 10 * U, vd(21.4e-6))]          # (R700/705/710 diode charge, R701/706/711, C700/701/702)
D703, D502, D500, D503 = vd(7.1e-6), vd(16e-6), vd(30e-6), vd(8.75e-6)
I_EN = VH / (2.2 * M_ + 1 * K)                             # R718 from the R1000 pull-up
ISINK, ISRC = 1.3e-3, 18e-3                                # LM3900 output: class-A sink, source (typ)
C707 = 2.2 * U
MIX_R = [750 * K, par(510 * K, 22 * K) + 33 * K, 1 * M_ + 1 * K, 56 * K]    # octopus, shot, hit, shark
MIX_C = 1 * U                                              # C305, C508, C606, C708
C900 = 10 * U
SHOT_DIV = 22 * K / (510 * K + 22 * K)
R_MAME = par(*MIX_R)                                       # MAME's mixer (same R for the shot: no divider / C599)
OUT_K = R_MAME * 90000 * 0.25 / 32768 * 0.30      # x 0.30: the shark (R730 56k) peaks at 3.1 x full scale on
                                                   # MAME's level (MAME clips); octopus / hit / shot follow
AMP_TAU = 100 * K * 0.1 * U


def jump(b):
    b.op("CMPI", imm=-(1 << 31))
    return b.skip_if(True)


def oneshot(b, out, trig, r1, r2, r3, r4, r5, c1, c2, g="T"):
    """DST_OP_AMP_1SHT type 1 Norton (MAME) with the + input at VBE: currents x r2 = (trig - vC2 - VBE) +
    (out - VBE) r2 / r5 vs max(0, vC1 - VBE) r2 / r3 + VH r2 / r1; C2 follows max(0, trig - VBE) through r2"""
    b.kreg(out + "_k2", rc_exp(r2 * c2), g)
    b.kreg(out + "_kc", rc_exp(par(r3, r4) * c1), g)
    b.kreg(out + "_kd", rc_exp(r4 * c1), g)
    b.op("BIT", 0, b.src(*trig))
    b.op("LDI", imm=0)
    b.op("LDIF", imm=q(12.0))
    b.op("ST", out + "_tv")
    b.op("SUB", out + "_c2")
    b.op("MACI", out, imm=q(r2 / r5))
    b.op("ADDI", imm=q(-VBE - VBE * r2 / r5))
    b.op("ST", out + "_ip")
    b.op("LD", out + "_tv")
    b.op("ADDI", imm=q(-VBE))
    b.op("MAXI", imm=0)
    b.op("RCM", out + "_c2", b.r(out + "_k2"))
    b.op("LD", out + "_c1")
    b.op("ADDI", imm=q(-VBE))
    b.op("MAXI", imm=0)
    b.op("MULI", imm=q(r2 / r3))
    b.op("ADDI", imm=q(r2 * VH / r1))
    b.op("SUB", out + "_ip")
    b.op("CMPI", imm=0)                                    # F = i_neg >= i_pos -> low
    b.op("LDI", imm=q(VH))
    b.op("LDIF", imm=0)
    b.op("ST", out)
    b.op("CMP", out + "_c1")                               # F = out >= c1: charge
    at = b.skip_if(True)
    b.op("ADDI", imm=q(0.6))                               # discharge: diode clamp at out + 0.6, else through r4
    b.op("CMP", out + "_c1")
    b.op("STNF", out + "_c1")
    at2 = b.skip_if(False)
    b.op("LD", out)
    b.op("RCM", out + "_c1", b.r(out + "_kd"))
    b.land(at2)
    at3 = jump(b)
    b.land(at)
    b.op("ADDI", imm=q(-VBE))
    b.op("MULI", imm=q(r3 / (r3 + r4)))
    b.op("ADDI", imm=q(VBE))
    b.op("RCM", out + "_c1", b.r(out + "_kc"))
    b.land(at3)


def noise(b):
    """N5 4006 / 4030: 7700 Hz clock (MAME's breadboard measurement), NOISE-2 (N5 13) = newest bit (nz), NOISE-1
    (N5 10) = the same sequence 4 clocks older (nz_h4); 0 / 12 V"""
    b.kreg("nz_inc", 7700 / FS, "O", clamp=False)
    b.op("LD", "nz_ph")
    b.op("ADD", "nz_inc")
    b.op("ST", "nz_ph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE)
    b.op("ST", "nz_ph")
    for d, s in (("nz_h4", "nz_h3"), ("nz_h3", "nz_h2"), ("nz_h2", "nz_h1"), ("nz_h1", "nz")):
        b.op("LD", s)
        b.op("ST", d)
    b.op("NOISE", 0, 0)
    b.op("LDI", imm=0)
    b.op("LDIF", imm=q(12.0))
    b.op("ST", "nz")
    b.land(at)


def octo_slow(b):
    """every other sample (C / 2 = rates x 2): LFO M5 6/1/5 integrator + 3/2/4 Schmitt (OSC_1 Norton), ramp =
    oc_l_v; envelope integrator J5 6/1/5 on the one-shot"""
    b.op_amp_osc_norton1("oc_l", 1 * M_, 33 * K, 100 * K, 120 * K, 1 * M_, 0.1 * U / 2, 12)
    b.integrate_norton1("oc_e", "oc_1", None, 1 * M_, 10 * K, 1 * U / 2, 12, 12)


def octo_fast(b):
    # VCO: M5 8/13/9 + 11/12/10, 4016 L5 shorts 13 while 10 is high (VCO_1 Norton); charges while low
    r1, r2, r3, r4, r5, c = 1 * M_, 470 * K, 680 * K, 1 * M_, 1 * M_, 470e-12
    i1 = VH / r5
    tl = (i1 - (12 - 2 * VBE) / r4) * r3 + VBE
    th = (i1 + VBE / r4) * r3 + VBE
    k0, k1 = DT / (r1 * c), (1 / r2 - 1 / r1) * DT / c
    b.kreg("oc_k0", k0, "O", clamp=False)
    b.kreg("oc_k1", k1, "O", clamp=False)
    b.kreg("oc_kd0", k0 * VCO_TD[1] * FS, "O", clamp=False)
    b.kreg("oc_kd1", k1 * VCO_TD[0] * FS, "O", clamp=False)
    b.op("LD", "oc_l_v")
    b.op("ADDI", imm=q(-VBE))
    b.op("ST", "oc_vm")
    b.op("LD", "oc_hi")
    b.op("CMPI", imm=q(0.5))
    at = b.skip_if(True)
    b.op("LD", "oc_vm"); b.op("MUL", "oc_kd1"); b.op("ADDI", imm=q(th)); b.op("ST", "_t")     # turn-around level
    b.op("LD", "oc_vm"); b.op("MUL", "oc_k1"); b.op("ADD", "oc_v"); b.op("ST", "oc_v")
    b.op("CMP", "_t")
    at2 = b.skip_if(False)
    b.op("SUB", "_t"); b.op("MULI", imm=q(-k0 / k1)); b.op("ADD", "_t"); b.op("ST", "oc_v")
    b.op("LDI", imm=ONE); b.op("ST", "oc_hi")
    b.land(at2)
    at3 = jump(b)
    b.land(at)
    b.op("LD", "oc_vm"); b.op("MUL", "oc_kd0"); b.op("ST", "_t")
    b.op("LDI", imm=q(tl)); b.op("SUB", "_t"); b.op("ST", "_t")
    b.op("LD", "oc_vm"); b.op("MUL", "oc_k0"); b.op("ST", "_u")
    b.op("LD", "oc_v"); b.op("SUB", "_u"); b.op("ST", "oc_v")
    b.op("CMP", "_t")
    at4 = b.skip_if(True)
    b.op("SUB", "_t"); b.op("MULI", imm=q(-k1 / k0)); b.op("ADD", "_t"); b.op("ST", "oc_v")
    b.op("LDI", imm=0); b.op("ST", "oc_hi")
    b.land(at4)
    b.land(at3)
    # one-shot J5 11/12/10 (D5 via F5 + R311), amp J5 3/2/4
    oneshot(b, "oc_1", (LATCH1, 5), 4.7 * M_, 100 * K, 1 * M_, 1 * M_, 2.2 * M_, 0.47 * U, 470e-12)
    b.op("LD", "oc_hi"); b.op("MULI", imm=q(VH)); b.op("ST", "oc_sq")
    b.op("LD", "oc_e"); b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("SUB", "oc_sq")
    b.op("ADDI", imm=q(2 * VBE - 680e3 * VH / 2.7e6)); b.op("MAXI", imm=0); b.op("MINI", imm=q(VH)); b.op("ST", "oc")


def shot_slow(b):
    """every other sample (time constants / 2): the three envelope caps"""
    b.kreg("st_kb", rc_exp(680e3 * 0.22e-6 / 2), "T")
    b.kreg("st_kc", rc_exp(par(330e3, 120e3) * 2.2e-6 / 2), "T")
    b.kreg("st_kd", rc_exp(120e3 * 2.2e-6 / 2), "T")
    b.kreg("st_ke", rc_exp(1.2e6 * 2.2e-6 / 2), "T")
    # C503: peak via D502, R510 into N4 13 (the input stops conducting below VBE)
    b.op("LD", "st_vb"); b.op("MINI", imm=q(VBE)); b.op("RCM", "st_vb", b.r("st_kb"))
    b.op("LD", "st_1"); b.op("ADDI", imm=q(-D502)); b.op("CMP", "st_vb"); b.op("STF", "st_vb")
    # C502: D500 + R506 from the one-shot, R507 into N4 2
    b.op("LD", "st_1"); b.op("ADDI", imm=q(-D500)); b.op("CMP", "st_vc")
    at = b.skip_if(False)
    p = par(330e3, 120e3)
    b.op("MULI", imm=q(p / 330e3)); b.op("ADDI", imm=q(VBE * p / 120e3)); b.op("RCM", "st_vc", b.r("st_kc"))
    at2 = jump(b)
    b.land(at)
    b.op("LD", "st_vc"); b.op("MINI", imm=q(VBE)); b.op("RCM", "st_vc", b.r("st_kd"))
    b.land(at2)
    # N4 3/2/4 comparator (R508) -> D503 -> C504 (18 mA source), R509 into N4 13
    b.op("LD", "st_ve"); b.op("MINI", imm=q(VBE)); b.op("RCM", "st_ve", b.r("st_ke"))
    b.op("LD", "st_vc"); b.op("CMPI", imm=q(VBE + 120e3 * VH / 2.7e6))
    at = b.skip_if(False)
    b.op("LD", "st_ve"); b.op("ADDI", imm=q(ISRC * 2 * DT / 2.2e-6)); b.op("MINI", imm=q(VH - D503))
    b.op("CMP", "st_ve"); b.op("STF", "st_ve")
    b.land(at)


def shot_fast(b):
    oneshot(b, "st_1", (LATCH1, 1), 4.7 * M_, 1 * M_, 1 * M_, 1 * M_, 2.2 * M_, 0.47 * U, 470e-12)
    # N4 8/13/9 VCA: VBE + 680k (max(0, (VB - VBE) / 680k + (VE - VBE) / 1.2M) - (NOISE-2 - VBE) / 680k - VH / 2.7M)
    b.op("LD", "st_ve"); b.op("ADDI", imm=q(-VBE)); b.op("MULI", imm=q(680 / 1200)); b.op("ADD", "st_vb")
    b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("SUB", "nz")
    b.op("ADDI", imm=q(2 * VBE - 680e3 * VH / 2.7e6)); b.op("MAXI", imm=0); b.op("MINI", imm=q(VH)); b.op("ST", "st_o9")
    # N4 11/12/10: + the trigger node (12 V while D1) / R519, - N4 9 / R515, R516; gated off when D1 drops
    b.op("BIT", 0, b.src(LATCH1, 1))
    b.op("LDI", imm=q(2 * VBE - 680e3 * VH / 2.7e6)); b.op("LDIF", imm=q(2 * VBE - 680e3 * VH / 2.7e6 + 12 - VBE))
    b.op("SUB", "st_o9"); b.op("MAXI", imm=0); b.op("MINI", imm=q(VH))
    # C599 .001 uF high pass (R520 + R521), divider R521 / (R520 + R521): Thevenin source for R529 / C508
    b.kreg("st_kh", rc_exp((510e3 + 22e3) * 0.001e-6), "F")
    b.op("SUB", "st_hp"); b.op("ST", "_t"); b.op("MUL", "st_kh"); b.op("ADD", "st_hp"); b.op("ST", "st_hp")
    b.op("LD", "_t"); b.op("MULI", imm=q(SHOT_DIV)); b.op("ST", "st")


def hit(b):
    U10 = 0.01                                             # clock timer unit: 10 ms
    rc = 47e3 * 2.2e-6
    caps = (0.015e-6 + 0.01e-6, 0.022e-6)
    b.kreg("h_dt", 1.0 / (FS * U10), "O", clamp=False)
    for n, c in zip("AB", caps):
        b.kreg("h_kc" + n, rc_exp(61e3 * c), "O")
        b.kreg("h_kd" + n, rc_exp(39e3 * c), "O")
    kc = sum(rc_exp(61e3 * c) for c in caps) / 2
    kd = sum(rc_exp(39e3 * c) for c in caps) / 2
    ic, idd = 12.0 / ((5 - 10 / 3) * kc), 12.0 / ((5 / 3) * kd)    # sample-fraction scale at the thresholds
    b.init("h_ch", 1.0)

    def swap():
        b.op("LD", "h_v"); b.op("ST", "_t"); b.op("LD", "h_vi"); b.op("ST", "h_v"); b.op("LD", "_t"); b.op("ST", "h_vi")

    b.op("BIT", 0, b.src(LATCH1, 2))
    at = b.skip_if(True)
    # reset: 74107 cleared (Q = 0: C602 + C603), 556 discharging, clock cap from 0 V
    b.op("LD", "h_q"); b.op("CMPI", imm=q(0.5))
    a2 = b.skip_if(False)
    swap()
    b.op("LDI", imm=0); b.op("ST", "h_q")
    b.land(a2)
    b.op("LDI", imm=0); b.op("RCM", "h_v", b.r("h_kdA"))
    b.op("ADDI", imm=q(0.6)); b.op("CMP", "h_vi"); b.op("STNF", "h_vi")
    b.op("LDI", imm=ONE); b.op("ST", "h_ch")
    b.op("LDI", imm=q(math.log(3) * rc / U10)); b.op("ST", "h_ck")
    b.op("LDI", imm=0); b.op("ST", "h_out")
    end = [jump(b)]
    b.land(at)
    # 556 H1 (9) clock: falling edge toggles the 74107, which swaps the tone cap
    b.op("LD", "h_ck"); b.op("SUB", "h_dt"); b.op("ST", "h_ck"); b.op("CMPI", imm=0)
    a2 = b.skip_if(True)
    b.op("ADDI", imm=q(math.log(2) * rc / U10)); b.op("ST", "h_ck")
    b.op("LDI", imm=ONE); b.op("SUB", "h_q"); b.op("ST", "h_q")
    swap()
    b.land(a2)
    b.op("LD", "h_v"); b.op("ADDI", imm=q(0.6)); b.op("CMP", "h_vi"); b.op("STNF", "h_vi")
    b.op("LD", "h_q"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "h_kcA"); b.op("LDF", "h_kcB"); b.op("ST", "h_kc")
    b.op("LD", "h_kdA"); b.op("LDF", "h_kdB"); b.op("ST", "h_kd")
    # 556 H1 tone (R602 22k, R603 39k, 5 V): output 12 V (H2 + R604) while charging, energy at the toggles
    b.op("LD", "h_ch"); b.op("CMPI", imm=q(0.5))
    a3 = b.skip_if(False)
    b.op("LDI", imm=q(5.0)); b.op("RCM", "h_v", b.r("h_kc")); b.op("CMPI", imm=q(10 / 3))
    a4 = b.skip_if(True)
    b.op("LDI", imm=q(12.0)); b.op("ST", "h_out")
    end.append(jump(b))
    b.land(a4)
    b.op("ADDI", imm=q(-10 / 3)); b.op("MULI", imm=q(-ic / 16)); b.op("MULI", imm=q(16)); b.op("ADDI", imm=q(12.0))
    b.op("MAXI", imm=0)
    b.op("ST", "h_out"); b.op("LDI", imm=0); b.op("ST", "h_ch")
    end.append(jump(b))
    b.land(a3)
    b.op("LDI", imm=0); b.op("RCM", "h_v", b.r("h_kd")); b.op("CMPI", imm=q(5 / 3) + 1)
    a5 = b.skip_if(False)
    b.op("LDI", imm=0); b.op("ST", "h_out")
    end.append(jump(b))
    b.land(a5)
    b.op("ADDI", imm=q(-5 / 3)); b.op("MULI", imm=q(-idd / 16)); b.op("MULI", imm=q(16)); b.op("MINI", imm=q(12.0))
    b.op("ST", "h_out"); b.op("LDI", imm=ONE); b.op("ST", "h_ch")
    for e in end:
        b.land(e)


def shark_slow(b):
    """every other sample (C / 2): three slow oscillators, the gated VCO"""
    # three slow oscillators (OSC_2 Norton: diode charge from the output, R to the - input, 1.2M bias, 1.5M hyst.)
    for n, (r6, r2, c, d) in enumerate(SH_OSC):
        o = f"s{n}"
        rc = par(r2, r6)
        v1 = ((VH - d) / r6 + VBE / r2) * rc
        tl = (VH / 1.2e6 - VBE / 1.5e6) * r2 + VBE
        th = (VH / 1.2e6 + (12 - 2 * VBE) / 1.5e6) * r2 + VBE
        b.kreg(o + "_k1", rc_exp(rc * c / 2), "O")
        b.kreg(o + "_k0", rc_exp(r2 * c / 2), "O")
        b.init(o + "_ch", 1.0)
        b.op("LD", o + "_ch"); b.op("CMPI", imm=q(0.5))
        b.op("LD", o + "_k0"); b.op("LDF", o + "_k1"); b.op("ST", "_k")
        b.op("LDI", imm=q(VBE)); b.op("LDIF", imm=q(v1))
        b.op("RCM", o + "_v", b.r("_k"))
        at = b.skip_if(False)
        b.op("CMPI", imm=q(th)); b.op("LDI", imm=0); b.op("STF", o + "_ch")
        at2 = jump(b)
        b.land(at)
        b.op("CMPI", imm=q(tl) + 1); b.op("LDI", imm=ONE); b.op("STNF", o + "_ch")
        b.land(at2)
    # M3 10: all three high -> D703 -> R717 into K3 3; K3 2 gets R722 (Schmitt) + R718 (D3 enable)
    c = 0.033e-6 / 2
    tl = (VH / 1.5e6 - VBE / 2.2e6) * 560e3 + VBE
    th = (VH / 1.5e6 + (12 - 2 * VBE) / 2.2e6) * 560e3 + VBE
    b.kreg("s_aen", I_EN * DT / c, "O", clamp=False)
    b.kreg("s_a1", (VH - VBE) / 2.2e6 * DT / c, "O", clamp=False)
    b.kreg("s_a0", -VBE / 2.2e6 * DT / c, "O", clamp=False)
    b.kreg("s_av", (VH - D703 - VBE) / 1.5e6 * DT / c, "O", clamp=False)
    b.init("s_ch", 1.0)
    b.op("BIT", 0, b.src(LATCH1, 3)); b.op("LDI", imm=0); b.op("LDF", "s_aen"); b.op("ST", "_t")
    b.op("LD", "s_ch"); b.op("CMPI", imm=q(0.5)); b.op("LD", "s_a0"); b.op("LDF", "s_a1"); b.op("ADD", "_t")
    b.op("MAXI", imm=0); b.op("ST", "_t")
    b.op("LD", "s0_ch"); b.op("MUL", "s1_ch"); b.op("MUL", "s2_ch"); b.op("MUL", "s_av"); b.op("ST", "_u")
    b.op("LD", "_t"); b.op("SUB", "_u"); b.op("ADD", "s_v"); b.op("MAXI", imm=0); b.op("MINI", imm=q(VH))
    b.op("ST", "s_v")
    b.op("LD", "s_ch"); b.op("CMPI", imm=q(0.5))
    at = b.skip_if(False)
    b.op("LD", "s_v"); b.op("CMPI", imm=q(th)); b.op("LDI", imm=0); b.op("STF", "s_ch")
    at2 = jump(b)
    b.land(at)
    b.op("LD", "s_v"); b.op("CMPI", imm=q(tl) + 1); b.op("LDI", imm=ONE); b.op("STNF", "s_ch")
    b.land(at2)


def shark_fast(b):
    # noise ladder: R724 / C704, R725 / C705, R726 into K3 11 (a Norton input: C706 stays at VBE)
    p1, p2 = par(12e3, 68e3), par(68e3, 330e3)
    b.kreg("s_k1", rc_exp(p1 * 0.015e-6), "F")
    b.kreg("s_k2", rc_exp(p2 * 0.015e-6), "F")
    b.init("s_l1", 6.0)
    b.init("s_l2", 6.0)
    b.op("LD", "nz_h4"); b.op("MULI", imm=q(p1 / 12e3)); b.op("MACI", "s_l2", imm=q(p1 / 68e3))
    b.op("RCM", "s_l1", b.r("s_k1"))
    b.op("MULI", imm=q(p2 / 68e3)); b.op("ADDI", imm=q(VBE * p2 / 330e3)); b.op("RCM", "s_l2", b.r("s_k2"))
    # K3 12/11/10: VBE + 1M (VCO / R723 - VH / R727 - (V2 - VBE) / R726)
    b.op("LD", "s_l2"); b.op("MULI", imm=q(-1e6 / 330e3))
    b.op("ADDI", imm=q(VBE - 1e6 * VH / 2.2e6 + VBE * 1e6 / 330e3)); b.op("ST", "_t")
    b.op("LD", "s_ch"); b.op("MULI", imm=q(1e6 * (VH - VBE) / 560e3)); b.op("ADD", "_t")
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VH)); b.op("ST", "s_vt")
    # C707 at K3 10: rise at the 18 mA source limit, fall at the 1.3 mA sink less R729 27k, plus the R730 load
    ks, kp, kl = DT / C707, DT / (27e3 * C707), DT / (MIX_R[3] * C707)
    b.op("CMP", "s_o")
    at = b.skip_if(False)
    b.op("LD", "s_o"); b.op("ADDI", imm=q(ISRC * ks)); b.op("CMP", "s_vt"); b.op("LDF", "s_vt")
    at2 = jump(b)
    b.land(at)
    b.op("LD", "s_o"); b.op("MULI", imm=q(-(kp + kl))); b.op("ADDI", imm=q(-ISINK * ks + 12 * kp))
    b.op("MACI", "m_c3", imm=q(kl)); b.op("MACI", "m_bus", imm=q(kl)); b.op("ADD", "s_o"); b.op("CMP", "s_vt")
    b.op("NOT"); b.op("LDF", "s_vt")
    b.land(at2)
    b.op("ST", "s_o")


def mixer(b):
    srcs = ["oc", "st", "h_out", "s_o"]
    for k, (s, r) in enumerate(zip(srcs, MIX_R)):
        b.kreg(f"m_a{k}", DT / (r * MIX_C), "F", clamp=False)
        b.op("LD", s); b.op("SUB", f"m_c{k}"); b.op("SUB", "m_bus"); b.op("ST", f"m_x{k}")
        b.op("MUL", f"m_a{k}"); b.op("ADD", f"m_c{k}"); b.op("ST", f"m_c{k}")
    b.op("LDI", imm=0)
    for k, r in enumerate(MIX_R):
        b.op("MACI", f"m_x{k}", imm=q(DT / (r * C900)))
    b.op("MUL", "_fF"); b.op("ADD", "m_bus"); b.op("ST", "m_bus")
    b.op("LDI", imm=0)
    for k, r in enumerate(MIX_R):
        b.op("MACI", f"m_x{k}", imm=q(OUT_K / r))
    # L-M2 / LM377 bridge: 2 (HP(a) + HP(HP(a))), C903 / C904 / C905 0.1 uF into 100k
    b.kreg("a_k", rc_exp(AMP_TAU), "F")
    b.op("SUB", "a_hp1"); b.op("ST", "_t"); b.op("MUL", "a_k"); b.op("ADD", "a_hp1"); b.op("ST", "a_hp1")
    b.op("LD", "_t"); b.op("SUB", "a_hp2"); b.op("ST", "_u"); b.op("MUL", "a_k"); b.op("ADD", "a_hp2")
    b.op("ST", "a_hp2")
    b.op("LD", "_u"); b.op("ADD", "_t"); b.op("MULI", imm=q(0.5))
    b.op("BIT", 1, b.src(LATCH1, 0))                       # D0 low: amplifier muted (D900 / D901)
    b.op("LDIF", imm=0)
    b.op("OUT")


def blueshrk():
    b = Board("blueshrk")
    noise(b)
    b.op("LDI", imm=ONE); b.op("SUB", "_ph"); b.op("ST", "_ph"); b.op("CMPI", imm=q(0.5))   # slow blocks: A / B
    at = b.skip_if(False)
    octo_slow(b)
    shot_slow(b)
    at2 = jump(b)
    b.land(at)
    shark_slow(b)
    b.land(at2)
    octo_fast(b)
    shot_fast(b)
    hit(b)
    shark_fast(b)
    mixer(b)
    return b.finish()


if __name__ == "__main__":
    p = blueshrk()
    fn = sys.argv[1] if len(sys.argv) > 1 else "blueshrk.bin"
    open(fn, "wb").write(p.binary())
    print(f"blueshrk: {len(p.code)} words, worst sample {p.worst_path()} units, {len(p.regs)} registers -> {fn}")
