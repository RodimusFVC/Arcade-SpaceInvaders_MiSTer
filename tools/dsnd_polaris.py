"""Polaris (Taito 1980) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

No schematic yet: MAME polaris_discrete (src/mame/midw8080/8080bw_a.cpp, D. Renaud) is the reference, node for node.
Sources (rtl/mw8080_board.sv, v_pol io_a renumbering): LATCH1 = port 4 (D0 SX0 shot, D1 SX1 sub hit, D2 SX2 ship,
D3 SX3 explosion, D5 SX5 sound enable), LATCH2 = port 6 (D0 SX6 plane down, D1 SX7 plane up, D3 SX9 / D4 SX10 hit),
LATCH3 = port 2 music data (FF = stopped).
Trimmers at MAME's defaults: VR1 80 %, VR2 70 %, VR3 90 % (x 0.5376 resistor scaling = the music input; the Game
Audio Music Volume scales it). Output = MAME's mixer x 40000 / 32768 (route 1.0); explosions clip, as in MAME.
Not modelled: the noise one-shot (5F / 1S1588, 1.5 ms every 8.5 s) that feeds ones into MAME's LFSRs (reset value 0):
it only keeps MAME's noise from locking up; the engine LFSRs free-run.
"""
import sys
from dsnd import Board, LATCH1, LATCH2, LATCH3, q, ONE, FS, rc_exp, VBE, par

K, M_, U, P_ = 1e3, 1e6, 1e-6, 1e-12
VP = 12.0
VH = VP - VBE                                        # Norton output high / input current reference


def vco1_norton_cap(b, out, vm, en, r1, r2, r3, r4, r5, c):
    """DSS_OP_AMP_OSC VCO_1 | NORTON | OUT_CAP, linear charge. vm = register holding MAME's v - VBE (Millman of
    the bias / modulation inputs, worked out by the caller). en = (source, bit) or None (always enabled); disabled =
    MAME force_charge (the cap charges on to the rail). out = the cap voltage (dsnd.op_amp_vco1_norton_cap with an
    optional enable and the Millman input folded in)"""
    tl = (VH / r5 - (VP - 2 * VBE) / r4) * r3 + VBE
    th = (VH / r5 + VBE / r4) * r3 + VBE
    b.kreg(out + "_nk0", -1.0 / (r1 * FS * c), "O", clamp=False)
    b.kreg(out + "_k1", (1.0 / r2 - 1.0 / r1) / (FS * c), "O", clamp=False)
    b.op("LD", out + "_hi")                           # _hi = MAME flip_flop: charges while low; power-up low
    b.op("CMPI", imm=q(0.5))
    if en is not None:
        b.op("FAND", 0, b.src(*en))
    at = b.skip_if(True)
    b.op("LD", vm)                                    # charging
    b.op("MUL", out + "_k1")
    b.op("ADD", out)
    if en is not None:
        b.op("MINI", imm=q(VH))                       # forced: the cap stops at the rail
    b.op("ST", out)
    b.op("CMPI", imm=q(th))
    at2 = b.skip_if(False)
    b.op("LDI", imm=ONE)
    b.op("ST", out + "_hi")
    at_f = None
    if en is not None:
        b.op("BIT", 0, b.src(*en))
        at_f = b.skip_if(False)                       # forced: keep charging, no carry
    b.op("LD", out)
    b.op("ADDI", imm=q(-th))
    b.op("MULI", imm=q(-r2 / (r1 - r2)))              # overshoot time x discharge rate (ratio fixed)
    b.op("ADDI", imm=q(th))
    b.op("ST", out)
    if at_f is not None:
        b.land(at_f)
    b.land(at2)
    at3 = b.jump()
    b.land(at)
    b.op("LD", vm)                                    # discharging
    b.op("MUL", out + "_nk0")
    b.op("ADD", out)
    b.op("ST", out)
    b.op("CMPI", imm=q(tl))
    at4 = b.skip_if(True)
    b.op("LDI", imm=0)
    b.op("ST", out + "_hi")
    b.op("LD", out)
    b.op("ADDI", imm=q(-tl))
    b.op("MULI", imm=q(-(r1 - r2) / r2))
    b.op("ADDI", imm=q(tl))
    b.op("ST", out)
    b.land(at4)
    b.land(at3)


def osc1_norton_cap(b, out, r1, r2, r3, r4, r5, c):
    """DSS_OP_AMP_OSC type 1 | NORTON | OUT_CAP (no enable), linear charge, overshoot carried; out = the cap.
    Power-up: charging from 0 V (MAME flip_flop 1)"""
    ch0 = VH / r1
    ch1 = (VH - VBE) / r2 - ch0
    i1 = VH / r5
    tl = (i1 - VBE / r4) * r3 + VBE
    th = (i1 + (VH - VBE) / r4) * r3 + VBE
    b.kreg(out + "_d0", ch0 / FS / c, "O", clamp=False)
    b.kreg(out + "_d1", ch1 / FS / c, "O", clamp=False)
    b.op("LD", out + "_lo")                           # _lo = 1 while discharging
    b.op("CMPI", imm=q(0.5))
    at = b.skip_if(True)
    b.op("LD", out)                                   # charging
    b.op("ADD", out + "_d1")
    b.op("ST", out)
    b.op("CMPI", imm=q(th))
    at2 = b.skip_if(False)
    b.op("ADDI", imm=q(-th))                          # overshoot -> discharge time
    b.op("MULI", imm=q(-ch0 / ch1))
    b.op("ADDI", imm=q(th))
    b.op("ST", out)
    b.op("LDI", imm=ONE)
    b.op("ST", out + "_lo")
    b.land(at2)
    at3 = b.jump()
    b.land(at)
    b.op("LD", out)                                   # discharging
    b.op("SUB", out + "_d0")
    b.op("ST", out)
    b.op("CMPI", imm=q(tl))
    at4 = b.skip_if(True)
    b.op("ADDI", imm=q(-tl))
    b.op("MULI", imm=q(-ch1 / ch0))
    b.op("ADDI", imm=q(tl))
    b.op("ST", out)
    b.op("LDI", imm=0)
    b.op("ST", out + "_lo")
    b.land(at4)
    b.land(at3)


def bp0_norton(b, out, src, r1, r2, r3, r4, rf, c1, c2, c3):
    """DST_OP_AMP_FILT BAND_PASS_0 | NORTON: v = max(0, in - VBE) -> c1 -> c2 -> high pass c3; out = (iFixed -
    (vC2 - vC3) / rTotal) x rF clipped 0 .. vP - VBE. c1 with k = 1 (tau << 1 sample) is skipped (identical)"""
    rt = r1 + r2 + r3
    k1 = rc_exp(par(r1, r2 + r3 + r4) * c1)
    k2 = rc_exp(par(r1 + r2, r3 + r4) * c2)
    k3 = rc_exp((r1 + r2 + r3 + r4) * c3)
    b.op("LD", src)
    b.op("ADDI", imm=q(-VBE))
    b.op("MAXI", imm=0)
    if q(k1) < ONE:
        b.kreg(out + "_k1", k1, "F")
        b.op("RCM", out + "_c1", b.r(out + "_k1"))
    b.kreg(out + "_k2", k2, "F")
    b.kreg(out + "_k3", k3, "F")
    b.op("RCM", out + "_c2", b.r(out + "_k2"))
    b.op("SUB", out + "_c3")
    b.op("ST", out + "_hp")
    b.op("LD", out + "_c2")
    b.op("RCM", out + "_c3", b.r(out + "_k3"))
    b.op("LD", out + "_hp")
    b.op("MULI", imm=q(-rf / rt))
    b.op("ADDI", imm=q(VH / r4 * rf))
    b.op("MAXI", imm=0)
    b.op("MINI", imm=q(VH))
    b.op("ST", out)
    b.op("ADDI", imm=q(-VBE))                         # out_i = max(0, out - VBE): a TVCA input current x r
    b.op("MAXI", imm=0)
    b.op("ST", out + "_i")


def tvca_c1(b, out, c1, nreg, r5=1 * K, r67=680 * K, r4=680 * K):
    """DST_OP_AMP_TRIG_VCA output with F = F2 on entry, F3 none (r6 = 0): c1 charges toward vt1 through r5 || r7
    while F2, else discharges toward VBE through r7; out = max(0, r4 (c1 - VBE) / r67 - n), n = the i_neg x r4
    register. i_pos's own clamp at 0 is dropped (n > 0: the result clamps to 0 either way); out < vP - VBE always"""
    vt1 = (12.0 - 0.6 - VBE) * r67 / (r5 + r67) + VBE
    b.kreg(out + "_kc", rc_exp(par(r5, r67) * c1), "T")
    b.kreg(out + "_kd", rc_exp(r67 * c1), "T")
    b.op("LD", out + "_kd"); b.op("LDF", out + "_kc"); b.op("ST", out + "_k")
    b.op("LDI", imm=q(VBE)); b.op("LDIF", imm=q(vt1))
    b.op("RCM", out + "_c1", b.r(out + "_k"))
    b.op("ADDI", imm=q(-VBE))
    if r4 != r67:
        b.op("MULI", imm=q(r4 / r67))
    b.op("SUB", nreg)
    b.op("MAXI", imm=0)
    b.op("ST", out)


def sonar_clock(b):
    """5L / 5F: SQUAREWFIX 60 / 16 Hz (6400 samples per half period exactly; power-up low for one half), counter
    on the falling edges (bit 2 = NODE_42), D flip-flop NODE_46 clocks NODE_45 = bit 2 | SX3 on the rising edges.
    Leaves F = NODE_45"""
    b.op("LD", "sq_t"); b.op("ADDI", imm=1); b.op("ST", "sq_t")       # raw sample counter
    b.op("CMPI", imm=6400)
    at = b.skip_if(False)
    b.op("LDI", imm=0); b.op("ST", "sq_t")
    b.op("LD", "sq"); b.op("CMPI", imm=q(0.5))                         # F = high now: falling edge
    b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", "sq")
    at_r = b.skip_if(False)
    b.op("LD", "cnt"); b.op("ADDI", imm=ONE); b.op("CMPI", imm=8 * ONE); b.op("LDIF", imm=0); b.op("ST", "cnt")
    b.op("CMPI", imm=4 * ONE); b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "b2")
    at_e = b.jump()
    b.land(at_r)
    b.op("LD", "b2"); b.op("CMPI", imm=q(0.5)); b.op("FOR", 0, b.src(LATCH1, 3))   # rising: dff = NODE_45
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "dff")
    b.land(at_e)
    b.land(at)
    b.op("LD", "b2"); b.op("CMPI", imm=q(0.5)); b.op("FOR", 0, b.src(LATCH1, 3))


def sonar_tvca(b, out, vco):
    """DST_OP_AMP_TRIG_VCA polaris_sonar_tvca_info with its dynamic F3: trg0 = NODE_45 (F on entry), trg1 = NODE_46.
    f2 = TRG1 (c1 charges while the flip-flop is high), f3 = TRG0_INV: c1 targets / rates switch with F3 and i_pos
    is cut while F3 is low. r2 = r4 = r6 + r7 = 680k: unit gains"""
    r1, r2, r4, r5, r6, r7, c1 = 2.7 * M_, 680 * K, 680 * K, 1 * K, 120 * K, 560 * K, 1 * U
    r67 = r6 + r7
    vt0 = (12.0 - 0.6) * r6 / (r5 + r6)
    vt1 = (12.0 - 0.6 - VBE) * r67 / (r5 + r67) + VBE
    assert r4 == r2 == r67
    b.kreg(out + "_kc0", rc_exp(par(r5, r6) * c1), "T")
    b.kreg(out + "_kc1", rc_exp(par(r5, r67) * c1), "T")
    b.kreg(out + "_kd0", rc_exp(r6 * c1), "T")
    b.kreg(out + "_kd1", rc_exp(r67 * c1), "T")
    # i_neg x r4 first (the VCO, unit gain): F is kept
    b.op("LD", vco + "_i"); b.op("ADDI", imm=q(r4 * VH / r1)); b.op("ST", out + "_n")
    # F = NODE_45 (F3 low) on entry
    b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", out + "_f3")
    b.op("LD", "dff"); b.op("CMPI", imm=q(0.5))       # F = F2
    at = b.skip_if(False)
    b.op("LD", out + "_f3"); b.op("CMPI", imm=q(0.5))     # charge: F = F3
    b.op("LD", out + "_kc0"); b.op("LDF", out + "_kc1"); b.op("ST", out + "_k")
    b.op("LDI", imm=q(vt0)); b.op("LDIF", imm=q(vt1))
    b.op("RCM", out + "_c1", b.r(out + "_k"))
    at2 = b.jump()
    b.land(at)
    b.op("LD", out + "_f3"); b.op("CMPI", imm=q(0.5))     # discharge: toward VBE via r6 + r7 (F3) / 0 V via r6
    b.op("LD", out + "_kd0"); b.op("LDF", out + "_kd1"); b.op("ST", out + "_k")
    b.op("LDI", imm=0); b.op("LDIF", imm=q(VBE))
    b.op("RCM", out + "_c1", b.r(out + "_k"))
    b.land(at2)
    b.op("ADDI", imm=q(-VBE))
    b.op("MUL", out + "_f3")                          # i_pos x r4 (0 while F3 is low)
    b.op("SUB", out + "_n")
    b.op("MAXI", imm=0)
    b.op("ST", out)


def music(b):
    """DISCRETE_NOTE 23396 Hz (555 / 2, Guru) / (256 - data), 2-bit counter 5H -> 47k / 12k DAC_R1 (180p: < 2 us,
    k = 1) -> DST_OP_AMP_FILT HIGH_PASS_0 | NORTON (r1 580k, r4 2.2M, rF 1M, c1 .01u). data FF holds the counter.
    MAME reloads the 8-bit counter with the latch at overflow; here the period follows the latch at once"""
    rt = par(47 * K, 12 * K)
    steps = (0.0, 3.4 * rt / (47 * K), 3.4 * rt / (12 * K), 3.4)
    b.kreg("mu_inc", 23396.0 / FS / 256.0, "O", clamp=False)          # Q16 clocks per sample
    xs = [max(0.0, v - VBE) for v in steps]           # the Norton input: max(0, DAC - VBE), one per 5H count
    for k, x in enumerate(xs):                        # mu_x0 = the current count's; a step rotates them
        b.init(f"mu_x{k}", x)
    b.op("LDLM", 0, b.src(LATCH3, 0), imm=0xFF | (16 << 8))
    b.op("MULI", imm=-ONE)
    b.op("ADDI", imm=256 << 16)
    b.op("ST", "mu_p")                                # (256 - data) clocks, Q16
    b.op("CMPI", imm=2 << 16)
    at = b.skip_if(False)                             # data FF: held
    b.op("LD", "mu_ph"); b.op("ADD", "mu_inc"); b.op("ST", "mu_ph")
    b.op("CMP", "mu_p")
    at2 = b.skip_if(False)
    b.op("SUB", "mu_p"); b.op("ST", "mu_ph")
    b.op("LD", "mu_x0"); b.op("ST", "mu_t")
    for k in range(3):
        b.op("LD", f"mu_x{k + 1}"); b.op("ST", f"mu_x{k}")
    b.op("LD", "mu_t"); b.op("ST", "mu_x3")
    b.land(at2)
    b.land(at)
    r1, r4, rf, c1 = 580 * K, 2.2 * M_, 1 * M_, 0.01 * U
    b.kreg("mu_k", rc_exp(r1 * c1), "F")
    b.op("LD", "mu_x0"); b.op("SUB", "mu_hc"); b.op("ST", "mu_hp")    # v - vC1 (before vC1 moves)
    b.op("LD", "mu_x0"); b.op("RCM", "mu_hc", b.r("mu_k"))
    b.op("LD", "mu_hp"); b.op("MULI", imm=q(-rf / r1)); b.op("ADDI", imm=q(VH / r4 * rf))
    b.op("ST", "mu")                                  # 0.23 .. 10.2 V: never clips


def polaris():
    b = Board("polaris")

    # ---- noise: 17-bit LFSRs (7K pins 4 / 10), clocks measured by Guru; outputs 0 / 11.5 V
    b.lfsr_noise("nlo", 800.8, VH, VH / 2, n=0)
    b.lfsr_noise("nhi", 23396.0, VH, VH / 2, n=1)
    bp0_norton(b, "nlf", "nlo", 560, 6.8 * K, 1002 * K, 2.2 * M_, 1 * M_, 0.22 * U, 0.22 * U, 1 * U)
    bp0_norton(b, "nhf", "nhi", 560, 6.8 * K, 1002 * K, 2.2 * M_, 1 * M_, 0.001 * U, 0.001 * U, 0.01 * U)

    # ---- sonar: VCO1 (r6 120k bias, r7 1M from NOISE_LO_FILT) into the clocked TVCA
    sonar_clock(b)                                    # F = NODE_45
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "n45")
    r1, r2, r6, r7 = 1 * M_, 680 * K, 120 * K, 1 * M_
    rt = par(r1, r2, r6, r7)
    ifx = VP / r6 + VBE / r1 + VBE / r2
    b.op("LD", "nlf"); b.op("MULI", imm=q(rt / r7)); b.op("ADDI", imm=q(ifx * rt - VBE)); b.op("ST", "svco_vm")
    vco1_norton_cap(b, "svco", "svco_vm", None, r1, r2, 680 * K, 1 * M_, 1 * M_, 180 * P_)
    b.op("LD", "svco"); b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("ST", "svco_i")
    b.op("LD", "n45"); b.op("CMPI", imm=q(0.5))
    sonar_tvca(b, "son", "svco")

    # ---- shot SX0, sub hit SX1: TVCAs on the filtered noises
    # polaris_shot_tvca_info: r1 2.7M, r2 = r3 = r4 = r7 680k, r5 1k, c1 1u; f0 TRG0_INV (r2 = hi noise), f1 TRG0
    # (r3 = lo noise), f2 TRG0. i_neg x r4 = r4 vH / r1 + the selected noise current (unit gain)
    fx = 680 * K * VH / (2.7 * M_)
    b.op("BIT", 0, b.src(LATCH1, 0))
    b.op("LD", "nhf_i"); b.op("LDF", "nlf_i"); b.op("ADDI", imm=q(fx)); b.op("ST", "shot_n")
    tvca_c1(b, "shot", 1 * U, "shot_n")               # F = SX0 still
    # polaris_shiphit_tvca_info: as the shot but f1 none (lo noise always)
    b.op("BIT", 0, b.src(LATCH1, 1))
    b.op("LD", "nhf_i"); b.op("LDIF", imm=0); b.op("ADD", "nlf_i"); b.op("ADDI", imm=q(fx)); b.op("ST", "shit_n")
    tvca_c1(b, "shit", 1 * U, "shit_n")

    # ---- ship SX2: 5.75 Hz |\|\ (cap of a Norton osc) sweeps a VCO1 (vmod fed straight in)
    osc1_norton_cap(b, "bm", 1 * M_, 10 * K, 100 * K, 120 * K, 1 * M_, 0.22 * U)
    b.op("LD", "bm"); b.op("ADDI", imm=q(-VBE)); b.op("ST", "ship_vm")
    vco1_norton_cap(b, "ship", "ship_vm", (LATCH1, 2), 1 * M_, 680 * K, 680 * K, 1 * M_, 1 * M_, 180 * P_)

    # ---- explosion SX3: TVCA on the raw low noise, two RC low passes
    # polaris_exp_tvca_info: r2 680k = raw lo noise always, c1 .33u, f2 TRG0
    b.op("LD", "nlo"); b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("ADDI", imm=q(fx)); b.op("ST", "ex0_n")
    b.op("BIT", 0, b.src(LATCH1, 3))
    tvca_c1(b, "ex0", 0.33 * U, "ex0_n")
    b.rcfilter("ex1", "ex0", 560.0, 0.22 * U)
    b.rcfilter("ex", "ex1", 6.8 * K, 0.22 * U)

    # ---- plane SX6 down / SX7 up: DISC_INTEGRATE_OP_AMP_2 | NORTON (2.2u, Guru) into VCO1 with r6 / r7 / r8
    ci = 2.2 * U
    up, dn, rst = VH / (1001 * K) / FS / ci, (VH - VBE) / (1001 * K) / FS / ci, (VH - VBE) / (101 * K) / FS / ci
    b.kreg("pi_up", up, "T", clamp=False)
    b.kreg("pi_dn", -dn, "T", clamp=False)
    b.kreg("pi_rs", rst, "T", clamp=False)
    b.op("BIT", 0, b.src(LATCH2, 0))
    b.op("LD", "pi_up"); b.op("LDF", "pi_dn")
    b.op("ADD", "pi")
    b.op("ST", "pi")
    b.op("BIT", 1, b.src(LATCH2, 1))                  # SX7 low: r3 charges it up fast
    b.op("ADD", "pi_rs")
    b.op("STF", "pi")
    b.op("LD", "pi"); b.op("MAXI", imm=0); b.op("MINI", imm=q(VH)); b.op("ST", "pi")
    r1, r2, r6, r7, r8 = 1 * M_, 680 * K, 100 * K, 10 * K, 100 * K
    rt = par(r1, r2, r6, r7, r8)
    ifx = VP / r6 + VBE / r1 + VBE / r2
    b.op("LD", "pi"); b.op("MULI", imm=q(rt / r7)); b.op("MACI", "nhf", imm=q(rt / r8))
    b.op("ADDI", imm=q(ifx * rt - VBE)); b.op("ST", "pl_vm")
    vco1_norton_cap(b, "pl", "pl_vm", (LATCH2, 1), r1, r2, 680 * K, 1 * M_, 1 * M_, 2000 * P_)

    # ---- hit SX10 / SX9 (c1 1u: Guru's board; the schematic shows .22u)
    # polaris_hit_tvca_info: r2 = r3 = 1360k (gain .5): f0 TRG0 = SX10 lo noise, f1 TRG1 = SX9 hi noise,
    # f2 TRG01_NAND
    b.op("BIT", 1, b.src(LATCH2, 4))
    b.op("LD", "nlf_i"); b.op("LDIF", imm=0); b.op("ST", "hit_n")
    b.op("BIT", 1, b.src(LATCH2, 3))
    b.op("LD", "nhf_i"); b.op("LDIF", imm=0); b.op("ADD", "hit_n")
    b.op("MULI", imm=q(0.5)); b.op("ADDI", imm=q(fx)); b.op("ST", "hit_n")
    b.op("BIT", 0, b.src(LATCH2, 4)); b.op("FAND", 0, b.src(LATCH2, 3)); b.op("NOT")
    tvca_c1(b, "hit", 1 * U, "hit_n")

    music(b)

    # ---- mixers VR1 (shot, sonar, hit, plane), VR2 (ship, sub hit), VR4 (VR1 x .8, explosion, VR2 x .7, music x
    # VR3): one Millman sum; input caps see r || rF 50k. Caps with equal time constants share one high pass
    # (linear: exact): shot + ship (66k), sonar + plane (43k), hit + explosion (20k), sub hit (110k)
    rf = 50 * K
    rt1 = par(66 * K, 43 * K, 20 * K, 43 * K, rf)
    rt2 = par(66 * K, 110 * K, rf)
    rt4 = par(22 * K, 20 * K, 22 * K, 22 * K, rf)
    g1, g2 = rt4 / (22 * K) * 0.8 * rt1, rt4 / (22 * K) * 0.7 * rt2
    w = dict(shot=g1 / (66 * K), son=g1 / (43 * K), hit=g1 / (20 * K), pl=g1 / (43 * K),
             ship=g2 / (66 * K), shit=g2 / (110 * K), ex=rt4 / (20 * K))
    w_mu = rt4 / (22 * K) * 0.9 * 0.5376
    chans = (("A", (("shot", w["shot"]), ("ship", w["ship"])), 66 * K), ("B", (("son", w["son"]), ("pl", w["pl"])), 43 * K),
             ("C", (("hit", w["hit"]), ("ex", w["ex"])), 20 * K), ("D", (("shit", w["shit"]),), 110 * K))
    for name, ins, r in chans:
        b.kreg("mx_k" + name, rc_exp(par(r, rf) * 1 * U), "F")
        b.op("LD", ins[0][0]); b.op("MULI", imm=q(ins[0][1]))
        for reg, k in ins[1:]:
            b.op("MACI", reg, imm=q(k))
        b.op("ST", "mx_" + name)
        b.op("RCM", "mx_h" + name, b.r("mx_k" + name))
    b.music = True                                    # Music Volume (Game Audio) scales the VR3 input
    b.op("LD", "mu"); b.op("MULI", imm=q(w_mu)); b.op("MUL", "_fM")
    for name, _, _ in chans:
        b.op("ADD", "mx_" + name)
    for name, _, _ in chans:
        b.op("SUB", "mx_h" + name)
    b.op("ST", "mx")
    # SX5 low: VR4 mixer disabled (output 0, its cAmp state held)
    b.kreg("mx_ka", rc_exp(100 * K * 1 * U), "F")
    b.op("BIT", 0, b.src(LATCH1, 5))
    at = b.skip_if(False)
    b.op("RCM", "mx_amp", b.r("mx_ka"))
    b.op("LD", "mx"); b.op("SUB", "mx_amp")
    b.op("MULI", imm=q(40000.0 / 32768.0))
    b.op("OUT")
    b.land(at)
    return b.finish()


if __name__ == "__main__":
    p = polaris()
    out = sys.argv[1] if len(sys.argv) > 1 else "polaris.bin"
    open(out, "wb").write(p.binary())
    print(f"polaris: {len(p.code)} words, worst sample {p.worst_path()} units, {len(p.regs)} registers -> {out}")
