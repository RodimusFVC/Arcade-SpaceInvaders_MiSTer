"""Clowns (Midway 1978) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Validated against the Game Logic schematic P.C. A084-90700-B630 (one sheet, `Clowns_Game_Logic_(A084-90700-B630).pdf`),
starting from dsnd_boards.clowns() (MAME clowns_discrete, HW-confirmed). Ports: S1 = port 3, S2 / S3 = tone generator
low / high (ports 5 / 6), S4 = port 7 (H3 74174: D0-D2 balloon pops bottom / middle / top, D3 sound enable (J4 7404 ->
D600 / D601 into both K1 inputs: low mutes), D4 springboard hit, D5 springboard miss).

Schematic vs MAME (defaults = schematic; clowns(mame=True) rebuilds the MAME program bit-exactly):
- C300 (top pop, R300 470K) .15 uF; MAME CAP_U(.015). C302 (bottom pop, R302 680K) .1 uF; MAME CAP_U(.082).
- Pop filter: K3 pin 9 -> R307 15K -> C303 .01 uF to ground -> .01 uF -> 39K to ground || R420 / R421 10K into the
  mixer (C602 1 uF into K1's virtual ground). One loaded network: V_B / V_in = s C2 Rb / (1 + s (C2 Rb + R1 (C1 + C2))
  + s^2 R1 Rb C1 C2), Rb = 39K || 10K, then V_B / 10K into the mixer. Poles 461 Hz (high-pass side) and 4.6 kHz; MAME
  cascades RC 15K (1.06 kHz) and CR 54K (295 Hz), x 39 / 54, into 10K + 15K || 39K.
- Springboard hit filter: M3 8/13/9 (R210 100K in, R211 100K bias, R213 300K || C203 .0082 uF) and M3 11/12/10
  (R214 27K, C204 .0082 uF integrator, R215 51K bias) with R216 30K from pin 10 back to pin 13: a two-integrator
  loop, H = 0.3 / (1 + s R214 C204 R216 / R213 + s^2 R214 R216 C203 C204): 682 Hz, Q 10.5, DC gain 0.3 (ngspice AC with
  the LM3900 macro: peak 670 Hz x 3.18, DC 0.307). MAME: FILTER2 low pass 500 Hz, Q 0.8, x 0.5 ("calculated with
  SPICE").
- Pop and springboard-hit VCAs (K3): Norton balance as drawn (tvca_norton): output + VBE, below-VBE input current,
  caps charged from J3 1K / +5 V through the diode and discharged into the + input down to VBE. MAME's TVCA comes out
  ~1 V low, so its hit envelope ends ~0.12 s after D4 drops instead of ~0.4 s (ngspice), its pops at about half level.
- Mixer: C602 1 uF between the summing bus and K1 high-passes every input through the bus source resistance (7.8K,
  20 Hz); MAME uses rF x cAmp (0.1 s), which turns the hit VCA's DC step into a 0.1 s thump.
- Springboard miss (MAME: a sample): built from the schematic, see clowns_miss() below. J2 is drawn with an output
  bubble but labelled 7417, and the same package's 5 -> 6 gate (music) is drawn without one: read as non-inverting
  (sound starts on D5 rising, as MAME starts the sample). The MAME sample (a recording) runs 323 -> 256 Hz with the
  same 0.79 pitch drop and ~0.5 s length as this circuit, which runs 785 -> 612 Hz with C403 = 3300 pF as drawn;
  clowns(c403=8.2e-9) gives 316 -> 283 Hz.
Music (Midway tone generator + its TVCA) is kept as MAME (HW-confirmed); the Norton output terms would raise it ~0.8 dB.
"""
import math

from dsnd import Board, LATCH2, LATCH3, LATCH4, FN_TRG0, FN_TRG1, FN_TRG2, q, ONE, FS, rc_exp, par, VBE

K, M_, U = 1e3, 1e6, 1e-6
OUT_SCALE = 11000 * 0.25 / 32768          # MAME: DISCRETE_OUTPUT gain x the 0.25 route, 16-bit full scale
MIDWAY_MUSIC_TVCA = dict(r1=3.3 * M_, r2=10 * K + 680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.001 * U,
                         v1=12, vP=12, f0=FN_TRG0, f2=FN_TRG1)

# ---- springboard miss (schematic "SPRINGBOARD MISS SOUND"), values and derived constants
VOH, VOL = 11.5, 0.09         # LM3900 output swing: V+ - VBE (MAME / the dsnd Norton nodes), datasheet VOL 0.09 V
C401 = 10 * U + 0.1 * U       # C401 10 uF tant + the 0.1 uF into K4 pin 11 (held at VBE): both load the node
# D5 low: J2 (7417, open collector) sinks, D400 off: C401 relaxes to the bias network 120K (+12 V) / 100K / R401 100K
# and R407 820K into Norton inputs (0.5 V)
G_LO = 1 / (120 * K) + 1 / (100 * K) + 1 / (100 * K) + 1 / (820 * K)
V_LO = (12 / (120 * K) + 0.5 / (100 * K) + 0.5 / (820 * K)) / G_LO           # 3.574 V
TAU_LO = C401 / G_LO                                                          # 0.342 s
# D5 high: J2 off, R400 1K + D400 charge C401 (ngspice: settles at 11.26 V, D400 ~0.51 V at ~0.23 mA)
V_HI = 11.26
TAU_HI = C401 * par(1 * K, 1 / G_LO)                                          # 9.8 ms
I_BN = (12 - 0.5) / (120 * K)                 # K4b pin 11 bias (R406 120K from +12 V DEC)
C_BN = 0.1 * U                                # C401 node -> K4b pin 11


def clowns_miss(b, c403=3300e-12):
    """K4a / K4b: C402 10 uF integrator (300K leak) whose - input K4b's output drives through 4.7K; K4b is a
    comparator with R403 100K positive feedback (+ input: R404 100K from K4a) against R406 120K bias + the C401
    0.1 uF coupling. B high pins A at VOL (sound off, latched); D5 rising (J2 releases) kicks C401 up, the 0.1 uF
    current resets B low; A then ramps (B's VOL draws ~87 uA out of A's - node through 4.7K, plus C401 through R401)
    until A reaches ~10.5 V, B flips and slams A down: one ~0.65 s burst per D5 rising edge.
    K4c / K4d: Norton VCO (R407 820K from C401, C403, R412 470K, R410 47K, R411 240K, R413 1M) = MAME VCO_3 Norton.
    K3a: Norton VCA, out = A - tri - (680K x 11.5 V / 2.7M) + 0.5 V (R / 680K on all three inputs), clamped.
    Output .022 uF -> 10K mixer input (the mixer's input cap)."""
    b.kreg("ms_kh", rc_exp(TAU_HI), "T")
    b.kreg("ms_kl", rc_exp(TAU_LO), "T")
    b.op("BIT", 0, b.src(LATCH4, 5))
    b.op("LD", "ms_kl"); b.op("LDF", "ms_kh"); b.op("ST", "ms_k")
    b.op("LDI", imm=q(V_LO)); b.op("LDIF", imm=q(V_HI))
    b.op("RCM", "ms_n", b.r("ms_k"))
    b.op("SUB", "ms_np"); b.op("ST", "ms_dn")         # dn = this sample's step
    b.op("LD", "ms_n"); b.op("ST", "ms_np")
    # K4b: + input (A - VBE + B - VBE) / 100K against (I_BN + C_BN dn / dt) -> B = VOH / VOL (in volts x 100K)
    kc = C_BN * FS * 100 * K                          # 480: split into two MULIs (Q24 range)
    b.op("LD", "ms_dn"); b.op("MULI", imm=q(-16.0)); b.op("MULI", imm=q(kc / 16.0))
    b.op("ADD", "ms_a"); b.op("ADD", "ms_b")
    b.op("ADDI", imm=q(-2 * 0.5 - I_BN * 100 * K))
    b.op("CMPI", imm=0)
    b.op("LDI", imm=q(VOL)); b.op("LDIF", imm=q(VOH)); b.op("ST", "ms_b")
    # K4a: C402 dA/dt = (n - VBE) / R401 - (A - VBE) / 300K - (B - VBE) / 4.7K, clamped to the output swing.
    # Worked in volts x 100K / 4 (Q24 range); the rate register carries the 4
    b.kreg("ms_ka", 4.0 / (FS * 10 * U * 100 * K), "T", clamp=False)
    b.op("LD", "ms_n"); b.op("MULI", imm=q(0.25))
    b.op("MACI", "ms_a", imm=q(-100 / 300 / 4))
    b.op("MACI", "ms_b", imm=q(-100 / 4.7 / 4))
    b.op("ADDI", imm=q((-0.5 + 0.5 * 100 / 300 + 0.5 * 100 / 4.7) / 4))
    b.op("MUL", "ms_ka"); b.op("ADD", "ms_a")
    b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", "ms_a")
    # K4c / K4d VCO (triangle = ms_o_v)
    b.op_amp_vco3_norton_exact("ms_o", "ms_n", 820 * K, 470 * K, 47 * K, 240 * K, 1 * M_, c403, 12)
    # K3a VCA
    b.op("LD", "ms_a"); b.op("SUB", "ms_o_v")
    b.op("ADDI", imm=q(-0.5 + 1.0 - 680 * K * 11.5 / (2.7 * M_)))
    b.op("MAXI", imm=0); b.op("MINI", imm=q(VOH)); b.op("ST", "miss")
    b.init("ms_n", V_LO); b.init("ms_np", V_LO); b.init("ms_a", VOL); b.init("ms_b", VOH)


V_TRIG = 4.64                 # 74174 Q high + J3 1K pull-up (+5 V) less the 1N4148 drop at the ~6 uA steady load
                              # (ngspice: C201 settles at 4.64 V; MAME: 5 - 0.6 V into a divider, then + VBE)


def tvca_norton(b, out, r1, r2, r4, inp, caps, vP=12.0):
    """Triggered Norton VCA as drawn (K3): out = VBE + r4 (I+ - (vP - VBE) / r1 - (inp - VBE) / r2), clamped to
    VOL .. VOH. I+ = sum of (cap - VBE) / r over the trigger caps; each cap charges from V_TRIG through J3 1K while its
    latch bit is high and discharges through r into the + input (down to VBE) while low. MAME's DST_TVCA_OP_AMP drops
    the output's VBE, clamps a below-VBE input current to 0, charges to 5 - 0.6 V into a divider, and runs c2 / c3 down
    to 0 V with v / r (no VBE): its outputs come out ~1 V low, so the envelopes end early. caps = [(bit, r, c)]"""
    for j, (bit, r, c) in enumerate(caps):
        vt = (V_TRIG / (1 * K) + VBE / r) / (1 / (1 * K) + 1 / r)
        b.kreg(f"{out}_kc{j}", rc_exp(par(1 * K, r) * c), "T")
        b.kreg(f"{out}_kd{j}", rc_exp(r * c), "T")
        b.op("BIT", 0, b.src(LATCH4, bit))
        b.op("LD", f"{out}_kd{j}"); b.op("LDF", f"{out}_kc{j}"); b.op("ST", out + "_k")
        b.op("LDI", imm=q(VBE)); b.op("LDIF", imm=q(vt))
        b.op("RCM", f"{out}_c{j}", b.r(out + "_k"))
        b.op("ADDI", imm=q(-VBE)); b.op("MAXI", imm=0); b.op("MULI", imm=q(r4 / r))
        if j:
            b.op("ADD", out + "_p")
        if j < len(caps) - 1:
            b.op("ST", out + "_p")
        b.init(f"{out}_c{j}", VBE)
    b.op("MACI", inp, imm=q(-r4 / r2))
    b.op("ADDI", imm=q(VBE - r4 * (vP - VBE) / r1 + r4 * VBE / r2))
    b.op("MAXI", imm=q(VOL)); b.op("MINI", imm=q(VOH)); b.op("ST", out)


def biquad_lp(b, out, src, g, a1, a2):
    """H(s) = g / (1 + a1 s + a2 s^2), bilinear transform pre-warped at the resonance (direct form I, MACI)"""
    w0 = 1 / math.sqrt(a2)
    k = w0 / math.tan(w0 / (2 * FS))
    d0 = a2 * k * k + a1 * k + 1
    d1 = (2 - 2 * a2 * k * k) / d0
    d2 = (a2 * k * k - a1 * k + 1) / d0
    b0 = g / d0
    b.load(src)
    b.op("ST", out + "_x0")
    b.op("MULI", imm=q(b0))
    for reg, kk in (("_x1", 2 * b0), ("_x2", b0), ("_y1", -d1), ("_y2", -d2)):
        b.op("MACI", out + reg, imm=q(kk))
    b.op("ST", out + "_s")
    b.op("LD", out + "_x1"); b.op("ST", out + "_x2")
    b.op("LD", out + "_x0"); b.op("ST", out + "_x1")
    b.op("LD", out + "_y1"); b.op("ST", out + "_y2")
    b.op("LD", out + "_s"); b.op("ST", out + "_y1")
    b.op("ST", out)


# pop filter network (R307 15K, C303 .01, .01 coupling, 39K || 10K mixer input)
R307, C303, C_POP, RB = 15 * K, 0.01 * U, 0.01 * U, par(39 * K, 10 * K)
_a = C_POP * RB + R307 * (C303 + C_POP)
_b = R307 * RB * C303 * C_POP
POP_T1 = (_a + math.sqrt(_a * _a - 4 * _b)) / 2      # 345 us (high-pass side, 461 Hz)
POP_T2 = (_a - math.sqrt(_a * _a - 4 * _b)) / 2      # 34.6 us (low pass, 4.6 kHz)
POP_K = C_POP * RB / POP_T1                          # mid-band gain 0.231
# springboard hit filter (R210 100K, R213 300K, C203 / C204 .0082 uF, R214 27K, R216 30K)
SB_G = 30 * K / (100 * K)
SB_A1 = 27 * K * 0.0082 * U * 30 * K / (300 * K)
SB_A2 = 27 * K * 30 * K * 0.0082 * U * 0.0082 * U


def clowns(mame=False, c403=3300e-12, c300=None, c302=None, pop_net=None, sb_filter=None, miss=None, tvca_hw=None,
           probe=None, out_gain=1.0):
    """mame=True: the MAME-derived program (dsnd_boards.clowns(), bit-exact); otherwise the schematic. The keyword
    overrides pick parts individually for A/B checks"""
    c300 = c300 if c300 is not None else (0.015 * U if mame else 0.15 * U)
    c302 = c302 if c302 is not None else (0.082 * U if mame else 0.1 * U)
    pop_net = pop_net if pop_net is not None else (not mame)
    sb_filter = sb_filter if sb_filter is not None else (not mame)
    miss = miss if miss is not None else (not mame)
    tvca_hw = tvca_hw if tvca_hw is not None else (not mame)
    b = Board("clowns")
    # tone generator (Midway music): enable = tone low D0
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    # balloon pops: noise through a three-trigger VCA (top / middle / bottom = C300 / C301 / C302)
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    if tvca_hw:     # K3 8/13/9: R304 2.7M, R303 680K (noise), R305 680K; R300 / C300, R301 / C301, R302 / C302
        tvca_norton(b, "pop_v", 2.7 * M_, 680 * K, 680 * K, "noise",
                    [(2, 470 * K, c300), (1, 510 * K, 0.1 * U), (0, 680 * K, c302)])
    else:
        b.tvca("pop_v", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=470 * K, r8=1 * K, r9=510 * K,
                             r10=1 * K, r11=680 * K, c1=c300, c2=0.1 * U, c3=c302, v1=5, v2=5, v3=5, vP=12,
                             f2=FN_TRG0, f4=FN_TRG1, f5=FN_TRG2),
               trig=(("bit", LATCH4, 2), ("bit", LATCH4, 1), ("bit", LATCH4, 0)), inp=("noise", 0.0))
    if pop_net:
        # the loaded R307 / C303 / C / 39K || 10K network as its two real poles; mixer input = V_B through 10K
        b.crfilter("pop_h", "pop_v", POP_T1, 1.0)
        b.rcfilter("pop_f", "pop_h", POP_T2, 1.0)
        b.gain("pop", "pop_f", POP_K)
        pop_r = 10 * K
    else:
        b.rcfilter("pop_f", "pop_v", 15 * K, 0.01 * U)
        b.crfilter("pop_h", "pop_f", 15 * K + 39 * K, 0.01 * U)
        b.gain("pop", "pop_h", 39 / (15 + 39))
        pop_r = 10 * K + 1 / (1 / (15 * K) + 1 / (39 * K))
    # springboard hit: Norton oscillator gated by a VCA, then the M3 two-integrator filter
    b.op_amp_osc_norton1("sb_osc", 820 * K, 33 * K, 150 * K, 240 * K, 1 * M_, 0.01 * U, 12)
    if tvca_hw:     # K3 3/2/4: R207 2.7M, R205 680K (oscillator), R208 680K; R206 680K / C201 1 uF
        tvca_norton(b, "sb_v", 2.7 * M_, 680 * K, 680 * K, "sb_osc", [(4, 680 * K, 1 * U)])
    else:
        b.tvca("sb_v", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=680 * K, c1=1 * U, v1=5, vP=12,
                            f2=FN_TRG0),
               trig=(("bit", LATCH4, 4), 0, 0), inp=("sb_osc", 0.0))
    if sb_filter:
        biquad_lp(b, "sb", "sb_v", SB_G, SB_A1, SB_A2)
    else:
        b.filter2("sb_f", "sb_v", 500, 1.0 / 0.8, "lp")
        b.gain("sb", "sb_f", 0.5)
    # springboard miss
    if miss:
        clowns_miss(b, c403)
    # mixer R422 / R421 / R420 10K, R419 1K + R507 (music volume, MAME adjuster 40 % on the log taper = 138k) into
    # C602 -> K1 (R606 100K)
    # C602 1 uF sits between the summing bus and K1's virtual ground: every input is high-passed through C602 and
    # the bus source resistance (R422 10K || R420 + 39K || R419 + R507 = 7.8K: 20 Hz). MAME: rF x cAmp = 0.1 s.
    # mixer_op_amp's c_amp runs through 100K: pass the equivalent capacitance
    c_amp = 1 * U if mame else par(10 * K, 10 * K + 39 * K, 1 * K + 138 * K) * 1 * U / (100 * K)
    b.mixer_op_amp("mix", [("sb", 10 * K, 0), ("miss" if miss else None, 10 * K, 0.022 * U), ("pop", pop_r, 0),
                           ("tone", 1 * K + 138 * K, 0, True)], rf=100 * K, c_amp=c_amp)
    if probe:                                         # test build: one internal node to the output (volts / 8)
        b.op("LD", probe)
        b.op("MULI", imm=q(1 / 8))
        b.op("OUT")
        return b.finish()
    # D3 low mutes the board (D600 / D601 into both K1 inputs)
    b.op("BIT", 0, b.src(LATCH4, 3))
    b.op("LD", "mix")
    b.op("NOT")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(OUT_SCALE * out_gain * (1 << 24))))
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    p = clowns()
    out = sys.argv[1] if len(sys.argv) > 1 else "clowns.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"clowns: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
