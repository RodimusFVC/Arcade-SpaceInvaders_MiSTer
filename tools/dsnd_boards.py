"""Sound board programs for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Circuit descriptions follow MAME src/mame/midw8080/mw8080bw_a.cpp (Derrick Renaud and others) where MAME models the
board, and the board schematics where it does not. program(machine) -> MRA index 5 bytes, or None.
Engine sources: LATCH1-4 = the board's sound latches 1-4 (FAMILY4 S1-S4 in tools/gen_mra.py), MISC bit 0 = VBLANK.
"""
from dsnd import (Board, LATCH1, LATCH2, LATCH3, LATCH4, FN_NONE, FN_TRG0, FN_TRG1, FN_TRG2, q, ONE, FS)

K, M_, U = 1e3, 1e6, 1e-6
OUT_SCALE = 11000 * 0.25 / 32768          # MAME: DISCRETE_OUTPUT gain x the 0.25 route, 16-bit full scale

MIDWAY_MUSIC_TVCA = dict(r1=3.3 * M_, r2=10 * K + 680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.001 * U,
                         v1=12, vP=12, f0=FN_TRG0, f2=FN_TRG1)


def clowns():
    """Clowns: S1 = port 3 (coin counter, controller select), S2 / S3 = tone generator low / high, S4 = port 7
    (D0-D2 balloon pops bottom / middle / top, D3 sound enable, D4 springboard hit, D5 springboard miss)"""
    b = Board("clowns")
    # tone generator (Midway music): enable = tone low D0
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    # balloon pops: noise through a three-trigger VCA (top / middle / bottom), then RC filters
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    b.tvca("pop_v", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=470 * K, r8=1 * K, r9=510 * K, r10=1 * K,
                         r11=680 * K, c1=0.015 * U, c2=0.1 * U, c3=0.082 * U, v1=5, v2=5, v3=5, vP=12,
                         f2=FN_TRG0, f4=FN_TRG1, f5=FN_TRG2),
           trig=(("bit", LATCH4, 2), ("bit", LATCH4, 1), ("bit", LATCH4, 0)), inp=("noise", 0.0))
    b.rcfilter("pop_f", "pop_v", 15 * K, 0.01 * U)
    b.crfilter("pop_h", "pop_f", 15 * K + 39 * K, 0.01 * U)
    b.gain("pop", "pop_h", 39 / (15 + 39))
    # springboard hit: Norton oscillator gated by a VCA, low-passed (MAME: SPICE-derived 500 Hz, gain 0.5)
    b.op_amp_osc_norton1("sb_osc", 820 * K, 33 * K, 150 * K, 240 * K, 1 * M_, 0.01 * U, 12)
    b.tvca("sb_v", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=680 * K, c1=1 * U, v1=5, vP=12,
                        f2=FN_TRG0),
           trig=(("bit", LATCH4, 4), 0, 0), inp=("sb_osc", 0.0))
    b.filter2("sb_f", "sb_v", 500, 1.0 / 0.8, "lp")
    b.gain("sb", "sb_f", 0.5)
    # springboard miss: not modelled by MAME (a sample there); no schematic yet
    # mixer: R507 music volume at its default (MAME adjuster 40 % on the log taper = 138k)
    b.mixer_op_amp("mix", [("sb", 10 * K, 0), (None, 10 * K, 0.022 * U), ("pop", 10 * K + 1 / (1 / (15 * K) + 1 / (39 * K)), 0),
                           ("tone", 1 * K + 138 * K, 0, True)], rf=100 * K, c_amp=1 * U)
    # D3 low mutes the board (MAME system_mute)
    b.op("BIT", 0, b.src(LATCH4, 3))
    b.op("LD", "mix")
    b.op("NOT")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(OUT_SCALE * (1 << 24))))
    b.op("OUT")
    return b.finish()


def spacwalk():
    """Space Walk: S1 = port 3 (D0 coin, D1 controller select, D2 sound enable, D3 space ship), S2 / S3 = tone
    generator, S4 = port 7 (D0-D2 target hit bottom / middle / top, D3 / D4 springboard hit 1 / 2, D5 springboard
    miss). Resistor mixer, R507 music volume at its 40 % default (1M..7k log), output 11000, MAME route 1.0."""
    b = Board("spacwalk")
    b.midway_tone("tone_sq", LATCH2, LATCH3, toggles=1)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    # target hit: noise through a three-trigger VCA (top / middle / bottom), two RC filters
    b.tvca("hit_v", dict(r1=1 * M_, r2=680 * K, r4=3680 * K, r5=1 * K, r7=270 * K, r8=1 * K, r9=300 * K, r10=1 * K,
                         r11=330 * K, c1=2.2 * U, c2=2.2 * U, c3=2.2 * U, v1=5, v2=5, v3=5, vP=12,
                         f2=FN_TRG0, f4=FN_TRG1, f5=FN_TRG2),
           trig=(("bit", LATCH4, 2), ("bit", LATCH4, 1), ("bit", LATCH4, 0)), inp=("noise", 0.0))
    b.rcfilter("hit_f", "hit_v", 20 * K, 0.0047 * U)
    b.rcfilter("hit", "hit_f", 40 * K, 0.0047 * U)
    # springboard hit 1 / 2: filtered noise modulates a Norton VCO, VCA, low pass (MAME: "wrong values", x 0.5 —
    # folded into the mixer weight). Both circuits' noise filters + VCO have identical inputs and parts in MAME, so
    # they are one shared copy here (identical output, half the instructions)
    b.rcfilter("sb_n1", "noise", 330 * K, 0.1 * U)
    b.rcfilter("sb_n2", "sb_n1", 480 * K, 0.1 * U)
    b.op_amp_vco3_norton("sb_o", "sb_n2", 510 * K, 82 * K, 150 * K, 240 * K, 1 * M_, 0.0022 * U, 12, r7=820 * K)
    for n in (1, 2):
        p = f"sb{n}"
        b.tvca(p + "_v", dict(r1=1 * M_, r2=220 * K, r4=300 * K, r5=1 * K, r7=120 * K, c1=3.3 * U, v1=5, vP=12,
                              f2=FN_TRG0), trig=(("bit", LATCH4, 2 + n), 0, 0), inp=("sb_o", 0.0))
        b.filter2(p, p + "_v", 2000.0 - n * 500, 1.0 / 0.8, "lp")
    # springboard miss: RCDISC2 envelope -> integrator + VCO -> CR -> Norton band pass -> Norton amp
    b.rcdisc2("miss_e", (LATCH4, 5), 0.5, 1 / (1 / (200 * K) + 1 / (820 * K)), 11.5, 1 * K, 0.68 * U)
    b.integrate_norton1("miss_i", "miss_e", 0, 1 * M_, 200 * K, 0.68 * U, 12, 12)
    b.op_amp_vco3_norton("miss_o", "miss_e", 820 * K, 330 * K, 47 * K, 300 * K, 1 * M_, 0.0022 * U, 12, sqw=True)
    b.crfilter("miss_c", "miss_o", 10 * K, 0.001 * U)
    b.op_amp_filt_bp1m_norton("miss_b", "miss_c", 10 * K, 1 * M_, 4.7 * M_, 0.001 * U, 0.001 * U, 12)
    b.op_amp_norton("miss", "miss_b", "miss_i", 100 * K, 220 * K, 0, 51 * K, 12)
    # space ship: Norton LFO (cap voltage) modulates a Norton VCO_1 (enable = D3), two RC filters
    b.op_amp_osc2_norton_cap("ship_l", 75 * K, 1 * M_, 6.8 * M_, 2.4 * M_, 2.2 * U, 12)
    b.op_amp_vco1_norton_cap("ship_o", "ship_l", (LATCH1, 3), 680 * K, 300 * K, 100 * K, 150 * K, 120 * K,
                             0.0012 * U, 12)
    b.rcfilter("ship_f", "ship_o", 1 * K, 0.15 * U)
    b.rcfilter("ship", "ship_f", 11 * K, 0.015 * U)
    b.mixer_resistor("mix", [("sb1", 75 * K, 0, False, 0.5), ("sb2", 75 * K, 0, False, 0.5), ("ship", 50 * K, 0), ("miss", 11 * K, 0),
                             ("hit", 20 * K, 0), ("tone", 2.7 * K + logadj(1e6, 7000, 40), 0, True)], c_amp=1 * U)
    b.op("BIT", 0, b.src(LATCH1, 2))                  # D2 low mutes the board (MAME system_mute)
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(11000 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def logadj(rmin, rmax, pct):
    """DISCRETE_ADJUSTMENT with DISC_LOGADJ at a PORT_ADJUSTER default (percent)"""
    import math
    return 10 ** (math.log10(rmin) + pct / 100 * (math.log10(rmax) - math.log10(rmin)))


def gated_out(b, src_l, src_r, scale, en):
    """stereo board folded to mono (MAME routes both channels at 1.0), gated by a game-on / enable bit"""
    b.op("LD", src_l)
    b.op("ADD", src_r)
    b.op("MULI", imm=int(round(0.5 * scale * (1 << 24))))
    b.op("ST", "_o")
    b.op("BIT", 0, b.src(*en))
    b.op("NOT")
    b.op("LD", "_o")
    b.op("LDIF", imm=0)
    b.op("OUT")


def shot_vca(b, name, en, noise, info, r_a, c_a, r_b, c_b):
    b.tvca(name + "_v", info, trig=(en, 0, 0), inp=(noise, 0.0))
    b.rcfilter(name + "_f", name + "_v", r_a, c_a)
    b.rcfilter(name, name + "_f", r_b, c_b)


SHOT_TVCA = dict(r1=2.7 * M_, r2=510 * K, r4=510 * K, r5=10 * K, r7=510 * K, c1=0.22 * U, v1=12, vP=12, f2=FN_TRG0)


def dogpatch():
    """Dog Patch: S1 = port 3 (D2 coin, D3 game on / sound enable, D4 left shot, D5 right shot, D6 hit),
    S2 / S3 = tone generator. Stereo in MAME (music right only); folded to mono. Hit sounds: MAME constant 0."""
    b = Board("dogpatch")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    shot_vca(b, "lshot", ("bit", LATCH1, 4), "noise", SHOT_TVCA, 12 * K, 0.01 * U, 80 * K, 0.0022 * U)
    shot_vca(b, "rshot", ("bit", LATCH1, 5), "noise", SHOT_TVCA, 12 * K, 0.01 * U, 80 * K, 0.0033 * U)
    b.mixer_op_amp("l", [("lshot", 113 * K, 0)], rf=100 * K, c_amp=0.1 * U)
    b.mixer_op_amp("r", [("rshot", 113 * K, 0), ("tone", 543 * K, 0, True)], rf=100 * K, c_amp=0.1 * U)
    gated_out(b, "l", "r", 32760.0 / 5.8 / 32768, (LATCH1, 3))
    return b.finish()


def boothill():
    """Boot Hill: S1 = port 3 (D2 coin, D3 game on, D4 / D5 left / right shot, D6 / D7 left / right hit),
    S2 / S3 = tone generator. Stereo in MAME, folded to mono. Music volume at the 35 % default (404k)."""
    b = Board("boothill")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", dict(MIDWAY_MUSIC_TVCA, r2=100 * K + 680 * K), trig=("tone_sq", ("bit", LATCH2, 0), 0),
           inp=(12.0, 0.0))
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    shot_vca(b, "lshot", ("bit", LATCH1, 4), "noise", SHOT_TVCA, 12 * K, 0.01 * U, 80 * K, 0.0022 * U)
    shot_vca(b, "rshot", ("bit", LATCH1, 5), "noise", SHOT_TVCA, 12 * K, 0.01 * U, 80 * K, 0.0033 * U)
    hit = dict(SHOT_TVCA, c1=0)                       # MAME boothill_hit_tvca_info: c1 0, the 1 uF sits in c2 (r9 0)
    shot_vca(b, "lhit", ("bit", LATCH1, 6), "noise", hit, 12 * K, 0.033 * U, 112 * K, 0.0033 * U)
    shot_vca(b, "rhit", ("bit", LATCH1, 7), "noise", hit, 12 * K, 0.0033 * U, 112 * K, 0.0022 * U)
    b.mixer_op_amp("l", [("lshot", 113 * K, 0), ("lhit", 145 * K, 0)], rf=100 * K, c_amp=0.1 * U)
    b.mixer_op_amp("r", [("rshot", 113 * K, 0), ("rhit", 145 * K, 0), ("tone", 33 * K + logadj(1e6, 75e3, 35), 0, True)],
                   rf=100 * K, c_amp=0.1 * U)
    gated_out(b, "l", "r", 7200.0 / 32768, (LATCH1, 3))
    return b.finish()


def tornbase():
    """Tornado Baseball: S1 (A0 AND-gate ports): D0 / D1 / D2 gate 240 / 960 / 120 Hz squares through 7403 NANDs
    wired together; 47k / 0.047 uF coupling. D3 siren / D4 cheer are not modelled (not in MAME either)."""
    b = Board("tornbase")
    b.op("LDI", imm=1 << 24)
    b.op("ST", "o")
    for k, f in enumerate((240, 960, 120)):
        b.squarewfix(f"sq{k}", f)
        b.op("LD", f"sq{k}")
        b.op("CMPI", imm=1 << 23)
        b.op("FAND", 0, b.src(LATCH1, k))
        b.op("LDI", imm=0)
        b.op("STF", "o")
    b.crfilter("snd", "o", 47 * K, 0.047 * U)
    b.output("snd", 32767 / 32768)
    return b.finish()


def desertgu():
    """Desert Gun / Road Runner: S1 = port 3 (D2 coin, D3 game on, D4 rifle shot, D5 bottle hit, D6 Road Runner hit,
    D7 creature hit), S2 / S3 = tone generator, S4 = port 7 (D0 beep-beep, D1 trigger click, D2 recoil, D3 controller
    select). Bottle / Road Runner / creature hits and beep-beep are 0 in MAME (incomplete there too)."""
    b = Board("desertgu")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.lfsr_noise("noise", 7515, 12.0, 6.0)
    b.tvca("shot_v", SHOT_TVCA, trig=(("bit", LATCH1, 4), 0, 0), inp=("noise", 0.0))
    b.rcfilter("shot_f", "shot_v", 12 * K, 0.01 * U)
    b.crfilter("shot", "shot_f", 80 * K, 0.0022 * U)
    # trigger click: 12 V through the 2k / 27k / 3k resistor mixer (other inputs 0), then the band-pass
    rmix = 1 / (1 / (2 * K) + 1 / (27 * K) + 1 / (3 * K))
    b.op("BIT", 0, b.src(LATCH4, 1))
    b.op("LDI", imm=0)
    b.op("LDIF", imm=int(round(12.0 / (3 * K) * rmix * (1 << 24))))
    b.op("ST", "click_in")
    b.op_amp_filt_bp1("click", "click_in", rmix, 39 * K, 0.033 * U, 0.033 * U, r3=68)
    music = 30 * K + logadj(1e6, 75e3, 60)
    b.mixer_op_amp("mix", [("shot", 110 * K, 0.1 * U), (None, 56 * K, 0.1 * U), (None, 180 * K, 0.1 * U),
                           ("click", 47 * K, 0.1 * U), ("tone", music, 0.1 * U, True)], rf=100 * K, c_amp=0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 3))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(6000 * 0.8 / 32768 * (1 << 24))))    # mixer gain 6000, MAME route 0.8
    b.op("OUT")
    return b.finish()


def bowler():
    """Bowling Alley: S1 = port 5 (D1 coin, D2 sound enable, D3 foul). MAME models only the foul: 180 Hz TTL square
    through an op-amp VCA, 68k / 0.1 uF coupling. Rolling / pin / strike / spare sounds are not modelled (MAME too)."""
    b = Board("bowler")
    b.squarewfix("sq", 180)
    b.op("LD", "sq")
    b.op("CMPI", imm=1 << 23)
    b.op("LDI", imm=0)
    b.op("LDIF", imm=int(round(3.4 * (1 << 24))))    # DEFAULT_TTL_V_LOGIC_1
    b.op("ST", "sqv")
    b.tvca("fowl_v", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=300 * K, c1=0.1 * U, v1=5, vP=12,
                          f2=FN_TRG0), trig=(("bit", LATCH1, 3), 0, 0), inp=("sqv", 0.0))
    b.crfilter("fowl", "fowl_v", 68 * K, 0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 2))
    b.op("NOT")
    b.op("LD", "fowl")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(10000 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def shuffle():
    """Shuffleboard: S1 = port 5 (D0 click, D1 rollover, D2 sound enable, D3 / D4 / D5 rolling 3 / 2 / 1),
    S2 = port 6 (D0 foul, D1 coin). Noise clock 1210 Hz."""
    b = Board("shuffle")
    b.lfsr_noise("noise", 1210, 12.0, 6.0)
    # rolling: three-trigger VCA with an output cap (C505), Norton amp mixing the noise, 800 Hz low pass, x 0.2
    b.tvca("roll_v", dict(r1=5.6 * M_, r4=2 * M_, r5=10 * K, r7=510 * K, r8=10 * K, r9=1 * M_, r10=10 * K,
                          r11=1.5 * M_, c1=1 * U, c2=1 * U, c3=1 * U, c4=0.33 * U, v1=12, v2=12, v3=12, vP=12,
                          f2=FN_TRG0, f4=FN_TRG1, f5=FN_TRG2),
           trig=(("bit", LATCH1, 5), ("bit", LATCH1, 4), ("bit", LATCH1, 3)), inp=(0.0, 0.0))
    b.op_amp_norton("roll_a", "noise", "roll_v", 680 * K, 680 * K, 2.7 * M_, 680 * K, 12)
    b.filter1_lp("roll_f", "roll_a", 800)
    b.gain("roll", "roll_f", 0.2)
    # foul: 120 Hz TTL square through a VCA
    b.squarewfix("sq", 120)
    b.op("LD", "sq")
    b.op("CMPI", imm=1 << 23)
    b.op("LDI", imm=0)
    b.op("LDIF", imm=int(round(3.4 * (1 << 24))))
    b.op("ST", "sqv")
    b.tvca("foul", dict(r1=2.7 * M_, r2=680 * K, r4=680 * K, r5=1 * K, r7=300 * K, c1=0.1 * U, v1=5, vP=12,
                        f2=FN_TRG0), trig=(("bit", LATCH2, 0), 0, 0), inp=("sqv", 0.0))
    # rollover: noise VCA, two RC filters
    b.tvca("ro_v", dict(r1=1 * M_, r2=680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.1 * U, v1=12, vP=12,
                        f2=FN_TRG0), trig=(("bit", LATCH1, 1), 0, 0), inp=("noise", 0.0))
    b.rcfilter("ro_f", "ro_v", 5.6 * K, 1 * U)
    b.rcfilter("ro", "ro_f", 11.2 * K, 1 * U)
    # click: 11.5 V logic, 300 Hz low pass, x 0.3
    b.op("BIT", 0, b.src(LATCH1, 0))
    b.op("LDI", imm=0)
    b.op("LDIF", imm=int(round(11.5 * (1 << 24))))
    b.op("ST", "click_in")
    b.filter1_lp("click_f", "click_in", 300)
    b.gain("click", "click_f", 0.3)
    b.mixer_resistor("mix", [("roll", 300 * K, 0.1 * U), ("foul", 200 * K, 0.1 * U), ("ro", 14.2 * K, 1 * U),
                             ("click", 33 * K, 0.1 * U)])
    b.op("BIT", 0, b.src(LATCH1, 2))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(59200 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def dplay():
    """Double Play / Extra Inning: S1 = port 3 (D0 tone on, D1 cheer, D2 siren, D3 whistle, D4 game on, D5 coin),
    S2 / S3 = tone generator. MAME route 0.8, mixer gain 2000, music pot 60 % of 1M..1k log."""
    b = Board("dplay")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    b.tvca("music", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.tvca("tone", MIDWAY_MUSIC_TVCA, trig=("tone_sq", ("bit", LATCH1, 0), 0), inp=(12.0, 0.0))
    b.integrate_norton1("siren_i", (LATCH1, 2), 5.0, 1 * M_, 100 * K, 3.3 * U, 12, 12)
    b.op_amp_vco2_norton("siren", "siren_i", 390 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 56 * K, 0.0022 * U, 12)
    b.integrate_norton1("wh_i", (LATCH1, 3), 12.0, 1 * M_, 230 * K, 3.3 * U, 12, 12)
    b.op_amp_vco2_norton("whistle", "wh_i", 510 * K, 5.6 * M_, 1 * M_, 1.5 * M_, 3.3 * M_, 300 * K, 220e-12, 12)
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    b.integrate_norton1("ch_i", (LATCH1, 1), 5.0, 1.5 * M_, 100 * K, 4.7 * U, 12, 12)
    b.op("LD", "noise")                               # DISCRETE_SWITCH: the noise bit gates the cheer envelope
    b.op("CMPI", imm=1 << 24)
    b.op("LDI", imm=0)
    b.op("LDF", "ch_i")
    b.op("ST", "ch_sw")
    b.op_amp_filt_bp1m("cheer", "ch_sw", 100 * K, 150 * K, 0.0047 * U, 0.0047 * U, r3=100 * K)
    music = 68 * K + logadj(1e6, 1000, 60)
    b.mixer_op_amp("mix", [("tone", 68 * K, 0.1 * U), ("siren", 68 * K, 0.1 * U), ("whistle", 68 * K, 0.1 * U),
                           ("cheer", 18 * K, 0.1 * U), ("music", music, 0.1 * U, True)], rf=100 * K, c_amp=0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 4))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(2000 * 0.8 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def checkmat():
    """Checkmate: S1 = ports 1 / 3 (D0 tone enable, D1 boom, D2 coin, D3 sound enable, D4-D5 / D6-D7 tone data ->
    comparator resistor networks). MAME route 0.4, output 300000. Pots R309 / R411 at their 50 % defaults."""
    b = Board("checkmat")
    # boom: uniform noise 1500 Hz -> TVCA -> 35 Hz band pass (d 1/8) x 15, clamped
    b.uniform_noise("noise", 1500, 2.0)
    b.tvca("boom_v", dict(r1=1.2 * M_, r2=1 * M_, r4=1.2 * M_, r5=1 * K, r7=1 * M_, c1=1 * U, v1=5, vP=5,
                          f2=FN_TRG0), trig=(("bit", LATCH1, 1), 0, 0), inp=("noise", 0.0))
    b.filter2("boom_f", "boom_v", 35, 1.0 / 8, "bp")
    b.gain("boom_g", "boom_f", 15)
    b.op("LD", "boom_g"); b.op("MAXI", imm=int(-6 * (1 << 24))); b.op("MINI", imm=int(5.5 * (1 << 24))); b.op("ST", "boom")
    # tone: Norton oscillator, R3 = 100k || (1.5M, 820k by D4 / D5), R4 = 330k || (1M, 510k by D6 / D7)
    i1 = 4.5 / 330e3 * 1e6                            # uA
    r3 = [1e-6 / (1 / 100e3 + (k & 1) / 1.5e6 + (k >> 1) / 820e3) for k in range(4)]          # MOhm
    g4 = [1e6 * 0 + (1 / 330e3 + (k & 1) / 1e6 + (k >> 1) / 510e3) * 1e6 for k in range(4)]   # 1 / MOhm
    b.chain("r3", 0, 0x30, 12, r3)
    b.chain("g4", 0, 0xC0, 10, g4)
    b.op("LD", "g4"); b.op("MULI", imm=-(1 << 23)); b.op("ADDI", imm=int(round(i1 * (1 << 24))))
    b.op("MUL", "r3"); b.op("ADDI", imm=1 << 23); b.op("ST", "tl")                # (i1 - VBE / r4) r3 + VBE
    b.op("LD", "g4"); b.op("MULI", imm=int(4.0 * (1 << 24))); b.op("ADDI", imm=int(round(i1 * (1 << 24))))
    b.op("MUL", "r3"); b.op("ADDI", imm=1 << 23); b.op("ST", "th")                # (i1 + (vh - VBE) / r4) r3 + VBE
    b.osc_norton1_dyn("osc", 1 * M_, 430 * K, 3300e-12, 5, "tl", "th")
    b.op("LD", "osc"); b.op("ADDI", imm=-int(2.5 * (1 << 24))); b.op("ST", "osc_c")   # CRFILTER_VREF 2.5: HP of (in - 2.5)
    b.crfilter("hp1", "osc_c", 250 * K, 0.1 * U)
    b.op("BIT", 0, b.src(LATCH1, 0))                  # DISCRETE_SWITCH: tone enable -> filtered osc, else 2.5 V
    b.op("LDI", imm=int(2.5 * (1 << 24)))
    b.op("ST", "sw")
    b.op("LD", "hp1"); b.op("ADDI", imm=int(2.5 * (1 << 24))); b.op("STF", "sw")
    b.crfilter("hp2", "sw", 303 * K, 0.01e-12)       # MAME: CAP_P(0.01) (as written there)
    b.rcfilter("tone", "hp2", 56 * K, 4700e-12)
    b.mixer_op_amp("mix", [("boom", 100 * K + logadj(100e3, 1000, 50), 10 * U),
                           ("tone", 103 * K + logadj(1e6, 1000, 50), 0.01 * U, True)], rf=100 * K, c_amp=1 * U)
    b.op("BIT", 0, b.src(LATCH1, 3))
    b.op("NOT")
    b.op("LD", "mix")
    b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(300000 * 0.4 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


def maze():
    """Amazing Maze: no sound latch. Inputs: IN0 joysticks (engine source 4, P1 D0-D3 / P2 D4-D7, active low), coin
    (misc bit 1). Tone timing = free 555 B1 (33k, 68k, 1 uF) toggling the timing FF; the player-select FF toggles on
    its falling edge; the 74147 encodes the selected player's stick into R305 / R306 / R308; R303 / R309 by player.
    Sound runs from a coin until the sticks sit idle for the 555 F2 monostable (1.1 x 270k x 100 uF)."""
    b = Board("maze")
    half = 0.693 * (33e3 + 2 * 68e3) * 1e-6         # PERIOD_OF_555_ASTABLE: the timing FF toggles each period
    b.op("LD", "tt_ph"); b.op("ADDI", imm=q(1.0 / (half * FS))); b.op("ST", "tt_ph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "tt_ph")
    b.op("LD", "tt"); b.op("CMPI", imm=q(0.5))       # falling edge of the timing FF toggles the player select
    b.op("LDI", imm=ONE); b.op("SUB", "psel"); b.op("STF", "psel")
    b.op("LDI", imm=ONE); b.op("SUB", "tt"); b.op("ST", "tt")
    b.land(at)
    # joystick in use: MAME controls = ~IN0 != FF, i.e. raw IN0 (active high) != 0
    b.op("LDL", 0, b.src(4, 0)); b.op("CMPI", imm=1 << 16)
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "use")
    # 555 F2 monostable: held discharged while a stick is used, else times out -> falling edge mutes (JK, coin clears)
    t_out = 1.1 * 270e3 * 100e-6
    b.op("LD", "use"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "mono"); b.op("ADDI", imm=q(1.0 / FS)); b.op("MINI", imm=q(t_out + 1)); b.op("LDIF", imm=0)
    b.op("ST", "mono")
    b.op("CMPI", imm=q(t_out)); b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", "go")   # 555 output high = timing
    b.op("LD", "go_d"); b.op("SUB", "go")             # 1 -> 0 edge: go_d - go = 1
    b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=ONE); b.op("STF", "mute")
    b.op("LD", "go"); b.op("ST", "go_d")
    b.op("BIT", 1, b.src(5, 1)); b.op("LDI", imm=0); b.op("STF", "mute")       # coin (line low) clears the mute FF
    # selected player's stick through the 74147 -> R3 network; R4 by the player select
    b.op("LDLM", 0, b.src(4, 0), imm=0x0F | (16 << 8)); b.op("ST", "_t")          # controls = ~IN0: 15 - nibble
    b.op("LDI", imm=15 << 16); b.op("SUB", "_t"); b.op("ST", "sel")
    b.op("LDLM", 0, b.src(4, 0), imm=0xF0 | (12 << 8)); b.op("ST", "_t")
    b.op("LD", "psel"); b.op("CMPI", imm=q(0.5))
    b.op("LDI", imm=15 << 16); b.op("SUB", "_t"); b.op("STF", "sel")
    b.op("LDI", imm=3 << 16); b.op("ST", "enc")
    for th, v in ((8, 0), (12, 1), (14, 2), (15, 3)):
        b.op("LD", "sel"); b.op("CMPI", imm=th << 16); b.op("LDI", imm=v << 16); b.op("STF", "enc")
    r3 = [1e-6 / (1 / 100e3 + (k & 1) / 1.5e6 + (k >> 1) / 820e3) for k in range(4)]
    b.op("LDI", imm=q(r3[0])); b.op("ST", "r3")
    for k in (1, 2, 3):
        b.op("LD", "enc"); b.op("CMPI", imm=k << 16); b.op("LDI", imm=q(r3[k])); b.op("STF", "r3")
    i1 = 4.5 / 330e3 * 1e6
    g4 = [(1 / 330e3) * 1e6, (1 / 330e3 + 1 / 1e6) * 1e6]
    b.op("LD", "psel"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=q(g4[0])); b.op("LDIF", imm=q(g4[1])); b.op("ST", "g4")
    b.op("LD", "g4"); b.op("MULI", imm=-(1 << 23)); b.op("ADDI", imm=int(round(i1 * (1 << 24))))
    b.op("MUL", "r3"); b.op("ADDI", imm=1 << 23); b.op("ST", "tl")
    b.op("LD", "g4"); b.op("MULI", imm=int(4.0 * (1 << 24))); b.op("ADDI", imm=int(round(i1 * (1 << 24))))
    b.op("MUL", "r3"); b.op("ADDI", imm=1 << 23); b.op("ST", "th")
    b.osc_norton1_dyn("osc", 1 * M_, 430 * K, 3300e-12, 5, "tl", "th")
    b.op("LD", "osc"); b.op("ADDI", imm=-int(2.5 * (1 << 24))); b.op("ST", "osc_c")
    b.crfilter("hp1", "osc_c", 250 * K, 0.1 * U)
    # switch: stick in use AND tone enabled (not muted) AND tone timing -> filtered osc, else 2.5 V
    b.op("LDI", imm=int(2.5 * (1 << 24))); b.op("ST", "sw")
    b.op("LD", "use"); b.op("MUL", "tt"); b.op("ST", "_u")
    b.op("LDI", imm=ONE); b.op("SUB", "mute"); b.op("MUL", "_u"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "hp1"); b.op("ADDI", imm=int(2.5 * (1 << 24))); b.op("STF", "sw")
    b.crfilter("hp2", "sw", 446 * K, 0.01e-12)       # MAME: CAP_P(0.01) as written
    b.rcfilter("snd", "hp2", 56 * K, 4700e-12)
    b.op("LD", "mute"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "snd"); b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(96200 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


BOARDS = {"clowns": clowns, "dogpatch": dogpatch, "boothill": boothill, "tornbase": tornbase, "desertgu": desertgu,
          "bowler": bowler, "shuffle": shuffle, "dplay": dplay, "checkmat": checkmat, "maze": maze,
          "spacwalk": spacwalk}


def program(machine):
    f = BOARDS.get(machine)
    return None if f is None else f().binary()


if __name__ == "__main__":
    import sys
    for m in sys.argv[1:] or BOARDS:
        p = BOARDS[m]()
        open(f"{m}.dsnd.bin", "wb").write(p.binary())
        print(f"{m}: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers")
