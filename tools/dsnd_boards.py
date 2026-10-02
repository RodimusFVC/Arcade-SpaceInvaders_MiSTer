"""Sound board programs for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Circuit descriptions follow MAME src/mame/midw8080/mw8080bw_a.cpp (Derrick Renaud and others) where MAME models the
board, and the board schematics where it does not. program(machine) -> MRA index 5 bytes, or None.
Engine sources: LATCH1-4 = the board's sound latches 1-4 (FAMILY4 S1-S4 in tools/gen_mra.py), MISC bit 0 = VBLANK.
"""
from dsnd import (Board, LATCH1, LATCH2, LATCH3, LATCH4, FN_NONE, FN_TRG0, FN_TRG1, FN_TRG2)

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


BOARDS = {"clowns": clowns, "dogpatch": dogpatch, "boothill": boothill, "tornbase": tornbase, "desertgu": desertgu,
          "bowler": bowler}


def program(machine):
    f = BOARDS.get(machine)
    return None if f is None else f().binary()


if __name__ == "__main__":
    import sys
    for m in sys.argv[1:] or BOARDS:
        p = BOARDS[m]()
        open(f"{m}.dsnd.bin", "wb").write(p.binary())
        print(f"{m}: {len(p.code)} instructions, {len(p.regs)} registers")
