"""Boot Hill (Midway 1977) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Board: PC 0612-00902A (Boot_Hill_Game_Logic_(0612-00902A).pdf). Started from dsnd_boards.boothill() (MAME
boothill_discrete, HW-confirmed 2026-10-02); three schematic corrections:
- hit envelope cap: the 1 uF tantalum sits on the trigger diode / 510K node (TVCA c1), like the shots' 0.22 uF.
  MAME's boothill_hit_tvca_info puts CAP_U(1) in the c2 slot (unused: r9 0, F4 NONE) with c1 = 0, so its hits
  gate on / off instantly.
- shot / hit filters: 12K / C1 / R2 / C2 / 33K is one loaded ladder into the summer's virtual ground (the 33K goes
  through the 1 uF tant straight to the LM3900 - input). MAME cascades two unloaded RC sections (12K, C1) and
  (12K + R2, C2); here the ladder's two real poles are computed exactly and run as two one-poles. DC gain stays
  1 / (12K + R2 + 33K), the mixer resistor.
- LM3900 TVCA output (ngspice of the schematic): Vout = VBE + r4 (I+ - I-), and the input resistor pulls current
  OUT of the - input when its source is at 0 V (noise / tone low): I2 = -VBE / r2. MAME's TVCA drops both terms
  (Vout = r4 (I+ - I-), I2 clamped >= 0): its "on" level is 1.0 V low and the release cuts off 1.0 V early. Both
  terms are constant while the source is low (and the output clamps at 0 while it is high), so they fold exactly
  into the fixed bias current: r1 -> norton_r1() below.
S1 = port 3 (D2 coin, D3 game on, D4 / D5 left / right shot, D6 / D7 left / right hit), S2 / S3 = tone generator.
Stereo board folded to mono 0.5 (L + R). Music volume (1M pot) at MAME's 35 % default (404K).
"""
import math
from dsnd import Board, LATCH1, LATCH2, LATCH3, FN_TRG0, FN_TRG1

K, M_, U = 1e3, 1e6, 1e-6
# copies of the dsnd_boards.py helpers (no import from dsnd_boards: it will import this file)
MIDWAY_MUSIC_TVCA = dict(r1=3.3 * M_, r2=10 * K + 680 * K, r4=680 * K, r5=10 * K, r7=680 * K, c1=0.001 * U,
                         v1=12, vP=12, f0=FN_TRG0, f2=FN_TRG1)
SHOT_TVCA = dict(r1=2.7 * M_, r2=510 * K, r4=510 * K, r5=10 * K, r7=510 * K, c1=0.22 * U, v1=12, vP=12, f2=FN_TRG0)


def logadj(rmin, rmax, pct):
    """DISCRETE_ADJUSTMENT with DISC_LOGADJ at a PORT_ADJUSTER default (percent)"""
    return 10 ** (math.log10(rmin) + pct / 100 * (math.log10(rmax) - math.log10(rmin)))


def gated_out(b, src_l, src_r, scale, en):
    """stereo board folded to mono, gated by the game-on bit"""
    b.op("LD", src_l)
    b.op("ADD", src_r)
    b.op("MULI", imm=int(round(0.5 * scale * (1 << 24))))
    b.op("ST", "_o")
    b.op("BIT", 0, b.src(*en))
    b.op("NOT")
    b.op("LD", "_o")
    b.op("LDIF", imm=0)
    b.op("OUT")


VBE = 0.5


def norton_r1(info, r2_low):
    """equivalent r1 for dsnd's TVCA: r4 vmax / r1' = r4 (vP - VBE) / r1 - VBE - r4 VBE / r2_low, where r2_low
    is the resistance from the - input to the 0 V source while the input is low (0 = no such path)"""
    r4, vmax = info["r4"], info["vP"] - VBE
    i = (info["vP"] - VBE) / info["r1"] - VBE / r4 - (VBE / r2_low if r2_low else 0.0)
    return dict(info, r1=vmax / i)


R1, R3 = 12 * K, 33 * K                              # TVCA output resistor, summing resistor


def ladder_poles(c1, r2, c2):
    """12K / c1 / r2 / c2 / 33K into a virtual ground: Vs / Vb = D0 (1 + a1 s + a2 s^2) -> time constants"""
    d0 = (R1 + r2 + R3) / R3
    a1 = (c2 * (R1 + r2) + R1 * c1 * (r2 + R3) / R3) / d0
    a2 = R1 * r2 * c1 * c2 / d0
    d = math.sqrt(a1 * a1 - 4 * a2)
    return (a1 + d) / 2, (a1 - d) / 2


def noise_vca(b, name, en, info, c1, r2, c2):
    b.tvca(name + "_v", info, trig=(en, 0, 0), inp=("noise", 0.0))
    ta, tb = ladder_poles(c1, r2, c2)
    b.rcfilter(name + "_f", name + "_v", ta, 1.0)     # filter group: both time constants scale with C
    b.rcfilter(name, name + "_f", tb, 1.0)


def boothill():
    b = Board("boothill")
    b.midway_tone("tone_sq", LATCH2, LATCH3)
    tone = dict(MIDWAY_MUSIC_TVCA, r2=100 * K + 680 * K)
    # tone low: the 7417 holds the 100K / 680K node at 0 V -> 680K from the - input to 0 V
    b.tvca("tone", norton_r1(tone, 680 * K), trig=("tone_sq", ("bit", LATCH2, 0), 0), inp=(12.0, 0.0))
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    shot = norton_r1(SHOT_TVCA, 510 * K)              # noise low: the 4006 output holds the 510K at 0 V
    hit = dict(shot, c1=1 * U)                        # schematic: 1 uF tant on the hit trigger node
    noise_vca(b, "lshot", ("bit", LATCH1, 4), shot, 0.01 * U, 68 * K, 0.0022 * U)
    noise_vca(b, "rshot", ("bit", LATCH1, 5), shot, 0.01 * U, 68 * K, 0.0033 * U)
    noise_vca(b, "lhit", ("bit", LATCH1, 6), hit, 0.033 * U, 100 * K, 0.0033 * U)
    noise_vca(b, "rhit", ("bit", LATCH1, 7), hit, 0.0033 * U, 100 * K, 0.0022 * U)
    b.mixer_op_amp("l", [("lshot", 113 * K, 0), ("lhit", 145 * K, 0)], rf=100 * K, c_amp=0.1 * U)
    b.mixer_op_amp("r", [("rshot", 113 * K, 0), ("rhit", 145 * K, 0), ("tone", 33 * K + logadj(1e6, 75e3, 35), 0, True)],
                   rf=100 * K, c_amp=0.1 * U)
    gated_out(b, "l", "r", 7200.0 / 32768, (LATCH1, 3))
    return b.finish()


if __name__ == "__main__":
    import sys
    p = boothill()
    out = sys.argv[1] if len(sys.argv) > 1 else "boothill.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"boothill: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
