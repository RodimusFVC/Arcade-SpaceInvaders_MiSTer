"""Space Chaser (Taito 1979) and Crash Road (Centromatic bootleg) sound for the discrete sound engine (tools/dsnd.py,
rtl/dsnd_engine.sv). Engine keys "8080bw:schaser" (schaser, schasera/b/c, schaserm) and "8080bw:crashrd".

No schematic: MAME 8080bw_a.cpp schaser_discrete + schaser_sh_port_1_w / _2_w / schaser_effect_555_cb, crashrd_port03_w
/ 05_w, the SN76477 config in 8080bw.cpp schaser() and the SN76477 model in devices/sound/sn76477.cpp (measured
formulas, per 48 kHz sample, as MAME runs it).
  LATCH1 = port 3: D0 dot enable (SX0), D1 dot pitch 8V (SX1), D2-D4 effect 555 select (SX2-SX4, diode / resistor
    array 15k / 39k / 82k + 20k, 1 uF; MAME: the schematic has SX2 / SX4 swapped), D5 explosion (SX5).
  LATCH2 = port 5: D0 music 1-bit DAC (SX6), D1 sound enable (SX7).
  Crash Road (bootleg ROM, same sound circuit, MAME crashrd_port03_w): port 3 D0 dot pitch, D2 explosion, D4 dot enable,
    D5 sound enable (MAME; the game holds it on through the explosion, as Space Chaser does SX7); no effect 555
    (D1 / D3 never written); port 5 D0 music.
SN76477: noise clock 47k, noise filter 330k / 470p, attack 4.7k / decay 2.2M on 1 uF, one-shot 220k / 0.1 uF,
feedback 33k, amplitude 200k (|| 68k during the explosion), VCO 39k / 0.1 uF swept by the SLF (120k / 1 uF), pitch
5 V (50 %), envelope one-shot, mixer VCO (B = explosion -> noise). ENABLE = !(555 low or explosion); the one-shot cap
is held at 0 V while disabled, so every enable restarts the one-shot and the attack.
Effect: the 555 is high (silent) 0.8873 R C, low (SN76477 enabled) 13.86 ms. Hardware behaviour where MAME differs:
effect 6 = 39k || 82k (MAME's table repeats 15k || 82k); effect 0 stops the 555 after the current low phase (MAME
keeps it running with the old timing, its known "missile sound continues" bug); a select change keeps the progress
through the high phase (same cap voltage) instead of restarting it.
Discrete: dot = 4V / 8V square (+-1.7 V) -> 560R / 0.1u -> 7.36k / 0.1u x VR3; SN76477 (scaled as MAME's stream
input, 0..3.5 V) -> 560R / 0.1u -> 7.36k / 0.1u -> high pass 59.36k / 1u x VR2; music bit x 3.4 V -> high pass
520k / 0.01u x VR1; sum gated by sound enable, x 33080 / 32768 (MAME's output). VR1 / VR2 / VR3 at MAME's adjuster
defaults 70 / 90 / 70 %. Filters start at their idle levels (no power-on thump).
"""
import math
from dsnd import Board, LATCH1, LATCH2, q, ONE, FS

K, U = 1e3, 1e-6
# SN76477 (sn76477.cpp constants)
SL_LO, SL_HI, DV = 0.33, 2.37, 0.35                 # SLF triangle, VCO top = SLF + 0.35 (VCO cap kept 0.35 V down)
AD_MAX, OS_MAX = 4.44, 2.5
OUT_HI, OUT_LO, CENTER = 3.51, 0.715, 2.57
K_IN = 32768.0 / 14100 / (CENTER - OUT_LO)          # SN76477 volts (from centre) -> discrete input volts
VR1 = 0.70 * 50 / (50 + 470)                         # music
VR2 = 0.90 * 50 / (50 + 0.56 + 6.8 + 2)              # explosion / effect
VR3 = 0.70 * 50 / (50 + 0.56 + 6.8 + 10)             # dot
HSYNC = 18352.94                                     # 19.968 MHz / 2 / 2 / 272
F4V = HSYNC / 2 / 4                                  # 4V square 2294 Hz (8V = half)


def effect_rc():
    """555 high-phase R C per effect select (index = port 3 D4-D2): diode array 15k (SX2) / 39k (SX3) / 82k (SX4)
    in parallel, + 20k, 1 uF. Index 6 = 39k || 82k (MAME's table has 15k || 82k there)."""
    rs = (15 * K, 39 * K, 82 * K)
    out = [0.0]
    for e in range(1, 8):
        g = sum(1.0 / r for k, r in enumerate(rs) if e >> k & 1)
        out.append((1.0 / g + 20 * K) * 1 * U)
    return out


def build(name, crash=False):
    b = Board(name)
    # bit assignments: (latch, bit)
    if crash:
        DOT_EN, DOT_SEL, EXPL, SND_EN = (LATCH1, 4), (LATCH1, 0), (LATCH1, 2), (LATCH1, 5)
    else:
        DOT_EN, DOT_SEL, EXPL, SND_EN = (LATCH1, 0), (LATCH1, 1), (LATCH1, 5), (LATCH2, 1)
    MUSIC = (LATCH2, 0)

    # ---------------------------------------------------------------- effect 555 (Space Chaser only)
    if not crash:
        t_low = math.log(2) * 2 * 20 * K * 1 * U / 2                      # MAME PERIOD_OF_555_ASTABLE(0, 20k, 1u) / 2
        rates = [0.0] + [1.0 / (FS * 0.8873 * rc) for rc in effect_rc()[1:]]
        b.chain("e_k", LATCH1, 0x1C, 14, rates)                           # high-phase step, 0 = held (effect off)
        b.op("LD", "e_k"); b.op("MUL", "_fT"); b.op("ST", "e_k")          # Game Audio: timing caps
        b.groups.add("T")
        b.kreg("e_kl", 1.0 / (FS * t_low), "T", clamp=False)
        b.op("LD", "e_low"); b.op("CMPI", imm=q(0.5))
        at = b.skip_if(False)
        b.op("LD", "e_lt"); b.op("ADD", "e_kl"); b.op("ST", "e_lt")        # low: 13.86 ms whatever the select
        b.op("CMPI", imm=ONE)
        at2 = b.skip_if(False)
        b.op("LDI", imm=0); b.op("ST", "e_low"); b.op("ST", "e_lt")
        b.land(at2)
        at3 = b.jump()
        b.land(at)
        b.op("LD", "e_ph"); b.op("ADD", "e_k"); b.op("ST", "e_ph")         # high: progress (cap voltage) held at 0
        b.op("CMPI", imm=ONE)
        at4 = b.skip_if(False)
        b.op("LDI", imm=0); b.op("ST", "e_ph"); b.op("LDI", imm=ONE); b.op("ST", "e_low")
        b.land(at4)
        b.land(at3)

    # ---------------------------------------------------------------- SN76477 enable, one-shot, attack / decay
    if crash:
        b.op("BIT", 0, b.src(*EXPL))
    else:
        b.op("LD", "e_low"); b.op("CMPI", imm=q(0.5)); b.op("FOR", 0, b.src(*EXPL))
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "en")
    b.op("SUB", "en_p"); b.op("CMPI", imm=q(0.5))                         # enabled this sample: restart
    at = b.skip_if(False)
    b.op("LDI", imm=0); b.op("ST", "ad"); b.op("ST", "os")
    b.land(at)
    b.op("LD", "en"); b.op("ST", "en_p")
    b.kreg("os_k", (OS_MAX / (0.8024 * 220 * K * 0.1 * U + 0.002079)) / FS / OS_MAX, "T", clamp=False)
    b.op("LD", "os"); b.op("ADD", "os_k"); b.op("MINI", imm=2 * ONE); b.op("ST", "os")
    b.op("LD", "en"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=0); b.op("STNF", "os")   # held at 0 V while disabled
    b.kreg("ad_a", AD_MAX / (4.7 * K * 1 * U) / FS, "T", clamp=False)
    b.kreg("ad_d", -AD_MAX / (2.2e6 * 1 * U) / FS, "T", clamp=False)
    b.op("LD", "os"); b.op("CMPI", imm=ONE)                               # one-shot over: decay
    b.op("LD", "ad_a"); b.op("LDF", "ad_d")
    b.op("ADD", "ad"); b.op("MINI", imm=q(AD_MAX)); b.op("MAXI", imm=0); b.op("ST", "ad")

    # ---------------------------------------------------------------- SN76477 noise: clock 47k, filter 330k / 470p
    # MAME's generator exactly: count (integer) += 48000 when <= the clock, then -= the clock; 31-bit shift
    # register (state 0 at power-up), out = b28 ^ b0, forced to 1 when b0-b4 and b28 are all 0 (not a plain
    # LFSR: the forcing takes 3-4 dB off the explosion's 20-200 Hz against the engine's NOISE op). Raw integers.
    nf = int(339100000 * pow(47 * K, -0.8849))                            # MAME's (uint32) noise clock, 24887 Hz
    ch = 5.0 / (0.1571 * 330 * K * 470e-12 + 0.00001430) / FS
    dis = 5.0 / (0.1331 * 330 * K * 470e-12 + 0.00001734) / FS
    b.op("LD", "n_ct"); b.op("CMPI", imm=nf + 1)
    at = b.skip_if(True)
    b.op("ADDI", imm=int(FS)); b.op("ST", "n_ct")
    b.op("LD", "n_s"); b.op("MULI", imm=1 << 23); b.op("ST", "n_h")      # h = s >> 1
    b.op("LD", "n_s"); b.op("SUB", "n_h"); b.op("SUB", "n_h"); b.op("ST", "n_b0")
    b.op("LD", "n_s"); b.op("MULI", imm=1 << 20); b.op("MULI", imm=1); b.op("ST", "n_t")   # s >> 28 (0..7)
    b.op("MULI", imm=1 << 23); b.op("ST", "n_u")
    b.op("LD", "n_t"); b.op("SUB", "n_u"); b.op("SUB", "n_u"); b.op("ST", "n_t")          # b28
    b.op("SUB", "n_b0"); b.op("ABS"); b.op("ST", "n_b")                  # b28 ^ b0
    b.op("LD", "n_s"); b.op("MULI", imm=1 << 19); b.op("MULI", imm=1 << 29); b.op("ST", "n_u")   # (s >> 5) << 5
    b.op("LD", "n_s"); b.op("SUB", "n_u"); b.op("ADD", "n_t"); b.op("CMPI", imm=1)       # b0-b4, b28 not all 0?
    b.op("LDI", imm=1); b.op("STNF", "n_b")
    b.op("LD", "n_b"); b.op("CMPI", imm=1); b.op("LDI", imm=0); b.op("LDIF", imm=1 << 30)
    b.op("ADD", "n_h"); b.op("ST", "n_s")
    b.land(at)
    b.op("LD", "n_ct"); b.op("ADDI", imm=-nf); b.op("ST", "n_ct")
    b.op("LD", "n_b"); b.op("CMPI", imm=1); b.op("LDI", imm=q(-dis)); b.op("LDIF", imm=q(ch))
    b.op("ADD", "n_c"); b.op("MINI", imm=q(5.0)); b.op("MAXI", imm=0); b.op("ST", "n_c")
    b.op("CMPI", imm=q(3.35)); b.op("LDI", imm=0); b.op("STF", "n_f")
    b.op("LD", "n_c"); b.op("CMPI", imm=q(0.74) + 1); b.op("LDI", imm=ONE); b.op("STNF", "n_f")

    # ---------------------------------------------------------------- SN76477 SLF (120k / 1u) sweeping the VCO (39k / .1u)
    rng = SL_HI - SL_LO
    st = 0.64 * 2 * (SL_HI + DV - SL_LO) / (39 * K * 0.1 * U) / FS
    b.kreg("s_up", rng / (0.5885 * 120 * K * 1 * U + 0.001300) / FS, "O", clamp=False)
    b.kreg("s_dn", -rng / (0.5413 * 120 * K * 1 * U + 0.001343) / FS, "O", clamp=False)
    b.kreg("v_st", st, "O", clamp=False)
    b.kreg("v_sn", -st, "O", clamp=False)
    b.init("s_v", SL_LO)
    b.init("v_v", SL_LO - DV)
    b.op("LD", "s_v"); b.op("ADD", "s_r"); b.op("MINI", imm=q(SL_HI)); b.op("MAXI", imm=q(SL_LO)); b.op("ST", "s_v")
    b.op("CMPI", imm=q(SL_HI)); b.op("LD", "s_dn"); b.op("STF", "s_r")
    b.op("LD", "s_v"); b.op("CMPI", imm=q(SL_LO) + 1); b.op("LD", "s_up"); b.op("STNF", "s_r")
    b.op("LD", "v_v"); b.op("ADD", "v_r"); b.op("CMP", "s_v"); b.op("LDF", "s_v")
    b.op("MAXI", imm=q(SL_LO - DV)); b.op("ST", "v_v")
    b.op("CMP", "s_v"); b.op("LD", "v_sn"); b.op("STF", "v_r")
    b.op("LD", "v_v"); b.op("CMPI", imm=q(SL_LO - DV) + 1); b.op("LD", "v_st"); b.op("STNF", "v_r")
    b.op("LD", "v_r"); b.op("CMPI", imm=0); b.op("LDI", imm=ONE); b.op("LDIF", imm=0); b.op("ST", "v_f")  # discharging = high

    # ---------------------------------------------------------------- SN76477 output: mixer bit, gain tables, clip
    cp0 = 3.818 * (33 * K / (200 * K)) + 0.03
    cp1 = 3.818 * (33 * K / (1.0 / (1 / (200 * K) + 1 / (68 * K)))) + 0.03
    b.op("BIT", 0, b.src(*EXPL)); b.op("LDI", imm=q(cp0)); b.op("LDIF", imm=q(cp1)); b.op("ST", "cp")
    b.op("LD", "v_f"); b.op("LDF", "n_f"); b.op("CMPI", imm=q(0.5))     # explosion: mixer B -> noise
    at = b.skip_if(False)
    # out_pos_gain[(int)(ad x 10)]: 0.2676 / V above 1.2 V (fit, max err 0.03), 0.6 / V from 0.98 V below
    b.op("LD", "ad"); b.op("MULI", imm=q(0.267583)); b.op("ADDI", imm=q(-0.187779)); b.op("ST", "_g")
    b.op("LD", "ad"); b.op("MULI", imm=q(0.6)); b.op("ADDI", imm=q(-0.6 * 0.98))
    b.op("CMP", "_g"); b.op("LDF", "_g"); b.op("MAXI", imm=0)
    b.op("MUL", "cp"); b.op("MINI", imm=q(OUT_HI - CENTER)); b.op("ST", "sn")
    at2 = b.jump()
    b.land(at)
    # out_neg_gain: -0.2288 / V above 1.2 V (max err 0.022), -0.5 / V from 0.98 V below
    b.op("LD", "ad"); b.op("MULI", imm=q(-0.228762)); b.op("ADDI", imm=q(0.165292)); b.op("ST", "_g")
    b.op("LD", "ad"); b.op("MULI", imm=q(-0.5)); b.op("ADDI", imm=q(0.5 * 0.98))
    b.op("CMP", "_g"); b.op("NOT"); b.op("LDF", "_g"); b.op("MINI", imm=0)
    b.op("MUL", "cp"); b.op("MAXI", imm=q(OUT_LO - CENTER)); b.op("ST", "sn")
    b.land(at2)
    b.op("LD", "en"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=0); b.op("STNF", "sn")   # disabled: centre

    # ---------------------------------------------------------------- discrete
    # explosion / effect: SN76477 -> 560R / .1u -> 7.36k / .1u -> high pass (7.36k + 2k + 50k) / 1u
    b.op("LD", "sn"); b.op("MULI", imm=q(K_IN)); b.op("ST", "x_in")
    b.rcfilter("x1", "x_in", 560, 0.1 * U)
    b.rcfilter("x2", "x1", 6.8 * K + 560, 0.1 * U)
    b.crfilter("x3", "x2", 6.8 * K + 560 + 2 * K + 50 * K, 1 * U)
    # dot: 4V square (hsync / 8), 8V = 4V / 2 (toggles on each 4V cycle); switch; 560R / .1u -> 7.36k / .1u
    b.init("d_s8", 1.7)
    b.op("LD", "d_ph"); b.op("ADDI", imm=q(F4V / FS)); b.op("CMPI", imm=ONE)
    b.op("ADDI", imm=-ONE); b.op("STF", "d_ph"); b.op("ADDI", imm=ONE); b.op("STNF", "d_ph")
    b.op("LDI", imm=0); b.op("SUB", "d_s8"); b.op("STF", "d_s8")
    b.op("LD", "d_ph"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=q(-1.7)); b.op("LDIF", imm=q(1.7))
    b.op("BIT", 0, b.src(*DOT_SEL)); b.op("LDF", "d_s8")
    b.op("BIT", 1, b.src(*DOT_EN)); b.op("LDIF", imm=0); b.op("ST", "d_sw")
    b.rcfilter("d1", "d_sw", 560, 0.1 * U)
    b.rcfilter("d2", "d1", 6.8 * K + 560, 0.1 * U)
    # music: 1-bit DAC (3.4 V) -> high pass (470k + 50k) / .01u; Game Audio music volume
    b.op("BIT", 0, b.src(*MUSIC)); b.op("LDI", imm=0); b.op("LDIF", imm=q(3.4)); b.op("ST", "m_in")
    b.crfilter("m1", "m_in", 470 * K + 50 * K, 0.01 * U)
    b.music = True
    # mix (DISCRETE_ADDER3 enabled by SX7), MAME output gain 33080 / 32768
    b.op("LD", "m1"); b.op("MULI", imm=q(VR1)); b.op("MUL", "_fM")
    b.op("MACI", "x3", imm=q(VR2)); b.op("MACI", "d2", imm=q(VR3))
    b.op("MULI", imm=q(33080.0 / 32768))
    b.op("BIT", 1, b.src(*SND_EN)); b.op("LDIF", imm=0)
    b.op("OUT")
    return b.finish()


def schaser():
    return build("schaser")


def crashrd():
    return build("crashrd", crash=True)


if __name__ == "__main__":
    import sys
    for f in (schaser, crashrd):
        p = f()
        print(f"{p.name}: {len(p.code)} words, worst path {p.worst_path()} units, {len(p.regs)} registers")
        if len(sys.argv) > 1:
            with open(f"{sys.argv[1]}/{p.name}.bin", "wb") as fh:
                fh.write(p.binary())
