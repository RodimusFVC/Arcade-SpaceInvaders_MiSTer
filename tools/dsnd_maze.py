"""Amazing Maze (Midway 1976) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

Board: optional game-board add-on 0611-00905 ("Sound Circuit (Optional) Game Board Add-On", one page,
Amazing_Maze_Sound_Board_Addon_(0611-00905).pdf). Started from dsnd_boards.maze() (MAME maze_discrete, sim-checked);
schematic corrections:
- 74147 E1 priority: the 74157 D1 takes UP on 1A / 1B (E1 input 1) and LEFT on 4A / 4B (E1 input 4); the sheet's own
  table reads DWN 00, RT 01, UP 10, LT 11 (B A). Priority LT > DN > RT > UP, idle = 11. MAME's maze_74147_table
  assumes IN0 bit order (bit 0 = LEFT on input 1), so it swaps UP and LEFT: UP gave 3 (idle pitch), LEFT gave 2.
- C401 is 0.01 uF (mylar); MAME writes CAP_P(0.01) = 0.01 pF, which turns the CR stage into a one-sample
  differentiator (the tone came out as clicks on each edge). With 0.01 uF the 26 Hz high pass passes the square with
  droop. The C401 / R404 / C400 / R405 ladder into J3's virtual ground is one 2nd-order section: poles 26.0 Hz
  (high pass) and 950 Hz (low pass), mid-band gain 0.637; MAME cascades CR 446K and RC 56K (605 Hz).
- 555 F2 game-over delay: Q200 (2N4403, base on E2 pin 8) clamps C204 at ~0.77 V (ngspice op point, 7410 low
  0.05-0.3 V -> 0.67-0.92 V) while a stick is used, so the timeout after the last stick release is
  270K x 100 uF x ln(4.23 / 1.667) = 25.2 s, not MAME's 1.1 RC = 29.7 s (that is from 0 V: power-up / long idle).
J3 (inverting Norton amp, R411 || R410 volume pot), J3b (unity inverter) and the bridged LM377 only scale the tone
(their 16 Hz / 0.06 Hz coupling corners are far below the 380-620 Hz tones): one output gain.
Tone oscillator J1 kept as MAME (LM3900 swing 0 .. V+ - 0.5 V; ngspice brackets it: -2.5 % / +16 % for the
behavioural / transistor-macro LM3900, the macro's 0.56 V output low is not the datasheet's 0.09 V).
Inputs: IN0 as the CPU reads it (engine source 4, P1 = right player D0-D3, P2 = left player D4-D7, active high:
D0 LEFT, D1 RIGHT, D2 DOWN, D3 UP), coin = misc bit 1 (~IN1 D3, low while the coin switch is closed).
"""
import math

from dsnd import Board, q, ONE, FS

K, M_, U = 1e3, 1e6, 1e-6
T_OUT = 1.1 * 270e3 * 100e-6                          # 555 F2: 2/3 Vcc from 0 V (29.7 s)
T_HOLD = 270e3 * 100e-6 * math.log(5 / (5 - 0.77))    # C204 time-equivalent of Q200's 0.77 V clamp (4.51 s)
R404, R405, C400, C401 = 56 * K, 390 * K, 4700e-12, 0.01 * U
# ladder p -> C401 -> R404 -> x (C400 to ground, R405 to the virtual ground): x / p = b s / (s^2 + (a + b + d) s + a d)
# with a = 1 / (R404 C401), b = 1 / (R404 C400), d = 1 / (R405 C400): poles 26.0 Hz and 950 Hz (not the 35.7 / 691 Hz
# of separate R404 + R405 / R404 || R405 sections), mid-band gain b / p2 = 0.637
_a, _b, _d = 1 / (R404 * C401), 1 / (R404 * C400), 1 / (R405 * C400)
_s = math.sqrt((_a + _b + _d) ** 2 - 4 * _a * _d)
LADDER_T1, LADDER_T2 = 2 / ((_a + _b + _d) - _s), 2 / ((_a + _b + _d) + _s)
LADDER_K = _b * LADDER_T2
OUT_GAIN = 0.11                                       # x 96200: tone RMS ~ the old (MAME-normalised) program


def maze(out_gain=None):
    g = OUT_GAIN if out_gain is None else out_gain
    b = Board("maze")
    b.init("go", 1.0); b.init("go_d", 1.0)           # 555 F2 timing from power-up (MAME: sound for ~30 s)
    half = 0.693 * (33e3 + 2 * 68e3) * 1e-6         # PERIOD_OF_555_ASTABLE: the timing FF toggles each period
    b.op("LD", "tt_ph"); b.op("ADDI", imm=q(1.0 / (half * FS))); b.op("ST", "tt_ph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE); b.op("ST", "tt_ph")
    b.op("LD", "tt"); b.op("CMPI", imm=q(0.5))       # falling edge of the timing FF toggles the player select
    b.op("LDI", imm=ONE); b.op("SUB", "psel"); b.op("STF", "psel")
    b.op("LDI", imm=ONE); b.op("SUB", "tt"); b.op("ST", "tt")
    b.land(at)
    # joystick in use (D2 7430): raw IN0 (active high) != 0
    b.op("LDL", 0, b.src(4, 0)); b.op("CMPI", imm=1 << 16)
    b.op("LDI", imm=0); b.op("LDIF", imm=ONE); b.op("ST", "use")
    # 555 F2 (R203 270K, C204 100 uF): a stick triggers it (output high, go = 1); C204 charges (mono = time-equivalent
    # of its voltage) but Q200 clamps it at T_HOLD while a stick is used; 2/3 Vcc (T_OUT) ends the timing and pin 7
    # discharges C204 (mono = 0). Times out from power-up (go = 1 at start, as MAME). go falling sets the mute FF.
    # (old: mono = 0 while used, counts to MINI t_out + 1 when idle, never reset after the timeout)
    b.op("LD", "use"); b.op("CMPI", imm=q(0.5))
    b.op("LDI", imm=ONE); b.op("STF", "go")
    b.op("LD", "mono"); b.op("ADDI", imm=q(1.0 / FS)); b.op("ST", "_m")
    b.op("MINI", imm=q(T_HOLD)); b.op("ST", "_h")
    b.op("LD", "use"); b.op("CMPI", imm=q(0.5)); b.op("LD", "_h"); b.op("STF", "_m")
    b.op("LD", "_m"); b.op("CMPI", imm=q(T_OUT)); b.op("LDI", imm=0); b.op("STF", "go")
    b.op("LD", "go"); b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=0); b.op("STNF", "_m")
    b.op("LD", "_m"); b.op("ST", "mono")
    b.op("LD", "go_d"); b.op("SUB", "go")             # 1 -> 0 edge: go_d - go = 1
    b.op("CMPI", imm=q(0.5)); b.op("LDI", imm=ONE); b.op("STF", "mute")
    b.op("LD", "go"); b.op("ST", "go_d")
    b.op("BIT", 1, b.src(5, 1)); b.op("LDI", imm=0); b.op("STF", "mute")       # coin (line low) clears the mute FF
    # selected player's stick (D1 74157: S = 0 right player = IN0 D0-D3, S = 1 left player = D4-D7) re-packed as a
    # priority value pv = 8 LEFT + 4 DOWN + 2 RIGHT + UP (x 2^16), so E1 (74147) is a monotonic compare chain:
    # pv >= 8 -> 3 (LT), >= 4 -> 0 (DN), >= 2 -> 1 (RT), >= 1 -> 2 (UP), 0 -> 3 (idle)
    # (old, MAME table on the ~nibble: UP -> 3, DN -> 0, RT -> 1, LEFT -> 2; priority UP > DN > RT > LEFT)
    for reg, lm in (("_p1", ((0x01, 19), (0x06, 16), (0x08, 13))), ("_p2", ((0x10, 15), (0x60, 12), (0x80, 9)))):
        b.op("LDLM", 0, b.src(4, 0), imm=lm[0][0] | (lm[0][1] << 8)); b.op("ST", reg)
        for mask, sh in lm[1:]:
            b.op("LDLM", 0, b.src(4, 0), imm=mask | (sh << 8)); b.op("ADD", reg); b.op("ST", reg)
    b.op("LD", "psel"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "_p1"); b.op("LDF", "_p2"); b.op("ST", "pv")
    b.op("LDI", imm=3 << 16); b.op("ST", "enc")
    for th, v in ((1, 2), (2, 1), (4, 0), (8, 3)):
        b.op("LD", "pv"); b.op("CMPI", imm=th << 16); b.op("LDI", imm=v << 16); b.op("STF", "enc")
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
    # H3 11-10: stick in use AND tone enabled (not muted) AND tone timing -> C301 side, else R402 / R403 (2.5 V);
    # centred on 2.5 V (C401 blocks the DC)
    b.op("LDI", imm=0); b.op("ST", "sw")
    b.op("LD", "use"); b.op("MUL", "tt"); b.op("ST", "_u")
    b.op("LDI", imm=ONE); b.op("SUB", "mute"); b.op("MUL", "_u"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "hp1"); b.op("STF", "sw")
    # C401 / R404 / C400 / R405 into J3's virtual ground (old: CR 446K / 0.01 pF, RC 56K / 4700 pF): exact as its two
    # real poles, a high pass and a low pass in cascade with gain LADDER_K (x - 0.5 V at the C400 node)
    b.crfilter("hp2", "sw", LADDER_T1, 1.0)
    b.rcfilter("snd", "hp2", LADDER_T2, 1.0)
    b.op("LD", "mute"); b.op("CMPI", imm=q(0.5))
    b.op("LD", "snd"); b.op("LDIF", imm=0)
    b.op("MULI", imm=int(round(g * LADDER_K * 96200 / 32768 * (1 << 24))))
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    p = maze(float(sys.argv[2]) if len(sys.argv) > 2 else None)
    out = sys.argv[1] if len(sys.argv) > 1 else "maze.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"maze: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
