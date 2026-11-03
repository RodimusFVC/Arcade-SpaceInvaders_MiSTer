#!/usr/bin/env python3
"""Space Invaders sound board (Midway A084-90700-B000 and its variants) as a discrete sound engine program
(tools/dsnd.py, rtl/rodimus/dsnd_engine/dsnd_engine.sv).

A circuit-for-circuit port of rtl/invaders_sound.sv (bit-exact C model verilator/snd/model.h): same topology, same
constants (tools/snd_consts.py BOARDS[board], Q20 volts / Q28 coefficients converted to Q24), same summer, rail clamp,
output scaling (OUT_G counts per volt) and P35 mute. `board` selects the constant set (snd_consts BOARD_PARAMS key);
MIDWAY and TAITO (L-shaped, RTL board 1: constants only, its trimmer mix is folded into the G_ gains by snd_consts)
are A/B-verified (Factory Game Audio). The other sets differ in constants only, plus INVAD2CT2's C37 low-pass, PV's
8V gate and MAME's unclamped summer. `grid` (default: every board; MIDWAY since 2026-11-03)
puts the saucer-hit / invader-hit VCO_1s on the RTL's exact substep grid (vco1_grid; their steps and reciprocals
move to the round robin's idle slots), which also lowers the static worst case (Taito 407 vs 409).

Differences against the RTL (format, not circuit):
  * Oscillators: the RTL steps its op-amp oscillators and the SN76477 VCO in 16 substeps per sample and averages the
    square outputs over them. Here each takes one 48 kHz step; a threshold crossing carries the overshoot time into
    the next phase (exact for linear ramps), and a square output gets the crossing sample's exact high fraction
    (overshoot / step; RCP where the step varies). The RTL clamps a crossing to the threshold and loses on average
    half a substep per edge: the carry subtracts that 1/32 sample, so the periods agree on average.
  * Slow parts run in an 8-sample round robin with 8-step coefficients (engine budget): the two OSC_1 LFOs (values
    held 8 samples), the bonus 556 gate timer and the three one-shots' timing caps (one-shot lengths quantized to 8
    samples). With `grid`, a VCO_1's phase still moves against the RTL's at each LFO turnaround (the held control
    integrates differently over that hold: up to ~1 sample per turnaround with the invader-hit LFO's fast charge);
    pitch contour and spectrum are unaffected.
  * Q24 / 32-bit arithmetic instead of Q20 volts / Q28 coefficients in 36 bits.
  * The engine has no 16V input: the bonus "480 Hz" gate is a free-running 487.5 Hz square (16V = bit 4 of the line
    count, 15600 lines / s, no frame wrap: model_wav's convention), so its phase against the video is lost. PV gates
    with 8V instead: a free-running 975 Hz square, same convention (power-up phase offset: see bonus).
  * sn_top (every board; MIDWAY / TAITO since 2026-11-02): the SN76477 VCO's top crossing is at most 15 substeps back
    (vmax can fall below the previous sample's value while the SLF falls); without it the VCO slips one substep there
    (PV / INVAD2CT1 8.2K / 0.1uF: audible phase drift; MIDWAY / TAITO: the same slips, smaller effect).
  * The 17-bit noise LFSR (taps 4 / 16, output bit 12, 7515 Hz phase clock) is reproduced bit for bit (not the
    engine's NOISE op).

Sources: P3 (port 3) = LATCH1, P5 (port 5) = LATCH2 (8080bw boards: sound ports 1 / 2), Game Audio = sources 6 / 7
(T timing / F filter / O oscillator capacitor groups + Aged Caps, as invaders_sound.sv `tweak`).
Every state register powers up 0 = the RTL's reset state (no init block).

    python3 dsnd_invaders.py [board] [out.bin]     -> program binary; prints size and worst-case budget per block
"""
import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dsnd import Board, LATCH1, LATCH2, q, ONE, FS, OPS        # noqa: E402
import snd_consts as SC                                         # noqa: E402

P3, P5 = LATCH1, LATCH2
SUB = SC.SUB
INT_MIN = -(1 << 31)
NRR = 8                                                         # round-robin length (samples)

# Q28 coefficients in snd_consts (everything else is a Q20 voltage)
COEF = ({"SH_VCO_KA", "SH_VCO_KB", "FL_KD", "FL_RC1", "FL_RC2", "BN_KC", "BN_KD", "OS_K10", "OS_E2", "IH_E1C",
         "IH_E1D", "EX_E1C", "EX_E1D", "MS_E1C", "MS_E1D", "IH_VCO_KA", "IH_VCO_KB", "IH_OUT_G", "EX_EC", "EX_ED",
         "EX_RC1", "EX_RC2", "MS_CR", "MS_NZ_G", "MS_VCO_K1", "MS_VCO_K6", "MS_A3_G", "MS_EC", "MS_ED", "MX_C11",
         "MX_C44", "MX_CAMP", "MX_LP", "OUT_G"} | {f"FL_KC{d}" for d in range(16)}
        | {f"G_{n}" for n in ("SH", "FL", "BN", "IH", "EX", "MS", "SN")})
# per-substep rates in snd_consts (x 16 = per sample)
PER_SUB = {"SH_OSC_D0", "SH_OSC_D1", "SH_VCO_KA", "SH_VCO_KB", "IH_OSC_D0", "IH_OSC_D1", "IH_VCO_KA", "IH_VCO_KB",
           "MS_VCO_IF", "MS_VCO_K1", "MS_VCO_K6", "MS_VCO_T1", "SN_VCO_STEP"}
# Game Audio scaling exactly as invaders_sound.sv KG (group) / KC (RC coefficient, capped at 1.0)
GROUP = dict(SH_ENV_UP="T", SH_ENV_DN="T", BN_KC="T", BN_KD="T", OS_E2="T", EX_E1C="T", EX_E1D="T", MS_E1C="T",
             MS_E1D="T", IH_E1C="T", IH_E1D="T", IH_ENV_UP="T", IH_ENV_DN="T", EX_EC="T", EX_ED="T", MS_ENV_UP="T",
             MS_ENV_DN="T", MS_CR="T", MS_EC="T", MS_ED="T", NZ_INC="O", SH_OSC_D0="O", SH_OSC_D1="O",
             SH_VCO_KA="O", SH_VCO_KB="O", FL_KD="O", IH_OSC_D0="O", IH_OSC_D1="O", IH_VCO_KA="O", IH_VCO_KB="O",
             SN_SLF_UP="O", SN_SLF_DN="O", SN_VCO_STEP="O", FL_RC1="F", FL_RC2="F", MX_CAMP="F", EX_RC1="F",
             EX_RC2="F", MX_C11="F", MX_C44="F", MX_LP="F", **{f"FL_KC{d}": "O" for d in range(16)})
CAPPED = ({"BN_KC", "BN_KD", "OS_E2", "EX_E1C", "EX_E1D", "MS_E1C", "MS_E1D", "IH_E1C", "IH_E1D", "EX_EC", "EX_ED",
           "MS_CR", "MS_EC", "MS_ED", "SH_VCO_KA", "SH_VCO_KB", "FL_KD", "IH_VCO_KA", "IH_VCO_KB", "FL_RC1",
           "FL_RC2", "MX_CAMP", "EX_RC1", "EX_RC2", "MX_C11", "MX_C44", "MX_LP"} | {f"FL_KC{d}" for d in range(16)})


class Inv(Board):
    def __init__(self, board="MIDWAY", tweak=True, grid=None, sn_top=None):
        super().__init__("invaders_" + board.lower())
        self.board = board
        self.grid = True if grid is None else grid     # VCO_1s on the exact substep grid (vco1_grid); MIDWAY too since 2026-11-03
        # SN76477 top crossing at most 15 substeps back (sn76477): off for the shipped MIDWAY / TAITO programs
        self.sn_top = True if sn_top is None else sn_top    # every board (MIDWAY / TAITO too: user 2026-11-02)
        self.c = SC.BOARDS[board]
        self.p = SC.BOARD_PARAMS[board]
        self.tweak = tweak
        self.marks = []
        self.pinit = []                               # (register, raw value) set once at power-up (settings block)
        self.kcuts = []

    # --- constants ---------------------------------------------------------------------------------------------
    def f(self, name):
        """snd_consts value as a float (volts / coefficient), per sample"""
        v = self.c[name] / (1 << (28 if name in COEF else 20))
        return v * SUB if name in PER_SUB else v

    def v(self, name):
        return q(self.f(name))

    def _kemit(self, emit):
        body, self.code = self.code, self.kcode
        emit()
        self.kcode, self.code = self.code, body
        self.kcuts.append(len(self.kcode))           # complete sequences: the settings block may split here

    def k(self, name, steps=1, scale=1.0, raw=False):
        """register = a snd_consts rate scaled by its Game Audio group (capped at 1.0 as the RTL), then x scale; steps
        > 1: an RC coefficient for that many samples, 1 - (1 - k)^steps (x the group factor). Built in the settings
        block (free per sample)"""
        reg = f"k_{name}" + (f"_n{steps}" if steps > 1 else "") + (f"_x{scale:g}" if scale != 1.0 else "") + \
            ("_raw" if raw else "")
        if reg in self.regs:
            return reg
        g = GROUP.get(name) if self.tweak else None
        cap = name in CAPPED and self.f(name) / 0.7 / 0.7 >= 1.0      # the RTL's 1.0 cap can act at all

        def emit():
            if steps > 1:                             # Factory value exact; settings scale it (error ~ steps x k / 2)
                self.op("LDI", imm=q(1.0 - (1.0 - self.f(name)) ** steps))
            else:
                self.op("LDI", imm=self.c[name] if raw else self.v(name))
            if g:
                self.groups.add(g)
                self.op("MUL", "_f" + g)
                if cap:
                    self.op("MINI", imm=(1 << 28) if raw else ONE)
            if scale != 1.0:
                self.op("MULI", imm=q(scale))
            self.op("ST", reg)
        self._kemit(emit)
        return reg

    # --- the RTL's own integer arithmetic (Q20 / Q28, model.h), for power-up states and exact phase counts -------
    def rtl_mul(self, a, b):
        return (a * b) >> 28

    def rtl_bonus_counts(self):
        """556 phase lengths in samples (Factory): first high run out of reset, low, high (cap clamped at the
        thresholds, so every later phase repeats exactly)"""
        c, mul = self.c, self.rtl_mul
        th, tr = c["V555_TH"], c["V555_TR"]

        def charge(v):
            n = 0
            while True:
                n += 1
                v += mul(c["V5"] - v, c["BN_KC"])
                if v >= th:
                    return n
        v, n2 = th, 0
        while True:
            n2 += 1
            v -= mul(v, c["BN_KD"])
            if v <= tr:
                break
        return charge(0) - 1, n2, charge(tr)

    def rtl_sn_start(self, nmin=2400):
        """(n, v0): the RTL's SN76477 VCO is rising at sample n; a dsnd VCO that powers up at v0 (rising, below the
        low limit) reaches exactly the RTL's value there without a crossing, so it skips the SLF power-up transient
        (VCO span below one sample step) that a one-step VCO cannot follow. v0 ~ -(n + 1) step must fit 32 bits: a
        faster VCO (Taito 3.9K / 0.22uF: 2.1 x Midway's step) starts earlier (Midway: nmin as given)"""
        c = self.c
        st = c["SN_VCO_STEP"]
        st24 = self.v("SN_VCO_STEP")
        nmin = min(nmin, (1 << 31) // st24 - 200)
        slf, sff, v, vff = 0, 0, 0, 0
        vmax24 = []
        for n in range(48000):
            slf = min(slf + c["SN_SLF_UP"], c["SN_SLF_MAX"]) if not sff else max(slf - c["SN_SLF_DN"], c["SN_SLF_MIN"])
            sff = 1 if slf >= c["SN_SLF_MAX"] else 0 if slf <= c["SN_SLF_MIN"] else sff
            vmax = slf + c["SN_VCO_DIFF"]
            vmax24.append(vmax * 16)
            for _ in range(SUB):
                v = min(v + st, vmax) if not vff else max(v - st, c["SN_SLF_MIN"])
                vff = 1 if v >= vmax else 0 if v <= c["SN_SLF_MIN"] else vff
            if n >= nmin and not vff and v < vmax:
                v0 = v * 16 - (n + 1) * st24
                if v0 >= INT_MIN and all(v0 + (k + 1) * st24 < vmax24[k] for k in range(n + 1)):
                    return n, v0
        raise AssertionError("no SN76477 start state")

    def mark(self, name):
        self.marks.append((name, len(self.code)))

    # --- small patterns ----------------------------------------------------------------------------------------
    def jump(self):
        self.op("CMPI", imm=INT_MIN)
        return self.skip_if(True)

    def flag01(self, reg):
        """F = (reg == 1.0) for a 0 / 1.0 flag"""
        self.op("LD", reg)
        self.op("CMPI", imm=q(0.5))

    def env(self, reg, on_reg, up, dn):
        """reg = clamp(reg + (on ? up : 0) - dn, 0, VOH); on_reg is 0 / 1.0; acc = reg"""
        self.op("LD", on_reg)
        self.op("MUL", self.k(up))
        self.op("SUB", self.k(dn))
        self.op("ADD", reg)
        self.op("MAXI", imm=0)
        self.op("MINI", imm=self.v("VOH"))
        self.op("ST", reg)

    # --- blocks ------------------------------------------------------------------------------------------------
    def noise(self):
        """17-bit LFSR, taps 4 / 16, output bit 12, stepped on the carry of a 32-bit phase (+NZ_INC per sample).
        Carry <=> the new phase, unsigned, is below the increment: flip the sign bit, one signed compare. The LFSR is
        kept complemented and left-aligned, W = ~l << 15 (power-up 0x1FFFF -> 0); new bit' = (b4' == b16'), tested as
        the sign of (W >> 15) x ((W << 12) + 1). nz = 12 V while l bit 12 = 1"""
        inc = self.c["NZ_INC"]
        self.pinit += [("nz_ph", INT_MIN), ("nz", self.v("V12"))]   # phase - 2^31; l = 0x1FFFF: bit 12 set
        self.op("LD", "nz_ph")
        if self.tweak:
            def emit():
                self.groups.add("O")
                self.op("LDI", imm=inc)
                self.op("MUL", "_fO")
                self.op("ST", "nz_inc")
                self.op("LDI", imm=0x7FFFFFFF)
                self.op("SUB", "nz_inc")
                self.op("ADDI", imm=1)
                self.op("ST", "nz_thr")                # 2^31 - inc
            self._kemit(emit)
            self.op("CMP", "nz_thr")
            self.op("ADD", "nz_inc")
        else:
            self.op("CMPI", imm=(1 << 31) - inc)
            self.op("ADDI", imm=inc)
        self.op("ST", "nz_ph")                        # (phase - 2^31) >= 2^31 - inc: carry
        at = self.skip_if(False)
        self.op("LD", "nz_w")
        self.op("MULI", imm=64 << 24)
        self.op("MULI", imm=64 << 24)
        self.op("ADDI", imm=1)
        self.op("ST", "nz_b")                         # (W << 12) + 1: sign = b4', never 0
        self.op("LD", "nz_w")
        self.op("MULI", imm=1 << 9)                   # W >> 15 = l' (sign = b16')
        self.op("MUL", "nz_b")
        self.op("CMPI", imm=0)                        # F = b4' == b16'
        self.op("LDI", imm=0)
        self.op("LDIF", imm=1 << 15)
        self.op("ADD", "nz_w")
        self.op("ADD", "nz_w")
        self.op("ST", "nz_w")
        self.op("MULI", imm=16 << 24)                 # l' bit 12 at the sign
        self.op("CMPI", imm=0)
        self.op("LDI", imm=0)
        self.op("LDIF", imm=self.v("V12"))
        self.op("ST", "nz")
        self.land(at)

    def osc1(self, n, d0, d1, steps=1):
        """Norton OSC_1 (constant charge / discharge steps), cap n_v, n_lo = 1.0 while discharging (power-up charging
        from 0 V), `steps` samples per call. A crossing carries the overshoot time less the RTL's substep loss"""
        tl, th = self.v("OSC_TL"), self.v("OSC_TH")
        D0, D1 = self.c[d0], self.c[d1]               # integer ratio: the RTL's substep rates exactly
        k0, k1 = self.k(d0, scale=steps), self.k(d1, scale=steps)
        self.flag01(n + "_lo")
        at = self.skip_if(True)
        self.op("LD", n + "_v")                       # charging
        self.op("ADD", k1)
        self.op("ST", n + "_v")
        self.op("CMPI", imm=th + 1)
        e1 = self.skip_if(False)
        self.op("ADDI", imm=-th)                      # v = th - (over / d1 - 1/32) d0
        self.op("MULI", imm=q(-D0 / D1))
        self.op("MACI", k0, imm=q(1 / 32 / steps))
        self.op("ADDI", imm=th)
        self.op("ST", n + "_v")
        self.op("LDI", imm=ONE)
        self.op("ST", n + "_lo")
        e2 = self.jump()
        self.land(at)
        self.op("LD", n + "_v")                       # discharging
        self.op("SUB", k0)
        self.op("ST", n + "_v")
        self.op("CMPI", imm=tl)
        e3 = self.skip_if(True)
        self.op("ADDI", imm=-tl)                      # v = tl + (under / d0 - 1/32) d1
        self.op("MULI", imm=q(-D1 / D0))
        self.op("MACI", k1, imm=q(-1 / 32 / steps))
        self.op("ADDI", imm=tl)
        self.op("ST", n + "_v")
        self.op("LDI", imm=0)
        self.op("ST", n + "_lo")
        for e in (e1, e2, e3):
            self.land(e)

    def vco1(self, n, ctl, ka, kb, frac):
        """Norton VCO_1: per sample d1 = (ctl - VBE) KB (charge, flip-flop n_hi = 0, power-up), d0 = (ctl - VBE) KA
        (discharge, n_hi = 1); thresholds VCO_TL / VCO_TH. The steps share ctl: the carry ratio is the constant
        KA / KB. frac: n_sq = fraction of the sample the flip-flop was high (the RTL's substep average)"""
        tl, th = self.v("VCO_TL"), self.v("VCO_TH")
        r = self.c[ka] / self.c[kb]
        # the RTL floors cv x K once per substep (Q20): on average its step is 16 x 0.5 Q20 LSB = 128 Q24 LSB short;
        # folded into the control offset: (cv - 128 LSB / K) K
        fb_b, fb_a = 128 / ONE / self.f(kb), 128 / ONE / self.f(ka)
        self.flag01(n + "_hi")
        at = self.skip_if(True)
        self.op("LD", ctl)                            # charging
        self.op("ADDI", imm=-self.v("VBE") - q(fb_b))
        self.op("MUL", self.k(kb))
        self.op("ST", n + "_d")
        self.op("ADD", n + "_v")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=th + 1)
        e1 = self.skip_if(False)
        self.op("ADDI", imm=-th)
        if frac:
            self.op("ST", n + "_ov")
        self.op("MULI", imm=q(-r))                    # th - (over / d1 - 1/32) d0, d0 = d1 KA / KB
        self.op("MACI", n + "_d", imm=q(r / 32))
        self.op("ADDI", imm=th)
        self.op("ST", n + "_v")
        self.op("LDI", imm=ONE)
        self.op("ST", n + "_hi")
        if frac:                                      # high for the time after the crossing
            self.op("LD", n + "_d")
            self.op("RCP")
            self.op("MUL", n + "_ov")
            self.op("ST", n + "_sq")
        e2 = self.jump()
        self.land(e1)
        if frac:
            self.op("LDI", imm=0)
            self.op("ST", n + "_sq")
        e3 = self.jump()
        self.land(at)
        self.op("LD", ctl)                            # discharging: n_d = -d0
        self.op("ADDI", imm=-self.v("VBE") - q(fb_a))
        self.op("MUL", self.k(ka, scale=-1.0))
        self.op("ST", n + "_d")
        self.op("ADD", n + "_v")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=tl)
        e4 = self.skip_if(True)
        self.op("ADDI", imm=-tl)
        if frac:
            self.op("ST", n + "_ov")
        self.op("MULI", imm=q(-1 / r))                # tl + (under / d0 - 1/32) d1
        self.op("MACI", n + "_d", imm=q(1 / r / 32))
        self.op("ADDI", imm=tl)
        self.op("ST", n + "_v")
        self.op("LDI", imm=0)
        self.op("ST", n + "_hi")
        if frac:                                      # high until the crossing: 1 - under / d0
            self.op("LD", n + "_d")
            self.op("RCP")
            self.op("MUL", n + "_ov")                 # under / d0
            self.op("MULI", imm=-ONE)
            self.op("ADDI", imm=ONE)
            self.op("ST", n + "_sq")
        e5 = self.jump()
        self.land(e4)
        if frac:
            self.op("LDI", imm=ONE)
            self.op("ST", n + "_sq")
        for e in (e2, e3, e5):
            self.land(e)

    # Norton VCO_1 on the exact substep grid: the cap register w holds v while charging (n_hi = 0) and s v + c while
    # discharging, so the flip-flop costs no register: IH (v drives the output) s = -1, c = 0 (|w| = v; its LFO
    # passes VBE within a few samples, so v stays >= 0); SH s = +1, c = -32 V (its slow LFO holds the control below
    # VBE for ~200 samples after power-up: v charges negative, as the RTL's, down to about -2.5 V)
    VCO1_ENC = {"ihv": (-1, 0.0), "shv": (1, -32.0)}

    def vco1_rr(self, n, ctl, ka, kb):
        """round-robin half of vco1_grid, run once per LFO hold (the slot after the LFO's): n_d1 = the charge step,
        n_dd = the discharge step on w (-s d0), both as vco1 (incl. the RTL's floor offset); n_i1 = 1 / d1,
        n_id = 1 / dd (= i1 KB / KA x -s)"""
        sg = self.VCO1_ENC[n][0]
        r = self.c[ka] / self.c[kb]
        for d, k, fb in (("_dd", self.k(ka, scale=-sg), 128 / ONE / self.f(ka)),
                         ("_d1", self.k(kb), 128 / ONE / self.f(kb))):
            self.op("LD", ctl)
            self.op("ADDI", imm=-self.v("VBE") - q(fb))
            self.op("MUL", k)
            self.op("ST", n + d)
        self.op("RCP")
        self.op("ST", n + "_i1")
        self.op("MULI", imm=q(-sg / r))
        self.op("ST", n + "_id")

    def rtl_vco1_start(self, osc, kb, slot):
        """power-up value of w: the RTL's VCO (model.h integer arithmetic) less this program's, both charging from 0
        over the first 3/4 of the samples before the RTL's first crossing. The program's steps exist only from the round-robin
        slot's first run (body sample slot - 1) and are held 8 samples, its LFO starting at the osc1 power-up lead;
        the remaining offset makes every later edge land on the RTL's substep grid"""
        c, mul = self.c, self.rtl_mul
        d0, d1 = c[osc + "_D0"], c[osc + "_D1"]
        lv, lff, v = 0, 1, 0                          # RTL: Osc1 (charging), Vco charging from 0
        rtl = []
        while True:
            for _ in range(SUB):
                if lff:
                    lv += d1
                    if lv > c["OSC_TH"]:
                        lv, lff = c["OSC_TH"], 0
                else:
                    lv -= d0
                    if lv < c["OSC_TL"]:
                        lv, lff = c["OSC_TL"], 1
            step = mul(lv - c["VBE"], c[kb])
            if not lff or v + SUB * step > c["VCO_TH"]:
                break
            v += SUB * step
            rtl.append(v / (1 << 20))
        nn = len(rtl) * 3 // 4
        assert nn >= 2 * NRR, "rtl_vco1_start: window too short"
        rtl = rtl[:nn]
        lead = ((slot - 1) % NRR) - (NRR - 1) / 2     # as the osc1 power-up value
        f1, kbs, vbe = self.f(osc + "_D1"), self.f(kb), SC.VBE
        fb = 128 / ONE / kbs
        w, d = 0.0, 0.0
        for k in range(nn):
            if k % NRR == (slot - 1) % NRR:
                ctl = f1 * (lead + NRR * (k // NRR + 1))
                d = (ctl - vbe - fb) * kbs
            w += d
        return q(rtl[-1] - w)

    def vco1_grid(self, n, frac):
        """Norton VCO_1 on the RTL's exact 16-substep edge grid (as vco3_ms / sn76477): after a crossing the new phase
        runs floor(16 tau) whole substeps, tau = overshoot / step (x the round robin's reciprocal: the steps are
        constant over an LFO hold). w = n_v: see VCO1_ENC. frac: n_sq = the RTL's substep average of the flip-flop
        (high while discharging): (n + 1) / 16 at the top crossing, (15 - n) / 16 at the bottom one"""
        sg, c = self.VCO1_ENC[n]
        c = q(c)
        tl, th = self.v("VCO_TL"), self.v("VCO_TH")
        w = n + "_v"
        self.op("LD", w)
        self.op("CMPI", imm=-tl // 2 if sg < 0 else c // 2)   # between the two ranges
        at = self.skip_if(False)                      # F: charging
        self.op("ADD", n + "_d1")
        self.op("ST", w)
        self.op("CMPI", imm=th + 1)
        e1 = self.skip_if(False)
        self.op("ADDI", imm=-th)
        self.op("MUL", n + "_i1")
        self.substeps("_ss")                          # n / 16
        if frac:
            self.op("ST", n + "_t")
            self.op("ADDI", imm=ONE >> 4)
            self.op("ST", n + "_sq")
            self.op("LD", n + "_t")
        self.op("MUL", n + "_dd")
        self.op("ADDI", imm=sg * th + c)
        self.op("ST", w)                              # s (th - n / 16 d0) + c: discharging
        ends = [self.jump()]
        if frac:
            self.land(e1)
            self.op("LDI", imm=0)
            self.op("ST", n + "_sq")
            ends.append(self.jump())
        else:
            ends.append(e1)
        self.land(at)
        self.op("ADD", n + "_dd")                     # discharging
        self.op("ST", w)
        if sg < 0:                                    # crossing <=> v < tl
            self.op("CMPI", imm=-tl + 1)
            cross = True                              # F = crossing
        else:
            self.op("CMPI", imm=tl + c)
            cross = False                             # F = no crossing
        if frac:                                      # no crossing first: the crossing falls through to the end
            e4 = self.skip_if(cross)
            self.op("LDI", imm=ONE)
            self.op("ST", n + "_sq")
            ends.append(self.jump())
            self.land(e4)
        else:
            ends.append(self.skip_if(not cross))
        self.op("ADDI", imm=-(sg * tl + c))           # s (v - tl)
        self.op("MUL", n + "_id")                     # x -s / d0
        self.substeps("_ss")
        if frac:
            self.op("ST", n + "_t")
            self.op("LDI", imm=ONE - (ONE >> 4))
            self.op("SUB", n + "_t")
            self.op("ST", n + "_sq")
            self.op("LD", n + "_t")
        self.op("MUL", n + "_d1")
        self.op("ADDI", imm=tl)
        self.op("ST", w)                              # tl + n / 16 d1: charging
        for e in ends:
            self.land(e)

    def ne555_fast(self, n, kd):
        """fleet 555 step in the RTL's own integer arithmetic: n_v in Q20, coefficients raw Q28, each step
        v += floor((target - v) k / 2^28) (= floor(floor(x k / 2^24) / 16)), so the threshold crossings - the tone's
        period - match the RTL sample for sample. Charging -> toward n_tg with n_k; discharging -> v -= floor(v KD);
        2/3 / 1/3 of 5 V clamp and flip. n_lo = 1.0 while discharging"""
        th, tr = self.c["V555_TH"], self.c["V555_TR"]
        self.flag01(n + "_lo")
        at = self.skip_if(True)
        self.op("LD", n + "_tg")
        self.op("SUB", n + "_v")
        self.op("MUL", n + "_k")
        self.op("MULI", imm=1 << 20)                  # floor(/ 16)
        self.op("ADD", n + "_v")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=th)
        self.op("LDI", imm=th)
        self.op("STF", n + "_v")
        self.op("LDI", imm=ONE)
        self.op("STF", n + "_lo")
        e = self.jump()
        self.land(at)
        self.op("LD", n + "_v")
        self.op("MUL", kd)
        self.op("MULI", imm=1 << 20)
        self.op("MULI", imm=-ONE)
        self.op("ADD", n + "_v")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=tr + 1)
        self.op("LDI", imm=tr)
        self.op("STNF", n + "_v")
        self.op("LDI", imm=0)
        self.op("STNF", n + "_lo")
        self.land(e)

    def fleet_kc(self):
        """fl_k = FL_KC[p5 & 15] (raw Q28; tree on D3-D1, D0 by LDIF), fl_tg = 4.4 V in Q20 (0 V for d = 0: the
        charge path is open, the cap bleeds through the 10M)"""
        vch = self.c["FL_VCH"]                                  # RTL units: Q20 volts, Q28 coefficients
        ks = [self.c[f"FL_KC{d}"] for d in range(16)]           # x Game Audio factor after the lookup
        # d = 0: the RTL bleeds v -= floor(v KC0); the charge step floor((tg - v) KC0) equals that for every v when
        # tg KC0 is just below 2^28 (misses only when v KC0 mod 2^28 is within 2^28 - tg KC0 of the top: ~1e-5)
        tg0 = ((1 << 28) - 1) // ks[0]
        self.op("LDI", imm=vch)
        self.op("ST", "fl_tg")
        ends = []

        def node(bit, base):
            if bit == 0:
                if base == 0:
                    self.op("BIT", 0, self.src(P5, 0))
                    self.op("LDI", imm=tg0)
                    self.op("LDIF", imm=vch)
                    self.op("ST", "fl_tg")
                    self.op("LDI", imm=ks[0])
                    self.op("LDIF", imm=ks[1])
                else:
                    self.op("LDI", imm=ks[base])
                    self.op("BIT", 0, self.src(P5, 0))
                    self.op("LDIF", imm=ks[base + 1])
                return
            self.op("BIT", 0, self.src(P5, bit))
            at = self.skip_if(True)
            node(bit - 1, base)
            ends.append(self.jump())
            self.land(at)
            node(bit - 1, base + (1 << bit))

        node(3, 0)
        for e in ends:
            self.land(e)
        if self.tweak:
            self.groups.add("O")
            self.op("MUL", "_fO")                     # (never reaches the RTL's 1.0 cap)
        self.op("ST", "fl_k")

    def bonus_timer(self, steps):
        """556 gate (P3 D4 = reset input, low: charging from 0 V) as phase counters with the RTL's exact phase lengths
        (Game Audio: x 1 / timing factor), `steps` samples per call; the remainder carries into the next phase, so
        edges land within `steps` - 1 samples of the RTL's and never drift. bn_lo = 1.0 while the output is low.
        Counts are samples << 16"""
        n1, n2, n3 = self.rtl_bonus_counts()
        if self.tweak:
            regs = []
            for nm, n in (("k_bn_n1", n1), ("k_bn_n2", n2), ("k_bn_n3", n3)):
                def emit(nm=nm, n=n):
                    if "_iT" not in self.regs:
                        self.op("LD", "_fT")
                        self.op("RCP")
                        self.op("ST", "_iT")
                    self.op("LDI", imm=n << 16)
                    self.op("MUL", "_iT")
                    self.op("ST", nm)
                self.groups.add("T")
                self._kemit(emit)
                regs.append(nm)
        self.op("BIT", 0, self.src(P3, 4))
        at = self.skip_if(True)
        if self.tweak:
            self.op("LD", regs[0])
        else:
            self.op("LDI", imm=n1 << 16)
        self.op("ST", "bn_c")
        self.op("LDI", imm=0)
        self.op("ST", "bn_lo")
        e1 = self.jump()
        self.land(at)
        self.op("LD", "bn_c")
        self.op("ADDI", imm=-(steps << 16))
        self.op("ST", "bn_c")
        self.op("CMPI", imm=1)
        e2 = self.skip_if(True)
        self.flag01("bn_lo")                          # phase over: low -> high (n3), high -> low (n2)
        if self.tweak:
            self.op("LD", regs[1])
            self.op("LDF", regs[2])
        else:
            self.op("LDI", imm=n2 << 16)
            self.op("LDIF", imm=n3 << 16)
        self.op("ADD", "bn_c")
        self.op("ST", "bn_c")
        self.op("LDI", imm=ONE)
        self.op("LDIF", imm=0)
        self.op("ST", "bn_lo")
        self.land(e1)
        self.land(e2)

    def one_shot(self, n, trig):
        """DST_OP_AMP_1SHT comparator, per sample (RTL): out = (trig - c2) r3 / r2 + vout r3 / r5 > pos(c1 - VBE) +
        i_fixed r3. n_on = 0 / 1.0. (trig - c2_old) = (trig - c2_new) / (1 - E2)"""
        self.pinit.append((n + "_r1", self.v("OS_IFIX")))
        self.op("BIT", 0, self.src(*trig))
        self.op("LDI", imm=0)
        self.op("LDIF", imm=self.v("OS_TRIG"))
        self.op("ST", n + "_tv")
        self.op("RCM", n + "_c2", self.r(self.k("OS_E2")))
        self.op("SUB", n + "_tv")
        if self.tweak:
            if "os_g" not in self.regs:
                def emit():
                    self.op("LDI", imm=ONE)
                    self.op("SUB", self.k("OS_E2"))
                    self.op("RCP")
                    self.op("MULI", imm=q(self.f("OS_K10")))
                    self.op("ST", "os_g")
                self._kemit(emit)
            self.op("MUL", "os_g")
        else:
            self.op("MULI", imm=q(self.f("OS_K10") / (1 - self.f("OS_E2"))))
        self.op("ADD", n + "_r1")                     # + pos(c1 - VBE) + i_fixed r3 (round robin)
        self.op("MACI", n + "_on", imm=-self.v("OS_VOUT_R5"))   # - the previous output's feedback
        self.op("CMPI", imm=0)                        # F = rhs >= lhs: output low
        self.op("LDI", imm=ONE)
        self.op("LDIF", imm=0)
        self.op("ST", n + "_on")

    def one_shot_c1(self, n, e1c, e1d, steps):
        """one-shot timing cap, `steps` samples per call: on -> charge toward OS_T1 (C1 to the r3 / r4 divider);
        off -> clamped to 0.6 V by the diode, else discharge through r4 (toward 0)"""
        self.flag01(n + "_on")
        at = self.skip_if(False)
        self.op("LDI", imm=self.v("OS_T1"))
        self.op("RCM", n + "_c1", self.r(self.k(e1c, steps)))
        e = self.jump()
        self.land(at)
        self.op("LDI", imm=self.v("V06"))
        self.op("CMP", n + "_c1")
        self.op("STNF", n + "_c1")
        e2 = self.skip_if(False)
        self.op("LDI", imm=0)
        self.op("RCM", n + "_c1", self.r(self.k(e1d, steps)))
        self.land(e)
        self.land(e2)
        self.op("ADDI", imm=-self.v("VBE"))           # acc = c1: the comparator's rhs term
        self.op("MAXI", imm=0)
        self.op("ADDI", imm=self.v("OS_IFIX"))
        self.op("ST", n + "_r1")

    def substeps(self, tmp):
        """acc = tau (samples after a crossing, 0 .. 1) -> floor(16 tau) / 16: the RTL clamps a crossing at the end of
        its substep, so the new phase runs that many whole substeps. (tau - 1/32) minus its remainder modulo 1/16,
        centred: (x << 12, wrapped) >> 12"""
        self.op("ADDI", imm=-(ONE >> 5))
        self.op("ST", tmp)
        self.op("MULI", imm=64 << 24)
        self.op("MULI", imm=64 << 24)
        self.op("MULI", imm=1 << 12)
        self.op("ST", tmp + "r")
        self.op("LD", tmp)
        self.op("SUB", tmp + "r")

    def vco3_ms(self):
        """missile VCO_3: charges by md1 while msv_dis = 0 (power-up), discharges by md0 (msv_dn = -md0). Both steps
        follow the noise / envelope: the carry divides by the step (RCP). msv_sq = fraction of the sample charging
        (the RTL's square: high while charging)"""
        tl, th = self.v("MS_VCO_TL"), self.v("MS_VCO_TH")
        n = "msv"
        self.flag01(n + "_dis")
        at = self.skip_if(True)
        self.op("LD", n + "_v")                       # charging
        self.op("ADD", "md1")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=th + 1)
        e1 = self.skip_if(False)
        self.op("ADDI", imm=-th)
        self.op("ST", n + "_ov")
        self.op("LD", "md1")
        self.op("RCP")
        self.op("MUL", n + "_ov")                     # tau: time after the crossing (samples)
        self.substeps("_ss")
        self.op("ST", n + "_tau")                     # whole substeps left, / 16
        self.op("MUL", "md0n")
        self.op("ADDI", imm=th)
        self.op("ST", n + "_v")                       # th - n / 16 md0
        self.op("LDI", imm=ONE)
        self.op("ST", n + "_dis")
        self.op("ADDI", imm=-(ONE >> 4))
        self.op("SUB", n + "_tau")
        self.op("ST", n + "_sq")                      # charging for the substeps before the crossing one: 15/16 - n/16
        e2 = self.jump()
        self.land(e1)
        self.op("LDI", imm=ONE)
        self.op("ST", n + "_sq")
        e3 = self.jump()
        self.land(at)
        self.op("LD", n + "_v")                       # discharging
        self.op("ADD", "md0n")
        self.op("ST", n + "_v")
        self.op("CMPI", imm=tl)
        e4 = self.skip_if(True)
        self.op("ADDI", imm=-tl)
        self.op("ST", n + "_ov")                      # < 0
        self.op("LD", "md0n")
        self.op("RCP")
        self.op("MUL", n + "_ov")                     # tau
        self.substeps("_ss")
        self.op("ST", n + "_tau")
        self.op("ADDI", imm=ONE >> 4)
        self.op("ST", n + "_sq")                      # charging from the crossing substep: (n + 1) / 16
        self.op("LD", n + "_tau")
        self.op("MUL", "md1")
        self.op("ADDI", imm=tl)
        self.op("ST", n + "_v")                       # tl + n / 16 md1
        self.op("LDI", imm=0)
        self.op("ST", n + "_dis")
        e5 = self.jump()
        self.land(e4)
        self.op("LDI", imm=0)
        self.op("ST", n + "_sq")
        for e in (e2, e3, e5):
            self.land(e)

    def sn76477(self):
        """SLF exactly as the RTL (per sample, clamp at the turning limit); VCO one step per sample with the carry
        mirrored about the limit (equal up / down steps) less the 1/32-sample substep loss. sn_sq = time high"""
        mn, mx, dv = self.v("SN_SLF_MIN"), self.v("SN_SLF_MAX"), self.v("SN_VCO_DIFF")
        self.sn_start = self.rtl_sn_start()
        self.pinit.append(("sn_v", self.sn_start[1]))
        self.flag01("sn_sff")
        self.op("LD", "sn_s")
        at = self.skip_if(True)
        self.op("ADD", self.k("SN_SLF_UP"))
        self.op("MINI", imm=mx)
        e1 = self.jump()
        self.land(at)
        self.op("SUB", self.k("SN_SLF_DN"))
        self.op("MAXI", imm=mn)
        self.land(e1)
        self.op("ST", "sn_s")
        self.op("ADDI", imm=dv)
        self.op("ST", "sn_vmax")
        self.op("CMPI", imm=mx + dv)
        self.op("LDI", imm=ONE)
        self.op("STF", "sn_sff")
        self.op("LD", "sn_s")
        self.op("CMPI", imm=mn + 1)
        self.op("LDI", imm=0)
        self.op("STNF", "sn_sff")
        st = self.k("SN_VCO_STEP")
        if self.tweak:                                # 1 / step for the crossing fraction
            def emit():
                self.op("LD", st)
                self.op("RCP")
                self.op("ST", "sn_ist")
            self._kemit(emit)
            ist = ("MUL", "sn_ist", 0)
        else:
            ist = ("MULI", 0, q(1.0 / self.f("SN_VCO_STEP")))
        self.flag01("sn_vff")
        at = self.skip_if(True)
        self.op("LD", "sn_v")                         # rising
        self.op("ADD", st)
        self.op("ST", "sn_v")
        self.op("CMP", "sn_vmax")
        e1 = self.skip_if(False)
        self.op("SUB", "sn_vmax")
        self.op(ist[0], ist[1], imm=ist[2])           # tau
        self.substeps("_ss")
        if self.sn_top:                               # vmax fell below last sample's v (SLF falling): the RTL clamps
            self.op("MINI", imm=ONE - (ONE >> 4))     # at its first substep, 15 substeps fall (not 16)
        self.op("ST", "sn_ov")                        # n / 16
        self.op("ADDI", imm=ONE >> 4)
        self.op("ST", "sn_sq")                        # high from the crossing substep: (n + 1) / 16
        self.op("LD", "sn_ov")
        self.op("MUL", st)
        self.op("SUB", "sn_vmax")
        self.op("MULI", imm=-ONE)
        self.op("ST", "sn_v")                         # vmax - n / 16 step
        self.op("LDI", imm=ONE)
        self.op("ST", "sn_vff")
        e2 = self.jump()
        self.land(e1)
        self.op("LDI", imm=0)
        self.op("ST", "sn_sq")
        e3 = self.jump()
        self.land(at)
        self.op("LD", "sn_v")                         # falling
        self.op("SUB", st)
        self.op("ST", "sn_v")
        self.op("CMPI", imm=mn + 1)
        e4 = self.skip_if(True)
        self.op("ADDI", imm=-mn)                      # <= 0
        self.op(ist[0], ist[1], imm=ist[2])
        self.op("MULI", imm=-ONE)                     # tau
        self.substeps("_ss")
        self.op("ST", "sn_ov")
        self.op("LDI", imm=ONE - (ONE >> 4))
        self.op("SUB", "sn_ov")
        self.op("ST", "sn_sq")                        # high before the crossing substep: 15/16 - n/16
        self.op("LD", "sn_ov")
        self.op("MUL", st)
        self.op("ADDI", imm=mn)
        self.op("ST", "sn_v")                         # mn + n / 16 step
        self.op("LDI", imm=0)
        self.op("ST", "sn_vff")
        e5 = self.jump()
        self.land(e4)
        self.op("LDI", imm=ONE)
        self.op("ST", "sn_sq")
        for e in (e2, e3, e5):
            self.land(e)
        lo, hi = self.v("SN_LO"), self.v("SN_HI")
        self.op("LD", "sn_sq")
        self.op("MULI", imm=hi - lo)
        self.op("ADDI", imm=lo)
        self.op("ST", "o_sn")
        self.op("BIT", 1, self.src(P3, 0))
        self.op("LDI", imm=self.v("SN_MID"))
        self.op("STF", "o_sn")

    def round_robin(self, blocks):
        """blocks[k] runs on samples k, k + 8, ...: a counter in steps of 2^29 (wraps by itself), binary dispatch;
        every leaf jumps straight to the end"""
        assert len(blocks) == 8
        self.op("LD", "_rr")
        self.op("ADDI", imm=1 << 29)
        self.op("ST", "_rr")
        vals = sorted(((k << 29) + (1 << 31)) % (1 << 32) - (1 << 31) for k in range(8))
        slot = {((k << 29) + (1 << 31)) % (1 << 32) - (1 << 31): k for k in range(8)}
        ends = []

        def empty(vs):
            return all(blocks[slot[v]] is None for v in vs)

        def node(vs):
            if len(vs) == 1:
                blocks[slot[vs[0]]]()
                ends.append(self.jump())
                return
            h = len(vs) // 2
            lo, hi = vs[:h], vs[h:]
            self.op("CMPI", imm=hi[0])
            if empty(hi):                             # an idle slot goes straight to the end
                ends.append(self.skip_if(True))
                node(lo)
            elif empty(lo):
                ends.append(self.skip_if(False))
                node(hi)
            else:
                at = self.skip_if(True)
                node(lo)
                self.land(at)
                node(hi)
        node(vals)
        del self.code[-2:]                            # the last leaf falls through to the end
        ends.pop()
        for e in ends:
            self.land(e)

    # --- the board ---------------------------------------------------------------------------------------------
    def build(self):
        VBE, VOH = self.v("VBE"), self.v("VOH")
        self.mark("noise")
        self.noise()

        self.mark("round robin")
        # the two LFOs (OSC_1, value held 8 samples): started 3.5 samples ahead, so the hold is centred on the RTL's
        # value and the VCOs' phase integrates the same over each hold
        # slot k runs at body samples n = k - 1 (mod 8); its first call steps 8 samples, so the state it holds over
        # n .. n + 7 is the RTL's at 7 + lead: lead = slot sample - 3.5 centres it
        for n, d1, k in (("sho", "SH_OSC_D1", 0), ("iho", "IH_OSC_D1", 4)):
            self.pinit.append((n + "_v", q(self.f(d1) * (((k - 1) % NRR) - (NRR - 1) / 2))))
            if self.grid:
                self.pinit.append((n[:2] + "v_v", self.rtl_vco1_start(n[:2].upper() + "_OSC", n[:2].upper() + "_VCO_KB",
                                                                      k)))
        if self.grid:                                 # LFO one slot earlier (slot 7 runs the sample before slot 0), the
            self.round_robin([                        # VCO steps / reciprocals in the LFO's old slot: same held values
                lambda: self.vco1_rr("shv", "sho_v", "SH_VCO_KA", "SH_VCO_KB"),
                lambda: self.bonus_timer(NRR),
                lambda: self.one_shot_c1("ihs", "IH_E1C", "IH_E1D", NRR),
                lambda: self.osc1("iho", "IH_OSC_D0", "IH_OSC_D1", steps=NRR),
                lambda: self.vco1_rr("ihv", "iho_v", "IH_VCO_KA", "IH_VCO_KB"),
                lambda: self.one_shot_c1("exs", "EX_E1C", "EX_E1D", NRR),
                lambda: self.one_shot_c1("mss", "MS_E1C", "MS_E1D", NRR),
                lambda: self.osc1("sho", "SH_OSC_D0", "SH_OSC_D1", steps=NRR),
            ])
        else:
            self.round_robin([
                lambda: self.osc1("sho", "SH_OSC_D0", "SH_OSC_D1", steps=NRR),
                lambda: self.bonus_timer(NRR),
                lambda: self.one_shot_c1("ihs", "IH_E1C", "IH_E1D", NRR),
                None,
                lambda: self.osc1("iho", "IH_OSC_D0", "IH_OSC_D1", steps=NRR),
                lambda: self.one_shot_c1("exs", "EX_E1C", "EX_E1D", NRR),
                lambda: self.one_shot_c1("mss", "MS_E1C", "MS_E1D", NRR),
                None,
            ])

        # ---- saucer hit (P5 D4): envelope, OSC_1 (round robin) -> VCO_1 square
        self.mark("saucer hit")
        kdn, kup = self.k("SH_ENV_DN"), self.k("SH_ENV_UP")

        def emit():
            self.op("LDI", imm=0)
            self.op("SUB", kdn)
            self.op("ST", "k_sh_dn")
            self.op("ADD", kup)
            self.op("ST", "k_sh_ud")
        self._kemit(emit)
        self.op("BIT", 0, self.src(P5, 4))
        self.op("LD", "k_sh_dn")
        self.op("LDF", "k_sh_ud")
        self.op("ADD", "sh_env")
        self.op("MAXI", imm=0)
        self.op("MINI", imm=VOH)
        self.op("ST", "sh_env")
        if self.grid:
            self.vco1_grid("shv", frac=True)
        else:
            self.vco1("shv", "sho_v", "SH_VCO_KA", "SH_VCO_KB", frac=True)
        self.op("LD", "shv_sq")                       # o_sh = clamp(pos(env - VBE) - pos(sq - VBE) - C, 0, VOH)
        self.op("MULI", imm=VOH)
        self.op("ADDI", imm=-VBE)
        self.op("MAXI", imm=0)
        self.op("ST", "sh_t")
        self.op("LD", "sh_env")
        self.op("SUB", "sh_t")
        self.op("ADDI", imm=-VBE - self.v("SH_OUT_C"))
        self.op("MAXI", imm=0)                        # (env < VBE: 0 either way; <= VOH - VBE: no top clamp)
        self.op("ST", "o_sh")

        # ---- fleet (P5 D0-D3): 555 (charge coefficient from the round robin), two RC low-passes
        self.mark("fleet")
        self.fleet_kc()
        self.ne555_fast("fl", self.k("FL_KD", raw=True))
        self.op("LD", "fl_lo")
        self.op("MULI", imm=-self.v("VTTL"))
        self.op("ADDI", imm=self.v("VTTL"))
        self.op("RCM", "fl_rc1", self.r(self.k("FL_RC1")))
        self.op("RCM", "o_fl", self.r(self.k("FL_RC2")))

        # ---- bonus missile base (P3 D4): 556 (round robin) output ANDed with the enable and 16V
        self.mark("bonus")
        bit = 3 if self.p.get("bn_8v") else 4         # 16V (PV: 8V): line count bit, sampled before the step
        if bit == 3:                                  # 8V: the rounded increment runs 0.2 LSB / sample slow, and
            # model_wav's line count hits an 8V edge exactly every 320 samples: start just under 1/40 line (the
            # smallest non-zero distance of a sample from an edge) ahead, so those edges land on the sample for
            # ~11 minutes. (16V: the increment rounds up, 0 is right for ~3 minutes)
            self.pinit.append(("v16_ph", (1 << 32) // (16 * 40) - 64))
        self.op("LD", "v16_ph")
        self.op("CMPI", imm=0)                        # F = !16V
        self.op("ADDI", imm=int(round(2 ** 32 * 15600 / FS / (2 << bit))))
        self.op("ST", "v16_ph")
        self.op("FOR", 1, self.src(P3, 4))            # | !enable
        self.op("LDI", imm=self.v("VTTL"))
        self.op("LDIF", imm=0)
        self.op("MACI", "bn_lo", imm=-self.v("VTTL"))
        self.op("MAXI", imm=0)                        # (bn_lo may lag a disable by up to 7 samples)
        self.op("ST", "o_bn")

        # ---- invader hit (P3 D3): one-shot, envelope, OSC_1 -> VCO_1 (cap voltage out)
        self.mark("invader hit")
        self.one_shot("ihs", (P3, 3))
        self.env("ih_env", "ihs_on", "IH_ENV_UP", "IH_ENV_DN")
        if self.grid:
            self.vco1_grid("ihv", frac=False)
        else:
            self.vco1("ihv", "iho_v", "IH_VCO_KA", "IH_VCO_KB", frac=False)
        g = self.f("IH_OUT_G")                        # o_ih = clamp(pos(env - VBE) - pos(v - VBE) G - C, 0, VOH)
        self.op("LD", "ihv_v")
        if self.grid:
            self.op("ABS")                            # (-v while discharging)
        self.op("MULI", imm=q(-g))
        self.op("ADD", "ih_env")
        self.op("ADDI", imm=q(g * 0.5) - VBE - self.v("IH_OUT_C"))
        self.op("MAXI", imm=0)
        self.op("ST", "o_ih")

        # ---- explosion (P3 D2): one-shot -> TVCA on the noise -> two RC low-passes
        self.mark("explosion")
        self.one_shot("exs", (P3, 2))
        self.flag01("exs_on")
        at = self.skip_if(True)
        self.op("LDI", imm=VBE)
        self.op("RCM", "ex_vc", self.r(self.k("EX_ED")))
        e = self.jump()
        self.land(at)
        self.op("LDI", imm=self.v("EX_VT"))
        self.op("RCM", "ex_vc", self.r(self.k("EX_EC")))
        self.land(e)
        self.op("MACI", "nz", imm=q(-(12.0 - SC.VBE) / 12.0))     # - pos(nz - VBE)
        self.op("ADDI", imm=-VBE - self.v("EX_C"))
        self.op("MAXI", imm=0)
        self.op("RCM", "ex_rc1", self.r(self.k("EX_RC1")))
        self.op("RCM", "o_ex", self.r(self.k("EX_RC2")))

        # ---- missile (P3 D1): one-shot envelope -> VCO_3 (noise + envelope modulated) -> TVCA
        self.mark("missile")
        self.one_shot("mss", (P3, 1))
        self.env("ms_env", "mss_on", "MS_ENV_UP", "MS_ENV_DN")
        self.op("ADDI", imm=-VBE)
        self.op("MAXI", imm=0)
        self.op("ST", "m73")                          # clamp(env - VBE, 0, 12)
        # (RTL: each floored product makes the substep step 0.5 Q20 LSB short on average = 128 Q24 LSB per sample:
        # the K6 term always, the K1 term while it is non-zero - folded into its input offset)
        self.op("ADDI", imm=-VBE)
        self.op("MULI", imm=-self.v("MS_VCO_K6"))
        self.op("ADDI", imm=-self.v("MS_VCO_IF") + 128)
        self.op("ST", "md0n")
        self.op("LD", "nz")
        self.op("SUB", "ms_cr")
        self.op("MULI", imm=self.v("MS_NZ_G"))
        self.op("ADDI", imm=-VBE - q(128 / ONE / self.f("MS_VCO_K1")))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=-self.v("MS_VCO_K1"))
        self.op("ADD", "md0n")
        self.op("ST", "md0n")                         # - discharge step
        self.op("ADDI", imm=self.v("MS_VCO_T1"))
        self.op("ST", "md1")                          # charge step
        self.op("LD", "nz")
        self.op("RCM", "ms_cr", self.r(self.k("MS_CR")))
        self.vco3_ms()
        self.op("LD", "m73")                          # -pos(m77 - VBE) = clamp(pos(sq - VBE) - a, -(VOH - VBE), 0),
        self.op("ADDI", imm=-VBE)                     # a = pos(m73 - VBE) A3_G - A3_C - VBE
        self.op("MAXI", imm=0)
        self.op("MULI", imm=self.v("MS_A3_G"))
        self.op("ADDI", imm=-self.v("MS_A3_C") - VBE)
        self.op("ST", "m77")
        self.op("LD", "msv_sq")
        self.op("MULI", imm=VOH)
        self.op("ADDI", imm=-VBE)
        self.op("MAXI", imm=0)
        self.op("SUB", "m77")
        self.op("MINI", imm=0)
        self.op("MAXI", imm=VBE - VOH)
        self.op("ST", "m77")
        self.op("BIT", 0, self.src(P3, 1))
        at = self.skip_if(True)
        self.op("LDI", imm=VBE)
        self.op("RCM", "ms_vc", self.r(self.k("MS_ED")))
        e = self.jump()
        self.land(at)
        self.op("LDI", imm=self.v("MS_VT"))
        self.op("RCM", "ms_vc", self.r(self.k("MS_EC")))
        self.land(e)
        self.op("ADD", "m77")                         # o_ms = clamp(pos(vc - VBE) - pos(m77 - VBE) - C, 0, VOH)
        self.op("ADDI", imm=-VBE - self.v("MS_C"))
        self.op("MAXI", imm=0)
        self.op("ST", "o_ms")

        # ---- SN76477 saucer (P3 D0 = run)
        self.mark("SN76477")
        self.sn76477()

        # ---- summing amplifier (signal kept negated after C44): C11 on the missile input, R105 || volume, C44
        # coupling, (C37), LM3900 rails, C45, OUT_G, P35 mute
        self.mark("summer")
        self.op("LD", "o_ms")
        self.op("RCM", "mx_c11", self.r(self.k("MX_C11")))
        self.op("SUB", "o_ms")
        self.op("ST", "mx_t")                         # -(o_ms - c11)
        self.op("LD", "o_sh")
        self.op("MULI", imm=-self.v("G_SH"))
        for src, gn in (("o_fl", "G_FL"), ("o_bn", "G_BN"), ("o_ih", "G_IH"), ("o_ex", "G_EX"), ("mx_t", "G_MS"),
                        ("o_sn", "G_SN")):
            self.op("MACI", src, imm=(1 if src == "mx_t" else -1) * self.v(gn))
        self.op("ST", "mx_v")                         # v
        if self.p["clamp"]:
            self.op("RCM", "mx_c44", self.r(self.k("MX_C44")))
            self.op("SUB", "mx_v")                    # -(v - c44)
            if self.p["c37"]:
                self.op("RCM", "mx_lp", self.r(self.k("MX_LP")))
            self.op("MAXI", imm=-self.v("MX_VHI"))
            self.op("MINI", imm=-self.v("MX_VLO"))
        else:
            self.op("MULI", imm=-ONE)
        self.op("ST", "mx_v")                         # -x
        self.op("RCM", "mx_amp", self.r(self.k("MX_CAMP")))
        self.op("SUB", "mx_v")                        # x - amp
        self.op("MULI", imm=q(self.p["out"] / 32768))   # OUT_G counts / V; engine 1.0 = 32768
        self.op("BIT", 1, self.src(P3, 5))            # P35 low: amplifier muted
        self.op("LDIF", imm=0)
        self.op("OUT")
        self.mark("end")
        self.breakdown = self.segments()
        return self.finish()

    def segments(self):
        """worst-case budget units per block of the body (blocks are single entry / single exit)"""
        code = self.code
        self.code = code + [(0, 0, 0, 0)]
        tail = {}
        for _, i in self.marks:
            full, self.code = self.code, self.code[i:]
            tail[i] = self.worst_path()
            self.code = full
        self.code = code
        return [(a, tail[i] - tail[j]) for (a, i), (_, j) in zip(self.marks, self.marks[1:])]

    def gfactor(self, reg, src, mask, sh, aged=False):
        """reg = 1 / capacitance scale for the setting index i (Board.SCALES: i <= 3 -> 1 - 0.1 i, else 0.7 + 0.1 i;
        Aged Caps: 1 - 0.15 i), as Board.chain's table but computed (fewer words)"""
        assert self.SCALES == (1.0, 0.9, 0.8, 0.7, 1.1, 1.2, 1.3) and self.AGED == (1.0, 0.85, 0.7)
        self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))     # i << 16
        self.op("MULI", imm=q((-0.15 if aged else -0.1) * 256))
        self.op("ADDI", imm=ONE)
        if not aged:
            self.op("ST", "_gs")
            self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))
            self.op("CMPI", imm=4 << 16)
            self.op("MULI", imm=q(0.1 * 256))
            self.op("ADDI", imm=q(0.7))
            self.op("STF", "_gs")
            self.op("LD", "_gs")
        self.op("RCP")
        self.op("ST", reg)

    def finish(self):
        """Board.finish without an init block (every register powers up 0 = reset state). Game Audio: key = settings
        + 1 against _glast (powers up 0); a change runs the settings block in two silent samples: part 1 (group
        factors + first half of the scaled constants) leaves _glast = key + 1 - 2^20, part 2 (the rest) sets key + 1"""
        assert not self.inits and (self.kcode or not self.pinit)
        body, self.code = self.code, []
        if self.kcode:
            ph2 = 1 << 20

            def key():
                self.op("LDLM", 0, self.src(6, 0), imm=0xFF)
                self.op("ST", "_t")
                self.op("LDLM", 0, self.src(7, 0), imm=0xFF | (8 << 8))
                self.op("ADD", "_t")
                self.op("ADDI", imm=1)
            key()
            self.op("SUB", "_glast")
            self.op("ABS")
            self.op("CMPI", imm=1)
            at = self.skip_if(False)
            self.op("CMPI", imm=ph2)                  # |diff| == 2^20: part 2
            p1 = self.skip_if(False)
            self.op("CMPI", imm=ph2 + 1)
            p1b = self.skip_if(True)
            half = min(self.kcuts, key=lambda c: abs(2 * c - len(self.kcode)))    # (acc does not survive a sample)
            assert all(o not in (OPS["SKF"], OPS["SKNF"]) for o, *_ in self.kcode)
            self.code += self.kcode[half:]
            key()
            self.op("ST", "_glast")
            self.op("END")
            self.land(p1)
            self.land(p1b)
            for g in sorted(self.groups):
                self.gfactor("_f" + g, *self.GROUP_SRC[g])
            aged = sorted(self.groups & {"T", "F"})
            if aged:
                self.gfactor("_fA", 7, 0x18, 13, aged=True)
                for g in aged:
                    self.op("LD", "_f" + g)
                    self.op("MUL", "_fA")
                    self.op("ST", "_f" + g)
            self.code += self.kcode[:half]
            if self.pinit:                            # power-up only (_glast == 0): initial states
                self.op("LD", "_glast")
                self.op("CMPI", imm=1)
                q1 = self.skip_if(True)
                self.op("CMPI", imm=0)
                q2 = self.skip_if(False)
                for reg, val in self.pinit:
                    assert INT_MIN <= val < (1 << 31), (reg, val)
                    self.op("LDI", imm=val)
                    self.op("ST", reg)
                self.land(q1)
                self.land(q2)
            key()
            self.op("ADDI", imm=-ph2)
            self.op("ST", "_glast")
            self.op("END")                            # a settings change costs two (silent) samples
            self.land(at)
        self.code += body
        self.op("END")
        worst = self.worst_path()
        assert worst <= 410, f"{self.name}: a sample can use {worst} budget units (engine budget 410)"
        return self


def invaders(board="MIDWAY", tweak=True, grid=None, sn_top=None):
    return Inv(board, tweak, grid, sn_top).build()


def report(b):
    print(f"{b.board}: {len(b.code)} words (1024), worst sample {b.worst_path()} / 410 units, {len(b.regs)} registers")
    print("  body blocks:", ", ".join(f"{a} {u}" for a, u in b.breakdown))


if __name__ == "__main__":
    bd = sys.argv[1].upper() if len(sys.argv) > 1 else "MIDWAY"
    b = Inv(bd)
    try:
        b.build()
    except AssertionError as e:
        print(e)
        b.marks and print("  body blocks:", ", ".join(f"{a} {u}" for a, u in b.breakdown))
        sys.exit(1)
    out = sys.argv[2] if len(sys.argv) > 2 else f"invaders_{bd.lower()}.dsnd.bin"
    open(out, "wb").write(b.binary())
    report(b)
