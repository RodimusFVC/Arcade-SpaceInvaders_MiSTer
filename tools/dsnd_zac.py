#!/usr/bin/env python3
"""Zaccaria 1B1120 sound board (The Invaders tinv2650 / sia2650, Dodgem) as a discrete sound engine program
(master tools/dsnd.py, rtl/rodimus/dsnd_engine/dsnd_engine.sv).

A port of rtl/zac_snd.sv (constants tools/zac_snd_consts.py): SN76477 with latch-switched components (one-shot,
attack / decay, SLF, VCO, noise generator + noise filter, mixer), the 555 VCO sweep (DB3) and the S2636 tone through
its filter. zac_snd updates 4 times per 48 kHz sample (192 kHz); every part here advances those 4 updates per
sample with zac_snd's own Q24 integers (rates per update, the same truncating multiplies): linear ramps and RC
steps unrolled or in closed form, turns / threshold crossings placed on their update (fix-up paths), the VCO's top
following the SLF (or the 555) update by update, the noise generator's fractional 192 kHz clock and the noise
filter per update, the tone filter (2 RC + coupling) per update. A/B against zac_snd:
/Work/Projects/verilator_local_files/Arcade-SpaceInvaders_MiSTer/dsnd_zac_ab/.

Programs: zac() = The Invaders (default: tone = the board's bit "raw", exact 555, the engine's LFSR with the SN76477
lock-up guard = zac_snd's noise bit for bit); zac(dodgem=True) = Dodgem: tone path only (its board never writes the
1E80 latch), 14.318 MHz line clock. Chosen by ear 2026-11-03 (raw tone and the exact noise were the closest).
Alternatives: pvi="gens" (tone generated from C7; 555 turn on the sample grid, exact555 does not fit next to it),
noise="exact" (the LFSR in program code; fits only without the tone path, pvi="none": validation builds).

Format differences against zac_snd (not circuit):
  * tone, "gens": generated from C7 (LATCH2) on the board's line clock (master / 908, free-running from power-up as
    the board's raster counter); a toggle lands on the first update of the sample holding its line event (up to 3
    updates early). "raw": source 5 bit 2 as read at the sample (up to 4 updates late). The update-exact version
    does not fit (words).
  * noise generator: the engine's LFSR with the lock-up guard (NOISE b[2]): taps 28 / 0 and the guard as zac_snd;
    seed 1 against zac_snd's 0 reaches the same state (1 << 30, new bit 1) on the first step, so the sequence is
    the same.
  * 555 5 V turn (default): at the end of its sample (up to 3 updates late per period: the free-running sweep's
    phase drifts against zac_snd's). exact555: on its update (no room next to "gens").
  * output gain index (attack / decay level, 45 steps): once per sample (after its 4 updates).
  * one-shot end (A/D decay start): at the end of the sample in which the one-shot cap reaches 2.5 V (the A/D cap
    is at its 4.44 V limit by then for every latch value).
  * VCO saturation mute (DB3: the 555 can push the VCO above 2.72 V): decided per sample by the last update.
  * VCO top in the <= 3 updates after an SLF turn keeps the SLF's limit (exact otherwise).
  * the noise filter only runs while the mixer selects noise (held otherwise; the games enter noise mode with the
    SN76477 disabled first, and the filter converges within a few updates).
  * power-up: zac_snd's VCO turns several times per sample while the SLF climbs from 0 V; the program holds it at
    0.33 V rising until zac_snd's is there too (vco_settle: Factory Game Audio, latch 0 at power-up).
  * Game Audio: group factors computed (RCP; Factory exactly 1.0, other settings within 1 LSB of zac_snd's integer
    tables); a settings change costs one silent sample (the free-running parts skip those 4 updates) and node 1 of
    the tone keeps its old coefficient until the next tone change.
  * zac_snd clips each update's mix to 16 bits; here the 4-update average is clipped.
Output = floor(sum of zac_snd's 4 outputs in the sample / 4), zac_snd's own 16-bit scale.

Sources: LATCH1 = 1E80 latch (bit 0 enable / one-shot, 1-5 component switches, 7 mixer noise else VCO; bit 6 not on
the board), LATCH2 = S2636 C7, MISC (source 5) bit 2 = the raw tone bit (pvi="raw" only). Game Audio = sources 6 / 7
exactly as zac_snd `tweak` = status[48:35] {music, aged, oscillator, filter, timing}: source 6 bits 2:0 timing (T),
5:3 filter (F); source 7 bits 2:0 oscillator (O), 4:3 Aged Caps (on T and F), 7:5 Music Volume (the tone level,
zac_snd's integer gains). zac_snd's groups: T = one-shot, A/D; F = noise filter, tone filter; O = SLF, VCO, 555,
noise clock.

    python3 dsnd_zac.py [zac | dodgem] [out.bin] [--gens] [--exact-noise --no-tone]   -> binary; size, worst case
"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dsnd_rate  # noqa: F401,E402  master assembler + this core's FS, before any dsnd import
from dsnd import Board, LATCH1, LATCH2, MISC, ONE             # noqa: E402
import zac_snd_consts as Z                                      # noqa: E402

INT_MIN = -(1 << 31)
L1 = LATCH1
HALF = ONE >> 1

# zac_snd thresholds (Q24, exactly as zac_snd_consts.py emits them)
OSMAX, SMIN, SMAX = Z.q(Z.ONE_SHOT_MAX), Z.q(Z.SLF_MIN), Z.q(Z.SLF_MAX)
DIFF, VMIN, VMAX = Z.q(Z.VCO_DIFF), Z.q(Z.VCO_MIN), Z.q(Z.VCO_MAX)
NMAX, NHI, NLO = Z.q(Z.NOISE_MAX), Z.q(Z.NOISE_HI), Z.q(Z.NOISE_LO)
ADMAX = Z.q(Z.AD_MAX)
V555, V5LO, V5HI = Z.q(Z.V555), Z.q(Z.V555 / 3), Z.q(2 * Z.V555 / 3)
PLO, PHI = Z.q(5.0 * 100 / (100 + 150)), Z.q(5.0)
PLO_ = PLO - PHI                                                # tone path registers hold volts - 5 V (power-up 0)
FS192 = int(Z.FS)
# Game Audio music gains (P1 x 1 / 0.7 / 0.5 / 1.4 / 2, integers as zac_snd)
MUSIC_G = [Z.PVI_GAIN, Z.PVI_GAIN * 7 // 10, Z.PVI_GAIN // 2, Z.PVI_GAIN * 14 // 10, Z.PVI_GAIN * 2]
GSRC = {"T": (6, 0x07, 16), "F": (6, 0x38, 13), "O": (7, 0x07, 16)}
# Board line clock (rtl/zac1b1120_board.sv): master clock enable = step / 39936 per 39.936 MHz clock, the tone
# counter acts on the enable with h == 0 (master clock k = 1 + 908 L, from power-up); zac_snd updates on clock
# 208 n + 207 and sees event k iff 39936 k <= step (208 n + 207).
LINE_L = 39936 * 908


def vco_settle():
    """zac_snd at power-up (latch 0, Factory): the SLF rises from 0 V, so for the first ~20 samples the VCO's top is
    within a few steps of its bottom and it turns several times per sample. Returns S0 = the SLF value after the
    first sample k whose predecessor ends with the VCO at 0.33 V rising and after which no sample holds two turns:
    the program holds its VCO there while the SLF is below S0 (zac_snd's own update rule, simulated here)"""
    slf, sff, v, ff = 0, 0, 0, 0
    st, up = Z.VCO_STEP[0], Z.SLF_CHG[0]
    ends = []                                         # per sample: (v, ff, slf, turns)
    for k in range(400):
        turns = 0
        for _ in range(4):
            slf = min(slf + up, SMAX) if not sff else slf
            vmax = slf + DIFF
            n = min(v + st, vmax) if not ff else max(v - st, VMIN)
            nff = 1 if n >= vmax else 0 if n <= VMIN else ff
            turns += nff != ff
            v, ff = n, nff
        ends.append((v, ff, slf, turns))
    last = max(k for k, e in enumerate(ends) if e[3] > 1)
    k0 = next(k for k in range(last + 1, 400) if ends[k][:2] == (VMIN, 0))
    assert all(e[3] <= 1 for e in ends[k0:])
    return ends[k0 + 1][2]


class Zac(Board):
    def __init__(self, dodgem=False, pvi="raw", noise="engine", exact555=None):
        super().__init__("zac_dodgem" if dodgem else "zac")
        assert pvi in ("gens", "raw", "none") and noise in ("engine", "exact")
        self.dodgem, self.pvi, self.noise = dodgem, pvi, noise
        self.exact555 = (pvi != "gens") if exact555 is None else exact555   # (does not fit next to "gens")
        self.marks, self.pinit, self.kregs = [], [], {}
        self.mstep = 14318 if dodgem else 15625

    # --- small patterns ----------------------------------------------------------------------------------------
    def mark(self, name):
        self.marks.append((name, len(self.code)))

    def jump(self):
        self.op("CMPI", imm=INT_MIN)
        return self.skip_if(True)

    def flag(self, reg):
        self.op("LD", reg)
        self.op("CMPI", imm=HALF)

    def k(self, reg, kb, g, cap=False, neg=False):
        """register = zac_snd's rate kb x the group factor (Factory: kb exactly), capped at 1.0 for its RC
        coefficients as zac_snd; neg: stored negated. Built in the settings block"""
        self.kregs.setdefault(reg, (kb, g, cap, neg))
        return reg

    def chain_raw(self, reg, src, mask, sh, values, default=None):
        """reg = values[index] (raw integers), index = (source byte & mask) >> ..; index len(values): default"""
        self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))
        self.op("ST", "_gv")
        self.op("LDI", imm=values[0])
        self.op("ST", reg)
        for i, v in list(enumerate(values))[1:] + ([(len(values), default)] if default is not None else []):
            self.op("LD", "_gv")
            self.op("CMPI", imm=i << 16)
            self.op("LDI", imm=v)
            self.op("STF", reg)

    # --- latch decode ------------------------------------------------------------------------------------------
    def decode(self):
        """oc4 / add4 = 4 updates of one-shot charge (R by DB2 | DB3, C by DB2, DB1 | DB5) / A/D decay (DB2, DB3,
        DB1 | DB5),
        nfc / nfn = noise filter charge / -discharge (R by DB2 | DB3, C by DB4), su / sdn = SLF charge / -discharge
        (C by DB1)"""
        oc = [self.k(f"k_oc{i}", 4 * Z.OS_CHG[i], "T") if i not in (2, 3) else None for i in range(8)]
        ad = [self.k(f"k_ad{i}", 4 * Z.AD_DIS[i], "T") for i in range(8)]
        nc = [self.k(f"k_nc{i}", Z.NF_CHG[i], "F") for i in range(4)]
        nn = [self.k(f"k_nn{i}", Z.NF_DIS[i], "F", neg=True) for i in range(4)]
        def sel(regs, i, out, b15=True):
            if b15:
                self.op("BIT", 0, self.src(L1, 1))
                self.op("FOR", 0, self.src(L1, 5))   # F = DB1 | DB5
            self.op("LD", regs[i])
            self.op("LDF", regs[i + 1])
            self.op("ST", out)

        def nfsel(i):
            self.op("BIT", 0, self.src(L1, 4))
            sel(nc, i, "nfc", False)
            sel(nn, i, "nfn", False)
        ends = []
        self.op("BIT", 0, self.src(L1, 2))
        self.op("FOR", 0, self.src(L1, 3))
        b23 = self.skip_if(True)
        sel(oc, 0, "oc4")                             # DB2 = DB3 = 0
        sel(ad, 0, "add4", False)
        nfsel(0)
        ends.append(self.jump())
        self.land(b23)
        nfsel(2)                                      # DB2 | DB3
        self.op("BIT", 0, self.src(L1, 2))
        b2 = self.skip_if(True)
        sel(oc, 4, "oc4")                             # DB3 only
        sel(ad, 2, "add4", False)
        ends.append(self.jump())
        self.land(b2)
        sel(oc, 6, "oc4")                             # DB2 (DB3 picks the decay)
        self.op("BIT", 0, self.src(L1, 3))
        b3 = self.skip_if(True)
        sel(ad, 4, "add4")
        ends.append(self.jump())
        self.land(b3)
        sel(ad, 6, "add4")
        for e in ends:
            self.land(e)
        su = [self.k(f"k_su{i}", Z.SLF_CHG[i], "O") for i in range(2)]
        sn = [self.k(f"k_sn{i}", Z.SLF_DIS[i], "O", neg=True) for i in range(2)]
        self.op("BIT", 0, self.src(L1, 1))
        for sel, reg in ((su, "su"), (sn, "sdn")):
            self.op("LD", sel[0])
            self.op("LDF", sel[1])
            self.op("ST", reg)
        self.op("LD", "sr")                           # the SLF's signed rate follows DB1 (power-up 0: rising)
        self.op("CMPI", imm=0)
        self.op("LD", "sdn")
        self.op("LDF", "su")
        self.op("ST", "sr")

    # --- one-shot, attack / decay, output gains ----------------------------------------------------------------
    def envelope(self):
        """en = DB0, start = en rising (first update of the sample: one-shot and A/D restart from 0 V). While the
        one-shot runs the A/D cap charges (4 updates per sample), else it decays; the one-shot run ends at the end
        of the sample in which its cap reaches 2.5 V (format: the decay starts up to 4 updates late; the A/D is at
        its 4.44 V limit by then for every latch value). Disabled: one-shot off, the A/D cap is not needed (the
        next enable restarts it). Then the gain index -> gd = OUT_P - OUT_N, gn = OUT_N"""
        adc = self.k("k_adc", 4 * Z.AD_CHG, "T")
        self.op("BIT", 0, self.src(L1, 0))
        on = self.skip_if(True)
        self.op("LDI", imm=0)
        self.op("ST", "en_d")
        self.op("ST", "osr")
        off = self.jump()
        self.land(on)
        self.flag("en_d")
        a = self.skip_if(True)                        # enabled before: no start
        self.op("LDI", imm=0)
        self.op("ST", "os")
        self.op("ST", "ad")
        self.op("LDI", imm=ONE)
        self.op("ST", "osr")
        self.op("ST", "en_d")
        self.land(a)
        self.flag("osr")
        idle = self.skip_if(False)
        self.op("LD", "os")
        self.op("ADD", "oc4")
        self.op("ST", "os")
        self.op("CMPI", imm=OSMAX)
        self.op("LDI", imm=0)
        self.op("STF", "osr")
        self.op("LD", "ad")
        self.op("ADD", adc)
        self.op("MINI", imm=ADMAX)
        g1 = self.jump()
        self.land(idle)
        self.op("LD", "ad")
        self.op("SUB", "add4")
        self.op("MAXI", imm=0)
        self.land(g1)
        self.op("ST", "ad")
        # gain index floor(10 x A/D); pairs of indices share a leaf (LDI / LDIF on the last compare)
        self.op("MULI", imm=10 * ONE)
        self.op("MULI", imm=1)
        ent = [0] + list(range(9, 45))               # 0: indices 0 .. 8 (no output)
        groups = [ent[i:i + 2] for i in range(0, len(ent), 2)]
        tails = []

        def tree(gs):
            if len(gs) == 1:
                g = gs[0]
                if len(g) == 2:
                    self.op("CMPI", imm=g[1])
                    self.op("LDI", imm=Z.OUT_P[g[0]] - Z.OUT_N[g[0]])
                    self.op("LDIF", imm=Z.OUT_P[g[1]] - Z.OUT_N[g[1]])
                    self.op("ST", "gd")
                    self.op("LDI", imm=Z.OUT_N[g[0]])
                    self.op("LDIF", imm=Z.OUT_N[g[1]])
                else:
                    self.op("LDI", imm=Z.OUT_P[g[0]] - Z.OUT_N[g[0]])
                    self.op("ST", "gd")
                    self.op("LDI", imm=Z.OUT_N[g[0]])
                tails.append(self.jump())
                return
            h = len(gs) // 2
            self.op("CMPI", imm=gs[h][0])
            at = self.skip_if(True)
            tree(gs[:h])
            self.land(at)
            tree(gs[h:])
        tree(groups)
        del self.code[-2:]
        tails.pop()
        for t in tails:
            self.land(t)
        self.op("ST", "gn")
        self.land(off)

    # --- SLF ---------------------------------------------------------------------------------------------------
    def tops(self):
        """m1 .. m4 = the VCO's top at updates 1 .. 4: 0.35 V + (DB3: the 555's max(w_(u-1) - 2.5 V, 0), else the
        SLF after update u (from the sample's start value along its signed rate, clamped at the limit it runs to:
        exact but for the <= 3 updates after an SLF turn, which keep the limit; format); st = the VCO step (C by DB3)"""
        vs0 = self.k("k_vs0", Z.VCO_STEP[0], "O")
        vs1 = self.k("k_vs1", Z.VCO_STEP[1], "O")
        self.op("BIT", 0, self.src(L1, 3))
        b3 = self.skip_if(True)
        self.op("LD", vs0)
        self.op("ST", "st")
        self.op("LD", "sr")
        self.op("CMPI", imm=0)
        fall = self.skip_if(False)
        e = []
        for up in (True, False):
            if not up:
                self.land(fall)
            self.op("LD", "s")
            self.op("ADDI", imm=DIFF)
            for u in range(1, 5):
                self.op("ADD", "sr")
                self.op("MINI" if up else "MAXI", imm=(SMAX if up else SMIN) + DIFF)
                self.op("ST", f"m{u}")
            e.append(self.jump())
        self.land(b3)
        for u in range(1, 5):
            self.op("LD", f"w{u - 1}")
            self.op("ADDI", imm=DIFF - V5LO)
            self.op("MAXI", imm=DIFF)
            self.op("ST", f"m{u}")
        self.op("LD", vs1)
        self.op("ST", "st")
        for x in e:
            self.land(x)

    def slf(self):
        """triangle 0.33 .. 2.37 V, signed rate sr (+su rising, sdn = -sd falling) per update, clamped at the
        turning limit (power-up from 0 V rising: no low clamp while rising); the turn lands on its update and the
        rest of the sample runs the other way"""
        ends = []
        self.op("LD", "sr")
        self.op("CMPI", imm=0)
        fall = self.skip_if(False)
        for up in (True, False):
            if not up:
                self.land(fall)
            lim, other = (SMAX, "sdn") if up else (SMIN, "su")
            imm = SMAX if up else SMIN + 1            # rising: F = reached; falling: NOT F = reached
            self.op("LD", "sr")
            self.op("MULI", imm=4 * ONE)
            self.op("ADD", "s")
            self.op("CMPI", imm=imm)
            turn = self.skip_if(up)
            self.op("ST", "s")
            ends.append(self.jump())
            self.land(turn)
            self.op("LD", "s")
            go = []
            for u in range(1, 4):
                self.op("ADD", "sr")
                self.op("CMPI", imm=imm)
                go.append(self.skip_if(up))
            tl = []
            for u in (4, 1, 2):                       # updates left after the turn -> the shared tail
                if u < 4:
                    self.land(go[u - 1])
                self.op("LDI", imm=(4 - u) * ONE)
                tl.append(self.jump())
            self.land(go[2])
            self.op("LDI", imm=ONE)
            for t in tl:
                self.land(t)
            self.op("MUL", other)
            self.op("ADDI", imm=lim)
            self.op("ST", "s")
            self.op("LD", other)
            self.op("ST", "sr")
            if up:
                ends.append(self.jump())
        for e in ends:
            self.land(e)

    # --- 555 ---------------------------------------------------------------------------------------------------
    def ne555(self):
        """CI5: charge toward 7.5 V (181K), discharge (1K), thresholds 2.5 / 5 V, zac_snd's truncating steps.
        Discharging, the working register v5 holds -v (an RC step toward 0 then truncates as zac_snd's
        v - mulq(v, k)). w0 .. w3 = v before updates 1 .. 4 (the VCO's top with DB3). The 2.5 V crossing lands on
        its update (the rest of the sample charges; w after it keeps the discharge values: the DB3 top for those
        <= 3 updates sits at 0 V instead of +0.1 mV). The 5 V crossing: exact555 -> on its update as well (w after
        it keeps the charge values); else at the end of its sample (format: up to 3 updates late each period, so the
        free-running sweep's phase drifts against zac_snd's, period +0.006 % on average; words)"""
        k5c = self.k("k_5c", Z.K555_CHG, "O", cap=True)
        k5d = self.k("k_5d", Z.K555_DIS, "O", cap=True)
        ends = []
        self.flag("d5")
        dis = self.skip_if(True)

        def step(charge, store):
            self.op("LDI", imm=V555 if charge else 0)
            self.op("RCM", "v5", self.r(k5c if charge else k5d))
            if store:
                if not charge:
                    self.op("MULI", imm=-ONE)
                self.op("ST", store)
        self.op("LD", "v5")
        self.op("ST", "w0")
        for u in range(1, 5):
            step(True, f"w{u}" if u < 4 else None)
        self.op("CMPI", imm=V5HI)
        ends.append(self.skip_if(False))
        self.op("LDI", imm=ONE)
        self.op("ST", "d5")
        if self.exact555:                             # the 5 V crossing on its update too
            go = []
            for u in range(1, 4):
                self.op("LD", f"w{u}")
                self.op("CMPI", imm=V5HI)
                go.append(self.skip_if(True))
        self.op("LD", "v5")
        self.op("MULI", imm=-ONE)
        self.op("ST", "v5")
        ends.append(self.jump())
        if self.exact555:
            ent = {}
            for u in (3, 2, 1):                       # restart from -w_u, then 4 - u discharge steps
                self.land(go[u - 1])
                self.op("LD", f"w{u}")
                self.op("MULI", imm=-ONE)
                self.op("ST", "v5")
                if u > 1:
                    ent[u] = self.jump()
            for n in (3, 2, 1):
                if n < 3:
                    self.land(ent[4 - n])
                step(False, None)
            ends.append(self.jump())
        self.land(dis)
        self.op("LD", "v5")
        self.op("MULI", imm=-ONE)
        self.op("ST", "w0")
        for u in range(1, 5):
            step(False, f"w{u}" if u < 4 else None)
        self.op("CMPI", imm=-V5LO)                    # -v >= -2.5 V: reached
        ends.append(self.skip_if(False))
        self.op("LDI", imm=0)
        self.op("ST", "d5")
        go = []
        for u in range(1, 4):
            self.op("LD", f"w{u}")
            self.op("CMPI", imm=V5LO + 1)
            go.append(self.skip_if(False))
        self.op("LD", "v5")                           # reached on update 4: the register changes sign
        self.op("MULI", imm=-ONE)
        self.op("ST", "v5")
        ends.append(self.jump())
        ent = {}
        for u in (3, 2):                              # restart from w_u, then 4 - u charge steps (fall-through)
            self.land(go[u - 1])
            self.op("LD", f"w{u}")
            self.op("ST", "v5")
            ent[u] = self.jump()
        self.land(go[0])
        self.op("LD", "w1")
        self.op("ST", "v5")
        for n in (3, 2, 1):
            if n < 3:
                self.land(ent[4 - n])
            step(True, None)
        for e in ends:
            self.land(e)

    # --- VCO ---------------------------------------------------------------------------------------------------
    def vco(self):
        """VCO v (volts): rising by st, clamped to the top m_u where it turns, falling (unclamped by the top) to
        0.33 V. At most one turn per sample once the SLF is above 0.33 V (half period >= 5 updates; DB3: >= 5 with the
        top at 0.35 V and the step x 1 / 0.7); the turn is found on its update (while rising the top moves less than
        a step per update, so the update-4 test decides). H = updates with the output high (falling). Mute (VCO >
        2.72 V, only reachable with DB3): G = 0 / 4 by the sample's last update. Power-up hold: vco_settle"""
        ends = []
        self.op("LD", "s")                            # power-up: held at 0.33 V rising (see vco_settle)
        self.op("CMPI", imm=vco_settle())
        run = self.skip_if(True)
        self.op("LDI", imm=VMIN)                      # (vd, H: still 0 from power-up)
        self.op("ST", "v")
        ends.append(self.jump())
        self.land(run)
        self.flag("vd")
        fall = self.skip_if(True)
        self.op("LD", "st")
        self.op("MULI", imm=4 * ONE)
        self.op("ADD", "v")
        self.op("CMP", "m4")
        turn = self.skip_if(True)
        self.op("ST", "v")
        self.op("LDI", imm=0)
        self.op("ST", "H")
        ends.append(self.jump())
        self.land(turn)
        self.op("LDI", imm=ONE)
        self.op("ST", "vd")
        self.op("LD", "v")
        go = []
        for u in range(1, 4):
            self.op("ADD", "st")
            self.op("CMP", f"m{u}")
            go.append(self.skip_if(True))
        tl = []
        for u in (4, 1, 2):                           # top on update u: v = m_u - (4 - u) st, H = 5 - u
            if u < 4:
                self.land(go[u - 1])
            self.op("LD", f"m{u}")
            self.op("ST", "_mt")
            self.op("LDI", imm=(u - 4) * ONE)
            tl.append(self.jump())
        self.land(go[2])
        self.op("LD", "m3")
        self.op("ST", "_mt")
        self.op("LDI", imm=-ONE)
        for t in tl:
            self.land(t)
        self.op("ST", "_k")
        self.op("MUL", "st")
        self.op("ADD", "_mt")
        self.op("ST", "v")
        self.op("LDI", imm=ONE)
        self.op("SUB", "_k")
        self.op("ST", "H")
        ends.append(self.jump())
        self.land(fall)
        self.op("LD", "st")
        self.op("MULI", imm=-4 * ONE)
        self.op("ADD", "v")
        self.op("CMPI", imm=VMIN + 1)
        hit = self.skip_if(False)
        self.op("ST", "v")
        self.op("LDI", imm=4 * ONE)
        self.op("ST", "H")
        ends.append(self.jump())
        self.land(hit)
        self.op("LDI", imm=0)
        self.op("ST", "vd")
        self.op("LD", "v")
        go = []
        for u in range(1, 4):
            self.op("SUB", "st")
            self.op("CMPI", imm=VMIN + 1)
            go.append(self.skip_if(False))
        tl = []
        for u in (4, 1, 2):                           # low from update u: v = 0.33 V + (4 - u) st, H = u - 1
            if u < 4:
                self.land(go[u - 1])
            self.op("LDI", imm=(4 - u) * ONE)
            tl.append(self.jump())
        self.land(go[2])
        self.op("LDI", imm=ONE)
        for t in tl:
            self.land(t)
        self.op("ST", "_k")
        self.op("MUL", "st")
        self.op("ADDI", imm=VMIN)
        self.op("ST", "v")
        self.op("LDI", imm=3 * ONE)
        self.op("SUB", "_k")
        self.op("ST", "H")
        for e in ends:
            self.land(e)
        self.op("LD", "v")                            # mute
        self.op("CMPI", imm=VMAX + 1)
        self.op("LDI", imm=4 * ONE)
        self.op("LDIF", imm=0)
        self.op("ST", "G")
        self.op("STF", "H")

    # --- noise generator ---------------------------------------------------------------------------------------
    def noise_gen(self):
        """zac_snd: per update, ncount <= F: step the LFSR, ncount += FS - F; else ncount -= F (F < FS / 5: at most
        one step per sample). The LFSR (31 bits, raw integer x): out = x0 ^ x28, forced 1 when bits 0-4 and 28 are
        0; x = x >> 1 | out << 30. r1 .. r4 = noise filter rate per update (charge for bit 1, -discharge for 0)"""
        nf = self.k("k_nfr", Z.NOISE_FREQ, "O")
        self.flag("nb")
        self.op("LD", "nfn")
        self.op("LDF", "nfc")
        for w in range(1, 5):
            self.op("ST", f"r{w}")
        self.op("LD", "nc")
        self.op("CMP", "k_nG4")                       # nc >= 4 F + 1: no step in this sample
        nost = self.skip_if(True)
        if self.noise == "engine":                    # the engine's LFSR 0 with the SN76477 lock-up guard (b[2])
            self.op("NOISE", 0, 4)
            self.op("LDI", imm=0)
            self.op("LDIF", imm=ONE)
            self.op("ST", "nb")
            self.op("LD", "nfn")
            self.op("LDF", "nfc")
            self.op("ST", "rB")
        else:
            self.lfsr()
        # update of the step: first u with nc <= u F; r_u .. r4 = the new bit's rate
        self.op("LD", "nc")
        go = []
        for u in range(1, 4):
            self.op("CMP", f"k_nG{u}")
            go.append(self.skip_if(False))
        ent = {}
        for u in (4, 3, 2):
            if u < 4:
                self.land(go[u - 1])
            self.op("LD", "rB")
            ent[u] = self.jump()
        self.land(go[0])
        self.op("LD", "rB")
        for w in range(1, 5):
            if w > 1:
                self.land(ent[w])
            self.op("ST", f"r{w}")
        self.op("LD", "nc")
        self.op("ADD", "k_nK")                        # + FS - 4 F
        end = self.jump()
        self.land(nost)
        self.op("LD", "nc")
        self.op("SUB", "k_nF4")
        self.land(end)
        self.op("ST", "nc")
        self.nfreq = nf

    def lfsr(self):
        """zac_snd's LFSR step (raw integer x): out = x0 ^ x28, 1 when bits 0-4 and 28 are 0; x = x >> 1 | out << 30.
        nb = out (0 / 1.0), rB = its noise filter rate"""
        self.op("LD", "x")
        self.op("MULI", imm=1 << 23)
        self.op("ST", "h")                            # x >> 1
        self.op("ADD", "h")
        self.op("SUB", "x")
        self.op("ST", "nb0")                          # -x0
        self.op("LD", "x")
        self.op("MULI", imm=1 << 27)                  # x << 3: sign = x28
        self.op("CMPI", imm=0)
        z28 = self.skip_if(True)
        self.op("LD", "nb0")                          # x28 = 1: out = 1 - x0
        self.op("ADDI", imm=1)
        o1 = self.jump()
        self.land(z28)
        self.op("LD", "h")                            # x28 = 0: out = x0, or 1 when x & 31 == 0
        self.op("MULI", imm=1 << 20)
        self.op("MULI", imm=1 << 29)
        self.op("SUB", "x")                           # -(x & 31)
        self.op("CMPI", imm=0)
        self.op("LD", "nb0")
        self.op("MULI", imm=-ONE)
        self.op("LDIF", imm=1)
        self.land(o1)
        self.op("CMPI", imm=1)                        # F = out
        self.op("LDI", imm=0)
        self.op("LDIF", imm=ONE)
        self.op("ST", "nb")
        self.op("LD", "nfn")
        self.op("LDF", "nfc")
        self.op("ST", "rB")
        self.op("LD", "nb")
        self.op("MULI", imm=64 << 24)                 # out << 30
        self.op("ADD", "h")
        self.op("ST", "x")

    def noise_filter(self):
        """per update: nf += r_u, clamped 0 .. 5 V; Schmitt: >= 3.35 V -> 0, <= 0.74 V -> 1. Hn = updates high"""
        for u in range(1, 5):
            self.op("LD", "nf")
            self.op("ADD", f"r{u}")
            self.op("MINI", imm=NMAX)
            self.op("MAXI", imm=0)
            self.op("ST", "nf")
            self.op("CMPI", imm=NHI)
            self.op("LDI", imm=0)
            self.op("STF", "fb")
            self.op("LD", "nf")
            self.op("CMPI", imm=NLO + 1)
            self.op("LDI", imm=ONE)
            self.op("STNF", "fb")
            if u == 1:
                self.op("LD", "fb")
            else:
                self.op("LD", "H")
                self.op("ADD", "fb")
            self.op("ST", "H")

    # --- S2636 tone --------------------------------------------------------------------------------------------
    def pvi_tone(self):
        """the tone drives node 1 (7406 on: 2.0 V through 60R, off: 5 V through 150R; Tc / Kc = target /
        coefficient for the current tone); q0 .. q3 = node 1 before updates 1 .. 4. "gens": generated from C7 on the
        board's line clock, switched on the first update of the sample holding the line event (format: edges on the
        48 kHz grid); "raw": the board's tone bit, sampled. (The update-exact "gen" variant does not fit.)"""
        k1lo = self.k("k_1lo", Z.K_PVI1_LO, "F", cap=True)
        k1hi = self.k("k_1hi", Z.K_PVI1_HI, "F", cap=True)
        self.k1 = (k1lo, k1hi)
        if self.pvi == "raw":
            self.op("BIT", 0, self.src(MISC, 2))
            self.tk(flag=False)
        else:
            inc = self.mstep * 208
            b0 = self.mstep * 207 - 39936             # B - b0 in the register (power-up 0)
            self.op("LD", "B")
            self.op("CMPI", imm=-3 * inc - b0)        # a line event in this sample
            ev = self.skip_if(True)
            self.op("ADDI", imm=4 * inc)
            self.op("ST", "B")
            nochg = [self.jump()]
            self.land(ev)
            self.op("ADDI", imm=4 * inc - LINE_L)
            self.op("ST", "B")
            # tone counter: C7 = 0 -> tone 0, count 0; count >= C7 -> toggle, count 0; else count + 1
            self.op("LDLM", 0, self.src(LATCH2, 0), imm=0xFF | (16 << 8))
            self.op("CMPI", imm=1 << 16)
            nz = self.skip_if(True)
            self.op("LDI", imm=0)
            self.op("ST", "cnt")
            self.op("ST", "tone")
            tk = self.jump()
            self.land(nz)
            self.op("SUB", "cnt")                     # C7 - count <= 0: toggle
            self.op("CMPI", imm=1)
            t2 = self.skip_if(False)
            self.op("LD", "cnt")
            self.op("ADDI", imm=1 << 16)
            self.op("ST", "cnt")
            nochg.append(self.jump())
            self.land(t2)
            self.op("LDI", imm=0)
            self.op("ST", "cnt")
            self.op("LDI", imm=ONE)
            self.op("SUB", "tone")
            self.op("ST", "tone")
            self.land(tk)
            self.tk()
            for a in nochg:
                self.land(a)
        self.p2php(0, "p1")                           # update 1 of nodes 2 / out uses node 1 before update 1
        for u in range(4):
            self.op("LD", "Tc")
            self.op("RCM", "p1", self.r("Kc"))
            if u < 3:
                self.op("ST", f"q{u + 1}")
        for u in range(1, 4):
            self.p2php(u, f"q{u}")

    def p2php(self, u, q):
        """node 2 (R17 100R / C17 10u, from node 1 before the update) and the output coupling (R19 + P1, C12 1u, from
        node 2 before the update): S += (p2 - php) x tone gain (zac_snd's truncating multiply)"""
        k2 = self.k("k_2", Z.K_PVI2, "F", cap=True)
        khp = self.k("k_hp", Z.K_PVI_HP, "F", cap=True)
        self.op("LD", "p2")
        self.op("RCM", "php", self.r(khp))
        self.op("LD", q)
        self.op("RCM", "p2", self.r(k2))
        self.op("SUB", "php")
        self.op("MUL", "g_pvi")
        self.op("ADD", "S")
        self.op("ST", "S")

    def tk(self, flag=True):
        """Tc / Kc for the current tone (flag=False: F = the tone already)"""
        k1lo, k1hi = self.k1
        if flag:
            self.flag("tone")
        self.op("LDI", imm=0)
        self.op("LDIF", imm=PLO_)
        self.op("ST", "Tc")
        self.op("LD", k1hi)
        self.op("LDF", k1lo)
        self.op("ST", "Kc")

    # --- the board ---------------------------------------------------------------------------------------------
    def build(self):
        if not self.dodgem:
            self.mark("decode")
            self.decode()
            self.mark("envelope")
            self.envelope()
            self.mark("555")
            self.ne555()
            self.mark("tops")
            self.tops()
            self.mark("slf")
            self.slf()
            self.mark("vco")
            self.vco()
            self.mark("noise")
            self.noise_gen()
            self.mark("mixer")
            # S = enabled ? G gn + H G / 4 gd : 0 (noise mode: H = the filtered noise's high updates)
            self.op("BIT", 0, self.src(L1, 7))
            vm = self.skip_if(False)
            self.noise_filter()                       # (H = the filtered noise's high updates)
            self.land(vm)
            self.op("LD", "G")
            self.op("MUL", "gn")
            self.op("ST", "S")
            self.op("LD", "H")
            self.op("MUL", "G")                       # (G 0 / 4: the mute also gates the noise mode's H)
            self.op("MULI", imm=ONE >> 2)
            self.op("MUL", "gd")
            self.op("ADD", "S")
            self.op("ST", "S")
            self.op("BIT", 0, self.src(L1, 0))
            self.op("LDI", imm=0)
            self.op("STNF", "S")
        else:
            self.op("LDI", imm=0)
            self.op("ST", "S")
        self.mark("tone")
        if self.pvi != "none":                        # "none": validation builds only (no tone path)
            self.pvi_tone()
        self.mark("out")
        self.op("LD", "S")
        self.op("MULI", imm=1 << 30)                  # x 64
        self.op("MULI", imm=2 * ONE)                  # x 2: OUT >> 9 = S / 4
        self.op("OUT")
        self.mark("end")
        self.breakdown = self.segments()
        return self.finish()

    def segments(self):
        code = self.code
        self.code = code + [(0, 0, 0, 0)]
        tail = {}
        for _, i in self.marks:
            full, self.code = self.code, self.code[i:]
            tail[i] = self.worst_path()
            self.code = full
        self.code = code
        return [(a, tail[i] - tail[j]) for (a, i), (_, j) in zip(self.marks, self.marks[1:])]

    # --- settings block (Game Audio) + power-up --------------------------------------------------------------
    def gfactor(self, reg, src, mask, sh, aged=False):
        """reg = 1 / capacitance scale for setting i (0 Factory, 1-3 = -10 / -20 / -30 %, 4-6 = +10 / +20 / +30 %;
        Aged Caps 1 - 0.15 i), computed (RCP); Factory is exactly 1.0. zac_snd's integer tables give the same
        rates within 1 LSB"""
        self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))     # i << 16
        self.op("MULI", imm=round((-0.15 if aged else -0.1) * 256 * ONE))
        self.op("ADDI", imm=ONE)
        if not aged:
            self.op("ST", "_gs")
            self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))
            self.op("CMPI", imm=4 << 16)
            self.op("MULI", imm=round(0.1 * 256 * ONE))
            self.op("ADDI", imm=round(0.7 * ONE))
            self.op("STF", "_gs")
            self.op("LD", "_gs")
        self.op("RCP")
        self.op("ST", reg)

    def settings(self):
        """group factors, every kr, the noise clock multiples, the tone gain"""
        used = {g for (_, g, _, _) in self.kregs.values() if g}
        for g in sorted(used):
            self.gfactor("_f" + g, *GSRC[g])
        aged = sorted(used & {"T", "F"})
        if aged:
            self.gfactor("_fA", 7, 0x18, 13, aged=True)
            for g in aged:
                self.op("LD", "_f" + g)
                self.op("MUL", "_fA")
                self.op("ST", "_f" + g)
        for reg, (kb, g, cap, neg) in self.kregs.items():
            assert not cap or kb * 2.05 < ONE, reg     # zac_snd's 1.0 cap cannot act
            self.op("LDI", imm=kb)
            self.op("MUL", "_f" + g)
            if neg:
                self.op("MULI", imm=-ONE)
            self.op("ST", reg)
        if not self.dodgem:
            for u in range(1, 5):                     # nc < u F + 1: the step is on update <= u
                self.op("LD", self.nfreq)
                self.op("MULI", imm=u * ONE)
                if u == 4:
                    self.op("ST", "k_nF4")
                    self.op("MULI", imm=-ONE)
                    self.op("ADDI", imm=FS192)
                    self.op("ST", "k_nK")
                    self.op("LD", "k_nF4")
                self.op("ADDI", imm=1)
                self.op("ST", f"k_nG{u}")
        if self.pvi == "none":
            return
        self.chain_raw("g_pvi", 7, 0xE0, 11, MUSIC_G)  # (OSD: 5 entries)
        # Kc (node 1's coefficient for the current tone) picks up a rescaled value at the next tone change (format);
        # at power-up it is 0 until then, with node 1 already at its target

    def finish(self):
        """no init block (state powers up 0 = zac_snd's reset state); key = settings + 1 against _glast
        (powers up 0); a change runs the settings block in one silent sample"""
        body, self.code = self.code, []

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
        kstart = len(self.code)
        assert not self.pinit
        self.settings()
        key()
        self.op("ST", "_glast")
        self.op("END")
        self.settings_len = len(self.code) - kstart
        self.land(at)
        self.code += body
        self.op("END")
        assert len(self.code) <= 1024, f"{self.name}: {len(self.code)} words (1024)"
        worst = self.worst_path()
        assert worst <= 410, f"{self.name}: a sample can use {worst} budget units (engine budget 410)"
        return self

    def power_up(self):
        """every register powers up 0 = zac_snd's reset state (the tone nodes and the line event counter are kept
        offset for that)"""


def zac(dodgem=False, pvi="raw", noise="engine", exact555=None):
    b = Zac(dodgem, pvi, noise, exact555)
    b.power_up()
    return b.build()


def report(b):
    print(f"{b.name} (tone {b.pvi}, noise {b.noise}, 555 {'exact' if b.exact555 else 'sample grid'}): "
          f"{len(b.code)} words (1024), worst sample "
          f"{b.worst_path()} / 410 units, {len(b.regs)} registers, settings block {b.settings_len} words")
    print("  body blocks:", ", ".join(f"{a} {u}" for a, u in b.breakdown))


if __name__ == "__main__":
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    which = args[0] if args else "zac"
    pv = "gens" if "--gens" in sys.argv else "none" if "--no-tone" in sys.argv else "raw"
    nz = "exact" if "--exact-noise" in sys.argv else "engine"
    b = Zac(which == "dodgem", pv, nz)
    b.power_up()
    try:
        b.build()
    except AssertionError as e:
        print(e)
        print("  body blocks:", ", ".join(f"{a} {u}" for a, u in getattr(b, "breakdown", [])))
        sys.exit(1)
    out = args[1] if len(args) > 1 else f"{b.name}.dsnd.bin"
    open(out, "wb").write(b.binary())
    report(b)
