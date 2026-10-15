"""Discrete sound engine assembler (rtl/dsnd_engine.sv, model verilator/snd/dsnd_model.h).

A board program runs once per 48 kHz sample. Values are Q24 volts (1.0 V = 1 << 24); the program scales what it
sends to OUT so that 1.0 = full scale. Board circuit programs live in tools/dsnd_boards.py.
"""
import math
import struct

FS = 48000.0
ONE = 1 << 24
OPS = dict(END=0, LD=1, LDI=2, ADD=3, SUB=4, ADDI=5, MULI=6, MUL=7, ST=8, STF=9, LDIF=10, RC=11, RCM=12, CMP=13,
           CMPI=14, MAXI=15, MINI=16, BIT=17, NOT=18, NOISE=19, OUT=20, LDF=21, SKF=22, SKNF=23, LDL=24, ABS=25,
           FAND=26, FOR=27, STNF=28, LDLM=29, MACI=30, EXP=31, RCP=32)
COST = {OPS["EXP"]: 4, OPS["RCP"]: 5}                 # engine budget units per instruction (others 1)
# bit sources: sound latches 1-4, latch 0, misc
LATCH1, LATCH2, LATCH3, LATCH4, LATCH0, MISC = range(6)


def q(v):
    """volts / gain -> Q24 int32"""
    i = int(round(v * ONE))
    assert -(1 << 31) <= i < (1 << 31), v
    return i


def k_rc(tau, fs=FS):
    """one-pole step toward the target: 1 - exp(-dt / tau)"""
    return q(1.0 - math.exp(-1.0 / (fs * tau)))


class Prog:
    def __init__(self, name):
        self.name, self.code, self.regs = name, [], {}

    # state memory: named registers, allocated on first use (255 = initialised flag)
    def r(self, name):
        if name not in self.regs:
            assert len(self.regs) < 255, "out of state registers"
            self.regs[name] = len(self.regs)
        return self.regs[name]

    def op(self, name, a=0, b=0, imm=0):
        a = self.r(a) if isinstance(a, str) else a
        self.code.append((OPS[name], a, b, imm & 0xFFFFFFFF))
        return len(self.code) - 1

    def src(self, source, bit):
        return (source << 3) | bit

    # --- skips: emit a placeholder, patch it to jump to here
    def skip_if(self, flag_true=True):
        return self.op("SKF" if flag_true else "SKNF")

    def land(self, at):
        o, a, b, _ = self.code[at]
        self.code[at] = (o, a, b, len(self.code) - at - 1)

    # --- building blocks -------------------------------------------------------------------------------------
    def rc(self, reg, tau):
        """acc = target; reg relaxes toward it with time constant tau (s); acc = reg"""
        self.op("RC", reg, imm=k_rc(tau))

    def rc_sel(self, reg, tau_hi, tau_lo, src, bit, invert=False):
        """RC whose time constant switches on a source bit (charge / discharge paths): bit set -> tau_hi"""
        self.op("BIT", 1 if invert else 0, self.src(src, bit))
        self.op("ST", "_t")
        self.op("LDI", imm=k_rc(tau_lo))
        self.op("LDIF", imm=k_rc(tau_hi))
        self.op("ST", "_k")
        self.op("LD", "_t")
        self.op("RCM", reg, self.r("_k"))

    def noise(self, n, amp):
        """acc = +amp / -amp white noise from LFSR n, stepped once per sample"""
        self.op("NOISE", 0, n)
        self.op("LDI", imm=q(-amp))
        self.op("LDIF", imm=q(amp))

    def binary(self):
        assert self.code[-1][0] == OPS["END"], self.name
        assert len(self.code) <= 1024, self.name
        return b"".join(struct.pack("<Q", (o << 56) | (a << 48) | (b << 40) | imm) for o, a, b, imm in self.code)


# ---------------------------------------------------------------- MAME discrete node equivalents
# (MAME src/devices/sound/disc_*.hxx, D. Renaud et al.). Signals live in named state registers as Q24 volts;
# logic signals are 0 / 1.0. Constants are worked out here exactly as the MAME reset functions do.

VBE = 0.5           # OP_AMP_NORTON_VBE
RAIL = 1.5          # OP_AMP_VP_RAIL_OFFSET
FN_NONE, FN_TRG0, FN_TRG0_INV, FN_TRG1, FN_TRG1_INV, FN_TRG2, FN_TRG2_INV, FN_TRG01_AND, FN_TRG01_NAND = range(9)


def wrap32(v):
    return (v + (1 << 31)) % (1 << 32) - (1 << 31)


def qmul(a, b):
    """the engine's Q24 multiply (MULI / MUL) on two Q24 integers, exactly"""
    return wrap32((a * b) >> 24)


def rc_exp(rc):
    return 1.0 if rc <= 0 else 1.0 - math.exp(-1.0 / (FS * rc))


def par(*rs):
    return 1.0 / sum(1.0 / r for r in rs if r)


class Board(Prog):
    """A board program: init block (runs once), then the per-sample body."""

    def __init__(self, name):
        super().__init__(name)
        self.inits = []
        self.groups = set()
        self.kcode = []                               # rate scaling, run only when a Game Audio setting changes
        self.music = False                            # a music trim pot is used (mixer input flagged "music")

    # --- Game Audio settings: capacitor groups T (timing / envelope), F (filters), O (oscillators). OSD index
    # 0 Factory, 1-3 = -10 / -20 / -30 %, 4-6 = +10 / +20 / +30 % capacitance; a rate per sample scales by 1 / C.
    GROUP_SRC = {"T": (6, 0x07, 16), "F": (6, 0x38, 13), "O": (7, 0x07, 16)}
    SCALES = (1.0, 0.9, 0.8, 0.7, 1.1, 1.2, 1.3)
    AGED = (1.0, 0.85, 0.7)                           # Aged Caps Off / Light / Heavy: electrolytics lose capacitance
    MUSIC = (1.0, 0.7, 0.5, 1.4, 2.0)                 # Music Volume Factory / Low / Lowest / High / Highest

    def chain(self, reg, src, mask, sh, values):
        """reg = values[setting index] (Q24), setting = (source byte & mask) >> ... as index << 16"""
        self.op("LDLM", 0, self.src(src, 0), imm=mask | (sh << 8))
        self.op("ST", "_gv")
        self.op("LDI", imm=q(values[0]))
        self.op("ST", reg)
        for i, v in enumerate(values[1:], 1):
            self.op("LD", "_gv")
            self.op("CMPI", imm=i << 16)
            self.op("LDI", imm=q(v))
            self.op("STF", reg)

    def kreg(self, reg, k, g, clamp=True):
        """reg = k x the group factor (k a per-sample rate: RC coefficient, slope or phase step); emitted into the
        on-change block, so it costs nothing per sample"""
        body, self.code = self.code, self.kcode
        self.op("LDI", imm=q(k))
        if g:
            self.groups.add(g)
            self.op("MUL", "_f" + g)
            if clamp:
                self.op("MINI", imm=ONE)
        self.op("ST", reg)
        self.kcode, self.code = self.code, body

    def init(self, reg, value):
        self.inits.append((reg, value))

    # --- logic: triggers are 0 / 1 constants, ('bit', source, bit) or a logic register name
    def flag_from(self, t):
        """F = trigger; returns False if t is a constant (then F is untouched and the value is returned)"""
        if isinstance(t, tuple):
            self.op("BIT", 0, self.src(t[1], t[2]))
            return None
        if isinstance(t, str):
            self.op("LD", t)
            self.op("CMPI", imm=q(0.5))
            return None
        return bool(t)

    def logic_reg(self, reg, fn, trig, store=True):
        """reg = dst_trigger_function(trig0, trig1, trig2, fn) as 0 / 1.0; returns the constant if it is one.
        A dynamic result also leaves F = the function value; store=False only sets F"""
        t0, t1, t2 = trig
        const = {FN_NONE: True}.get(fn)
        if const is None:
            a = {FN_TRG0: t0, FN_TRG0_INV: t0, FN_TRG1: t1, FN_TRG1_INV: t1, FN_TRG2: t2, FN_TRG2_INV: t2,
                 FN_TRG01_AND: t0, FN_TRG01_NAND: t0}[fn]
            inv = fn in (FN_TRG0_INV, FN_TRG1_INV, FN_TRG2_INV, FN_TRG01_NAND)
            if fn in (FN_TRG01_AND, FN_TRG01_NAND):
                ca, cb = self.flag_from(t0), None
                if ca is not None:
                    if not ca:
                        const = inv
                    else:
                        cb = self.flag_from(t1)
                        if cb is not None:
                            const = cb != inv
                else:
                    self.op("LDI", imm=0); self.op("LDIF", imm=ONE); self.op("ST", "_fa")
                    cb = self.flag_from(t1)
                    if cb is not None:
                        if not cb:
                            const = inv
                        else:
                            self.op("LD", "_fa"); self.op("CMPI", imm=q(0.5))
                    else:
                        self.op("LDI", imm=0); self.op("LDIF", imm=ONE); self.op("MUL", "_fa"); self.op("CMPI", imm=q(0.5))
            else:
                c = self.flag_from(a)
                if c is not None:
                    const = c != inv
            if const is None:
                if inv:
                    self.op("NOT")
                if store:
                    self.op("LDI", imm=0)
                    self.op("LDIF", imm=ONE)
                    self.op("ST", reg)
                return None
        return bool(const)

    def load(self, v):
        """acc = a register or a constant"""
        if isinstance(v, str):
            self.op("LD", v)
        else:
            self.op("LDI", imm=q(v))

    # --- nodes -----------------------------------------------------------------------------------------------
    def midway_tone(self, out, lo, hi, clock=19968000 / 10 / 2, toggles=2):
        """DISCRETE_NOTE fed by the Midway tone latches: lo = (source, ) D5-D1, hi = D5-D0; out = 0 / 1.0. toggles =
        output edges handled per sample (a second one only matters for notes above 24 kHz)"""
        n = out
        self.op("LDLM", 0, self.src(hi, 0), imm=0x3F | (22 << 8))
        self.op("ST", n + "_d")
        self.op("LDLM", 0, self.src(lo, 0), imm=0x3E | (16 << 8))
        self.op("ADD", n + "_d")
        self.op("ST", n + "_d")
        self.op("LDI", imm=4096 << 16)
        self.op("SUB", n + "_d")
        self.op("ST", n + "_p")                       # period in clocks, Q16
        self.op("LD", n + "_ph")
        self.op("ADDI", imm=int(round(clock / FS * 65536)))
        self.op("ST", n + "_ph")
        for _ in range(toggles):                      # anything faster is ultrasonic
            self.op("LD", n + "_ph")
            self.op("CMP", n + "_p")
            self.op("SUB", n + "_p")
            self.op("STF", n + "_ph")
            self.op("LDI", imm=ONE)
            self.op("SUB", out)
            self.op("STF", out)

    def lfsr_noise(self, out, freq, amp, bias, n=0):
        """DSS_LFSR_NOISE clocked at freq (an RC clock on these boards: oscillator group)"""
        self.kreg(out + "_inc", freq / FS, "O", clamp=False)
        self.op("LD", out + "_ph")
        self.op("ADD", out + "_inc")
        self.op("ST", out + "_ph")
        self.op("CMPI", imm=ONE)
        at = self.skip_if(False)
        self.op("ADDI", imm=-ONE)
        self.op("ST", out + "_ph")
        self.op("NOISE", 0, n)
        self.op("LDI", imm=q(bias - amp / 2))
        self.op("LDIF", imm=q(bias + amp / 2))
        self.op("ST", out)
        self.land(at)
        self.init(out, bias - amp / 2)

    def tvca(self, out, info, trig, inp):
        """DST_TVCA_OP_AMP (no c4 support needed yet). info keys r1..r11, c1..c3, v1..v3, vP, f0..f5"""
        g = lambda k: info.get(k, 0)
        r4 = g("r4")
        r67 = g("r6") + g("r7")
        vmax = g("vP") - VBE
        div = lambda a, b: b / (a + b) if (a + b) else 0
        vt1 = (g("v1") - 0.6 - VBE) * div(g("r5"), r67) + VBE
        kc1 = rc_exp(par(g("r5"), r67) * g("c1"))
        kd1 = rc_exp(r67 * g("c1"))
        fixed = r4 * vmax / g("r1")
        # i2 / i3 input terms, scaled by r4 into volts. n stays a known constant (no register) until a term needs
        # it in one; constant inputs are folded exactly as the engine would compute them
        n_const = q(fixed)
        for rk, fk, ik in (("r2", "f0", 0), ("r3", "f1", 1)):
            if not g(rk):
                continue
            src = inp[ik]
            if n_const is not None and not isinstance(src, str):
                v = qmul(max(wrap32(q(src) + q(-VBE)), 0), q(r4 / g(rk)))
                c = self.logic_reg(out + "_g", g(fk), trig, store=False)
                if c is True:
                    n_const = wrap32(n_const + v)
                elif c is None:                       # F = the gate
                    self.op("LDI", imm=n_const)
                    self.op("LDIF", imm=wrap32(n_const + v))
                    self.op("ST", out + "_n")
                    n_const = None
                continue
            c = self.logic_reg(out + "_g", g(fk), trig)
            if c is False:
                continue
            self.load(src)
            self.op("ADDI", imm=q(-VBE))
            self.op("MAXI", imm=0)
            self.op("MULI", imm=q(r4 / g(rk)))
            if c is None:
                self.op("MUL", out + "_g")
            if n_const is not None:
                self.op("ADDI", imm=n_const)
                n_const = None
            else:
                self.op("ADD", out + "_n")
            self.op("ST", out + "_n")
        f3 = self.logic_reg(out + "_f3", g("f3"), trig)
        assert f3 is True, "dynamic F3 not needed by the boards so far"
        # c1: charge toward vt1 while F2 is open, else discharge toward VBE through r6 + r7
        c2 = self.logic_reg(out + "_f2", g("f2"), trig, store=False)
        if c2 is None:                                # F = F2: pick the pre-scaled charge / discharge rate
            self.kreg(out + "_kd1", kd1, "T")
            self.kreg(out + "_kc1", kc1, "T")
            self.op("LD", out + "_kd1"); self.op("LDF", out + "_kc1"); self.op("ST", out + "_k")
            self.op("LDI", imm=q(VBE)); self.op("LDIF", imm=q(vt1))
            self.op("RCM", out + "_c1", self.r(out + "_k"))
        else:
            self.kreg(out + "_k", kc1 if c2 else kd1, "T")
            self.op("LDI", imm=q(vt1 if c2 else VBE))
            self.op("RCM", out + "_c1", self.r(out + "_k"))
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=q(r4 / r67))
        caps = [x for x in (("r8", "r9", "c2", "v2", "f4"), ("r10", "r11", "c3", "v3", "f5")) if g(x[1])]
        if caps:
            self.op("ST", out + "_p")
        for j, (rv, rr, cc, vk, fk) in enumerate(caps):
            vt = (g(vk) - 0.6 - VBE) * div(g(rv), g(rr))
            k0, k1 = rc_exp(g(rr) * g(cc)), rc_exp(par(g(rv), g(rr)) * g(cc))
            cf = self.logic_reg(out + "_g", g(fk), trig, store=False)
            if cf is None:
                self.kreg(f"{out}_k0{cc}", k0, "T")
                self.kreg(f"{out}_k1{cc}", k1, "T")
                self.op("LD", f"{out}_k0{cc}"); self.op("LDF", f"{out}_k1{cc}"); self.op("ST", out + "_k")
                self.op("LDI", imm=0); self.op("LDIF", imm=q(vt))
                self.op("RCM", out + "_" + cc, self.r(out + "_k"))
            else:
                self.kreg(out + "_k", k1 if cf else k0, "T")
                self.op("LDI", imm=q(vt if cf else 0))
                self.op("RCM", out + "_" + cc, self.r(out + "_k"))
            self.op("MULI", imm=q(r4 / g(rr)))
            self.op("ADD", out + "_p")
            if j < len(caps) - 1:
                self.op("ST", out + "_p")
        if g("c4"):                                   # output cap through r4: exponential charge toward i_out x r4
            self.kreg(out + "_k4", rc_exp(r4 * g("c4")), "T")
        # acc = p here
        if n_const is not None:
            self.op("ADDI", imm=wrap32(-n_const))
        else:
            self.op("SUB", out + "_n")
        self.op("MAXI", imm=0)
        if g("c4"):
            self.op("RCM", out + "_c4", self.r(out + "_k4"))
            self.op("MAXI", imm=0)
        self.op("MINI", imm=q(vmax))
        self.op("ST", out)

    def op_amp_osc_norton1(self, out, r1, r2, r3, r4, r5, c, vP):
        """DSS_OP_AMP_OSC type 1 Norton, linear charge, square-wave output (enable tied high, no r6)"""
        vh = vP - VBE
        ch0 = vh / r1                                 # discharge current (flip-flop low)
        ch1 = (vh - VBE) / r2 - ch0                   # charge current (flip-flop high)
        i1 = vh / r5
        tl = (i1 - VBE / r4) * r3 + VBE
        th = (i1 + (vh - VBE) / r4) * r3 + VBE
        d0, d1 = ch0 / FS / c, ch1 / FS / c
        self.kreg(out + "_d0", d0, "O", clamp=False)
        self.kreg(out + "_d1", d1, "O", clamp=False)
        # state: _v cap voltage, _lo = 1.0 while discharging (power-up: charging, MAME m_flip_flop = 1)
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        at = self.skip_if(True)
        self.op("LD", out + "_v")                     # charging
        self.op("ADD", out + "_d1")
        self.op("CMPI", imm=q(th))
        at2 = self.skip_if(False)
        self.op("ADDI", imm=q(-th))                   # overshoot turns into discharge time
        self.op("MULI", imm=q(-ch0 / ch1))
        self.op("ADDI", imm=q(th))
        self.op("ST", out + "_v")
        self.op("LDI", imm=ONE)
        self.op("ST", out + "_lo")
        self.op("LDI", imm=ONE)                       # F = 1 -> skip the not-crossed store
        self.op("CMPI", imm=0)
        self.land(at2)
        at3 = self.skip_if(True)
        self.op("ST", out + "_v")
        self.land(at3)
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at4 = self.skip_if(True)                      # always: skip the discharge branch
        self.land(at)
        self.op("LD", out + "_v")                     # discharging
        self.op("SUB", out + "_d0")
        self.op("CMPI", imm=q(tl))
        at5 = self.skip_if(True)
        self.op("ADDI", imm=q(-tl))
        self.op("MULI", imm=q(-ch1 / ch0))
        self.op("ADDI", imm=q(tl))
        self.op("ST", out + "_v")
        self.op("LDI", imm=0)
        self.op("ST", out + "_lo")
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at6 = self.skip_if(True)
        self.land(at5)
        self.op("ST", out + "_v")
        self.land(at6)
        self.land(at4)
        self.op("LD", out + "_lo")                    # output: high while charging
        self.op("CMPI", imm=q(0.5))
        self.op("LDI", imm=q(vh))
        self.op("LDIF", imm=0)
        self.op("ST", out)

    def op_amp_filt_bp1(self, out, src, r1, rf, c1, c2, r2=0, r3=0, vref=0.0, vp=12.0, vn=0.0):
        """DST_OP_AMP_FILT, DISC_OP_AMP_FILTER_IS_BAND_PASS_1 (non-Norton), output clipped to the rails"""
        rt = par(r1, r2, r3)
        gain = -rf / rt
        self.kreg(out + "_k2", rc_exp(rt * c2), "F")
        self.kreg(out + "_k1", rc_exp(rf * c1), "F")
        self.load(src)
        self.op("ADDI", imm=q(-vref))
        self.op("MULI", imm=q(rt / r1))               # Millman: (in - vref) / r1 * rTotal
        self.op("ST", out + "_v")
        self.op("SUB", out + "_c2")
        self.op("ST", out + "_hp")                    # v - vC2 (before vC2 moves)
        self.op("LD", out + "_v")
        self.op("RCM", out + "_c2", self.r(out + "_k2"))
        self.op("LD", out + "_hp")
        self.op("RCM", out + "_c1", self.r(out + "_k1"))
        lo, hi = sorted(((vn - vref) / gain, (vp - RAIL - vref) / gain))
        self.op("MAXI", imm=q(lo))                    # clip at the rails before the (large) gain: no Q24 overflow
        self.op("MINI", imm=q(hi))
        steps = 1
        while abs(gain) / steps > 64:
            steps *= 16
        self.op("MULI", imm=q(gain / steps))
        if steps > 1:
            self.op("MULI", imm=q(steps))
        self.op("ADDI", imm=q(vref))
        self.op("ST", out)

    def op_amp_norton(self, out, inp0, inp1, r1, r2, r3, r4, vP, vN=0.0):
        """DST_OP_AMP, DISC_OP_AMP_IS_NORTON, no cap: out = r4 x (i+ - i-), clamped to vN .. vP - VBE"""
        vmax = vP - VBE
        self.load(inp0)
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=q(r4 / r1))
        self.op("ADDI", imm=q(r4 * vmax / r3 if r3 else 0))
        self.op("ST", out + "_n")
        self.load(inp1)
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=q(r4 / r2))
        self.op("SUB", out + "_n")
        self.op("MAXI", imm=q(vN))
        self.op("MINI", imm=q(vmax))
        self.op("ST", out)

    def filter1_lp(self, out, src, fc):
        """DST_FILTER1 low pass (bilinear, pre-warped as MAME calculate_filter1_coefficients)"""
        wc = FS * 2.0 * math.tan(math.pi * fc / FS)
        t2 = 2.0 * FS
        den = wc + t2
        a1, b0 = (wc - t2) / den, wc / den
        self.load(src)
        self.op("ST", out + "_x0")
        self.op("MULI", imm=q(b0))
        self.op("MACI", out + "_x1", imm=q(b0))
        self.op("MACI", out, imm=q(-a1))
        self.op("ST", out)
        self.op("LD", out + "_x0"); self.op("ST", out + "_x1")

    def mixer_resistor(self, out, ins, c_amp=0):
        """DST_MIXER, DISC_MIXER_IS_RESISTOR (no rF): Millman sum of the (high-passed) inputs, ins = [(src, r, c
        [, music [, gain]])]; output high-passed by c_amp (100k)"""
        rt = par(*[inp[1] for inp in ins])
        if c_amp:
            self.kreg(out + "_ka", rc_exp(100e3 * c_amp), "F")
        terms = [(k, inp[0], inp[1], inp[2], (inp[4] if len(inp) > 4 else 1.0) * rt / inp[1], len(inp) > 3 and inp[3])
                 for k, inp in enumerate(ins)]       # inp[4]: a gain stage in front of this input, folded in
        self._mix_sum(out, terms, 0.0)
        if c_amp:
            self.op("RCM", out + "_amp", self.r(out + "_ka"))
            self.op("LD", out + "_s")
            self.op("SUB", out + "_amp")
        self.op("ST", out)

    def _mix_sum(self, out, terms, vref):
        """out_s = sum of w x input over terms (k, src, r, c, w, music); acc = out_s after. An input with a coupling
        cap (high-passed through r * c, after subtracting vref) or the music trim pot takes the long path; a plain
        register input is one MACI. Each term rounds the same either way and integer sums are exact: the order
        does not change the result"""
        slow = [t for t in terms if t[1] is not None and (t[3] or t[5] or not isinstance(t[1], str))]
        fast = [t for t in terms if t[1] is not None and t not in slow]
        self.op("LDI", imm=0)
        if slow or not fast:
            self.op("ST", out + "_s")
        for k, src, r, c, w, music in slow:
            if c:
                self.kreg(f"{out}_k{k}", rc_exp(r * c), "F")
            self.load(src)
            if c:
                if vref:
                    self.op("ADDI", imm=q(-vref))
                self.op("RCM", f"{out}_hp{k}", self.r(f"{out}_k{k}"))
                self.load(src)
                self.op("SUB", f"{out}_hp{k}")
            self.op("MULI", imm=q(w))
            if music:                                 # the music trim pot
                self.music = True
                self.op("MUL", "_fM")
            self.op("ADD", out + "_s")
            self.op("ST", out + "_s")
        if fast:
            if slow:
                self.op("LD", out + "_s")
            for k, src, r, c, w, music in fast:
                self.op("MACI", src, imm=q(w))
            self.op("ST", out + "_s")

    def rcdisc2(self, out, sw, in0, r0, in1, r1, c):
        """DST_RCDISC2: relaxes toward in0 through r0 while the switch bit is low, toward in1 through r1 when high"""
        self.kreg(out + "_k0", rc_exp(r0 * c), "T")
        self.kreg(out + "_k1", rc_exp(r1 * c), "T")
        self.op("BIT", 0, self.src(*sw))
        self.op("LD", out + "_k0")
        self.op("LDF", out + "_k1")
        self.op("ST", out + "_k")
        self.op("LDI", imm=q(in0))
        self.op("LDIF", imm=q(in1))
        self.op("RCM", out, self.r(out + "_k"))

    def op_amp_osc2_norton_cap(self, out, r1, r2, r3, r4, c, vP):
        """DSS_OP_AMP_OSC type 2 | NORTON, OUT_CAP (r5 = r6 = 0): exponential charge toward v1 / discharge toward v0
        through r1 || r2; a threshold crossing clamps to the threshold (sub-sample time lost: LFO-rate, negligible).
        out = the cap voltage"""
        vh = vP - VBE
        rc = par(r1, r2)
        v0 = VBE / r2 * rc
        v1 = (vh / r1 + VBE / r2) * rc
        tl = vh / r4 * r2 + VBE
        th = (vh / r4 + (vP - 2 * VBE) / r3) * r2 + VBE
        self.kreg(out + "_k", rc_exp(rc * c), "O")
        self.op("LD", out + "_dis")                   # _dis = 1 while discharging (power-up: charging from 0 V)
        self.op("CMPI", imm=q(0.5))
        self.op("LDI", imm=q(v1))
        self.op("LDIF", imm=q(v0))
        self.op("RCM", out, self.r(out + "_k"))
        self.op("CMPI", imm=q(th))
        self.op("LDI", imm=q(th))
        self.op("STF", out)
        self.op("LDI", imm=ONE)
        self.op("STF", out + "_dis")
        self.op("LD", out + "_dis")
        self.op("CMPI", imm=q(0.5))
        at = self.skip_if(False)
        self.op("LD", out)
        self.op("CMPI", imm=q(tl))
        self.op("LDI", imm=q(tl))
        self.op("STNF", out)
        self.op("LDI", imm=0)
        self.op("STNF", out + "_dis")
        self.land(at)

    def op_amp_vco1_norton_cap(self, out, vmod, en, r1, r2, r3, r4, r5, c, vP):
        """DSS_OP_AMP_OSC VCO_1 | NORTON, OUT_CAP, r6 = r7 = r8 = 0 (vmod fed straight in), linear charge. en low =
        MAME force_charge (cap charges to the rail). Charge / discharge currents both scale with vmod, so the
        overshoot carry ratio is a constant: exact. out = the cap voltage"""
        vh = vP - VBE
        i1 = vh / r5
        tl = (i1 - (vP - 2 * VBE) / r4) * r3 + VBE
        th = (i1 + VBE / r4) * r3 + VBE
        self.kreg(out + "_nk0", -1.0 / (r1 * FS * c), "O", clamp=False)
        self.kreg(out + "_k1", (1.0 / r2 - 1.0 / r1) / (FS * c), "O", clamp=False)
        self.load(vmod)
        self.op("ADDI", imm=q(-VBE))
        self.op("ST", out + "_vm")
        # _hi = MAME flip_flop: charges while low (xor 1), power-up low
        self.op("LD", out + "_hi")
        self.op("CMPI", imm=q(0.5))
        self.op("FAND", 0, self.src(*en))
        at = self.skip_if(True)
        self.op("LD", out + "_vm")                    # charging
        self.op("MUL", out + "_k1")
        self.op("ADD", out)
        self.op("MINI", imm=q(vh))                    # forced: the cap stops at the rail
        self.op("ST", out)
        self.op("CMPI", imm=q(th))
        at2 = self.skip_if(False)
        self.op("LDI", imm=ONE)
        self.op("ST", out + "_hi")
        self.op("BIT", 0, self.src(*en))
        at_f = self.skip_if(False)                    # forced (disabled): keep charging, no carry
        self.op("LD", out)
        self.op("ADDI", imm=q(-th))
        self.op("MULI", imm=q(-r2 / (r1 - r2)))       # overshoot time x discharge rate (d0 / d1 constant)
        self.op("ADDI", imm=q(th))
        self.op("ST", out)
        self.land(at_f)
        self.land(at2)
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at3 = self.skip_if(True)
        self.land(at)
        self.op("LD", out + "_vm")                    # discharging
        self.op("MUL", out + "_nk0")
        self.op("ADD", out)
        self.op("ST", out)
        self.op("CMPI", imm=q(tl))
        at4 = self.skip_if(True)
        self.op("LDI", imm=0)
        self.op("ST", out + "_hi")
        self.op("LD", out)
        self.op("ADDI", imm=q(-tl))
        self.op("MULI", imm=q(-(r1 - r2) / r2))
        self.op("ADDI", imm=q(tl))
        self.op("ST", out)
        self.land(at4)
        self.land(at3)

    def op_amp_vco3_norton(self, out, vmod, r1, r2, r3, r4, r5, c, vP, r7=0, sqw=False):
        """DSS_OP_AMP_OSC VCO_3 | NORTON (no r6 / r8 enable), linear charge, OUT_CAP or OUT_SQW. Discharge current
        = fixed (r7) + max(0, vmod - VBE) / r1, charge = (vP - 2 VBE) / r2 - discharge. The overshoot carry ratio
        follows vmod (no divide in the engine): a crossing lands half a step past the threshold, the expected value
        of the exact carry -> no pitch bias, edge jitter within one sample. The cap stays inside tl - step .. th, so
        MAME's 0 .. vh clamp never acts (the assert keeps the charge current positive for any vmod <= vP).
        OUT_CAP: out = the cap voltage. SQW: out = vh while charging, 0 while discharging; the cap is out_v"""
        vh = vP - VBE
        i1 = vh / r5
        tl = (i1 - VBE / r4) * r3 + VBE
        th = (i1 + (vP - 2 * VBE) / r4) * r3 + VBE
        assert vh / r1 + (vh / r7 if r7 else 0.0) < (vP - 2 * VBE) / r2, "charge current must stay positive"
        cap = out + "_v" if sqw else out
        self.kreg(out + "_ka", 1.0 / (r1 * FS * c), "O", clamp=False)
        self.kreg(out + "_kf", (vh / r7 if r7 else 0.0) / (FS * c), "O", clamp=False)
        self.kreg(out + "_kt", (vP - 2 * VBE) / r2 / (FS * c), "O", clamp=False)
        self.load(vmod)
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MUL", out + "_ka")
        self.op("ADD", out + "_kf")
        self.op("ST", out + "_d0")                    # discharge step; charge step = kt - d0
        if sqw:                                       # power-up: charging from 0 V
            self.init(out, vh)
            self.op("LD", out)
            self.op("CMPI", imm=q(vh / 2))
            at = self.skip_if(False)
        else:
            self.op("LD", out + "_lo")                # _lo = 1 while discharging
            self.op("CMPI", imm=q(0.5))
            at = self.skip_if(True)
        self.op("LD", cap)                            # charging
        self.op("ADD", out + "_kt")
        self.op("SUB", out + "_d0")
        self.op("ST", cap)
        self.op("CMPI", imm=q(th))
        at2 = self.skip_if(False)
        self.op("LD", out + "_d0")
        self.op("MULI", imm=q(-0.5))
        self.op("ADDI", imm=q(th))
        self.op("ST", cap)
        if sqw:
            self.op("LDI", imm=0)
            self.op("ST", out)
        else:
            self.op("LDI", imm=ONE)
            self.op("ST", out + "_lo")
        self.land(at2)
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at3 = self.skip_if(True)
        self.land(at)
        self.op("LD", cap)                            # discharging
        self.op("SUB", out + "_d0")
        self.op("ST", cap)
        self.op("CMPI", imm=q(tl))
        at4 = self.skip_if(True)
        self.op("LD", out + "_kt")
        self.op("SUB", out + "_d0")
        self.op("MULI", imm=q(0.5))
        self.op("ADDI", imm=q(tl))
        self.op("ST", cap)
        if sqw:
            self.op("LDI", imm=q(vh))
            self.op("ST", out)
        else:
            self.op("LDI", imm=0)
            self.op("ST", out + "_lo")
        self.land(at4)
        self.land(at3)

    def op_amp_filt_bp1m_norton(self, out, src, r1, r3, rf, c1, c2, vP, vN=0.0, r2=0):
        """DST_OP_AMP_FILT BAND_PASS_1M | NORTON: input max(0, in - VBE), MAME bilinear band pass with the circuit
        gain, vRef = (vP - VBE) / r3 x rF, clipped vN .. vP - VBE; the clipped output feeds back (as MAME)"""
        rt = par(r1, r2) if r2 else r1
        fc = 1.0 / (2 * math.pi * math.sqrt(rt * rf * c1 * c2))
        d = (c1 + c2) / math.sqrt(rf / rt * c1 * c2)
        gain = -rf / rt * c2 / (c1 + c2)
        wc = FS * 2.0 * math.tan(math.pi * fc / FS)
        t2 = 2 * FS
        den = t2 * t2 + d * wc * t2 + wc * wc
        a1 = 2.0 * (-t2 * t2 + wc * wc) / den
        a2 = (t2 * t2 - d * wc * t2 + wc * wc) / den
        b0 = d * wc * t2 / den * gain
        vref = (vP - VBE) / r3 * rf
        vmax = vP - VBE
        assert vref - vN < 120, "feedback state outside Q24"
        self.load(src)
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("ST", out + "_x0")
        self.op("MULI", imm=q(b0))
        for reg, k in (("_x2", -b0), ("_y2", -a2), ("_y1", -a1)):     # y2 before y1: partial sums stay small
            self.op("MACI", out + reg, imm=q(k))
        self.op("ST", out + "_s")
        self.op("LD", out + "_x1"); self.op("ST", out + "_x2")
        self.op("LD", out + "_x0"); self.op("ST", out + "_x1")
        self.op("LD", out + "_y1"); self.op("ST", out + "_y2")
        self.op("LD", out + "_s")
        self.op("ADDI", imm=q(vref))
        self.op("MAXI", imm=q(vN))
        self.op("MINI", imm=q(vmax))
        self.op("ST", out)
        self.op("ADDI", imm=q(-vref))
        self.op("ST", out + "_y1")

    def integrate_norton1(self, out, src, v_on, r1, r2, c, v1, vP):
        """DST_INTEGRATE, DISC_INTEGRATE_OP_AMP_1 | NORTON: dv = (max(0, (in - VBE) / r2) - (v1 - VBE) / r1) / FS / C,
        clipped 0 .. vP - VBE. src = (source, bit): the input is v_on when the bit is set (INPUTX_LOGIC);
        src = a register name: the input is that voltage (v_on unused)"""
        self.kreg(out + "_ka", 1.0 / (r2 * FS * c), "T", clamp=False)
        self.kreg(out + "_kb", (v1 - VBE) / (r1 * FS * c), "T", clamp=False)
        if isinstance(src, str):
            self.op("LD", src)
            self.op("ADDI", imm=q(-VBE))
            self.op("MAXI", imm=0)
        else:
            self.op("BIT", 0, self.src(*src))
            self.op("LDI", imm=0)
            self.op("LDIF", imm=q(v_on - VBE))
        self.op("MUL", out + "_ka")
        self.op("SUB", out + "_kb")
        self.op("ADD", out)
        self.op("MAXI", imm=0)
        self.op("MINI", imm=q(vP - VBE))
        self.op("ST", out)

    def op_amp_vco2_norton(self, out, vmod, r1, r2, r3, r4, r5, r6, c, vP):
        """DSS_OP_AMP_OSC VCO_2 | NORTON, linear charge, square-wave output. Charge rates follow vmod each sample;
        a threshold crossing clamps to the threshold (MAME carries the overshoot time: small pitch difference)"""
        vh = vP - VBE
        t1, t2 = vh / r2, vh * (1.0 / r2 + 1.0 / r6)
        i1 = vh / r5
        tl = (i1 - VBE / r4) * r3 + VBE
        th = (i1 + (vh - VBE) / r4) * r3 + VBE
        self.kreg(out + "_ka", 1.0 / (r1 * FS * c), "O", clamp=False)
        self.kreg(out + "_k0", t1 / (FS * c), "O", clamp=False)
        self.kreg(out + "_k1", t2 / (FS * c), "O", clamp=False)
        self.load(vmod)
        self.op("MUL", out + "_ka")
        self.op("ST", out + "_iv")                    # i / (FS C)
        self.op("SUB", out + "_k0")
        self.op("ST", out + "_d0")                    # discharge step (negative = osc held charging)
        self.op("LD", out + "_k1")
        self.op("SUB", out + "_iv")
        self.op("ST", out + "_d1")                    # charge step
        self.op("LD", out + "_d0")                    # force charge when the discharge current reverses
        self.op("CMPI", imm=0)
        self.op("LDI", imm=0)
        self.op("LDIF", imm=ONE)
        self.op("ST", out + "_run")
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        at = self.skip_if(True)
        self.op("LD", out + "_v")                     # charging
        self.op("ADD", out + "_d1")
        self.op("MINI", imm=q(vh))
        self.op("ST", out + "_v")
        self.op("CMPI", imm=q(th))
        self.op("LDI", imm=0)
        self.op("LDIF", imm=ONE)
        self.op("MUL", out + "_run")                  # crossed and the oscillator is running -> discharge
        self.op("CMPI", imm=q(0.5))
        self.op("LDI", imm=q(th))
        self.op("STF", out + "_v")
        self.op("LDI", imm=ONE)
        self.op("STF", out + "_lo")
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at2 = self.skip_if(True)
        self.land(at)
        self.op("LD", out + "_v")                     # discharging
        self.op("SUB", out + "_d0")
        self.op("ST", out + "_v")
        self.op("CMPI", imm=q(tl))
        at3 = self.skip_if(True)
        self.op("LDI", imm=q(tl))
        self.op("ST", out + "_v")
        self.op("LDI", imm=0)
        self.op("ST", out + "_lo")
        self.land(at3)
        self.land(at2)
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        self.op("LDI", imm=q(vh))
        self.op("LDIF", imm=0)
        self.op("ST", out)

    def op_amp_filt_bp1m(self, out, src, r1, rf, c1, c2, r2=0, r3=0, vref=0.0, vp=12.0, vn=0.0):
        """DST_OP_AMP_FILT BAND_PASS_1M (non-Norton): MAME's bilinear band pass with the circuit gain, clipped"""
        rt = par(r1, r2, r3)
        fc = 1.0 / (2 * math.pi * math.sqrt(rt * rf * c1 * c2))
        d = (c1 + c2) / math.sqrt(rf / rt * c1 * c2)
        gain = -rf / rt * c2 / (c1 + c2)
        wc = FS * 2.0 * math.tan(math.pi * fc / FS)
        t2 = 2 * FS
        den = t2 * t2 + d * wc * t2 + wc * wc
        a1 = 2.0 * (-t2 * t2 + wc * wc) / den
        a2 = (t2 * t2 - d * wc * t2 + wc * wc) / den
        b0 = d * wc * t2 / den * gain
        self.load(src)
        self.op("ADDI", imm=q(-vref))
        self.op("MULI", imm=q(rt / r1))
        self.op("ST", out + "_x0")
        self.op("MULI", imm=q(b0))
        for reg, k in (("_x2", -b0), ("_y1", -a1), ("_y2", -a2)):
            self.op("MACI", out + reg, imm=q(k))
        self.op("ST", out + "_s")
        self.op("LD", out + "_x1"); self.op("ST", out + "_x2")
        self.op("LD", out + "_x0"); self.op("ST", out + "_x1")
        self.op("LD", out + "_y1"); self.op("ST", out + "_y2")
        self.op("LD", out + "_s"); self.op("ST", out + "_y1")
        self.op("ADDI", imm=q(vref))
        self.op("MAXI", imm=q(vn))
        self.op("MINI", imm=q(vp - RAIL))
        self.op("ST", out)

    def osc_norton1_dyn(self, out, r1, r2, c, vP, tl_reg, th_reg):
        """DSS_OP_AMP_OSC type 1 Norton (SQW) with thresholds from registers (r3 / r4 switched by data bits)"""
        vh = vP - VBE
        ch0 = vh / r1
        ch1 = (vh - VBE) / r2 - ch0
        self.kreg(out + "_d0", ch0 / FS / c, "O", clamp=False)
        self.kreg(out + "_d1", ch1 / FS / c, "O", clamp=False)
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        at = self.skip_if(True)
        self.op("LD", out + "_v")                     # charging
        self.op("ADD", out + "_d1")
        self.op("ST", out + "_v")
        self.op("CMP", th_reg)
        at2 = self.skip_if(False)
        self.op("SUB", th_reg)                        # overshoot -> discharge time (rates fixed: ratio is a constant)
        self.op("MULI", imm=q(-ch0 / ch1))
        self.op("ADD", th_reg)
        self.op("ST", out + "_v")
        self.op("LDI", imm=ONE)
        self.op("ST", out + "_lo")
        self.land(at2)
        self.op("LDI", imm=ONE)
        self.op("CMPI", imm=0)
        at3 = self.skip_if(True)
        self.land(at)
        self.op("LD", out + "_v")                     # discharging
        self.op("SUB", out + "_d0")
        self.op("ST", out + "_v")
        self.op("CMP", tl_reg)
        at4 = self.skip_if(True)
        self.op("SUB", tl_reg)
        self.op("MULI", imm=q(-ch1 / ch0))
        self.op("ADD", tl_reg)
        self.op("ST", out + "_v")
        self.op("LDI", imm=0)
        self.op("ST", out + "_lo")
        self.land(at4)
        self.land(at3)
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        self.op("LDI", imm=q(vh))
        self.op("LDIF", imm=0)
        self.op("ST", out)

    def uniform_noise(self, out, freq, amp, bias=0.0, n=1):
        """DSS_NOISE: a new uniform random value (4-bit) each 1 / freq, peak-to-peak amp"""
        self.kreg(out + "_inc", freq / FS, "O", clamp=False)
        self.op("LD", out + "_ph")
        self.op("ADD", out + "_inc")
        self.op("ST", out + "_ph")
        self.op("CMPI", imm=ONE)
        at = self.skip_if(False)
        self.op("ADDI", imm=-ONE)
        self.op("ST", out + "_ph")
        self.op("LDI", imm=q(bias - amp / 2))
        self.op("ST", out)
        for k in range(4):
            self.op("NOISE", 0, n)
            self.op("LD", out)
            self.op("ADDI", imm=q(amp * (1 << k) / 15))
            self.op("STF", out)
        self.land(at)

    def squarewfix(self, ph, freq):
        """DSS_SQUAREWFIX phase (50 % duty): reg ph runs 0 .. 1.0; high while ph >= 0.5"""
        self.op("LD", ph)
        self.op("ADDI", imm=q(freq / FS))
        self.op("CMPI", imm=ONE)
        self.op("ADDI", imm=-ONE)
        self.op("STF", ph)
        self.op("ADDI", imm=ONE)
        self.op("STNF", ph)

    def rcfilter(self, out, src, r, c):
        self.kreg(out + "_k", rc_exp(r * c), "F")
        self.load(src)
        self.op("RCM", out, self.r(out + "_k"))

    def crfilter(self, out, src, r, c):
        """high pass: out = in - cap; cap follows"""
        self.kreg(out + "_k", rc_exp(r * c), "F")
        self.load(src)
        self.op("SUB", out + "_c")
        self.op("ST", out)
        self.op("MUL", out + "_k")
        self.op("ADD", out + "_c")
        self.op("ST", out + "_c")

    def filter2(self, out, src, fc, d, kind="lp"):
        """DST_FILTER2 (bilinear, pre-warped), as MAME calculate_filter2_coefficients"""
        wc = FS * 2.0 * math.tan(math.pi * fc / FS)
        t2 = 2 * FS
        den = t2 * t2 + d * wc * t2 + wc * wc
        a1 = 2.0 * (-t2 * t2 + wc * wc) / den
        a2 = (t2 * t2 - d * wc * t2 + wc * wc) / den
        if kind == "lp":
            b0 = b2 = wc * wc / den
            b1 = 2.0 * b0
        elif kind == "bp":
            b0, b1, b2 = d * wc * t2 / den, 0.0, -d * wc * t2 / den
        else:
            b0 = b2 = t2 * t2 / den
            b1 = -2.0 * b0
        self.load(src)
        self.op("ST", out + "_x0")
        self.op("MULI", imm=q(b0))
        for reg, k in (("_x1", b1), ("_x2", b2), ("_y1", -a1), ("_y2", -a2)):
            self.op("MACI", out + reg, imm=q(k))
        self.op("ST", out + "_s")
        self.op("LD", out + "_x1"); self.op("ST", out + "_x2")
        self.op("LD", out + "_x0"); self.op("ST", out + "_x1")
        self.op("LD", out + "_y1"); self.op("ST", out + "_y2")
        self.op("LD", out + "_s"); self.op("ST", out + "_y1")
        self.op("ST", out)

    def gain(self, out, src, k):
        self.load(src)
        self.op("MULI", imm=q(k))
        self.op("ST", out)

    def mixer_op_amp(self, out, ins, rf, c_amp=0, vref=0.0):
        """DST_MIXER, DISC_MIXER_IS_OP_AMP: ins = [(src, r, c)], output high-passed by c_amp (100k)"""
        if c_amp:
            self.kreg(out + "_ka", rc_exp(100e3 * c_amp), "F")
        terms = [(k, inp[0], inp[1], inp[2], -rf / inp[1], len(inp) > 3 and inp[3])    # (vref - in) / r * rf, vref 0
                 for k, inp in enumerate(ins)]
        self._mix_sum(out, terms, vref)
        if c_amp:
            self.op("RCM", out + "_amp", self.r(out + "_ka"))
            self.op("LD", out + "_s")
            self.op("SUB", out + "_amp")
        self.op("ST", out)

    def jump(self):
        """unconditional forward skip (CMPI INT_MIN: F = 1); land() it like a skip"""
        self.op("CMPI", imm=-(1 << 31))
        return self.skip_if(True)

    def op_amp_norton_cap(self, out, inp1, r2, r3, r4, c, vP, v_on=12.0, vN=0.0, g="T"):
        """DST_OP_AMP, DISC_OP_AMP_IS_NORTON with a cap and no r1: i = max(0, (in1 - VBE) / r2) - (vP - VBE) / r3;
        r4: exponential charge toward i x r4 (tau r4 C), else linear (i / C); clamped vN .. vP - VBE.
        inp1 = (source, bit) at v_on / 0 V (INPUTX_LOGIC) or a register"""
        vmax = vP - VBE
        ifix = vmax / r3 if r3 else 0.0
        if r4:
            self.kreg(out + "_k", rc_exp(r4 * c), g)
            if isinstance(inp1, tuple):
                self.op("BIT", 0, self.src(*inp1))
                self.op("LDI", imm=q((max(0.0, -VBE) / r2 - ifix) * r4))
                self.op("LDIF", imm=q((max(0.0, v_on - VBE) / r2 - ifix) * r4))
            else:
                self.load(inp1)
                self.op("ADDI", imm=q(-VBE))
                self.op("MAXI", imm=0)
                self.op("MULI", imm=q(r4 / r2))
                self.op("ADDI", imm=q(-ifix * r4))
            self.op("RCM", out, self.r(out + "_k"))
        else:
            self.kreg(out + "_ka", 1.0 / (r2 * FS * c), g, clamp=False)
            self.kreg(out + "_kb", ifix / (FS * c), g, clamp=False)
            if isinstance(inp1, tuple):
                self.op("BIT", 0, self.src(*inp1))
                self.op("LDI", imm=0)
                self.op("LDIF", imm=q(max(0.0, v_on - VBE)))
            else:
                self.load(inp1)
                self.op("ADDI", imm=q(-VBE))
                self.op("MAXI", imm=0)
            self.op("MUL", out + "_ka")
            self.op("SUB", out + "_kb")
            self.op("ADD", out)
        self.op("MAXI", imm=q(vN))
        self.op("MINI", imm=q(vmax))
        self.op("ST", out)

    def op_amp_1sht_norton(self, out, trig, v_trig, r1, r2, r3, r4, r5, c1, c2, vP, vN=0.0, g="T"):
        """DST_OP_AMP_1SHT, DISC_OP_AMP_1SHT_1 | NORTON. trig = (source, bit) at v_trig / 0 V. Currents are kept
        x r2 (volts). out = vP - VBE while the trigger current (edge through c2 + feedback through r5) beats c1's
        current through r3 + the r1 bias; c1 charges toward the r3 / r4 divider, discharges through r4 or the diode"""
        vmax = vP - VBE
        self.kreg(out + "_k2", rc_exp(r2 * c2), g)
        self.kreg(out + "_kc", rc_exp(par(r3, r4) * c1), g)
        self.kreg(out + "_kd", rc_exp(r4 * c1), g)
        self.op("BIT", 0, self.src(*trig))
        self.op("LDI", imm=0)
        self.op("LDIF", imm=q(v_trig))
        self.op("ST", out + "_tv")
        self.op("SUB", out + "_c2")
        self.op("MACI", out, imm=q(r2 / r5))          # i_pos x r2, with the previous output
        self.op("ST", out + "_ip")
        self.op("LD", out + "_tv")
        self.op("RCM", out + "_c2", self.r(out + "_k2"))
        self.op("LD", out + "_c1")
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=q(r2 / r3))
        self.op("ADDI", imm=q(r2 * vmax / r1))        # i_neg x r2
        self.op("SUB", out + "_ip")
        self.op("CMPI", imm=0)                        # F = i_neg >= i_pos -> low
        self.op("LDI", imm=q(vmax))
        self.op("LDIF", imm=q(vN))
        self.op("ST", out)
        self.op("CMP", out + "_c1")                   # F = out >= c1: charge
        at = self.skip_if(True)
        self.op("ADDI", imm=q(0.6))                   # discharge: diode clamp at out + 0.6, else through r4
        self.op("CMP", out + "_c1")
        self.op("STNF", out + "_c1")
        at2 = self.skip_if(False)
        self.op("LD", out)
        self.op("RCM", out + "_c1", self.r(out + "_kd"))
        self.land(at2)
        at3 = self.jump()
        self.land(at)
        self.op("ADDI", imm=q(-VBE))                  # charge toward (out - VBE) r3 / (r3 + r4) + VBE
        self.op("MULI", imm=q(r3 / (r3 + r4)))
        self.op("ADDI", imm=q(VBE))
        self.op("RCM", out + "_c1", self.r(out + "_kc"))
        self.land(at3)

    def op_amp_vco3_norton_exact(self, out, vmod1, r1, r2, r3, r4, r5, c, vP, vmod2=None, r6=0, r7=0, g="O",
                                 vm1_gain=1.0, vm1_max=None, vm2_max=None, rcp=False):
        """DSS_OP_AMP_OSC VCO_3 | NORTON, OUT_SQW, linear charge, with MAME's exact overshoot carry (RCP of the rate
        at each threshold crossing) and its output rule (two toggles in a sample: high if it was discharging).
        Discharge current = r7 bias + max(0, vm1_gain x vmod1 - VBE) / r1 + (vmod2 - VBE) / r6, charge =
        (vP - 2 VBE) / r2 - discharge. A phase that can last under one sample gets the second toggle (asserted from
        the largest rates, Game Audio -30 % included). out = vP - VBE while charging; out_v = the cap, out_ch = 1
        while charging (power-up: charging from 0 V). rcp=True: the reciprocals come from out_r0 / out_r1 (refreshed
        elsewhere: slowly varying vmod), out_d0 / out_d1 then hold the rates for that"""
        vh = vP - VBE
        i1 = vh / r5
        tl = (i1 - VBE / r4) * r3 + VBE
        th = (i1 + (vP - 2 * VBE) / r4) * r3 + VBE
        vm1, vm2 = (vh if vm1_max is None else vm1_max), (vh if vm2_max is None else vm2_max)   # largest vmods
        ifix = vh / r7 if r7 else 0.0
        dmax = ifix + max(0.0, vm1 * vm1_gain - VBE) / r1 + ((vm2 - VBE) / r6 if r6 else 0.0)
        dmin = ifix + (-VBE / r6 if r6 else 0.0)
        it = (vP - 2 * VBE) / r2
        assert dmax < it, "charge current must stay positive"
        fast = 1 / 0.7 / (FS * c)                     # V per sample per A at Game Audio -30 %
        short_d = (th - tl) / (dmax * fast) <= 1.0
        short_c = (th - tl) / ((it - dmin) * fast) <= 1.0
        assert not (short_d and short_c), "both phases under a sample: more toggles needed"
        self.init(out + "_ch", 1.0)
        self.kreg(out + "_ka", vm1_gain / (r1 * FS * c), g, clamp=False)
        if ifix:
            self.kreg(out + "_kf", ifix / (FS * c), g, clamp=False)
        self.kreg(out + "_kt", it / (FS * c), g, clamp=False)
        if vmod2 is not None:
            self.kreg(out + "_k6", 1.0 / (r6 * FS * c), g, clamp=False)
        if vmod2 is not None:
            self.load(vmod2)
            self.op("ADDI", imm=q(-VBE))
            self.op("MUL", out + "_k6")
            self.op("ST", out + "_d0")
        self.load(vmod1)
        self.op("ADDI", imm=q(-VBE / vm1_gain))
        self.op("MAXI", imm=0)
        self.op("MUL", out + "_ka")
        if ifix:
            self.op("ADD", out + "_kf")
        if vmod2 is not None:
            self.op("ADD", out + "_d0")
        self.op("ST", out + "_d0")                    # discharge step (V / sample)
        self.op("LD", out + "_kt")
        self.op("SUB", out + "_d0")
        self.op("ST", out + "_d1")                    # charge step
        ends = []

        def cross_d():                                # acc = rem = (tl - v) / d0
            self.op("LDI", imm=q(tl)); self.op("SUB", out + "_v")
            if rcp:
                self.op("MUL", out + "_r0")
            else:
                self.op("ST", out + "_ov"); self.op("LD", out + "_d0"); self.op("RCP"); self.op("MUL", out + "_ov")

        def cross_c():                                # acc = v - th -> rem = (v - th) / d1
            self.op("ADDI", imm=q(-th))
            if rcp:
                self.op("MUL", out + "_r1")
            else:
                self.op("ST", out + "_ov"); self.op("LD", out + "_d1"); self.op("RCP"); self.op("MUL", out + "_ov")

        def term(ch, o):                              # final state, output; jump to the end
            if ch is not None:
                self.op("LDI", imm=ONE if ch else 0); self.op("ST", out + "_ch")
            self.op("LDI", imm=q(o)); self.op("ST", out)
            ends.append(self.jump())

        self.op("LD", out + "_ch")
        self.op("CMPI", imm=q(0.5))
        at_c = self.skip_if(True)
        # start discharging
        self.op("LD", out + "_v"); self.op("SUB", out + "_d0"); self.op("ST", out + "_v")
        self.op("CMPI", imm=q(tl))
        at = self.skip_if(False)
        term(None, 0.0)                               # no toggle
        self.land(at)
        cross_d()
        self.op("MUL", out + "_d1"); self.op("ADDI", imm=q(tl)); self.op("ST", out + "_v")     # charge the rest
        if short_c:
            self.op("CMPI", imm=q(th) + 1)
            at = self.skip_if(True)
            term(True, vh)
            self.land(at)
            cross_c()
            self.op("MUL", out + "_d0"); self.op("ST", out + "_ov")                           # discharge the rest
            self.op("LDI", imm=q(th)); self.op("SUB", out + "_ov"); self.op("ST", out + "_v")
            term(False, vh)                           # two toggles, was discharging: high
        else:
            term(True, vh)
        self.land(at_c)
        # start charging
        self.op("LD", out + "_v"); self.op("ADD", out + "_d1"); self.op("ST", out + "_v")
        self.op("CMPI", imm=q(th) + 1)
        at = self.skip_if(True)
        term(None, vh)
        self.land(at)
        cross_c()
        self.op("MUL", out + "_d0"); self.op("ST", out + "_ov")                               # discharge the rest
        self.op("LDI", imm=q(th)); self.op("SUB", out + "_ov"); self.op("ST", out + "_v")
        if short_d:
            self.op("CMPI", imm=q(tl))
            at = self.skip_if(False)
            term(False, 0.0)
            self.land(at)
            cross_d()
            self.op("MUL", out + "_d1"); self.op("ADDI", imm=q(tl)); self.op("ST", out + "_v")
            term(True, 0.0)                           # two toggles, was charging: low
        else:
            term(False, 0.0)
        del self.code[-2:]                            # the last jump (CMPI, SKF) would land right here
        ends.pop()
        for e in ends:
            self.land(e)

    def op_amp_norton_sqw(self, out, sqw, v_sq, inp1, r1, r2, r3, r4, vP, vN=0.0):
        """op_amp_norton (DST_OP_AMP Norton, no cap) for IN0 = a square wave that is exactly 0 or v_sq and IN1 >= 0:
        out = r4 x ((in1 - VBE) / r2 - in0 term - (vP - VBE) / r3), clamped (IN1's max(0, ..) only matters where the
        result clamps to vN = 0 anyway)"""
        vmax = vP - VBE
        assert vN == 0.0
        g2 = r4 / r2
        self.load(sqw)
        self.op("MULI", imm=q(-(v_sq - VBE) * r4 / r1 / v_sq / g2))
        self.op("ADD", inp1)
        if g2 != 1.0:
            self.op("MULI", imm=q(g2))
        self.op("ADDI", imm=q(-VBE * g2 - r4 * vmax / r3))
        self.op("MAXI", imm=0)
        self.op("MINI", imm=q(vmax))
        self.op("ST", out)

    def ne555_astable2(self, outs, en, parts, level, g="O"):
        """DSD_555_ASTBL x n sharing one RESET (no CV, no fast-charge diode), outputs ANDed (DISCRETE_LOGIC_AND with
        the enable) at `level`: exact phase timers, high (r1 + r2) C ln 2 (ln 3 on the first cycle out of reset: the
        cap starts at 0 V), low r2 C ln 2. en = (source, bit): low = RESET. parts = [(r1, r2, c)]; outs[0] = the
        AND, outs[k]_p the phases"""
        U = 0.01                                      # timer unit: 10 ms
        o = outs[0]
        self.kreg(o + "_dt", 1.0 / (FS * U), g, clamp=False)
        tm = [((r1 + r2) * c * math.log(2.0) / U, r2 * c * math.log(2.0) / U, (r1 + r2) * c * math.log(3.0) / U)
              for r1, r2, c in parts]
        self.op("BIT", 0, self.src(*en))
        at = self.skip_if(True)
        for k, (th, tl, t1) in enumerate(tm):         # reset: phase -(first high - high): high for the first cycle
            self.op("LDI", imm=q(th - t1))
            self.op("ST", f"{o}_p{k}")
        self.op("LDI", imm=0)
        self.op("ST", o)
        at2 = self.jump()
        self.land(at)
        for k, (th, tl, t1) in enumerate(tm):         # phase 0 .. high + low, high while below the high time
            p = f"{o}_p{k}"
            self.op("LD", p)
            self.op("ADD", o + "_dt")
            self.op("CMPI", imm=q(th + tl))
            self.op("ADDI", imm=q(-(th + tl)))
            self.op("STF", p)
            self.op("ADDI", imm=q(th + tl))
            self.op("STNF", p)
        for k, (th, tl, t1) in enumerate(tm):
            self.op("LD", f"{o}_p{k}")
            self.op("CMPI", imm=q(th))
            if k == 0:
                self.op("LDI", imm=q(level))
                self.op("LDIF", imm=0)
                self.op("ST", o)
            else:
                self.op("LDI", imm=0)
                self.op("STF", o)
        self.land(at2)

    def sn76477_slf_vco(self, out, inhibit, slf_res, slf_cap, vco_res, vco_cap, amp_res, fb_res, scale=1.0, g="O"):
        """SN76477 as MAME sn76477_device (measured formulas): SLF triangle 0.33 .. 2.37 V sweeps the VCO top (VCO
        mode 1), mixer = VCO, envelope one-shot with no one-shot parts (attack instant, full level after the first
        enable), pitch 5 V (50 % duty). inhibit = (source, bit): high = ENABLE pin high (silent, out 0).
        out = MAME's stream value x scale. Rate registers carry the flip-flops (negative = discharging); the VCO cap
        is kept 0.35 V down (its top = the SLF voltage)"""
        slf_lo, slf_hi, vco_lo, dv = 0.33, 2.37, 0.33, 0.35
        rng = slf_hi - slf_lo
        up = rng / (0.5885 * slf_res * slf_cap + 0.001300) / FS
        dn = rng / (0.5413 * slf_res * slf_cap + 0.001343) / FS
        st = 0.64 * 2 * (slf_hi + dv - vco_lo) / (vco_res * vco_cap) / FS
        cp = 3.818 * (fb_res / amp_res) + 0.03       # centre to peak
        hi = min(2.57 + cp * 1.00, 3.51)              # gain tables at the 4.44 V A/D cap
        lo = max(2.57 + cp * -0.85, 0.715)
        to_fs = lambda v: ((v - 0.715) / (2.57 - 0.715) - 1.0) * scale
        self.init(out + "_s", slf_lo)                 # rates power up 0: the low-limit test starts both charging
        self.init(out + "_v", vco_lo - dv)
        self.kreg(out + "_up", up, g, clamp=False)
        self.kreg(out + "_dn", -dn, g, clamp=False)
        self.kreg(out + "_st", st, g, clamp=False)
        self.kreg(out + "_sn", -st, g, clamp=False)
        self.op("LD", out + "_s")                     # SLF
        self.op("ADD", out + "_sr")
        self.op("MINI", imm=q(slf_hi))
        self.op("MAXI", imm=q(slf_lo))
        self.op("ST", out + "_s")
        self.op("CMPI", imm=q(slf_hi))
        self.op("LD", out + "_dn")
        self.op("STF", out + "_sr")
        self.op("LD", out + "_s")
        self.op("CMPI", imm=q(slf_lo) + 1)
        self.op("LD", out + "_up")
        self.op("STNF", out + "_sr")
        self.op("LD", out + "_v")                     # VCO: up to the SLF voltage, down to the low limit
        self.op("ADD", out + "_vr")
        self.op("CMP", out + "_s")
        self.op("LDF", out + "_s")
        self.op("MAXI", imm=q(vco_lo - dv))
        self.op("ST", out + "_v")
        self.op("CMP", out + "_s")
        self.op("LD", out + "_sn")
        self.op("STF", out + "_vr")
        self.op("LD", out + "_v")
        self.op("CMPI", imm=q(vco_lo - dv) + 1)
        self.op("LD", out + "_st")
        self.op("STNF", out + "_vr")
        self.op("LD", out + "_vr")                    # output: VCO flip-flop high (discharging) -> + peak
        self.op("CMPI", imm=0)
        self.op("LDI", imm=q(to_fs(hi)))
        self.op("LDIF", imm=q(to_fs(lo)))
        self.op("ST", out)
        self.op("BIT", 0, self.src(*inhibit))
        self.op("LDI", imm=0)
        self.op("STF", out)

    def output(self, src, scale):
        """OUT: src volts * scale, 1.0 = full scale"""
        self.load(src)
        self.op("MULI", imm=q(scale))
        self.op("OUT")

    def finish(self):
        """wrap: init block guarded by M[255], body, END"""
        body = self.code
        self.code = []
        tweak = bool(self.groups or self.kcode or self.music)    # no Game Audio settings used: no change block
        if tweak:
            self.inits.append(("_glast", -1.0))
        if self.inits:
            self.op("LD", 255)
            self.op("CMPI", imm=1)
            at = self.skip_if(True)
            for reg, v in self.inits:
                self.op("LDI", imm=q(v))
                self.op("ST", reg)
            self.op("LDI", imm=1)
            self.op("ST", 255)
            self.land(at)
        if tweak:
            # Game Audio settings changed (or first sample: _glast powers up as -1)? -> group factors + scaled rates
            self.op("LDLM", 0, self.src(6, 0), imm=0xFF)
            self.op("ST", "_t")
            self.op("LDLM", 0, self.src(7, 0), imm=0xFF | (8 << 8))
            self.op("ADD", "_t")
            self.op("ST", "_key")
            self.op("SUB", "_glast")
            self.op("ABS")
            self.op("CMPI", imm=1)
            at = self.skip_if(False)
            for g in sorted(self.groups):
                src, mask, sh = self.GROUP_SRC[g]
                self.chain("_f" + g, src, mask, sh, [1.0 / sc for sc in self.SCALES])
            aged = sorted(self.groups & {"T", "F"})
            if aged:                                  # Aged Caps (src7 bits 4:3) on the timing / coupling caps
                self.chain("_fA", 7, 0x18, 13, [1.0 / a for a in self.AGED])
                for g in aged:
                    self.op("LD", "_f" + g)
                    self.op("MUL", "_fA")
                    self.op("ST", "_f" + g)
            if self.music:                            # Music Volume (src7 bits 7:5)
                self.chain("_fM", 7, 0xE0, 11, self.MUSIC)
            self.code += self.kcode
            self.op("LD", "_key")
            self.op("ST", "_glast")
            self.op("END")                            # a settings change costs one (silent) sample
            self.land(at)
        self.code += body
        self.op("END")
        worst = self.worst_path()
        assert worst <= 410, f"{self.name}: a sample can use {worst} budget units (engine budget 410)"
        return self

    def worst_path(self):
        """most budget units any sample can use (EXP 4, RCP 5, others 1): longest path through the forward skips"""
        n = len(self.code)
        L = [0] * (n + 1)
        for i in range(n - 1, -1, -1):
            o, _, _, imm = self.code[i]
            if o == OPS["END"]:
                L[i] = 1
            elif o in (OPS["SKF"], OPS["SKNF"]):
                j = i + 1 + imm
                always = o == OPS["SKF"] and i and self.code[i - 1][0] == OPS["CMPI"] and self.code[i - 1][3] == 1 << 31
                L[i] = 1 + (L[j] if always else max(L[i + 1], L[j] if j < n else 0))   # CMPI INT_MIN; SKF = jump
            else:
                L[i] = COST.get(o, 1) + L[i + 1]
        return L[0]
