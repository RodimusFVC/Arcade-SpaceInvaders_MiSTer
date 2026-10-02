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
           FAND=26, FOR=27, STNF=28, LDLM=29)
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
        assert len(self.code) <= 512, self.name
        return b"".join(struct.pack("<Q", (o << 56) | (a << 48) | (b << 40) | imm) for o, a, b, imm in self.code)


# ---------------------------------------------------------------- MAME discrete node equivalents
# (MAME src/devices/sound/disc_*.hxx, D. Renaud et al.). Signals live in named state registers as Q24 volts;
# logic signals are 0 / 1.0. Constants are worked out here exactly as the MAME reset functions do.

VBE = 0.5           # OP_AMP_NORTON_VBE
RAIL = 1.5          # OP_AMP_VP_RAIL_OFFSET
FN_NONE, FN_TRG0, FN_TRG0_INV, FN_TRG1, FN_TRG1_INV, FN_TRG2, FN_TRG2_INV, FN_TRG01_AND, FN_TRG01_NAND = range(9)


def rc_exp(rc):
    return 1.0 if rc <= 0 else 1.0 - math.exp(-1.0 / (FS * rc))


def par(*rs):
    return 1.0 / sum(1.0 / r for r in rs if r)


class Board(Prog):
    """A board program: init block (runs once), then the per-sample body."""

    def __init__(self, name):
        super().__init__(name)
        self.inits = []

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

    def logic_reg(self, reg, fn, trig):
        """reg = dst_trigger_function(trig0, trig1, trig2, fn) as 0 / 1.0; returns the constant if it is one"""
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
    def midway_tone(self, out, lo, hi, clock=19968000 / 10 / 2):
        """DISCRETE_NOTE fed by the Midway tone latches: lo = (source, ) D5-D1, hi = D5-D0; out = 0 / 1.0"""
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
        for _ in range(2):                            # up to two toggles a sample (anything faster is ultrasonic)
            self.op("LD", n + "_ph")
            self.op("CMP", n + "_p")
            self.op("SUB", n + "_p")
            self.op("STF", n + "_ph")
            self.op("LDI", imm=ONE)
            self.op("SUB", out)
            self.op("STF", out)

    def lfsr_noise(self, out, freq, amp, bias, n=0):
        """DSS_LFSR_NOISE clocked at freq: out = bias +/- amp / 2, held between clocks"""
        self.op("LD", out + "_ph")
        self.op("ADDI", imm=q(freq / FS))
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
        # i2 / i3 input terms, scaled by r4 into volts
        self.op("LDI", imm=q(fixed))
        self.op("ST", out + "_n")
        for rk, fk, ik in (("r2", "f0", 0), ("r3", "f1", 1)):
            if not g(rk):
                continue
            c = self.logic_reg(out + "_g", g(fk), trig)
            if c is False:
                continue
            self.load(inp[ik])
            self.op("ADDI", imm=q(-VBE))
            self.op("MAXI", imm=0)
            self.op("MULI", imm=q(r4 / g(rk)))
            if c is None:
                self.op("MUL", out + "_g")
            self.op("ADD", out + "_n")
            self.op("ST", out + "_n")
        f3 = self.logic_reg(out + "_f3", g("f3"), trig)
        assert f3 is True, "dynamic F3 not needed by the boards so far"
        # c1: charge toward vt1 while F2 is open, else discharge toward VBE through r6 + r7
        c2 = self.logic_reg(out + "_f2", g("f2"), trig)
        if c2 is None:
            self.op("LD", out + "_f2"); self.op("CMPI", imm=q(0.5))
            self.op("LDI", imm=q(kd1)); self.op("LDIF", imm=q(kc1)); self.op("ST", out + "_k")
            self.op("LDI", imm=q(VBE)); self.op("LDIF", imm=q(vt1))
            self.op("RCM", out + "_c1", self.r(out + "_k"))
        else:
            self.op("LDI", imm=q(vt1 if c2 else VBE))
            self.op("RC", out + "_c1", imm=q(kc1 if c2 else kd1))
        self.op("ADDI", imm=q(-VBE))
        self.op("MAXI", imm=0)
        self.op("MULI", imm=q(r4 / r67))
        self.op("ST", out + "_p")
        for rv, rr, cc, vk, fk in (("r8", "r9", "c2", "v2", "f4"), ("r10", "r11", "c3", "v3", "f5")):
            if not g(rr):
                continue
            vt = (g(vk) - 0.6 - VBE) * div(g(rv), g(rr))
            k0, k1 = rc_exp(g(rr) * g(cc)), rc_exp(par(g(rv), g(rr)) * g(cc))
            cf = self.logic_reg(out + "_g", g(fk), trig)
            if cf is None:
                self.op("LD", out + "_g"); self.op("CMPI", imm=q(0.5))
                self.op("LDI", imm=q(k0)); self.op("LDIF", imm=q(k1)); self.op("ST", out + "_k")
                self.op("LDI", imm=0); self.op("LDIF", imm=q(vt))
                self.op("RCM", out + "_" + cc, self.r(out + "_k"))
            else:
                self.op("LDI", imm=q(vt if cf else 0))
                self.op("RC", out + "_" + cc, imm=q(k1 if cf else k0))
            self.op("MULI", imm=q(r4 / g(rr)))
            self.op("ADD", out + "_p")
            self.op("ST", out + "_p")
        self.op("LD", out + "_p")
        self.op("SUB", out + "_n")
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
        # state: _v cap voltage, _lo = 1.0 while discharging (power-up: charging, MAME m_flip_flop = 1)
        self.op("LD", out + "_lo")
        self.op("CMPI", imm=q(0.5))
        at = self.skip_if(True)
        self.op("LD", out + "_v")                     # charging
        self.op("ADDI", imm=q(d1))
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
        self.op("ADDI", imm=q(-d0))
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
        self.load(src)
        self.op("ADDI", imm=q(-vref))
        self.op("MULI", imm=q(rt / r1))               # Millman: (in - vref) / r1 * rTotal
        self.op("ST", out + "_v")
        self.op("SUB", out + "_c2")
        self.op("ST", out + "_hp")                    # v - vC2 (before vC2 moves)
        self.op("LD", out + "_v")
        self.op("RC", out + "_c2", imm=q(rc_exp(rt * c2)))
        self.op("LD", out + "_hp")
        self.op("RC", out + "_c1", imm=q(rc_exp(rf * c1)))
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
        self.load(src)
        self.op("RC", out, imm=q(rc_exp(r * c)))

    def crfilter(self, out, src, r, c):
        """high pass: out = in - cap; cap follows"""
        self.load(src)
        self.op("SUB", out + "_c")
        self.op("ST", out)
        self.op("MULI", imm=q(rc_exp(r * c)))
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
        self.op("MULI", imm=q(b0)); self.op("ST", out + "_s")
        for reg, k in (("_x1", b1), ("_x2", b2), ("_y1", -a1), ("_y2", -a2)):
            self.op("LD", out + reg); self.op("MULI", imm=q(k)); self.op("ADD", out + "_s"); self.op("ST", out + "_s")
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
        self.op("LDI", imm=0)
        self.op("ST", out + "_s")
        for k, (src, r, c) in enumerate(ins):
            if src is None:
                continue
            self.load(src)
            if c:
                self.op("ADDI", imm=q(-vref))
                self.op("RC", f"{out}_hp{k}", imm=q(rc_exp(r * c)))
                self.load(src)
                self.op("SUB", f"{out}_hp{k}")
            self.op("MULI", imm=q(-rf / r))          # (vref - in) / r * rf, vref = 0
            self.op("ADD", out + "_s")
            self.op("ST", out + "_s")
        if c_amp:
            self.op("RC", out + "_amp", imm=q(rc_exp(100e3 * c_amp)))
            self.op("LD", out + "_s")
            self.op("SUB", out + "_amp")
        self.op("ST", out)

    def output(self, src, scale):
        """OUT: src volts * scale, 1.0 = full scale"""
        self.load(src)
        self.op("MULI", imm=q(scale))
        self.op("OUT")

    def finish(self):
        """wrap: init block guarded by M[255], body, END"""
        body = self.code
        self.code = []
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
        self.code += body
        self.op("END")
        return self
