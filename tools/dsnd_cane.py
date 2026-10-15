"""Cane (Model Racing prototype, 1979?) sound for the discrete sound engine (tools/dsnd.py, rtl/dsnd_engine.sv).

No schematic exists (unreleased prototype; MAME's driver is derived from the game's source code). Reference = MAME
8080bw_a.cpp CANE_AUDIO (SN76477 + 555 one-shot noise + TOS music, J.P. Piccato), node for node:
  port 3 (LATCH1): D0 SX0 mixer enable (ADDER3 enable: 0 = everything silent), D1 / D2 / D3 = SN76477 mixer C / A / B,
                   D4 SX4 = 555 one-shot trigger (falling edge, 50 ms, retriggerable) gating 1280 Hz noise (guessed)
  port 5 (LATCH2): TOS note = LS273 -> 2 x LS161 preload: count from D up to 255 at 1.248 MHz, J-K toggle (DISCRETE_NOTE),
                   D = 255 stops the counter (pause); 10k / 0.1 uF high pass; VR2
  SN76477 (VR1): MAME's measured model: noise 39k / 1k / 1000 pF (29.4 kHz clock), SLF 1 uF / 33k (25 Hz), VCO 3300 pF
                   / 100k external, one-shot envelope 10 uF / 100k (0.80 s), attack 1 uF / 47k, decay 1M,
                   amplitude 20.1k / feedback 4.7k; output = MAME stream value (gain tables in 0.1 V steps)
  SN enable (MAME's code): every port-3 write pulses ENABLE (one-shot + attack restart); every port-5 write inhibits
                   the SN until the next port-3 write. Port 5 is rewritten at <= 34 ms intervals during play (MAME log),
                   so MAME's SN is only short blips after each port-3 write.
The engine sees latch values, not write strobes. A port-3 write is seen when LATCH1 changes, a port-5 write when
LATCH2 changes; the repeated port-5 writes of 255 need a write strobe (ROUTING, shared RTL change). Until it exists the
SN window closes after `sn_window` s (default 36 ms: the longest port-3 -> port-5 gap measured in play), or at a
LATCH2 change. With the strobe routed, the program follows MAME's gating exactly (the window never closes first).
MAME bug: sh_port_1_w sets the mixer with set_mixer_params(), a config-time setter that never reaches the live mixer,
so MAME's SN always runs in mode 0 (VCO, ~24 kHz here with the ~0 V control voltage): MAME's SN blips are near-Nyquist
and inaudible. Default = the port-selected mode the driver intends (D1 C, D2 A, D3 B); mame_mixer=True = MAME as it
runs (matches the MAME capture sample for sample: corr 0.9996 with the strobes, verilator/snd/cane/).
Levels: MAME DISCRETE_OUTPUT gain 1 (/ 32768), route 1.0: SFX x VR3 13860, SN x 0.5 x 32768 x VR1 1.584, music x VR2
17820 (MAME adjuster defaults 70 / 80 / 90 %). No output high pass (MAME has none).
MAME guesses / doubts kept as MAME: 555 noise 1280 Hz "Guessed"; SN amplitude resistor (470k pot, MAME 20.1k); VCO
control voltage 5 (1 - exp(-t / 47)) with t in SECONDS since the last port-3 write (47 s; 47k x 1 uF would be 47 ms);
the game never uses an SN mixer mode with the VCO while SX0 is on, so the VCO choice is inaudible in this game.
"""
import math
from dsnd import Board, LATCH1, LATCH2, MISC, q, ONE, FS

P5_STROBE_BIT, P3_STROBE_BIT = 2, 3                  # MISC (source 5) bits that would toggle on each port 5 / 3 write
ROUTING = ("cane: MISC (src 5) bit 2 toggles on every port-5 write, bit 3 on every port-3 write "
           "(bit 3 optional: only same-value port-3 rewrites, in mixer-inhibit mode, depend on it)")
TOS_CLOCK = 19968000.0 / 16                          # CANE_H64 = 1.248 MHz: 26 counts per 48 kHz sample
VR1, VR2, VR3 = 0.80 * 0.33 * 6, 0.90 * 0.33 * 60000, 0.70 * 0.33 * 60000
K_SX, K_SN, K_MU = VR3 / 32768, 0.5 * VR1, VR2 / 32768   # node -> engine full scale (1.0 = 32768)
STEP = 2.0 ** -12                                    # one-shot / window timers: 1 sample
# SN76477 (MAME sn76477.cpp measured formulas and constants)
OS_MAX, SLF_LO, SLF_HI, VCO_LO, VCO_DV, VCO_SAT, AD_MAX = 2.5, 0.33, 2.37, 0.33, 0.35, 2.72, 4.44
R_NCLK, R_NFLT, C_NFLT = 39e3, 1e3, 1000e-12
C_SLF, R_SLF, C_VCO, R_VCO = 1e-6, 33e3, 3300e-12, 100e3
C_OS, R_OS, C_AD, R_ATT, R_DEC = 10e-6, 100e3, 1e-6, 47e3, 1e6
R_AMP, R_FB = 100 + 20e3, 4.7e3
CP = 3.818 * (R_FB / R_AMP) + 0.03                   # centre-to-peak volts
# gain tables (out_pos_gain / out_neg_gain, index = floor(A/D cap x 10)): exact for index <= 10, least-squares line
# for 11..44 (max error 0.017 / 0.011)
GP_A, GP_B, GN_A, GN_B = -0.17462, 0.02676, 0.15403, -0.02287
GLOW = {9: (0.01, -0.01), 10: (0.03, -0.02)}


def sn_rates():
    noise = 339100000 * R_NCLK ** -0.8849
    os_up = (OS_MAX - 0.0) / (0.8024 * R_OS * C_OS + 0.002079)
    os_dn = (OS_MAX - 0.0) / (854.7 * C_OS + 0.00001795)
    slf_up = (SLF_HI - SLF_LO) / (0.5885 * R_SLF * C_SLF + 0.001300)
    slf_dn = (SLF_HI - SLF_LO) / (0.5413 * R_SLF * C_SLF + 0.001343)
    vco = 0.64 * 2 * (SLF_HI + VCO_DV - VCO_LO) / (R_VCO * C_VCO)      # 50 % duty (pitch 5 V)
    ad_up, ad_dn = AD_MAX / (R_ATT * C_AD), AD_MAX / (R_DEC * C_AD)
    # the noise filter (1k x 1000 pF) swings rail to rail within a sample: filtered bit = NOT the generator bit
    assert 5.0 / (0.1571 * R_NFLT * C_NFLT + 0.00001430) / FS > 5.0 and 5.0 / (0.1331 * R_NFLT * C_NFLT + 0.00001734) / FS > 5.0
    return dict(noise=noise, os_up=os_up, os_dn=os_dn, slf_up=slf_up, slf_dn=slf_dn, vco=vco, ad_up=ad_up, ad_dn=ad_dn)


def write_event(b, out, latch, strobe_bit):
    """F = this sample sees a write: the latch value or the strobe toggle bit differs from the last sample"""
    b.op("LDLM", 0, b.src(MISC, 0), imm=(1 << strobe_bit) | ((24 - strobe_bit) << 8))   # toggle bit -> 1.0
    b.op("ST", "_t")
    b.op("LDL", 0, b.src(latch, 0))                 # latch byte / 256
    b.op("ADD", "_t")
    b.op("ST", out + "_n")
    b.op("SUB", out + "_o")
    b.op("ABS")
    b.op("CMPI", imm=1)
    b.op("LD", out + "_n")
    b.op("ST", out + "_o")                           # (F kept)


def music(b):
    """DISCRETE_NOTE(1, 1.248 MHz, port 5, max1 255, max2 1) -> CRFILTER 10k / 0.1 uF; counts / 256 in Q24"""
    inc = TOS_CLOCK / FS
    assert inc == 26.0
    b.op("LDLM", 0, b.src(LATCH2, 0), imm=0xFF | (16 << 8))
    b.op("ST", "cn_md")
    b.op("CMPI", imm=q(255 / 256))                 # data == max1: the counter holds (pause)
    at = b.skip_if(True)
    b.op("LD", "cn_mc")
    b.op("ADDI", imm=q(inc / 256))
    b.op("ST", "cn_mc")
    for _ in range(2):                               # a second reload only for data > 230 (ultrasonic)
        b.op("LD", "cn_mc")
        b.op("CMPI", imm=ONE)                        # count > 255: reload with the data, toggle the J-K
        b.op("ADDI", imm=-ONE)
        b.op("ADD", "cn_md")
        b.op("STF", "cn_mc")
        b.op("LDI", imm=ONE)
        b.op("SUB", "cn_mq")
        b.op("STF", "cn_mq")
    b.land(at)
    b.crfilter("cn_mo", "cn_mq", 10e3, 0.1e-6)       # out = q - cap; cap += out x k   (F group)
    b.music = True
    b.op("LD", "cn_mo")
    b.op("MUL", "_fM")                               # Music Volume (Game Audio)
    b.op("ST", "cn_mo")


def sfx(b):
    """555 one-shot (DISCRETE_ONESHOT falling edge of SX4, 50 ms, retrigger) x clamped DISCRETE_NOISE 1280 Hz"""
    b.kreg("cn_xdt", STEP, "T", clamp=False)
    b.kreg("cn_xinc", 1280.0 / FS, "O", clamp=False)
    b.op("BIT", 0, b.src(LATCH1, 4))
    b.op("LDI", imm=0)
    b.op("LDIF", imm=ONE)
    b.op("ST", "cn_xb")
    b.op("LD", "cn_xp")                              # previous - now = 1.0 on a falling edge
    b.op("SUB", "cn_xb")
    b.op("CMPI", imm=q(0.5))
    b.op("LD", "cn_xb")
    b.op("ST", "cn_xp")
    b.op("LD", "cn_xt")                              # countdown, not on the trigger sample
    b.op("SUB", "cn_xdt")
    b.op("MAXI", imm=0)
    b.op("LDIF", imm=q(round(0.05 * FS) * STEP))
    b.op("ST", "cn_xt")
    # DSS_NOISE amp 1 bias 0 (uniform -0.5 .. 0.5 at 1280 Hz), CLAMP 0 .. 5: 16 levels (k + 0.5) / 16 - 0.5
    b.op("LD", "cn_xph")
    b.op("ADD", "cn_xinc")
    b.op("ST", "cn_xph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE)
    b.op("ST", "cn_xph")
    b.op("LDI", imm=q(0.5 / 16 - 0.5))
    b.op("ST", "cn_xn")
    for k in range(4):
        b.op("NOISE", 0, 1)
        b.op("LD", "cn_xn")
        b.op("ADDI", imm=q((1 << k) / 16))
        b.op("STF", "cn_xn")
    b.op("LD", "cn_xn")
    b.op("MAXI", imm=0)
    b.op("ST", "cn_xn")
    b.land(at)
    b.op("LD", "cn_xt")
    b.op("CMPI", imm=1)
    b.op("LDI", imm=0)
    b.op("LDF", "cn_xn")
    b.op("ST", "cn_xo")


def sn76477(b, sn_window, mame_mixer):
    r = sn_rates()
    b.kreg("cn_nin", r["noise"] / FS, "O", clamp=False)
    b.kreg("cn_oup", r["os_up"] / FS, "T", clamp=False)
    b.kreg("cn_odn", -r["os_dn"] / FS, "T", clamp=False)
    b.kreg("cn_sup", r["slf_up"] / FS, "O", clamp=False)
    b.kreg("cn_sdn", -r["slf_dn"] / FS, "O", clamp=False)
    b.kreg("cn_vup", r["vco"] / FS, "O", clamp=False)
    b.kreg("cn_vdn", -r["vco"] / FS, "O", clamp=False)
    b.kreg("cn_aup", r["ad_up"] / FS, "T", clamp=False)
    b.kreg("cn_adn", -r["ad_dn"] / FS, "T", clamp=False)
    b.init("cn_s", SLF_LO)
    b.init("cn_v", VCO_LO)
    b.init("cn_vm", VCO_DV)
    b.init("cn_en", 1.0)                             # set_enable(0): enabled at power-up
    b.init("cn_or", -r["os_dn"] / FS)                # one-shot idle (discharging)

    # --- writes since the last sample: port 5 inhibits, port 3 pulses ENABLE (port 3 wins within one sample)
    write_event(b, "cn_w5", LATCH2, P5_STROBE_BIT)
    b.op("LDI", imm=0)
    b.op("STF", "cn_en")
    write_event(b, "cn_w3", LATCH1, P3_STROBE_BIT)
    b.op("LDI", imm=ONE)
    b.op("STF", "cn_en")
    b.op("LD", "cn_oup")
    b.op("STF", "cn_or")                             # one-shot runs (cap not reset)
    b.op("LDI", imm=0)
    b.op("STF", "cn_ad")                             # attack restarts from 0 V
    b.op("STF", "cn_vv")                             # VCO control timer restarts: 0 V
    b.op("STF", "cn_vt")
    b.op("LDI", imm=q(VCO_DV))
    b.op("STF", "cn_vm")
    if sn_window:
        b.op("LDI", imm=q(round(sn_window * FS) * STEP))
        b.op("STF", "cn_win")
        b.op("LD", "cn_win")                         # no port-5 strobe yet: close the window after sn_window
        b.op("ADDI", imm=-q(STEP))
        b.op("MAXI", imm=0)
        b.op("ST", "cn_win")
        b.op("CMPI", imm=1)
        b.op("LDI", imm=0)
        b.op("STNF", "cn_en")

    # --- VCO control voltage 5 (1 - exp(-t / 47 s)), MAME's 1 kHz timer
    b.op("LD", "cn_vt")
    b.op("ADDI", imm=q(1000.0 / FS))
    b.op("CMPI", imm=ONE)
    b.op("ADDI", imm=-ONE)
    b.op("STF", "cn_vt")
    b.op("ADDI", imm=ONE)
    b.op("STNF", "cn_vt")
    at = b.skip_if(False)
    b.op("LDI", imm=q(5.0))
    b.op("RC", "cn_vv", imm=q(1.0 - math.exp(-0.001 / 47.0)))
    b.op("ADDI", imm=q(VCO_DV))
    b.op("ST", "cn_vm")
    b.land(at)

    # --- one-shot cap (rate register: + charging while running, - discharging)
    b.op("LD", "cn_os")
    b.op("ADD", "cn_or")
    b.op("MINI", imm=q(OS_MAX))
    b.op("MAXI", imm=0)
    b.op("ST", "cn_os")
    b.op("CMPI", imm=q(OS_MAX))
    b.op("LD", "cn_odn")
    b.op("STF", "cn_or")
    # --- SLF triangle 0.33 .. 2.37 V (rate register; flip-flop = discharging)
    b.op("LD", "cn_s")
    b.op("ADD", "cn_sr")
    b.op("MINI", imm=q(SLF_HI))
    b.op("MAXI", imm=q(SLF_LO))
    b.op("ST", "cn_s")
    b.op("CMPI", imm=q(SLF_HI))
    b.op("LD", "cn_sdn")
    b.op("STF", "cn_sr")
    b.op("LD", "cn_s")
    b.op("CMPI", imm=q(SLF_LO) + 1)
    b.op("LD", "cn_sup")
    b.op("STNF", "cn_sr")
    # --- VCO (external control): 0.33 V .. control + 0.35 V
    b.op("LD", "cn_v")
    b.op("ADD", "cn_vr")
    b.op("CMP", "cn_vm")
    b.op("LDF", "cn_vm")
    b.op("MAXI", imm=q(VCO_LO))
    b.op("ST", "cn_v")
    b.op("CMP", "cn_vm")
    b.op("LD", "cn_vdn")
    b.op("STF", "cn_vr")
    b.op("LD", "cn_v")
    b.op("CMPI", imm=q(VCO_LO) + 1)
    b.op("LD", "cn_vup")
    b.op("STNF", "cn_vr")
    # --- noise generator (31-bit LFSR, taps 28 / 0 as MAME) at 29.4 kHz; filtered bit = NOT the generator bit
    b.op("LD", "cn_nph")
    b.op("ADD", "cn_nin")
    b.op("ST", "cn_nph")
    b.op("CMPI", imm=ONE)
    at = b.skip_if(False)
    b.op("ADDI", imm=-ONE)
    b.op("ST", "cn_nph")
    b.op("NOISE", 0, 0)
    b.op("LDI", imm=ONE)
    b.op("LDIF", imm=0)
    b.op("ST", "cn_N")
    b.land(at)
    # --- attack / decay cap (one-shot envelope mode: charging while the one-shot runs)
    b.op("LD", "cn_or")
    b.op("CMPI", imm=1)
    b.op("LD", "cn_adn")
    b.op("LDF", "cn_aup")
    b.op("ADD", "cn_ad")
    b.op("MINI", imm=q(AD_MAX))
    b.op("MAXI", imm=0)
    b.op("ST", "cn_ad")
    # --- gains: index n = floor(10 x A/D); n << 12 via the Q24 truncation
    b.op("MULI", imm=q(10.0))
    b.op("MULI", imm=1)                              # floor -> raw integer n
    b.op("MULI", imm=1 << 30)
    b.op("MULI", imm=1 << 30)                        # n << 12
    b.op("ST", "cn_n")
    b.op("MULI", imm=int(round(GP_B * 2 ** 36)))
    b.op("ADDI", imm=q(GP_A))
    b.op("ST", "cn_gp")
    b.op("LD", "cn_n")
    b.op("MULI", imm=int(round(GN_B * 2 ** 36)))
    b.op("ADDI", imm=q(GN_A))
    b.op("ST", "cn_gn")
    b.op("LD", "cn_n")
    b.op("CMPI", imm=11 << 12)
    at = b.skip_if(True)
    b.op("LDI", imm=0)
    b.op("ST", "cn_gp")
    b.op("ST", "cn_gn")
    for n, (gp, gn) in sorted(GLOW.items()):
        b.op("LD", "cn_n")
        b.op("CMPI", imm=n << 12)
        b.op("LDI", imm=q(gp))
        b.op("STF", "cn_gp")
        b.op("LDI", imm=q(gn))
        b.op("STF", "cn_gn")
    b.land(at)
    # --- mixer select: p = B (D3) x 4 + A (D2) x 2 + C (D1); mode = C x 4 + B x 2 + A
    # components (1.0 / 0): V = VCO flip-flop (discharging), S = SLF flip-flop (discharging), N = filtered noise
    b.op("LD", "cn_vr")
    b.op("CMPI", imm=0)
    b.op("LDI", imm=ONE)
    b.op("LDIF", imm=0)
    b.op("ST", "cn_V")
    b.op("LD", "cn_sr")
    b.op("CMPI", imm=0)
    b.op("LDI", imm=ONE)
    b.op("LDIF", imm=0)
    b.op("ST", "cn_S")
    leaves = {0: ["V"], 1: ["S", "N"], 2: ["S"], 3: ["V", "S", "N"], 4: ["N"], 5: ["V", "S"], 6: ["V", "N"], 7: []}
    ends = []

    def tree(bits, p):
        if not bits:
            comps = leaves[p]
            if comps:
                b.op("LD", "cn_" + comps[0])
                for c in comps[1:]:
                    b.op("MUL", "cn_" + c)
            else:
                b.op("LDI", imm=0)
            ends.append(b.jump())
            return
        bit, w = bits[0]
        b.op("BIT", 0, b.src(LATCH1, bit))
        at = b.skip_if(True)
        tree(bits[1:], p)
        b.land(at)
        tree(bits[1:], p + w)
    if mame_mixer:                                   # MAME as it runs: mixer stuck at mode 0 (VCO)
        b.op("LD", "cn_V")
    else:
        tree([(3, 4), (2, 2), (1, 1)], 0)
        for e in ends:
            b.land(e)
    # --- output: out ? + : - gain, x CP / 1.855 (MAME stream value); silent when inhibited or the VCO saturates
    b.op("CMPI", imm=q(0.5))
    b.op("LD", "cn_gn")
    b.op("LDF", "cn_gp")
    b.op("MULI", imm=q(CP / (2.57 - 0.715) * K_SN))
    b.op("ST", "cn_so")
    b.op("LD", "cn_v")
    b.op("CMPI", imm=q(VCO_SAT) + 1)
    b.op("LDI", imm=0)
    b.op("STF", "cn_so")
    b.op("LD", "cn_en")
    b.op("CMPI", imm=q(0.5))
    b.op("LDI", imm=0)
    b.op("STNF", "cn_so")


def cane(sn=True, tos=True, shot=True, sn_window=0.036, mame_mixer=False):
    """sn / tos / shot: include the SN76477, the TOS music, the 555 noise (False = that part silent, for
    comparisons). sn_window: seconds after a port-3 write until the SN is inhibited when no port-5 write is seen
    (None = only a seen port-5 write inhibits it). mame_mixer: MAME as it runs (sh_port_1_w calls the config-time
    set_mixer_params(), which never reaches the live mixer: the SN stays in mode 0, VCO, ~24 kHz here) instead of the
    port-selected mixer mode the driver intends"""
    b = Board("cane")
    terms = []
    if tos:
        music(b)
        terms.append(("cn_mo", K_MU))
    if shot:
        sfx(b)
        terms.append(("cn_xo", K_SX))
    if sn:
        sn76477(b, sn_window, mame_mixer)
        terms.append(("cn_so", 1.0))
    # DISCRETE_ADDER3 with enable SX0, DISCRETE_OUTPUT gain 1, route 1.0
    for i, (reg, k) in enumerate(terms):
        if i == 0:
            b.op("LD", reg)
            if k != 1.0:
                b.op("MULI", imm=q(k))
        else:
            b.op("MACI", reg, imm=q(k))
    if not terms:
        b.op("LDI", imm=0)
    b.op("ST", "_t")
    b.op("BIT", 0, b.src(LATCH1, 0))
    b.op("LDI", imm=0)
    b.op("LDF", "_t")
    b.op("OUT")
    return b.finish()


if __name__ == "__main__":
    import sys
    p = cane()
    out = sys.argv[1] if len(sys.argv) > 1 else "cane.dsnd.bin"
    open(out, "wb").write(p.binary())
    print(f"cane: {len(p.code)} instructions, worst sample {p.worst_path()}, {len(p.regs)} registers -> {out}")
