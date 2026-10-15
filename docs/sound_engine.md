# Discrete sound engine (dsnd): master reference

This document is for developers (human or AI) who add boards to the engine or reuse it in another core.
It describes the code as of 2026-10-14. Board agents were adding `tools/dsnd_<board>.py` files at the time, so re-run the
inventory commands in §7.6 and §10 before you rely on the tables.

Every fact comes from the code. Citations use `file:line`, and paths are relative to the core root
(`Projects/Arcade-SpaceInvaders_MiSTer/`). Facts that come only from the vault log
(`Claude/spaceinvaders_modernize_2026-09-30.md`, cited as **VAULT:line**) cover history or hardware status that the code
cannot show.

| Short name | File |
|---|---|
| ENG | `rtl/dsnd_engine.sv` |
| MODEL | `verilator/snd/dsnd_model.h` |
| ASM | `tools/dsnd.py` |
| BOARDS | `tools/dsnd_boards.py` (+ `tools/dsnd_<board>.py`) |
| GEN | `tools/gen_mra.py` |
| TOP | `Arcade-SpaceInvaders.sv` |
| MW | `rtl/mw8080_board.sv` |
| LOCK | `verilator/snd/dsnd_lockstep.cpp` |

---

## 1. Overview

- **What it is.** A small fixed-point processor that runs a per-game "circuit program" once per audio sample. Each
  Midway one-off sound board then costs a program in the MRA instead of dedicated logic (ENG:3-4).
- **Why one engine plus a program.** One dedicated `invaders_sound` board costs about 2.8k ALMs. About 20 dedicated
  boards would not fit the Cyclone V (VAULT:138-139). The engine added about 630 ALMs, 8 RAM blocks and 3 DSPs
  (VAULT:223-225). The program arrives as MRA **ioctl index 5** (TOP:568-589).
- **Sample rate.** 48 kHz. One sample is 832 clocks of the 39.936 MHz core clock (`go = tick == 831`, ENG:98-100;
  `FS = 48000.0`, ASM:9).
- **Number format.** Q24 two's complement, so 1.0 = `1 << 24` (ENG:25-26, ASM:10). The accumulator and state words are
  32-bit and **wrap** on overflow (ENG:26, MODEL:63-68). The representable range is -128.0 to just under +128.0
  (`q()` asserts it, ASM:19-23).
- **Units.** The convention belongs to the program, not the engine:
  - Op-amp / MAME-node boards: volts (ASM:3, ASM:86).
  - Transistor-level boards: V, mA and kΩ, so V = mA × kΩ stays inside Q24 (`tools/dsnd_gunfight.py:8`,
    `tools/dsnd_seawolf.py:8`).
  - The M-4 prototype uses V, µA, MΩ, µF and s (`verilator/snd/m4/m4_proto.py:6`).
- **Budget.** At most **410 budget units** per sample. Every instruction costs 1 unit, except EXP (4) and RCP (5).
  At 2 clocks per unit, the budget uses 824 of the 832 clocks (ENG:26-27, ENG:39-40, ENG:136, ENG:290).
- **Output.** The program accumulates `mix` with OUT. At the end of the sample:
  `out = saturate16(mix >>> 9)`, so Q24 1.0 = 32768 = full scale (ENG:27, ENG:292-293, MODEL:103-104).

---

## 2. MRA index 5 byte format

### 2.1 Layout

- **No header.** Index 5 is the raw program image. Byte `n` goes to program word `n >> 3`, byte lane `n & 7`. The
  program RAM is written through byte enables `8'd1 << prog_addr[2:0]` (ENG:63-70).
- **Byte order.** Each 64-bit word is **little endian**: the assembler uses `struct.pack("<Q", ...)` (ASM:82), and the
  harness loads the same way (LOCK:20-24).
- **Instruction word** (ENG:25, ENG:114-117, ASM:82):

| Bits | Field | Byte in the MRA (within the 8-byte group) | Use |
|---|---|---|---|
| 63:56 | `op` | byte 7 | opcode 0-32. Any other value is a NOP that costs 1 unit (ENG:179, MODEL:96). |
| 55:48 | `a` | byte 6 | state address for read/write (M[a]). For BIT/FAND/FOR, bit 0 = invert. |
| 47:40 | `b` | byte 5 | Source selector `b[5:3]` = source, `b[2:0]` = bit. Also the NOISE LFSR number `b[1:0]` and the RCM coefficient address M[b]. |
| 39:32 | (none) | byte 4 | Unused, always 0. |
| 31:0 | `imm` | bytes 0-3 | Signed 32-bit immediate (Q24 value, raw integer, skip count, or LDLM mask/shift). |

- **Maximum size.** 1024 words = **8192 bytes**. The address is 13 bits (`prog_addr[12:0]`, ENG:49, TOP:585), and the
  program counter is 10 bits (ENG:61). The assembler asserts `len(code) <= 1024` and that the program ends with END
  (ASM:79-82). A longer download would wrap around and overwrite the start.
- **Program RAM is never cleared.** Words past a shorter, newer program keep old contents, which is harmless because
  execution stops at END. The **state** RAM is cleared on every reset (§4).
- **The init block and settings data are part of the program.** They are not a separate header. `Board.finish()`
  emits them as instructions (ASM:1292-1341). Structure:

```text
[init block]     LD M[255]; CMPI 1; SKF +n            ; skip if M[255] >= 1 (already initialised)
                 { LDI q(v); ST reg } ...             ; Board.init() values, plus _glast = -1.0
                 LDI 1; ST M[255]                     ; raw 1, not Q24
[change block]   LDLM src6 & 0xFF; ST _t; LDLM (src7 & 0xFF) << 8; ADD _t; ST _key
                 SUB _glast; ABS; CMPI 1; SKNF +n     ; settings unchanged -> skip to the body
                 group factor chains (_fT/_fF/_fO), Aged chain + multiply, Music chain, kreg code
                 LD _key; ST _glast; END              ; the sample that sees a change is silent
[body]           ... per-sample circuit ...; END
```

  The init block is emitted only if `inits` is non-empty (ASM:1299). The change block is emitted only if the board
  uses a Game Audio group, a kreg or a music input (ASM:1296, ASM:1309).

### 2.2 MRA XML

`gen_mra.mra()` emits index 5 only for `mw8080bw.cpp` sets whose MAME **machine config** name is a key of `BOARDS`
(GEN:1057). Data is 16 hex bytes per line:

```xml
    <!-- Index 5: sound board program for the discrete sound engine (tools/dsnd_boards.py) -->
    <rom index="5">
        <part>
            00 00 00 00 00 00 FF 01 01 00 00 00 00 00 00 0E
            ...
        </part>
    </rom>
```

**The order matters.** `ds_on` is cleared by the index 1 byte 0 write and set by any index 5 write (TOP:569-574), so
index 5 must come **after** index 1. `gen_mra` places it after index 1 (GEN:1102-1108). Index 6 is the artwork
(`ART_INDEX = 6`, "index 5 = discrete sound program", GEN:763).

### 2.3 Worked example: `releases/Tornado Baseball - Ball Park.mra`

The index 5 block is 912 bytes = 114 words. The first lines of the `<part>` are:

```text
00 00 00 00 00 00 FF 01 01 00 00 00 00 00 00 0E
04 00 00 00 00 00 00 16 00 00 00 FF 00 00 00 02
00 00 00 00 00 00 08 08 01 00 00 00 00 00 00 02
00 00 00 00 00 00 FF 08 FF 00 00 00 00 30 00 1D
00 00 00 00 00 00 09 08 FF 08 00 00 00 38 00 1D
```

Registers allocated by the assembler: `o`=0, `sq0`=1, `sq1`=2, `sq2`=3, `_fF`=4, `snd_k`=5, `snd_c`=6, `snd`=7,
`_glast`=8, `_t`=9, `_key`=10, `_gv`=11, `_fA`=12.

| Word | Bytes (LE) | op | a | b | imm | Meaning |
|---|---|---|---|---|---|---|
| 0 | `00 00 00 00 00 00 FF 01` | 1 LD | 255 | 0 | 0 | acc = M[255] (init flag) |
| 1 | `01 00 00 00 00 00 00 0E` | 14 CMPI | 0 | 0 | 1 | F = acc >= 1 |
| 2 | `04 00 00 00 00 00 00 16` | 22 SKF | 0 | 0 | 4 | already initialised: skip words 3-6 |
| 3 | `00 00 00 FF 00 00 00 02` | 2 LDI | 0 | 0 | 0xFF000000 = -1.0 | |
| 4 | `00 00 00 00 00 00 08 08` | 8 ST | 8 | 0 | 0 | `_glast` = -1.0 |
| 5 | `01 00 00 00 00 00 00 02` | 2 LDI | 0 | 0 | 1 | |
| 6 | `00 00 00 00 00 00 FF 08` | 8 ST | 255 | 0 | 0 | M[255] = 1 |
| 7 | `FF 00 00 00 00 30 00 1D` | 29 LDLM | 0 | 0x30 (src 6) | 0x00FF | acc = src6 & 0xFF |
| 8 | `00 00 00 00 00 00 09 08` | 8 ST | 9 | 0 | 0 | `_t` |
| 9 | `FF 08 00 00 00 38 00 1D` | 29 LDLM | 0 | 0x38 (src 7) | 0x08FF | acc = (src7 & 0xFF) << 8 |
| 10-13 | | ADD 9; ST 10; SUB 8; ABS | | | | `_key`, then \|key - _glast\| |
| 14-15 | | CMPI 1; SKNF 50 | | | | unchanged: jump to word 66 (the body) |
| 16-19 | | LDLM src6 imm 0x0D38; ST 11; LDI 1.0; ST 4 | | | | Filter index = ((src6 & 0x38) << 13), `_fF` = 1.0 |
| 20-43 | | LD 11; CMPI k<<16; LDI 1/scale; STF 4 (k = 1..6) | | | | `_fF` = 1.111, 1.25, 1.4286, 0.909, 0.833, 0.769 |
| 44-58 | | Aged chain into `_fA` (1.176, 1.4286); `_fF *= _fA` | | | | |
| 59-62 | | LDI 0.009387; MUL 4; MINI 1.0; ST 5 | | | | kreg: `snd_k` = k_RC × `_fF`, clamped to 1.0 |
| 63-65 | | LD 10; ST 8; END | | | | `_glast` = key, silent sample |
| 66-67 | | LDI 1.0; ST 0 | | | | body starts |
| 111-113 | | LD 7; MULI 0.999969; OUT; END | | | | `output("snd", 32767/32768)` |

Byte `0x30` in the `b` field is source 6, bit 0 (`src()` = `source << 3 | bit`, ASM:47-48). `imm` 0x08FF for LDLM means
mask 0xFF and shift 8 (ENG:174).

---

## 3. Instruction set

Notation:

- `M[x]` is a 32-bit state word.
- `imm` is the signed 32-bit immediate.
- `q24(p)` = `(int32)(p >>> 24)` of the full product, an arithmetic shift with the result truncated to 32 bits
  (MODEL:16, ENG:123-124).
- `S` = `src[b[5:3]]`, a source byte (§5).
- `sbit` = `S[b[2:0]] ^ a[0]` (ENG:120-122, MODEL:49).
- Every add and subtract wraps mod 2³².
- Compares are signed.
- Semantics come from ENG:151-181 and MODEL:59-97. The two are bit-exact (ENG:27, MODEL:1).

| op | Mnemonic | Operands | Semantics | Clocks | Units |
|---|---|---|---|---|---|
| 0 | END | — | End of sample: publish `out`. A program word of 0 is END. | 2 | 1 in `worst_path` (the model does not count it, MODEL:60) |
| 1 | LD | a | `acc = M[a]` | 2 | 1 |
| 2 | LDI | imm | `acc = imm` | 2 | 1 |
| 3 | ADD | a | `acc = acc + M[a]` | 2 | 1 |
| 4 | SUB | a | `acc = acc - M[a]` | 2 | 1 |
| 5 | ADDI | imm | `acc = acc + imm` | 2 | 1 |
| 6 | MULI | imm | `acc = q24(acc * imm)` | 2 | 1 |
| 7 | MUL | a | `acc = q24(acc * M[a])` | 2 | 1 |
| 8 | ST | a | `M[a] = acc` | 2 | 1 |
| 9 | STF | a | `if F: M[a] = acc` | 2 | 1 |
| 10 | LDIF | imm | `if F: acc = imm` | 2 | 1 |
| 11 | RC | a, imm | `acc = M[a] + q24((acc - M[a]) * imm); M[a] = acc`. The difference is formed in 33 bits (ENG:221) / int64 (MODEL:75), so it cannot overflow. | 2 | 1 |
| 12 | RCM | a, b | Like RC with the coefficient `M[b]` (ENG:222) | 2 | 1 |
| 13 | CMP | a | `F = acc >= M[a]` | 2 | 1 |
| 14 | CMPI | imm | `F = acc >= imm` | 2 | 1 |
| 15 | MAXI | imm | `if acc < imm: acc = imm` | 2 | 1 |
| 16 | MINI | imm | `if acc > imm: acc = imm` | 2 | 1 |
| 17 | BIT | a, b | `F = sbit` (a[0] inverts) | 2 | 1 |
| 18 | NOT | — | `F = !F` | 2 | 1 |
| 19 | NOISE | b | `L = lfsr[b[1:0]]` (31 bit); `nb = L[28] ^ L[0]`; `L = {nb, L[30:1]}`; `F = nb` (ENG:276-281, MODEL:84) | 2 | 1 |
| 20 | OUT | — | `mix = mix + acc` | 2 | 1 |
| 21 | LDF | a | `if F: acc = M[a]` | 2 | 1 |
| 22 | SKF | imm | `if F: pc = pc_next + imm` (10-bit wrap, relative to the next word, ENG:184, MODEL:87) | 2 | 1 |
| 23 | SKNF | imm | `if !F: pc = pc_next + imm` | 2 | 1 |
| 24 | LDL | b | `acc = S << 16` (unsigned byte, ENG:173) | 2 | 1 |
| 25 | ABS | — | `if acc < 0: acc = -acc` (INT_MIN stays INT_MIN) | 2 | 1 |
| 26 | FAND | a, b | `F = F & sbit` | 2 | 1 |
| 27 | FOR | a, b | `F = F \| sbit` | 2 | 1 |
| 28 | STNF | a | `if !F: M[a] = acc` | 2 | 1 |
| 29 | LDLM | b, imm | `acc = (S & imm[7:0]) << imm[12:8]` (32-bit, zero-filled, ENG:174, MODEL:90) | 2 | 1 |
| 30 | MACI | a, imm | `acc = acc + q24(M[a] * imm)` | 2 | 1 |
| 31 | EXP | — | `acc = 2^acc` (below) | 8 | 4 |
| 32 | RCP | — | `acc = 1/acc` (below) | 10 | 5 |

**EXP** (MODEL:19-28, ENG:223-227, ENG:240-251). Q28 coefficients
`C = {268435436, 186066863, 64465738, 14985772, 2413057, 504005}` (ENG:127-128):

```c
if (a >= 7<<24)   return 0x7FFFFFFF;        // >= 7.0 saturates
if (a < -(24<<24)) return 0;               // < -24.0 underflows
ip = a >> 24; fr = a & 0xFFFFFF; p = C[5];
for (k = 4; k >= 0; k--) p = (int32)(C[k] + q24(p * fr));   // Horner on the fraction, Q28
sh = ip - 4;  return sh >= 0 ? (int32)(p << sh) : p >> -sh; // Q28 -> Q24 and 2^ip
```

**RCP** (MODEL:30-45, ENG:228-231, ENG:252-270):

```c
if (d == 0) return 0x7FFFFFFF;
neg = d < 0; m0 = |d|; z = clz32(m0); D = (m0 << z) >> 8;          // D in 0.5 .. 1 (Q24)
Y = (int32)(47370962 + q24(D * -31580641));                          // seed 48/17 - 32/17 D
repeat 3: { e = q24(D * Y); Y = q24(Y * (int32)(0x02000000 - e)); } // Newton-Raphson
sh = z - 8;
if (sh < 0) r = Y >> -sh; else r = min(Y << sh, 0x7FFFFFFF);         // saturates at +/-0x7FFFFFFF
return neg ? -r : r;
```

**Timing.** E1 reads M[a]/M[b], registers the multiplier operands and starts the next fetch. E2 multiplies, writes back
and addresses the next M read. A write in E2 is forwarded to the next instruction's read (ENG:55-57, ENG:285-287), so
programs never see a read-after-write hazard. EXP and RCP insert 6 and 8 `S_X` micro-steps on the same multiplier
(ENG:236-272).

**Skips are the only control flow.** There are no loops or calls. The ISA allows a backward skip through the 10-bit
wrap, but the assembler emits forward skips only (`skip_if`/`land`, ASM:51-56), and `worst_path` assumes that
(ASM:1352).

---

## 4. Machine state and the per-sample loop

| Item | Width | Reset / initial | Notes |
|---|---|---|---|
| State RAM M[0..255] | 256 × 32 | **Cleared on reset**: `S_CLR` writes 0 to all 256 words (ENG:192-201); the model starts at 0 (MODEL:8) | Two dpram copies (M[a], M[b] read ports) share one write port (ENG:80-95) |
| M[255] | 32 | 0 after reset | **Reserved init flag**: the assembler allocates registers 0..254 only (`assert len(self.regs) < 255`, ASM:35-40), and `finish()` guards the init block with it (ASM:1299-1308) |
| Program RAM | 1024 × 64 | Not cleared | Loaded byte-wise from index 5 (ENG:63-78) |
| acc | 32 signed | 0 at power-up only | **Persists across samples**, and the reset branch does not clear it (ENG:103, ENG:192-196; MODEL:9) |
| F (flag) | 1 | 0 at power-up only | Persists across samples |
| pc | 10 | 0 at the start of each sample | (ENG:203) |
| mix | 32 signed | 0 at the start of each sample | Wraps (ENG:204, ENG:282) |
| lfsr[0..3] | 31 each | 1 at power-up; **not reset** | Stepped only by NOISE (ENG:105, MODEL:11) |
| icnt (budget) | 9 | 0 at the start of each sample | (ENG:106, ENG:288) |
| out | 16 signed | 0 on reset | Changes once per sample (ENG:292-293) |

Do not rely on the values of acc and F at the start of a sample. Do not rely on the LFSR phase after a reset.

**Per-sample loop** (ENG:97-100, ENG:202-295):

1. Every 832 clocks (`tick == 831`, frozen while `pause`) `S_IDLE` sets `pc = 0`, `mix = 0` and `icnt = 0`.
2. `S_LOAD` fetches word 0. A NOP E2 (`ir = 0`, skipped as END because `icnt == 0`) addresses its M reads
   (ENG:208-214).
3. E1/E2 then run per instruction (with `S_X` for EXP/RCP).
4. The sample ends on END, or **after the instruction that reaches 410 units**:
   `icnt + cost - 1 >= 410` (ENG:290); in the model, `while (n < 410)` (MODEL:53).
   - An over-budget program is **truncated silently**: instructions after the cut, including OUT, do not run that
     sample. `finish()` therefore asserts the static worst case (§7.5).
5. `S_PUB` sets `out = sat16(mix >>> 9)` (ENG:292-293).

**Output path at the top level.**

| Condition | Result | Source |
|---|---|---|
| `zac` | Zaccaria board audio | TOP:80-81 |
| else `ds_on` | `ds_audio` on **both** channels (mono) | TOP:80-81 |
| else | `invaders_sound` board audio | TOP:80-81 |
| Master Volume | applied after the mux: ×1, 0.71, 0.5, 0.25, 1.41 or 2, saturated | TOP:82-91 |
| `pause_cpu` | AUDIO_L/R forced to 0 | TOP:90-91 |

When `ds_on` is set, the board's own audio (normally silent, sound map 1) is ignored.

**Reset** = `RESET | status[0] | buttons[1] | ioctl_download | ~locked` (TOP:275). The engine is held in `S_CLR` for the
whole MRA download and clears M after it. Every load therefore re-runs the init block.

---

## 5. Inputs (sources)

`src` is 64 bits, one byte per source (ENG:51). It is wired at TOP:576-577:

```systemverilog
wire [63:0] ds_src = {status[48:46], status[45:44], status[43:41], 2'd0, status[40:35], 6'd0, ~in_port[1][3], vblank, in_port[0], b8_lat};
```

| Source (`b[5:3]`) | Content | Assembler name (ASM:16) |
|---|---|---|
| 0 | Sound latch 1 (`snd1`) | `LATCH1` |
| 1 | Sound latch 2 (`snd2`) | `LATCH2` |
| 2 | Sound latch 3 (`snd3`) | `LATCH3` |
| 3 | Sound latch 4 (`snd4`) | `LATCH4` |
| 4 | **IN0** = `in_port[0]` (after the DIP idle level / control map, TOP:476-480) | `LATCH0` (misnamed, see Discrepancies) |
| 5 | bit 0 = `vblank` (top level, after the artwork module, TOP:558); bit 1 = `~in_port[1][3]` (coin, used by maze); bits 7:2 = 0 | `MISC` |
| 6 | [2:0] Timing Caps `status[37:35]`; [5:3] Filter Caps `status[40:38]`; [7:6] = 0 | (numeric `6`) |
| 7 | [2:0] Oscillator Caps `status[43:41]`; [4:3] Aged Caps `status[45:44]`; [7:5] Music Volume `status[48:46]` | (numeric `7`) |

`b8_lat` = {snd4, snd3, snd2, snd1} from `mw8080_board` (TOP:535-537, TOP:684-685; MW:93-95, MW:596).

### 5.1 How port writes reach the latches

**Variant 20 (Midway one-offs).** Writes are table-driven. Index 1 bytes 56-63 hold one write-target byte per port
A2-A0 (`io_tab[127:64]`, MW:417-419). Bits [3]-[6] of that byte load latches 1-4 with the written byte (MW:644-647).

`gen_mra` builds the table from `FAMILY4[...]["wr"]` (GEN:96-130). The bit names are `WC=1` shift count, `WD=2` shift
data, `WW=4` watchdog, `S1..S4 = 8,16,32,64`, `RC=128` reversible shift count (GEN:104). Ports 8-15 are ignored when
`a3=1` (MW:417).

Current routing for engine boards (port → latch):

| Board (machine) | Port → latch | Source |
|---|---|---|
| clowns, spacwalk, desertgu | 3→S1, 5→S2, 6→S3, 7→S4 | GEN:119-120, 127 |
| dogpatch, boothill, dplay | 3→S1, 5→S2, 6→S3 | GEN:112, 116-117 |
| gunfight, tornbase | every odd port → S1 (A0 AND gate; `_and3`) | GEN:105-106, 111, 115 |
| checkmat | ports 1 and 3 (and 5 and 7) → S1 | GEN:110 |
| shuffle, bowler | 5→S1, 6→S2 | GEN:128-129 |
| maze | no latch (uses source 4 IN0 and source 5 bit 1) | GEN:109, TOP:576 |
| spcenctr | own decode in RTL: port 1, A4:A3 = 0/1/2 → S1/S2/S3 when A6 = 0 | MW:648-656 |
| seawolf | 5→S1 | GEN:118 |
| blueshrk | 3→S1 | GEN:121 |
| phantom2 | 5→S1, 6→S2 | GEN:113 |
| zzzap, lagunar | 2→S1, 5→S2 | GEN:122-123 |
| m4 | 3→S1, 5→S2 | GEN:124 |
| gmissile | 3→S1, 5→S2, 7→S4 | GEN:125 |

**Other variants.** The latches are `snd1`/`snd2` from the invaders-style decode (MW:690-697 region). `gen_mra` emits
index 5 only for `mw8080bw.cpp` machines (GEN:1057).

### 5.2 Program only, or RTL too?

| Need | Where |
|---|---|
| Sound bits are bytes the CPU writes to up to 4 ports (A2-A0 decode) | **Program only**, plus a `FAMILY4` entry (`wr` table, `snd=1`) in GEN. No RTL. |
| Edge-triggered sounds, one-shots, pulse widths, AND-gated ports | Program (keep the previous level in a register; see `op_amp_1sht_norton`, `ne555_astable2`) |
| More than 4 sound latches, or a decode on address bits above A2 (spcenctr A4/A3/A6) | RTL in MW (as `v_spc`, MW:648-656) |
| Inputs that are not port writes (joysticks, coin, video timing) | Top `ds_src`. Free bits: source 5 [7:2] and source 6 [7:6] (TOP:577). maze shows the pattern. |
| A sound CPU (6802/AY, 8035, etc.) | Not this engine |

---

## 6. Game Audio settings

**OSD** (TOP:182-188, page P5 "Game Audio", saved per game):

| Option | status | Values (OSD index order) |
|---|---|---|
| Timing Caps | [37:35] | Factory, -10%, -20%, -30%, +10%, +20%, +30% |
| Filter Caps | [40:38] | same |
| Oscillator Caps | [43:41] | same |
| Aged Caps | [45:44] | Off, Light, Heavy |
| Music Volume | [48:46] | Factory, Low, Lowest, High, Highest |
| Master Volume | [51:49] | Factory, -3dB, -6dB, -12dB, +3dB, +6dB. Applied in TOP:82-91, outside the engine. |

**Groups in the assembler** (ASM:121-126):

| Group | Source byte, mask, shift (`GROUP_SRC`) | Factor register | Factor value |
|---|---|---|---|
| T (timing / envelope caps) | src 6, `0x07`, 16 | `_fT` | `1 / SCALES[i]`, SCALES = (1.0, 0.9, 0.8, 0.7, 1.1, 1.2, 1.3); × `_fA` |
| F (filter / coupling caps) | src 6, `0x38`, 13 | `_fF` | `1 / SCALES[i]`; × `_fA` |
| O (oscillator caps, RC noise clocks) | src 7, `0x07`, 16 | `_fO` | `1 / SCALES[i]` (no aging) |
| Aged | src 7, `0x18`, 13 | `_fA` | `1 / AGED[i]`, AGED = (1.0, 0.85, 0.7); multiplied into `_fT` and `_fF` only (ASM:1323-1329) |
| Music | src 7, `0xE0`, 11 | `_fM` | MUSIC = (1.0, 0.7, 0.5, 1.4, 2.0), a direct gain (ASM:1330-1331) |

**How the settings reach the program.**

- `chain(reg, src, mask, sh, values)` emits LDLM, which turns the setting into `index << 16`. It then stores
  `values[i]` through `CMPI i<<16` + STF steps (ASM:128-138). OSD indices past the table use the last entry.
- `kreg(reg, k, g, clamp=True)` emits `reg = q(k) × _f<g>` into the **change block**, not the body, so a scaled rate
  costs nothing per sample. By default it clamps with `MINI 1.0` so an RC coefficient never exceeds 1.0. Use
  `clamp=False` for slopes and phase steps (ASM:140-151).
- The change block runs when `key = src6 | src7 << 8` differs from `_glast`, and on the first sample after reset
  (`_glast` initialised to -1.0). That sample ENDs without OUT, so it is silent (ASM:1309-1336).
- Music: a mixer input with tuple element 3 = True multiplies by `_fM` (`_mix_sum`, ASM:509-511). This sets
  `self.music` so the chain is emitted.

**What is legal.**

- A rate per sample scales by 1/C: a factor > 1 means a smaller capacitance and a faster rate (ASM:121-122).
- Crystal-derived clocks are left unscaled: `midway_tone`, `squarewfix` and plain `rc`/`rc_sel` use fixed constants
  (ASM:59-71, ASM:218-240, ASM:902-910).
- A program may read sources 6 and 7 directly (BIT/LDL/LDLM), but then it handles changes itself. The zzzap program
  maps Timing → C18/C19 and Oscillator → engine oscillator caps (`tools/dsnd_zzzap.py` docstring).
- Boards with no group, kreg or music input get no change block, and only Master Volume affects them. Currently that
  is gunfight and seawolf (§7.6).
- `invaders_sound.sv` and `zac_snd.sv` receive the same OSD bits through `snd_tweak` ports, not through the engine
  (TOP:670, TOP:714; MW:80).

---

## 7. Assembler and node library (`tools/dsnd.py`)

### 7.1 Core API (`Prog`, ASM:31-82)

| Call | Effect |
|---|---|
| `Prog.r(name)` | Allocates a named state register on first use (0..254). **Names are global**, so node-internal suffixes must not collide (§8.3). |
| `op(name, a, b, imm)` | Appends an instruction. If `a` is a string it is resolved through `r()`, and `imm` is masked to 32 bits. |
| `src(source, bit)` | `(source << 3) \| bit` for the `b` field |
| `skip_if(flag_true)` / `land(at)` | Emits a SKF/SKNF placeholder, then patches it to skip to the current position |
| `rc(reg, tau)` | `RC reg, k_rc(tau)` (fixed, no group) |
| `rc_sel(reg, tau_hi, tau_lo, src, bit, invert)` | RC whose tau switches on a source bit |
| `noise(n, amp)` | acc = ±amp from LFSR n |
| `binary()` | Packs to MRA bytes. Asserts a trailing END and ≤ 1024 words. |

Helpers:

| Helper | Meaning | Source |
|---|---|---|
| `q(v)` | volts → Q24, range-checked | ASM:19 |
| `k_rc(tau)` | `1 - exp(-1/(FS tau))` | ASM:26 |
| `rc_exp(rc)` | same; returns 1.0 for rc ≤ 0 | ASM:103 |
| `par(*rs)` | parallel resistance | ASM:107 |
| `qmul(a, b)` | the engine's exact Q24 multiply | ASM:98 |
| `wrap32` | 32-bit wrap | ASM:94 |
| `VBE = 0.5`, `RAIL = 1.5` | MAME `OP_AMP_NORTON_VBE` / `OP_AMP_VP_RAIL_OFFSET` | ASM:89-90 |
| `FN_*` | trigger functions | ASM:91 |

### 7.2 `Board(Prog)` infrastructure (ASM:111-216, 972-975, 1286-1357)

| Method | Purpose |
|---|---|
| `init(reg, value)` | One-time value (float, through `q()`) set by the init block |
| `kreg(reg, k, g, clamp)` | Group-scaled rate (§6) |
| `chain(reg, src, mask, sh, values)` | Lookup table on a source bit-field (also used by checkmat for the tone data bits, BOARDS:329-330) |
| `flag_from(t)` / `logic_reg(reg, fn, trig, store)` | MAME `dst_trigger_function`. A trigger is a constant, `('bit', source, bit)` or a logic register. Constants are folded at assembly time. |
| `load(v)` | acc = register or constant |
| `jump()` | Unconditional forward skip (`CMPI INT_MIN; SKF`, which `worst_path` treats as always taken) |
| `output(src, scale)` | `LD; MULI scale; OUT` (1.0 = full scale) |
| `finish()` | Wraps init block + change block + body + END. **Asserts `worst_path() <= 410`.** |
| `worst_path()` | Static longest path in budget units (§7.5) |

### 7.3 Node methods and their MAME discrete equivalents

"Group" is the Game Audio group used by the node's `kreg` calls. "—" means the node uses fixed coefficients.

| Method (ASM line) | MAME equivalent | Group | Notes |
|---|---|---|---|
| `midway_tone(out, lo, hi, clock, toggles)` (218) | `DISCRETE_NOTE` fed by the Midway tone latches | — | Q16 phase. lo = D5-D1, hi = D5-D0. out = 0 / 1.0. |
| `lfsr_noise(out, freq, amp, bias, n)` (242) | `DSS_LFSR_NOISE` (RC clock) | O | Engine LFSR, not MAME's sequence |
| `tvca(out, info, trig, inp)` (259) | `DST_TVCA_OP_AMP` | T | r1..r11, c1..c4, v1..v3, f0..f5. Dynamic F3 is not supported (asserts). |
| `op_amp_osc_norton1(out, r1..r5, c, vP)` (355) | `DSS_OP_AMP_OSC` type 1 Norton, SQW | O | Overshoot carried |
| `op_amp_filt_bp1(out, src, r1, rf, c1, c2, ...)` (413) | `DST_OP_AMP_FILT` `BAND_PASS_1` (non-Norton) | F | Clips at the rails before the gain, and splits the gain into ×16 steps |
| `op_amp_norton(out, inp0, inp1, r1..r4, vP, vN)` (441) | `DST_OP_AMP` `IS_NORTON`, no cap | — | |
| `filter1_lp(out, src, fc)` (459) | `DST_FILTER1` LP (bilinear, pre-warped) | — | |
| `mixer_resistor(out, ins, c_amp)` (473) | `DST_MIXER` `IS_RESISTOR` (no rF) | F | ins = `(src, r, c[, music[, gain]])` |
| `rcdisc2(out, sw, in0, r0, in1, r1, c)` (521) | `DST_RCDISC2` | T | |
| `op_amp_osc2_norton_cap(out, r1..r4, c, vP)` (533) | `DSS_OP_AMP_OSC` type 2 Norton, OUT_CAP | O | Clamps at the threshold |
| `op_amp_vco1_norton_cap(out, vmod, en, r1..r5, c, vP)` (565) | `DSS_OP_AMP_OSC` VCO_1 Norton, OUT_CAP | O | Exact carry. en low = force_charge. |
| `op_amp_vco3_norton(out, vmod, r1..r5, c, vP, r7, sqw)` (621) | `DSS_OP_AMP_OSC` VCO_3 Norton | O | Half-step carry |
| `op_amp_filt_bp1m_norton(out, src, r1, r3, rf, c1, c2, vP, vN, r2)` (692) | `DST_OP_AMP_FILT` `BAND_PASS_1M` Norton | — | Asserts the feedback state is inside Q24 |
| `integrate_norton1(out, src, v_on, r1, r2, c, v1, vP)` (727) | `DST_INTEGRATE` `OP_AMP_1` Norton | T | |
| `op_amp_vco2_norton(out, vmod, r1..r6, c, vP)` (748) | `DSS_OP_AMP_OSC` VCO_2 Norton, SQW | O | Clamps at the threshold (small pitch error vs MAME) |
| `op_amp_filt_bp1m(out, src, r1, rf, c1, c2, ...)` (809) | `DST_OP_AMP_FILT` `BAND_PASS_1M` (non-Norton) | — | |
| `osc_norton1_dyn(out, r1, r2, c, vP, tl_reg, th_reg)` (838) | Type 1 Norton osc with thresholds from registers (`DST_COMP_ADDER`-driven) | O | |
| `uniform_noise(out, freq, amp, bias, n)` (883) | `DSS_NOISE` (4-bit uniform) | O | |
| `squarewfix(ph, freq)` (902) | `DSS_SQUAREWFIX` (phase only) | — | |
| `rcfilter(out, src, r, c)` (912) | `DST_RCFILTER` | F | |
| `crfilter(out, src, r, c)` (917) | `DST_CRFILTER` | F | |
| `filter2(out, src, fc, d, kind)` (927) | `DST_FILTER2` lp/bp/hp | — | |
| `gain(out, src, k)` (954) | `DST_GAIN` | — | |
| `mixer_op_amp(out, ins, rf, c_amp, vref)` (959) | `DST_MIXER` `IS_OP_AMP` | F | |
| `op_amp_norton_cap(out, inp1, r2, r3, r4, c, vP, v_on, vN, g)` (977) | `DST_OP_AMP` `IS_NORTON` with cap | `g` (default T) | |
| `op_amp_1sht_norton(out, trig, v_trig, r1..r5, c1, c2, vP, vN, g)` (1014) | `DST_OP_AMP_1SHT` `1SHT_1` Norton | `g` (default T) | |
| `op_amp_vco3_norton_exact(out, vmod1, ..., rcp)` (1058) | VCO_3 Norton SQW with MAME's exact carry | `g` (default O) | Uses **RCP** at crossings. Asserts the phases are not both under one sample. |
| `op_amp_norton_sqw(out, sqw, v_sq, inp1, r1..r4, vP, vN)` (1174) | `DST_OP_AMP` Norton, IN0 a 0/v_sq square | — | |
| `ne555_astable2(outs, en, parts, level, g)` (1191) | `DSD_555_ASTBL` × n sharing RESET, ANDed | `g` (default O) | |
| `sn76477_slf_vco(out, inhibit, ...)` (1231) | SN76477 (MAME `sn76477_device`), SLF + VCO mode 1 | `g` (default O) | |
| `output(src, scale)` (1286) | `DISCRETE_OUTPUT` | — | |

**Transistor-level helpers** (not in ASM; copy from a board file):

- `newton()` + `bjt()`: one Newton step on Vbe per sample using **EXP + RCP**, with the step written to stay inside Q24.
  See `tools/dsnd_gunfight.py:74-127`, and the same pattern in `dsnd_seawolf.py`.
- `lin(b, dst, const, terms)`: linear combination using MACI. Each board file defines its own copy.

**Scheduling helpers** (`tools/dsnd_spcenctr.py:45-79`):

- `round_robin(b, blocks)`: block k runs on samples k, k+n, ..., so each block counts once in the worst path.
- `alternate(b, a, b)`: even/odd samples.
- Gun Fight alternates L/R sides at 24 kHz each (`tools/dsnd_gunfight.py:247-258`).

### 7.4 Writing and registering a board

```python
# tools/dsnd_<board>.py
from dsnd import Board, LATCH1, FN_TRG0, q

OUT_GAIN, ROUTE = 10000, 1.0                 # MAME DISCRETE_OUTPUT gain, add_route gain (example values)

def myboard():
    b = Board("myboard")                       # register names are global inside b
    b.lfsr_noise("noise", 7700, 12.0, 6.0)
    b.tvca("shot_v", dict(r1=2.7e6, r2=510e3, r4=510e3, r5=10e3, r7=510e3, c1=0.22e-6, v1=12, vP=12, f2=FN_TRG0),
           trig=(("bit", LATCH1, 4), 0, 0), inp=("noise", 0.0))
    b.rcfilter("shot", "shot_v", 12e3, 0.01e-6)
    b.op("BIT", 0, b.src(LATCH1, 3)); b.op("NOT")          # enable bit low -> mute
    b.op("LD", "shot"); b.op("LDIF", imm=0)
    b.op("MULI", imm=q(OUT_GAIN * ROUTE / 32768)); b.op("OUT")   # MAME DISCRETE_OUTPUT gain x add_route
    return b.finish()                          # asserts worst path <= 410
```

To register the board:

1. Import it in `tools/dsnd_boards.py` and add it to `BOARDS` (BOARDS:7-9, BOARDS:486-488). The key is the **MAME machine
   config name**: einning uses `dplay`, roadrunm uses `desertgu`, gunfighto uses `gunfight`, and clowns1 uses `clowns`.
   `program(machine)` returns the bytes (BOARDS:491-493).
2. Add or verify the `FAMILY4` entry in `tools/gen_mra.py` (GEN:107-130). `snd=1` makes the board's own audio silent.
3. Regenerate the MRAs (GEN:11-12):

```bash
CORE=/Work/Projects/ZZZ_Obsidian_FPGA/FPGA-Brain/Projects/Arcade-SpaceInvaders_MiSTer
python3 $CORE/tools/gen_mra.py /Work/Build/mame/src/mame/midw8080/mw8080bw.cpp $CORE/releases <set> [<set> ...]
```

`gen_mra` reads MAME source from `MAME_DIR = /Work/Build/mame/src/mame/midw8080` (GEN:25).

### 7.5 Budget rules

- `worst_path()` (ASM:1343-1357) does dynamic programming backwards over the code:
  - END = 1 unit.
  - EXP = 4, RCP = 5, every other instruction = 1.
  - SKF/SKNF = 1 + max(fall-through, target), except `CMPI INT_MIN; SKF` (`jump()`), which always takes the target.
  - It covers the init and change blocks too. The change block ends in END, so the result is
    max(change path, body path).
- `finish()` asserts `worst <= 410` (ASM:1339-1340). This is an exact static bound, not a simulation maximum.
- If a board is over budget:
  - Move rate scaling into `kreg` (the change block).
  - Share identical sub-circuits (spacwalk shares one noise filter + VCO for sb1/sb2, BOARDS:70-75).
  - Run slow parts at a lower rate with `round_robin` / `alternate`.
  - Keep EXP/RCP rare. RCP can be refreshed in a slow slot (`op_amp_vco3_norton_exact(..., rcp=True)`).

### 7.6 Current programs (assembled 2026-10-14)

Inventory command:

```bash
python3 -B -c "import sys; sys.path.insert(0,'$CORE/tools'); import dsnd_boards as d
for m,f in d.BOARDS.items(): p=f(); print(m, len(p.code), p.worst_path(), len(p.regs))"
```

| Program | Words | Worst units | Registers | Groups | Music | In `BOARDS` |
|---|---|---|---|---|---|---|
| clowns | 411 | 225 | 64 | F O T | yes | yes |
| dogpatch | 328 | 194 | 51 | F O T | yes | yes |
| boothill | 410 | 226 | 71 | F O T | yes | yes |
| tornbase | 114 | 66 | 13 | F | no | yes |
| desertgu | 332 | 194 | 51 | F O T | yes | yes |
| bowler | 150 | 105 | 18 | F T | no | yes |
| shuffle | 375 | 199 | 64 | F O T | no | yes |
| dplay | 490 | 286 | 79 | F O T | yes | yes |
| checkmat | 379 | 213 | 54 | F O T | yes | yes |
| maze | 279 | 187 | 37 | F O | no | yes |
| spacwalk | 657 | 407 | 116 | F O T | yes | yes |
| gunfight | 997 | 365 | 80 | — | no | yes |
| spcenctr | 786 | **410** | 134 | O T | no | yes |
| blueshrk | 811 | 392 | 115 | F O T | no | **no** |
| phantom2 | 606 | 406 | 87 | F O T | no | **no** |
| seawolf | 506 | 271 | 59 | — | no | **no** |
| zzzap / lagunar | 786 | 379 | 66 | O T | no | **no** |

---

## 8. Adding a new board, step by step

### 8.1 Reference first

1. **Schematic.** Put it under `Useful Information/schematics/` and trace the circuit (e.g. `tools/dsnd_spcenctr.py:3-5`,
   `dsnd_blueshrk.py:3-5`, `dsnd_seawolf.py:3-4`).
2. **MAME diff.** Read `mw8080bw_a.cpp` for the board. Note what MAME models, what it plays as samples, and what it
   leaves out. Several boards are partly constant 0 in MAME (desertgu BOARDS:186-189, bowler BOARDS:217-218,
   clowns BOARDS:41). **Samples are measurement targets and are never played** (`dsnd_seawolf.py:6`).
3. **ngspice reference.** Write a deck of the traced circuit at `verilator/snd/<board>/<x>_spice.py`, writing CSV on a
   48 kHz grid. Examples: `m4_spice.py`, `gm_spice.py`, `bs_spice.py`, `sw_spice.py`, `zz_spice.py`, `sc_spice.py`,
   `gf_spice.py`, `ph_spice.py`.
4. **Float prototype**, if the circuit is not a stock node. Write `verilator/snd/<board>/<x>_proto.py`, a per-sample
   model with the same equations and order the program will use. It is the spec for `tools/dsnd_<board>.py`
   (`gf_proto.py:1-4`, `m4_proto.py:1`, `gm_proto.py:1-2`).
5. **Program.** Write `tools/dsnd_<board>.py` with `Board` nodes, or engine code that mirrors the prototype.

### 8.2 Compare against references

**MAME poke capture.** The Lua scripts are loaded with MAME `-autoboot_script` together with `-wavwrite` (VAULT:155):

| Script | Environment | Purpose |
|---|---|---|
| `verilator/snd/mame_cap.lua` | `CAPLOG`, `CAPPRESS="port,field,t0,t1;..."`, `CAPSWEEP="port,field,period"` | Logs `time port data` for every I/O write and presses inputs on a schedule |
| `verilator/snd/mame_force.lua` | `CAPLOG`, `CAPFORCE="port,value,t0,t1;..."`, `CAPPOKE="port,value,t;..."` | Rewrites a port's writes in time windows, or pokes a port once (for games that rarely write it) |
| `verilator/snd/mame_fields.lua` | `CAPLOG` | Dumps every `port \| field` name, then exits |

**Replay the log through the engine.** Convert `time port data` into `--ev=t:src:hex` (changes only):

- `verilator/snd/blueshrk/mkev.py` (port 3 → source 0).
- `verilator/snd/phantom2/ph_eng.sh` (port 5 → source 0, otherwise source 1).

**Compare.** The board scripts measure:

- 10 ms RMS envelope correlation.
- RMS ratio and peak.
- Spectral peak / centroid.
- Fundamental frequency by autocorrelation.
- Decay times.

Scripts: `blueshrk/cmp.py:1-3` (30 Hz high pass first, because MAME has no amplifier high pass), `gmissile/gm_cmp.py:1-2`,
`seawolf/cmp.py:1-3`, `zzzap/cmp_eng.py:1-2`.

### 8.3 RTL-vs-model check, stress run, MRA install

```bash
CORE=/Work/Projects/ZZZ_Obsidian_FPGA/FPGA-Brain/Projects/Arcade-SpaceInvaders_MiSTer
make -C $CORE/verilator obj_dsnd/Vdsnd_engine                 # Makefile:18-22 (RTL + ram_sim.sv + LOCK + MODEL)
E=$CORE/verilator/obj_dsnd/Vdsnd_engine
# program binary (tools/dsnd_boards.py __main__ writes <m>.dsnd.bin into the CURRENT directory, BOARDS:496-501;
# write it explicitly instead):
python3 -B -c "import sys; sys.path.insert(0,'$CORE/tools'); import dsnd_boards as d; open('$SCRATCH/m.bin','wb').write(d.program('<machine>'))"
# lockstep RTL vs model with events, WAV out (LOCK:2-4, 59-118)
$E --prog=$SCRATCH/m.bin --len=10 --ev=0.5:0:09 --ev=1.0:0:19 --wav=$SCRATCH/m.wav
EV=$(python3 $CORE/verilator/snd/blueshrk/stress_ev.py 60 1 | sed 's/^/--ev=/')
$E --prog=$SCRATCH/m.bin --len=60 ${=EV}                       # zsh: ${=EV} splits the list
# random-program fuzz, optionally restricted to opcodes
$E --fuzz=1000 [--ops=31,32]
# directed EXP / RCP bit coverage (dsnd_optest.py:1-4)
python3 $CORE/verilator/snd/dsnd_optest.py $SCRATCH            # then each {exp,rcp}_w<k>.bin with --len=2
```

- **Pass criterion.** `lockstep: N samples, 0 mismatches`, and the exit status is 0 (LOCK:118-121).
- The harness also prints `most instructions in a sample` (the executed maximum, which is at most the static worst
  path) and `peak` (LOCK:111).
- **Stress run.** Use random overlapping triggers at the game's write rate for a long run, e.g.
  `verilator/snd/blueshrk/stress_ev.py <len> <seed>`. Use several seeds.
- **MRA install.**
  1. Register the board in `BOARDS`.
  2. Run `gen_mra.py` for each of the board's sets (§7.4).
  3. Confirm `<rom index="5">` appears after index 1.
  4. The engine hardware does not change, so no rebuild is needed for a new program alone. A new source wiring in
     `ds_src` or an MW decode change does need a rebuild.

### 8.4 Rules learned so far

| Rule | Why / where |
|---|---|
| **Apply each board's MAME `add_route` gain** in the output scale (with `DISCRETE_OUTPUT` gain / 32768) | Missing it showed as a constant 1.25× level on desertgu (VAULT:173-174). In code: clowns 0.25 (BOARDS:12), desertgu 0.8 (BOARDS:211), dplay 0.8 (BOARDS:309), checkmat 0.4 (BOARDS:350), spcenctr 0.45 (`dsnd_spcenctr.py:27`). |
| **Node register names must not collide.** `Prog.r()` is one global namespace, and nodes derive internals as `out + "_x"`. | A VCO's `_i` collided with the integrator output `siren_i` and was renamed `_iv` (VAULT:189-190, ASM:761). Prefix board-level temporaries uniquely. Names starting with `_` are used by the assembler (`_t`, `_k`, `_gv`, `_glast`, `_key`, `_f*`, `_fa`) and by board helpers (`_o`, `_rr`, `_ph`, `_dv`, `_re`, `_rd`, `_ie`, `_side`, `_nx`). |
| **zsh does not word-split `$EV`.** Use `${=EV}` (sh scripts can use `$EV`, `ph_eng.sh:4-5`). | An unsplit list passes only the first event, which gives a silent run (VAULT:220). |
| **Fold stereo to mono.** The engine has one output. | `gated_out()` = 0.5 (L + R) gated by the enable (BOARDS:111-121). Gun Fight folds L/R into its final scale (`dsnd_gunfight.py:259-260`). |
| **Respect the Q24 range.** ±128 per value; products are truncated `>>24` and wrap. | Clip **before** a large gain and split the gain into steps: `op_amp_filt_bp1` clips at the rails, then applies gain in ×16 steps (ASM:429-437). Newton splits a 1/rt ≥ 127 (`dsnd_gunfight.py:91-93`). Asserts guard feedback state (ASM:707). Pick units that keep values small (V/mA/kΩ for transistor boards). |
| **MAME `-wavwrite` ignores `system_mute`.** | A MAME WAV is not a reference for the enable/mute bit. Test the mute against the schematic (VAULT:159). |
| **Check the static budget, not the sim maximum.** | `finish()` asserts `worst_path() <= 410`. A truncated sample silently loses its OUT (ENG:290). |
| **Index 5 must follow index 1 in the MRA** | `ds_on` is cleared by index 1 byte 0 (TOP:571-572) |
| **A settings change costs one silent sample** | The change block ENDs (ASM:1335) |

---

## 9. Reusing the engine in another core

**Files.**

| File | Role |
|---|---|
| `rtl/dsnd_engine.sv` | The engine (SystemVerilog, Quartus 17 compatible, ENG:146) |
| `rtl/ram_rom/dpram_dc.vhd` | `altsyncram` wrapper with `byteena_a` (generics `widthad_a`, `width_a`), used for the program RAM and both state RAMs |
| `tools/dsnd.py` (+ board files) | Assembler and node library |
| `verilator/snd/dsnd_model.h`, `verilator/snd/dsnd_lockstep.cpp`, `verilator/ram_sim.sv` (sim `dpram_dc`), `verilator/snd/dsnd_optest.py` | Verification |

**Top-level hookup** (from TOP:568-589):

```systemverilog
reg ds_on = 1'b0;                                   // program present for this game
always @(posedge clk) if (ioctl_wr) begin
    if (ioctl_index == 8'd1 && ioctl_addr == 25'd0) ds_on <= 1'b0;   // any MRA load clears it (pick a byte every MRA has)
    if (ioctl_index == 8'd5) ds_on <= 1'b1;
end
wire [63:0] ds_src = {settings7, settings6, misc, in0, latch4, latch3, latch2, latch1};
wire signed [15:0] ds_audio;
dsnd_engine dsnd (
    .clk(clk),                                      // 832 x sample rate (39.936 MHz -> 48 kHz)
    .reset(reset),                                  // must cover the download: clears M, re-runs the init block
    .pause(pause),                                  // freezes the sample tick
    .prog_wr(ioctl_wr && ioctl_index == 8'd5),
    .prog_addr(ioctl_addr[12:0]),
    .prog_data(ioctl_dout),
    .src(ds_src),
    .out(ds_audio)
);
assign AUDIO_L = ds_on ? ds_audio : board_audio;    // mono: same on R
```

**Constraints when porting.**

- **Clock.** The sample period is fixed at 832 clocks (ENG:99). At another clock either:
  - generate `clk` at 832 × fs, or
  - change the `tick` compare. Then `FS` in `tools/dsnd.py:9` must equal the real sample rate, and the 410-unit budget
    must fit the new period (2 clocks per unit plus about 4 overhead, ENG:27).
- **Source byte meanings** are a contract between `ds_src` and the programs (`LATCH1..`, `MISC`, `GROUP_SRC`). Keep
  sources 6 and 7 as the Game Audio layout if you use the assembler's groups.
- `ioctl_index` 5 is a choice made in this core. Avoid index 3 and 4 if the core uses the MiSTer hiscore system.
- Byte loads need a RAM with byte enables. The engine writes the 64-bit word lane by lane (ENG:66-69).

---

## 10. Coverage table (this core)

Sources:

- RTL selection: MW:1109-1200 (sound map `smap` = index 1 byte 14, MW:286), TOP:80-81.
- Engine programs: `BOARDS` (BOARDS:486-488).
- Index 5 presence: `releases/**/*.mra`, scanned 2026-10-14.
- Variant / sound map: GEN:31-130 and the scanned MRAs.
- Hardware status: VAULT (cited), or the coordinator's list where the code and vault cannot show it (marked ⚠).

### 10.1 Hard-coded RTL

| Sets | Implementation | Status | Schematic |
|---|---|---|---|
| Every `invaders`-board set with sound map 0. Variant 0: invaders, spcking2ir (Midway); 49 Taito/bootleg sets incl. invadpt2, sitv, sicv, spaceatt, cosmicmo, galxwars... Variant 1: cosmo, galactic, invaderl, ozmawarsmr, searthin, solfight, spaceph, spacmiss, starw, superinv, tst_invd. Variant 2: astropal. Variants 7, 8, 9, 12: vortex, spacecom, invmulti ×10, starw1. | `invaders_sound.sv` `u_sound` (MW:1165-1176). `taito_snd` = Taito L-shaped variant for `8080bw.cpp` sets (GEN:1006). | ✅ HW (coordinator) ⚠ | Midway A084-90700-B000 (`invaders_sound.sv` header; VAULT:576) |
| invad2ct | Two boards: `u_sound` on latches 1/2 → left, `u_sound2` on latches 3/4 → right (smap 9, MW:1178-1192) | ✅ HW for board 1 ⚠. Board 2 **not validated** against `Space Invaders 2 Cocktail.pdf` p21/22 (coordinator; not checkable in code). | Same board ×2 |
| tinv2650, sia2650, dodgem | `zac_snd.sv` (`zac1b1120_board.sv:254`) via `zac` (TOP:80) | ✅ HW for The Invaders (coordinator) ⚠ dodgem not separately recorded | Zaccaria "The Invaders" sheet (`zac_snd.sv` header; VAULT:542) |
| invasion, invasiona, invasiona2, invasionb, invasionrz | Variant 2, smap 0, Taito flag → **Taito invaders board** | Plays, but MAME uses a Sidam board (coordinator) ⚠ | Needed |

### 10.2 Discrete engine (index 5 in the MRA)

| Sets (machine) | Status | Schematic / reference | Notes from code |
|---|---|---|---|
| clowns, clowns1 (clowns) | ✅ HW 2026-10-02 (VAULT:223) | MAME only. Only the parts catalog, no schematic (VAULT:158-160). | Springboard miss missing (BOARDS:41) |
| dogpatch | ✅ HW (VAULT:223) | MAME only | Stereo folded; hit sounds are constant 0 in MAME (BOARDS:135) |
| boothill | ✅ HW (VAULT:223) | MAME only | Stereo folded |
| tornbase | ✅ HW (VAULT:223) | MAME only | Siren / cheer not modelled, as in MAME (BOARDS:170) |
| desertgu, roadrunm (desertgu) | ✅ HW (VAULT:223) | MAME only | Bottle / Road Runner / creature hits and beep-beep are 0 in MAME too (BOARDS:189) |
| gunfight, gunfighto (gunfight) | ✅ HW 2026-10-14 (VAULT:301) | MAME `nl_gunfight` netlist values + ngspice (`dsnd_gunfight.py:4-5`). No schematic cited. | Transistor level, EXP/RCP |
| spacwalk | Sim (VAULT:210-219) | MAME only | |
| shuffle | Sim (VAULT:178-182) | MAME only | |
| dplay, einning (dplay) | Sim (VAULT:183-190) | MAME only | |
| checkmat | Sim (VAULT:191-195) | MAME only | |
| maze | Sim (VAULT:196-205) | MAME only | Uses IN0 / coin sources |
| bowler | Sim (VAULT:176-177) | MAME only | Only the foul sound; MAME models nothing else (BOARDS:217-218) |
| spcenctr | ⚠ not recorded (coordinator: in progress) | Schematic `Space_Encounters_Wiring_and_Schematics.pdf` p3 + `sc_spice.py` (`dsnd_spcenctr.py:3-5`) | Worst path exactly 410. MAME reference variant `spcenctr_mame` (BOARDS:420) is not registered. |

### 10.3 Engine program written, not registered (no index 5, so the set is **silent**: smap 1, `ds_on` = 0)

| Sets | Program | Schematic | Status |
|---|---|---|---|
| blueshrk, blueshrkmr, blueshrkmr2 | `tools/dsnd_blueshrk.py` | M051-00742-A005 + `bs_spice.py` | In progress (coordinator) |
| phantom2 | `tools/dsnd_phantom2.py` | Manual PDF p19 + `ph_spice.py` | In progress |
| seawolf, seawolfa, seawolfo | `tools/dsnd_seawolf.py` | Kush N' Stuff manual p23 + `sw_spice.py` | In progress |
| 280zzzap, 280zzzapa, lagunar | `tools/dsnd_zzzap.py` (`zzzap()`, `lagunar()`) | 0610-00907A + MAME `nl_280zzzap` corrections + `zz_spice.py` | In progress |
| m4 | **No program yet.** `verilator/snd/m4/` prototype + ngspice only. | M-4 Game Logic A084-90700-D626 (`m4_spice.py:2`) | In progress |
| gmissile | **No program yet.** `verilator/snd/gmissile/` prototype + ngspice only. | Gameboard schematic sheet (`gm_spice.py:2`) | In progress |

### 10.4 Silent or remapped (no discrete model)

| Sets | Code path | Notes |
|---|---|---|
| polaris ×5, schaser ×5, crashrd, lupin3, lupin3a, cane, ozmawars, ozmawars2, ozmawars3, mraker, orbite | Family 3, sound map 1 = silent (MW:1125, GEN:62-82) | Schematic needed. ozmawarsmr is **not** silent (smap 0, invaders board). |
| invrvnge, invrvngea, invrvngeb, invrvngedu, invrvngegw | Sound map 1 silent. "6802 + AY sound not yet" (GEN:83). | `rtl/sound/jt49` exists but is not in `files.qip` |
| sflush | FAMILY3 sound map 1. "MAME: no sound" (GEN:84). | **No MRA in `releases/`.** Variant 21 RTL exists (MW:165). |
| attackfc, attackfcu, claybust, gunchamp | Variants 6 / 11 force sp1 = sp2 = 0 (MW:1117) | No sound in MAME (GEN:41, 47). gunchamp runs on the claybust variant. |
| spcewars, spacewr3, swipeout (variant 3); spcewarla (smap 8) | Remapped onto the invaders board, plus a 1-bit tune speaker (MW:1112, 1135, 1194-1200) | Bootlegs: samples in MAME |
| yosakdon, yosakdona (4); darthvdr (5); shuttlei, skylove (10) | Remapped onto the invaders board (MW:1113-1119). shuttlei FE codes become 10 ms pulses (MW:1140-1162). | |
| ballbomb (2), indianbt (3), indianbtbr (4), schasercv (5, + tune), rollingc (6) | Family 3 sample maps onto invaders voices (MW:1126-1134) | Not in the coordinator's list |
| lrescue, lrescueabl, lrescuem, lrescuem2, grescue, desterth, escmars, mlander, mlandera, resclunar | Sound map 7 = tune speaker only (MW:1125, 1196) | Not in the coordinator's list |

### 10.5 Not built

| Set | Code evidence |
|---|---|
| rotaryf | Not in `SUPPORTED` / `FAMILY*` (GEN:31-130), and no RTL references it. The coordinator's reason (needs an 8085 core) is not stated in the code. ⚠ |

---

## Discrepancies (code wins)

1. **Program RAM size.** VAULT:141 says 512 words. The code has 1024 words (ENG:25, 61-63; MODEL:7). VAULT:188 records
   the change.
2. **Budget.** VAULT:142 says "<= 400 instructions per sample". The code has 410 **budget units**, with EXP = 4 and
   RCP = 5 (ENG:39-40, 136, 290; ASM:14, 1340). VAULT:211 has 410 instructions but predates EXP/RCP units.
3. **Opcode count.** VAULT:144 says "30 ops". The code has 33 (0-32), including MACI 30, EXP 31 and RCP 32 (ENG:28-38).
4. **Source 4.** VAULT:143 says "latch 0 (0)". The top level wires **IN0** (`in_port[0]`, TOP:576-577). The code
   contradicts itself: the engine header (ENG:41), `ASM:16` (`LATCH0`) and the harness help (LOCK:4) still say
   "latch 0".
5. **Misc source.** The `dsnd_boards.py` docstring (BOARDS:5) lists only "MISC bit 0 = VBLANK". Bit 1 = coin
   (`~IN1[3]`) is also wired (TOP:577).
6. **Source byte 7.** VAULT:230-231 has "src7 = osc", later superseded at VAULT:254 by {music, aged, osc}. The code matches
   the latter (TOP:577, ASM:123-126).
7. **spcenctr status.** VAULT:301 lists spcenctr as TODO, but it is in `BOARDS` (BOARDS:488) and
   `releases/Space Encounters.mra` has index 5.
8. **Program location.** The `ASM:4` docstring says board programs live in `tools/dsnd_boards.py`. Many now live in
   `tools/dsnd_<board>.py` and are imported there (BOARDS:8-9).
9. **Coordinator list vs code.**
   - m4 and gmissile have no `tools/dsnd_*.py` yet.
   - blueshrk, phantom2, seawolf and zzzap have programs but are not registered, so their sets are silent today.
   - ozmawarsmr is not silent.
   - ballbomb, indianbt, indianbtbr, schasercv, rollingc, spcewarla, the lrescue family and skylove were missing
     from the list.
   - sflush has no MRA.
   - invrvnge is silent, with no 6802/AY RTL in the build.
10. **Harness nit.** `run(..., uint8_t src[6], ...)` reads 8 source bytes (LOCK:43-48). This is harmless (the array
    decays to a pointer, and callers pass `src[8]`), but the declared size is wrong.
11. **END accounting.** `worst_path` counts END as 1 unit (ASM:1349-1350). The model does not count it toward the
    budget (MODEL:60), and the RTL's stop test ignores it. `worst_path` is therefore conservative by 1.
