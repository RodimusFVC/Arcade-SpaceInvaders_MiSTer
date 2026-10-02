#!/usr/bin/env python3
"""Generate MRAs for the Space Invaders (Midway / Taito 8080) core straight from MAME mw8080bw.cpp / 8080bw.cpp.

Region placement matches rtl/mw8080_board.sv; images are copied as dumped.
DIP switches and the input map come from the set's MAME INPUT_PORTS.

Release layout: a parent set (MAME parent 0) goes to <releases>/<Title>.mra with the
trailing parentheses dropped; a clone goes to
<releases>/_alternatives/_<Parent Title>/<Full Title>.mra.

Usage:
    gen_mra.py <driver.cpp[,driver.cpp...]> <releases_dir> [set ...]      (no sets: every supported set)
"""
import copy
import re
import sys
import xml.etree.ElementTree as ET
from pathlib import Path

MAME_VERSION = "0289"
RBF = "SpaceInvaders"
MAME_DIR = Path("/Work/Build/mame/src/mame/midw8080")
HISCORE_DAT = Path("/CybertronMD/Mame/plugins/hiscore/hiscore.dat")   # current file (user, 2026-09-27)

# ---------------------------------------------------------------- boards

# (driver, machine config, init) -> board variant (rtl/mw8080_board.sv)
SUPPORTED = {
    ("mw8080bw.cpp", "invaders", "empty_init"): 0,     # Midway Space Invaders board
    ("8080bw.cpp", "invaders", "empty_init"): 0,       # Taito / bootleg sets on the same board
    ("8080bw.cpp", "cosmicmo", "empty_init"): 0,       # invaders board (cocktail flip gated by a DIP)
    ("8080bw.cpp", "cosmicbat", "empty_init"): 0,      # invaders board, 20 MHz / 10 CPU
    ("8080bw.cpp", "invnomb", "empty_init"): 1,        # no shifter: port 3 reads open (MAME unmapped = 0)
    ("8080bw.cpp", "invasion", "empty_init"): 2,       # no shifter: port 3 reads IN3
    ("8080bw.cpp", "spcewars", "empty_init"): 3,       # Sanritsu: port 3 bit 4 = 1-bit tune speaker, no watchdog
    ("8080bw.cpp", "yosakdon", "empty_init"): 4,       # no shifter, inputs on ports 1-2, own sound bits
    ("8080bw.cpp", "darthvdr", "empty_init"): 5,       # ROM 0-17FF, work RAM 1800, VRAM 4000, vblank RST 7
    ("8080bw.cpp", "attackfc", "init_attackfc"): 6,    # shifter on ports 3 / 7, no sound in MAME, no watchdog
    ("8080bw.cpp", "attackfcu", "empty_init"): 6,
    ("8080bw.cpp", "vortex", "init_vortex"): 7,        # I/O A1 inverted, ROM A0/A3/A9 scrambled, colour per column
    ("8080bw.cpp", "spacecom", "init_spacecom"): 8,    # ports 41 / 42 / 44, 256-pixel picture
    ("8080bw.cpp", "invmulti", "init_invmulti"): 9,    # 128K scrambled ROM in 8 banks, 93C46 EEPROM at 6000
}

# ioctl index 0: maincpu 0x0000-0x7FFF (the board keeps 0000-1FFF and 4000-5FFF); invmulti: the raw 128K user1 dump
REGIONS = {"maincpu": (0x0000, 0x8000)}
VARIANT_REGIONS = {9: {"user1": (0x0000, 0x20000)}}
INDEX0_TEXT = {9: "raw 128K user1 dump (banked and unscrambled on the read path)"}
IGNORED_REGIONS = {"plds", "unknown", "unk"}
BOARD_IGNORED = {0: {"proms"}, 1: {"user1"}}            # dumps the MAME config never reads (invadernc, spacmiss)
RAM_WINDOWS = [(0x2000, 0x4000), (0x6000, 0x8000)]      # CPU reads RAM here; ROM bytes loaded into it are dead (jspecter)
NAME_OVERRIDE = {"invaders": "Space Invaders"}

F_VERT, F_ROT90 = 0x10, 0x80
S_TAITO = 0x01                                          # index 1 byte 2: Taito L-shaped sound board
TAITO_SOUND_DRIVERS = {"8080bw.cpp"}                   # Taito SV/TV sets and their bootlegs; mw8080bw.cpp = Midway

# port order on the board: DIP bytes 0-3 and input map bytes 16-47
PORTS = ["IN0", "IN1", "IN2", "IN3"]
PORT_TAGS = {"yosakdon": [None, "IN0", "IN1", None],   # input port tags at board ports 0-3, per machine config
             "darthvdr": ["P1", "P2", None, None],
             "attackfc": ["IN0", None, None, None],
             "attackfcu": [None, "IN0", None, None],
             "spacecom": [None, "IN0", "IN1", "IN2"]}   # ports 41 / 42 / 44: board slots 1-3
LINE_BASE = 40      # control ids 40-47 = DIP byte 3 bits: MAME fake-port DIPs read through a custom handler
CTL_VBLANK = 36     # control id 36 = VBLANK (spacecom IN2 bit 0)
ROM_DECODE = {"init_attackfc": 0x01,                   # index 1 byte 3: [0] A8/A9 swapped, [1] vortex A0/A3/A9 XOR
              "init_vortex": 0x02}
ROM_PATCH = {"init_spacecom": [(0x10, 0xF5)]}           # MAME: bad dump, "should be push a at RST 10h"

# control ids (Arcade-SpaceInvaders.sv)
CTL = {("JOYSTICK_UP", 1): 1, ("JOYSTICK_DOWN", 1): 2, ("JOYSTICK_LEFT", 1): 3, ("JOYSTICK_RIGHT", 1): 4,
       ("BUTTON1", 1): 5, ("BUTTON2", 1): 6, ("BUTTON3", 1): 7, ("BUTTON4", 1): 8,
       ("JOYSTICK_UP", 2): 9, ("JOYSTICK_DOWN", 2): 10, ("JOYSTICK_LEFT", 2): 11, ("JOYSTICK_RIGHT", 2): 12,
       ("BUTTON1", 2): 13, ("BUTTON2", 2): 14, ("BUTTON3", 2): 15, ("BUTTON4", 2): 16,
       ("BUTTON5", 1): 32, ("BUTTON6", 1): 33, ("BUTTON5", 2): 34, ("BUTTON6", 2): 35,
       ("COIN1", 0): 17, ("COIN2", 0): 18, ("START1", 0): 19, ("START2", 0): 20,
       ("TILT", 0): 21, ("SERVICE1", 0): 22, ("SERVICE", 0): 22, ("COIN3", 0): 23,
       ("MEMORY_RESET", 0): 22}                          # operator "Name Reset" (invaddlx): service key
IGNORED_TYPES = {"UNUSED", "UNKNOWN"}
READ_LINE_FIXED = {"cosmicmo_cab_r": 0}                # cabinet type: upright

# PORT_CUSTOM_MEMBER handlers, upright cabinet: the fake port whose bits the handler returns
CUSTOM = {
    "invaders_in0_control_r": "CONTP1",
    "invaders_in1_control_r": "CONTP1",
    "invaders_in2_control_r": "CONTP1",
    "sicv_in2_control_r": "CONTP1",          # | P2GATE factory DIPs ("leave on" = 0)
    "invadpt2_in1_control_r": "CONTP1",      # upright: P1 | P2, both players share the controls
    "invadpt2_in2_control_r": "CONTP1",
    "invaders_sw6_sw7_r": "SW6SW7",
    "invaders_sw5_r": "SW5",
}

REGION_WORDS = [("US", "US"), ("Japan", "Japan"), ("Spanish", "Spain"), ("Italian", "Italy"), ("French", "France"),
                ("Brazil", "Brazil"), ("Argentina", "Argentina"), ("Greek", "Greece"), ("Hungarian", "Hungary")]

# ---------------------------------------------------------------- parsing

GAME_RE = re.compile(r'^\s*(?:/\*[^*]*\*/)?\s*GAMEL?\(\s*([\w?]+),\s*(\w+),\s*(\w+),\s*(\w+),\s*(\w+),\s*\w+,\s*(\w+),'
                     r'\s*(ROT\d+),\s*"([^"]*)",\s*"([^"]*)",\s*([^)]*)\)', re.M)
LOAD_RE = re.compile(r'ROM_LOAD\(\s*"([^"]+)"\s*,\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+),\s*(?:BAD_DUMP\s+)?CRC\(([0-9a-fA-F]+)\)')
CONT_RE = re.compile(r'ROM_CONTINUE\(\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+)\s*\)')
RELOAD_RE = re.compile(r'ROM_RELOAD\(\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+)\s*\)')
FILL_RE = re.compile(r'ROM_FILL\(\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+),\s*(0x[0-9a-fA-F]+)\s*\)')
REGION_RE = re.compile(r'ROM_REGION\(\s*(0x[0-9a-fA-F]+),\s*"([^"]+)"')
MACRO_RE = re.compile(r'\b(PORT_\w+|INPUT_PORTS_\w+)\b(\s*\(((?:[^()"]|"[^"]*"|\([^()]*\))*)\))?')
DEFINE_RE = re.compile(r'^#define\s+(\w+)(\((\w+)\))?[ \t]*((?:.*\\\n)*.*)$', re.M)


def header_macros():
    """#define NAME / NAME(arg) bodies from the driver headers (input port macros, port tags)."""
    out = {}
    for h in ("mw8080bw.h", "8080bw.h"):
        for m in DEFINE_RE.finditer((MAME_DIR / h).read_text()):
            out[m.group(1)] = (m.group(3), m.group(4).replace("\\\n", "\n"))
    return out


MACROS = header_macros()


def expand(text):
    """Expand the header macros used inside INPUT_PORTS blocks."""
    for _ in range(4):
        changed = False
        for name, (arg, body) in MACROS.items():
            if arg is None:
                new = re.sub(r'\b%s\b(?!\s*\()' % name, lambda _: body, text)
            else:
                new = re.sub(r'\b%s\(\s*([^()]*?)\s*\)' % name,
                             lambda m: re.sub(r'\b%s\b' % arg, m.group(1), body), text)
            if new != text:
                text, changed = new, True
        if not changed:
            break
    return text


def parse_games(src):
    games = {}
    for m in GAME_RE.finditer(src):
        year, name, parent, machine, inputs, init, rot, manuf, desc, flags = m.groups()
        games[name] = dict(year=year, name=name, parent=None if parent == "0" else parent,
                           machine=machine, inputs=inputs, init=init, rot=rot, manuf=manuf, desc=desc,
                           working="MACHINE_NOT_WORKING" not in flags,
                           layout=(re.search(r'\blayout_(\w+)', flags) or [None, None])[1])
    return games


def parse_roms(src, setname):
    m = re.search(r'ROM_START\(\s*%s\s*\)(.*?)ROM_END' % re.escape(setname), src, re.S)
    if not m:
        raise SystemExit(f"ROM_START({setname}) not found")
    segs, region, rsize, last = [], None, 0, None
    for line in m.group(1).splitlines():
        line = line.split("//")[0]
        if "NO_DUMP" in line:
            continue
        if (r := REGION_RE.search(line)):
            region, rsize, last = r.group(2), int(r.group(1), 16), None
        elif (l := LOAD_RE.search(line)):
            name, off, length, crc = l.group(1), int(l.group(2), 16), int(l.group(3), 16), l.group(4).lower()
            last = dict(name=name, crc=crc, src=0, dst=off, len=length, flen=length, region=region, rsize=rsize)
            segs.append(last)
        elif (c := CONT_RE.search(line)):
            off, length = int(c.group(1), 16), int(c.group(2), 16)
            nxt = dict(last, src=last["src"] + last["len"], dst=off, len=length)
            segs.append(nxt)
            last = nxt
        elif (r := RELOAD_RE.search(line)):
            base = next(s for s in reversed(segs) if s["name"] == last["name"] and s["src"] == 0)
            last = dict(base, dst=int(r.group(1), 16), len=int(r.group(2), 16))
            segs.append(last)
        elif (f := FILL_RE.search(line)):
            off, length, val = int(f.group(1), 16), int(f.group(2), 16), int(f.group(3), 16)
            segs.append(dict(name=None, crc=None, fill=val, src=0, dst=off, len=length, region=region, rsize=rsize))
            last = None
        elif "ROM_" in line and any(k in line for k in ("ROM_LOAD", "ROM_COPY", "ROM_FILL", "ROM_RELOAD")):
            raise SystemExit(f"{setname}: unhandled ROM statement: {line.strip()}")
    return segs


def def_str(s):
    s = s.strip()
    m = re.fullmatch(r'DEF_STR\(\s*(\w+)\s*\)', s)
    if not m:
        return s.strip('"')
    w = m.group(1)
    c = re.fullmatch(r'(\d+)C_(\d+)C', w)
    if c:
        return f"{c.group(1)}C/{c.group(2)}C"
    return w.replace("_", " ")


def idle_level(arg, mask):
    """Released level from an IP_ACTIVE_LOW / IP_ACTIVE_HIGH keyword or a numeric default."""
    if arg == "IP_ACTIVE_LOW":
        return mask
    if arg == "IP_ACTIVE_HIGH":
        return 0
    return int(arg, 0) & mask


def split_args(a):
    out, depth, cur, q = [], 0, "", False
    for ch in a:
        if ch == '"':
            q = not q
        if not q and ch == "(":
            depth += 1
        elif not q and ch == ")":
            depth -= 1
        if ch == "," and depth == 0 and not q:
            out.append(cur.strip())
            cur = ""
        else:
            cur += ch
    if cur.strip():
        out.append(cur.strip())
    return out


def parse_inputs(src):
    """INPUT_PORTS name -> {tag: [field]}; field = dict(mask, kind, default, type, player, name, settings)."""
    blocks = {m.group(1): expand(m.group(2)) for m in re.finditer(
        r'INPUT_PORTS_START\(\s*(\w+)\s*\)(.*?)INPUT_PORTS_END', src, re.S)}
    cache = {}

    def build(name):
        if name in cache:
            return copy.deepcopy(cache[name])
        ports, cur, fld = {}, None, None
        body = "\n".join(l.split("//")[0] for l in blocks[name].splitlines())
        for m in MACRO_RE.finditer(body):
            mac, args = m.group(1), split_args(m.group(3) or "")
            if mac == "PORT_INCLUDE":
                ports.update(build(args[0]))
            elif mac in ("PORT_START", "PORT_MODIFY"):
                cur = args[0].strip('"()')
                if mac == "PORT_START":
                    ports[cur] = []
                fld = None
            elif mac in ("PORT_BIT", "PORT_DIPNAME", "PORT_CONFNAME", "PORT_SERVICE", "PORT_SERVICE_NO_TOGGLE",
                         "PORT_DIPUNUSED", "PORT_DIPUNUSED_DIPLOC", "PORT_DIPUNKNOWN", "PORT_DIPUNKNOWN_DIPLOC"):
                mask = int(args[0], 0)
                ports[cur] = [f for f in ports[cur] if not (f["mask"] & mask)]
                if mac == "PORT_BIT":
                    t = args[2].replace("IPT_", "")
                    fld = dict(mask=mask, kind="unused" if t in IGNORED_TYPES else "input",
                               default=idle_level(args[1], mask), type=t, player=1, name=None, settings=[])
                elif mac in ("PORT_DIPNAME", "PORT_CONFNAME"):
                    fld = dict(mask=mask, kind="dip", default=int(args[1], 0), name=def_str(args[2]), settings=[])
                elif mac == "PORT_SERVICE":
                    off = idle_level(args[1], mask)
                    fld = dict(mask=mask, kind="dip", default=off, name="Service Mode",
                               settings=[(off, "Off", None), (off ^ mask, "On", None)])
                elif mac == "PORT_SERVICE_NO_TOGGLE":
                    fld = dict(mask=mask, kind="input", default=idle_level(args[1], mask), type="SERVICE", player=0,
                               name=None, settings=[])
                else:
                    d = {"IP_ACTIVE_LOW": mask, "IP_ACTIVE_HIGH": 0}.get(args[1])
                    fld = dict(mask=mask, kind="unused", default=int(args[1], 0) if d is None else d)
                ports[cur].append(fld)
            elif mac in ("PORT_DIPSETTING", "PORT_CONFSETTING"):
                fld["settings"].append((int(args[0], 0), def_str(args[1]), None))
            elif mac == "PORT_CONDITION":
                cond = (args[0].strip('"'), int(args[1], 0), args[2], int(args[3], 0))
                if fld.get("settings"):
                    v, n, _ = fld["settings"][-1]
                    fld["settings"][-1] = (v, n, cond)    # this setting only exists under the condition
                else:
                    fld["cond"] = cond
            elif mac == "PORT_PLAYER":
                fld["player"] = int(args[0])
            elif mac == "PORT_COCKTAIL":
                fld["player"] = 2
            elif mac == "PORT_NAME":
                fld["name"] = args[0].strip('"')
            elif mac == "PORT_READ_LINE_MEMBER":
                fn = re.fullmatch(r'FUNC\(\w+::(\w+)\)', args[0])
                if not fn or fn.group(1) not in READ_LINE_FIXED:
                    raise SystemExit(f"INPUT_PORTS({name}): unhandled read line {args[0]}")
                fld["default"] = READ_LINE_FIXED[fn.group(1)] * fld["mask"]
                fld["custom"] = "fixed"
            elif mac == "PORT_READ_LINE_DEVICE_MEMBER":
                if not (args[0].strip('"') == "screen" and "vblank" in args[1]):
                    raise SystemExit(f"INPUT_PORTS({name}): unhandled device read line {args}")
                fld["custom"] = "vblank"
            elif mac == "PORT_CUSTOM_MEMBER":
                fn = re.fullmatch(r'FUNC\(\w+::(\w+)\)', args[0])
                if not fn:
                    raise SystemExit(f"INPUT_PORTS({name}): unhandled custom {args[0]}")
                fld["custom"] = fn.group(1)
            elif mac in ("PORT_DIPLOCATION", "PORT_CODE", "PORT_TOGGLE", "PORT_2WAY", "PORT_4WAY", "PORT_8WAY",
                         "PORT_SENSITIVITY", "PORT_KEYDELTA", "PORT_CHANGED_MEMBER",
                         "PORT_IMPULSE"):
                pass
            else:
                raise SystemExit(f"INPUT_PORTS({name}): unhandled {mac}")
        cache[name] = ports
        return copy.deepcopy(ports)

    return build


def place(segs, setname, ignored=frozenset(), regions=REGIONS):
    out = []
    for s in clip_ram(segs):
        if s["region"] in IGNORED_REGIONS or s["region"] in ignored:
            continue
        if s["region"] not in regions:
            raise SystemExit(f"{setname}: unmapped region {s['region']}")
        base, size = regions[s["region"]]
        if s["dst"] + s["len"] > size:
            raise SystemExit(f"{setname}: {s['name']} overruns {s['region']}")
        a, e = base + s["dst"], base + s["dst"] + s["len"]
        kept = []
        for o in out:                                  # later loads and fills overwrite earlier bytes
            oe = o["addr"] + o["len"]
            if oe <= a or o["addr"] >= e:
                kept.append(o)
                continue
            if o["addr"] < a:
                kept.append(dict(o, len=a - o["addr"]))
            if oe > e:
                kept.append(dict(o, addr=e, src=o["src"] + e - o["addr"], len=oe - e))
        out = kept + [dict(s, addr=a)]
    out.sort(key=lambda s: s["addr"])
    for a, b in zip(out, out[1:]):
        assert a["addr"] + a["len"] <= b["addr"], f"{setname}: overlap {a['name']} / {b['name']}"
    return out


def clip_ram(segs):
    """Drop the maincpu bytes that fall in the RAM windows."""
    out = []
    for s in segs:
        pieces = [s]
        if s["region"] == "maincpu":
            for lo, hi in RAM_WINDOWS:
                nxt = []
                for p in pieces:
                    a, e = p["dst"], p["dst"] + p["len"]
                    if e <= lo or a >= hi:
                        nxt.append(p)
                        continue
                    if a < lo:
                        nxt.append(dict(p, len=lo - a))
                    if e > hi:
                        nxt.append(dict(p, src=p["src"] + hi - a, dst=hi, len=e - hi))
                pieces = nxt
        out += pieces
    return out


def whole_file(seg, segs):
    """True when this segment is the file's only load (no ROM_CONTINUE pieces)."""
    return (sum(1 for s in segs if s["name"] == seg["name"] and s["crc"] == seg["crc"]) == 1 and seg["src"] == 0
            and seg["len"] == seg["flen"])

# ---------------------------------------------------------------- inputs -> MRA


def contiguous(mask):
    lo = (mask & -mask).bit_length() - 1
    hi = mask.bit_length() - 1
    return lo, hi, mask == ((1 << (hi + 1)) - (1 << lo))


def port_default(fields):
    level = 0xFF
    for f in fields:
        level = (level & ~f["mask"]) | (f["default"] & f["mask"])
    return level


def input_config(g, ports):
    """-> (idle bytes, dip list, input map, P1 button names)."""
    idle, dips, imap, buttons = [], [], [0] * 32, {}
    lines = []                                         # fake-port DIP bits routed through DIP byte 3

    def holds(cond):
        """PORT_CONDITION under every other DIP at its default (the MRA has no conditional settings)."""
        if cond is None:
            return True
        tag, mask, op, val = cond
        v = port_default(ports[tag]) & mask
        return {"EQUALS": v == val, "NOTEQUALS": v != val}[op]

    def dip(f, byte, lo, hi):
        if not holds(f.get("cond")):
            return
        sets = [(v, n) for v, n, c in f["settings"] if holds(c)]
        vals = [v >> f["lo_src"] for v, _ in sets]
        ids = ",".join(n.replace(",", ";") for _, n in sets)
        bits = f"{byte * 8 + lo}" if lo == hi else f"{byte * 8 + lo},{byte * 8 + hi}"
        seq = vals == list(range(len(vals))) and len(vals) == 1 << (hi - lo + 1)
        dips.append((f["name"], bits, ids, None if seq else ",".join(str(v) for v in vals)))

    def control(f, bit):
        t, pl = f["type"], f["player"]
        key = (t, pl if t.startswith(("JOYSTICK", "BUTTON")) else 0)
        if key not in CTL:
            raise SystemExit(f'{g["name"]}: unmapped input {t} player {pl}')
        imap[bit] = CTL[key]
        if t.startswith("BUTTON") and pl == 1:
            buttons[int(t[6:])] = f["name"] or ("Fire" if t == "BUTTON1" else f"Button {t[6:]}")

    for p, tag in enumerate(PORT_TAGS.get(g["machine"], PORTS)):
        fields = ports.get(tag) if tag else None
        if fields is None:
            idle.append(None)
            continue
        if p == 3 and lines:
            raise SystemExit(f'{g["name"]}: IN3 and fake-port DIPs both need DIP byte 3')
        level = 0xFF
        for f in fields:
            level = (level & ~f["mask"]) | (f["default"] & f["mask"])
            lo, hi, ok = contiguous(f["mask"])
            if f["kind"] == "dip":
                if not ok:
                    raise SystemExit(f'{g["name"]}: non-contiguous DIP {f["name"]} mask {f["mask"]:#x}')
                dip(dict(f, lo_src=lo), p, lo, hi)
            elif f["kind"] == "input" and f["type"] == "CUSTOM" and f.get("custom") in (None, "fixed"):
                continue                               # a fixed level (galxwars protection value, cabinet line)
            elif f["kind"] == "input" and f["type"] == "OTHER":
                if "reset" in (f["name"] or "").lower():
                    control(dict(f, type="MEMORY_RESET", player=0), p * 8 + lo)   # operator name-reset button
                continue                               # spare switches: idle level
            elif f["kind"] == "input" and f["type"] == "CUSTOM" and f.get("custom") == "vblank":
                imap[p * 8 + lo] = CTL_VBLANK              # the board's VBLANK
            elif f["kind"] == "input" and f["type"] == "CUSTOM":
                src_tag = CUSTOM.get(f.get("custom"))
                if src_tag is None:
                    raise SystemExit(f'{g["name"]}: unhandled custom handler {f.get("custom")} in {tag}')
                for sf in ports[src_tag]:                # an active-low custom field reads its source inverted
                    slo, shi, sok = contiguous(sf["mask"])
                    if sf["kind"] == "input":
                        control(sf, p * 8 + lo + slo)
                    elif sf["kind"] == "dip":
                        base = len(lines)
                        for b in range(slo, shi + 1):
                            lines.append((sf["default"] >> b) & 1)
                            imap[p * 8 + lo + b] = LINE_BASE + base + b - slo
                        dip(dict(sf, lo_src=slo), 3, base, base + shi - slo)
            elif f["kind"] == "input":
                if f["mask"] & (f["mask"] - 1):
                    raise SystemExit(f'{g["name"]}: multi-bit input {f["type"]} in {tag}')
                control(f, p * 8 + lo)
        idle.append(level & 0xFF)
    if lines:
        idle[3] = sum(v << i for i, v in enumerate(lines)) | (0xFF << len(lines)) & 0xFF
    idle = [0xFF if b is None else b for b in idle]
    return idle, dips, imap, buttons


def parse_hiscores(path):
    """hiscore.dat -> {set: [entry lines]}; sets listed together share the entry lines below them."""
    out, names, body = {}, [], False
    for line in path.read_text(encoding="latin-1").splitlines():
        line = line.strip()
        if not line or line.startswith(";"):
            names, body = ([], False) if not line else (names, body)
            continue
        if line.endswith(":"):
            if body:
                names, body = [], False
            names.append(line[:-1])
        elif line.startswith("@delay"):
            continue
        elif line.startswith("@"):
            for n in names:
                out.setdefault(n, []).append(line)
            body = True
    return out


def board_cfg(g):
    return SUPPORTED.get((g["drv"], g["machine"], g["init"]))


# ---------------------------------------------------------------- colour overlays (MAME layouts)

LAYOUT_DIR = Path("/Work/Build/mame/src/mame/layout")
RAW_W, RAW_H = 260, 224                                 # the core's picture: hx 1-260 -> x 0-259, rows 0-223
OV_BASE, OV_MAX = 64, 16                                # index 1: byte 64 = count, then 8 bytes per rectangle


def _bounds(e):
    b = e.find("bounds")
    if b is None:
        return None
    if "x" in b.attrib:
        x, y = float(b.get("x")), float(b.get("y"))
        return x, y, x + float(b.get("width")), y + float(b.get("height"))
    return float(b.get("left")), float(b.get("top")), float(b.get("right")), float(b.get("bottom"))


def overlay_rects(g):
    """MAME layout colour overlay -> [(x0, x1, y0, y1, r, g, b)] in raw scan coordinates (half-open), later wins."""
    if not g.get("layout"):
        return []
    path = LAYOUT_DIR / f'{g["layout"]}.lay'
    if not path.exists():
        raise SystemExit(f'{g["name"]}: layout {path.name} not found')
    root = ET.parse(path).getroot()
    elements = {e.get("name"): e for e in root.findall("element")}
    for view in root.findall("view"):
        scr = view.find("screen")
        ovs = [e for e in view.findall("element") if e.get("blend") == "multiply"]
        if scr is None or not ovs:
            continue
        sx0, sy0, sx1, sy1 = _bounds(scr)
        rw, rh = (RAW_H, RAW_W) if g["rot"] in ("ROT90", "ROT270") else (RAW_W, RAW_H)
        out = []
        for ov in ovs:
            el = elements[ov.get("ref")]
            rects = el.findall("rect")
            ib = [_bounds(r) for r in rects]
            ix0, iy0 = min(b[0] for b in ib), min(b[1] for b in ib)
            ix1, iy1 = max(b[2] for b in ib), max(b[3] for b in ib)
            ex0, ey0, ex1, ey1 = _bounds(ov)
            for r, b in zip(rects, ib):
                col = r.find("color")
                rgb = [round(255 * float(col.get(k, "1"))) if col is not None else 255 for k in ("red", "green", "blue")]
                # element space -> view -> rotated-screen pixels
                vx = [ex0 + (b[i] - ix0) * (ex1 - ex0) / (ix1 - ix0) for i in (0, 2)]
                vy = [ey0 + (b[i] - iy0) * (ey1 - ey0) / (iy1 - iy0) for i in (1, 3)]
                X = [round((v - sx0) / (sx1 - sx0) * rw) for v in vx]
                Y = [round((v - sy0) / (sy1 - sy0) * rh) for v in vy]
                X = [min(max(v, 0), rw) for v in X]
                Y = [min(max(v, 0), rh) for v in Y]
                # rotated screen -> raw scan (half-open)
                if g["rot"] == "ROT270":        # rotated (X, Y) = (raw_y, RAW_W - 1 - raw_x)
                    x0, x1, y0, y1 = RAW_W - Y[1], RAW_W - Y[0], X[0], X[1]
                elif g["rot"] == "ROT90":       # rotated (X, Y) = (RAW_H - 1 - raw_y, raw_x)
                    x0, x1, y0, y1 = Y[0], Y[1], RAW_H - X[1], RAW_H - X[0]
                else:
                    x0, x1, y0, y1 = X[0], X[1], Y[0], Y[1]
                if x1 > x0 and y1 > y0:
                    out.append((x0, x1, y0, y1, *rgb))
        if len(out) > OV_MAX:
            raise SystemExit(f'{g["name"]}: {len(out)} overlay rectangles (max {OV_MAX})')
        return out
    return []


def overlay_bytes(rects):
    out = [len(rects)]
    for x0, x1, y0, y1, r, gr, b in rects:
        out += [x0 & 0xFF, (x0 >> 8) | (x1 >> 8) << 1, x1 & 0xFF, y0, y1, r, gr, b]
    return out


def config_bytes(g, idle, imap):
    """MRA index 1: variant, flags, sound board, ROM decode, idle levels (bytes 4-11), input map, overlay."""
    flags = (F_VERT if g["rot"] in ("ROT90", "ROT270") else 0) | (F_ROT90 if g["rot"] == "ROT90" else 0)
    sflags = S_TAITO if g["drv"] in TAITO_SOUND_DRIVERS else 0
    assert len(idle) <= 8, g["name"]
    # the idle levels repeat <switches default>: MiSTer sends DIP bytes (index 254) only when an MRA has a <dip>
    cfg = [board_cfg(g), flags, sflags, ROM_DECODE.get(g["init"], 0)] + idle + [0] * (12 - len(idle)) + imap
    return cfg + [0] * (OV_BASE - len(cfg)) + overlay_bytes(overlay_rects(g))


def mra(g, games, segs, build_inputs):
    variant = board_cfg(g)
    ports = build_inputs(g["inputs"])
    idle, dips, imap, buttons = input_config(g, ports)
    flags = (F_VERT if g["rot"] in ("ROT90", "ROT270") else 0) | (F_ROT90 if g["rot"] == "ROT90" else 0)
    nbtn = max(buttons) if buttons else 0
    names = ",".join([buttons.get(i, "Not Used") for i in range(1, 5)] + ["Coin", "Start 1P", "Start 2P", "Pause"] +
                     [buttons.get(i, "Not Used") for i in (5, 6)])
    parent = g["parent"] or g["name"]
    zipname = f'{g["name"]}.zip' + (f'|{g["parent"]}.zip' if g["parent"] else "")
    rotation = {"ROT0": "horizontal", "ROT270": "vertical (ccw)", "ROT90": "vertical (cw)"}[g["rot"]]
    bootleg = "yes" if "bootleg" in g["manuf"].lower() or "hack" in g["desc"].lower() else "no"
    top = games.get(parent, g)
    series = title_case(clean_title(top["desc"]))
    region = next((r for w, r in REGION_WORDS if w in g["desc"]), "World")

    lines = []
    pos = 0
    for s in segs:
        if s["addr"] > pos:
            lines.append(f'        <part repeat="0x{s["addr"] - pos:X}">00</part>')
        if s.get("fill") is not None:
            lines.append(f'        <part repeat="0x{s["len"]:X}">{s["fill"]:02X}</part>')
        elif whole_file(s, segs):
            lines.append(f'        <part crc="{s["crc"]}" name="{s["name"]}"/>')
        else:
            lines.append(f'        <part crc="{s["crc"]}" name="{s["name"]}" offset="0x{s["src"]:X}" length="0x{s["len"]:X}"/>')
        pos = s["addr"] + s["len"]
    lines += [f'        <patch offset="0x{a:X}">{v:02X}</patch>' for a, v in ROM_PATCH.get(g["init"], [])]

    dip_lines = "\n".join(f'        <dip name="{n}" bits="{b}" ids="{i}"' + (f' values="{v}"' if v else "") + "/>"
                          for n, b, i, v in dips)
    cfg = config_bytes(g, idle, imap)
    cfg_rows = "\n".join("            " + " ".join(f"{b:02X}" for b in cfg[i:i + 16]) for i in range(0, len(cfg), 16))
    return f"""<misterromdescription>
    <name>{display_name(g)}</name>
    <region>{region}</region>
    <homebrew>no</homebrew>
    <bootleg>{bootleg}</bootleg>
    <version></version>
    <alternative></alternative>
    <platform></platform>
    <series>{series}</series>
    <year>{g["year"]}</year>
    <manufacturer>{g["manuf"]}</manufacturer>
    <category>Shooter</category>

    <setname>{g["name"]}</setname>
    <parent>{parent}</parent>
    <mameversion>{MAME_VERSION}</mameversion>
    <rbf>{RBF}</rbf>
    <about></about>

    <resolution>15kHz</resolution>
    <rotation>{rotation}</rotation>
    <flip>yes</flip>

    <players>2 (alternating)</players>
    <joystick>2-way horizontal</joystick>
    <special_controls></special_controls>
    <num_buttons>{nbtn}</num_buttons>
    <buttons names="{names}" default="A,Y,B,X,Select,Start,R,L"/>

    <switches default="{",".join(f"{b:02X}" for b in idle)}">
{dip_lines}
    </switches>

    <!-- Index 0: {INDEX0_TEXT.get(variant, "CPU 0x0000-0x7FFF (MAME maincpu)")} -->
    <rom index="0" md5="none" zip="{zipname}">
{chr(10).join(lines)}
    </rom>

    <!-- Index 1: board variant, flags, sound board, input map, colour overlay (see Arcade-SpaceInvaders.sv) -->
    <rom index="1">
        <part>
{cfg_rows}
        </part>
    </rom>

    <remark>{remark(g)}</remark>
    <mratimestamp>20260930000000</mratimestamp>
</misterromdescription>
"""


def title_case(text):
    """Capitalise every word, small words included; acronyms (US, II, PCB) stay as written."""
    out = []
    for word in re.split(r"(\s+|[()/,-])", text):
        if word and word[0].isalpha():
            word = word[0].upper() + word[1:]
        out.append(word)
    return "".join(out)


def clean_title(desc):
    """Parent title: the description without its trailing parenthesised qualifiers."""
    return re.sub(r"(\s*\([^()]*\))+$", "", desc).strip()


def display_name(g):
    if g["name"] in NAME_OVERRIDE:
        return NAME_OVERRIDE[g["name"]]
    return title_case(clean_title(g["desc"]) if g["parent"] is None else g["desc"])


def remark(g):
    """'Title (Maker)', without repeating a maker the title already names."""
    m = re.fullmatch(r'bootleg \((.+)\)', g["manuf"])
    maker = f"{m.group(1)} bootleg" if m else g["manuf"]
    desc = title_case(g["desc"])
    return desc if maker.lower() in g["desc"].lower() else f"{desc} ({title_case(maker)})"


def safe(name):
    return re.sub(r'\s*/\s*', " - ", re.sub(r'[\\:*?"<>|]', "-", name))


def out_path(root, g, games):
    if g["parent"] is None:
        return root / f"{safe(display_name(g))}.mra"
    top = games.get(g["parent"])
    parent_title = display_name(top) if top else title_case(clean_title(g["desc"]))
    return root / "_alternatives" / f"_{safe(parent_title)}" / f"{safe(display_name(g))}.mra"


def rom_segments(g, src):
    """ioctl index 0 layout for a set (shared with verilator/build_rom.py)."""
    v = board_cfg(g)
    return place(parse_roms(src, g["name"]), g["name"], BOARD_IGNORED.get(v, frozenset()), VARIANT_REGIONS.get(v, REGIONS))


def main():
    """Each driver is parsed on its own: INPUT_PORTS / config names repeat across the drivers."""
    out_dir = Path(sys.argv[2])
    wanted = set(sys.argv[3:])
    drivers = [Path(f) for f in sys.argv[1].split(",")]
    all_games = {}
    for d in drivers:
        for n, g in parse_games(d.read_text()).items():
            all_games.setdefault(n, dict(g, drv=d.name))
    for d in drivers:
        src = d.read_text()
        games = {n: dict(g, drv=d.name) for n, g in parse_games(src).items()}
        build_inputs = parse_inputs(src)
        names = [n for n in games if n in wanted] if wanted else \
                [n for n, g in games.items() if board_cfg(g) is not None and g["working"]]
        for name in names:
            try:
                main_one(name, games, all_games, src, build_inputs, out_dir)
            except SystemExit as e:
                if wanted:
                    raise
                print(f"SKIPPED {name}: {e}")
    missing = wanted - set(all_games)
    if missing:
        raise SystemExit(f"unknown sets: {sorted(missing)}")


def main_one(name, games, all_games, src, build_inputs, out_dir):
    g = games[name]
    if board_cfg(g) is None:
        raise SystemExit(f'{name}: board {g["drv"]} {g["machine"]} / {g["init"]} not implemented')
    segs = rom_segments(g, src)
    path = out_path(out_dir, g, all_games)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(mra(g, all_games, segs, build_inputs))
    print(f'{name:12s} {path.relative_to(out_dir)}')


if __name__ == "__main__":
    main()
