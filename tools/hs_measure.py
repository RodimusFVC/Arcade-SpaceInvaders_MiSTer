#!/usr/bin/env python3
"""Measure each set's hiscore table init frame in MAME (write taps on its hiscore.dat ranges, cold boot, no input).
   hs_measure.py set...   -> per set: last boot-time write frame (addresses written in > LIVE frames excluded as live
                             counters), last value change, write clusters. One MAME at a time, scratch cfg / nvram.
   The core boots in step with MAME (invaders attract pixel-identical at the same frame), so MAME frames apply."""
import subprocess, sys, tempfile
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
argv, sys.argv = sys.argv, sys.argv[:1]
import gen_mra as g

MAME = "/CybertronMD/Mame/mame"
ROMS = Path(__file__).resolve().parent.parent / "games/mame"
FRAMES, LIVE, GAP = 1800, 100, 30
DETAIL = "--detail" in argv                         # per-address value-change frames
argv = [a for a in argv if a != "--detail"]

LUA = """local scr = manager.machine.screens[":screen"]
local sp = manager.machine.devices[":maincpu"].spaces["program"]
taps = {}
for _, r in ipairs({%s}) do
  taps[#taps + 1] = sp:install_write_tap(r[1], r[1] + r[2] - 1, "hs" .. r[1], function(o, d, m)
    print(string.format("W %%d %%04X %%02X", scr:frame_number(), o, d))
  end)
end
sub = emu.add_machine_frame_notifier(function() if scr:frame_number() >= %d then manager.machine:exit() end end)
"""


def measure(name, lines, tmp):
    ranges = [(int(l.split(",")[2], 16), int(l.split(",")[3], 16)) for l in lines]
    lua = Path(tmp) / f"{name}.lua"
    lua.write_text(LUA % (", ".join(f"{{{a}, {n}}}" for a, n in ranges), FRAMES))
    r = subprocess.run([MAME, name, "-rompath", str(ROMS), "-video", "none", "-sound", "none", "-nothrottle",
                        "-noplugins", "-cfg_directory", f"{tmp}/cfg", "-nvram_directory", f"{tmp}/nv",
                        "-snapshot_directory", f"{tmp}/snap", "-autoboot_script", str(lua)],
                       capture_output=True, text=True, timeout=600)
    frames, val, changes = {}, {}, []
    for ln in r.stdout.splitlines():
        if not ln.startswith("W "):
            continue
        _, f, a, v = ln.split()
        f = int(f)
        frames.setdefault(a, set()).add(f)
        if val.get(a) != v:
            val[a] = v
            changes.append((f, a))
    if not frames:
        return f"{name:12s} NO WRITES ({r.stderr.strip().splitlines()[-1] if r.stderr.strip() else 'no output'})"
    live = {a for a, fs in frames.items() if len(fs) > LIVE}
    boot = sorted({f for a, fs in frames.items() if a not in live for f in fs})
    clusters, start = [], None
    for i, f in enumerate(boot):
        if start is None:
            start = f
        if i + 1 == len(boot) or boot[i + 1] - f > GAP:
            clusters.append(f"{start}-{f}" if f != start else f"{f}")
            start = None
    last_w = boot[-1] if boot else -1
    last_c = max((f for f, a in changes if a not in live), default=-1)
    # init = last value change to work RAM (2000-23FF), but not before the boot write burst ends (the game's own
    # digit draw); later video RAM writes (2400+) are attract redraws from the work RAM copy
    ram = [f for f, a in changes if a not in live and int(a, 16) < 0x2400]
    init = max(ram + [int(clusters[0].split("-")[-1])] if clusters else ram, default=-1)
    flag = "  LATE" if init > 600 else ""
    detail = "".join(f"\n    {a}  changes at {' '.join(str(f) for f, b in changes if b == a)[:150]}"
                     for a in sorted(frames)) if DETAIL else ""
    return (f"{name:12s} init {init:4d}  last-write {last_w:4d}  last-change {last_c:4d}  clusters {' '.join(clusters)}"
            + (f"  live {' '.join(sorted(live))}" if live else "") + flag + detail)


if __name__ == "__main__":
    with tempfile.TemporaryDirectory() as tmp:
        for name in argv[1:]:
            lines = g.HISCORES.get(name) if hasattr(g, "HISCORES") else g.parse_hiscores(g.HISCORE_DAT).get(name)
            print(measure(name, lines, tmp) if lines else f"{name:12s} no hiscore.dat entry", flush=True)
