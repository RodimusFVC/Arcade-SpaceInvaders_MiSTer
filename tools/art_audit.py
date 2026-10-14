# Artwork audit: which MAME artwork zips in the archive match sets of the user's cores, and what each layout needs
# (2026-10-14). usage: art_audit.py <clones.txt from "mame -listclones"> <out.json>   (scans /Work/Projects/Arcade-*)
import sys, re, json, struct, zipfile, pathlib, collections
import xml.etree.ElementTree as ET

ARCH = pathlib.Path("/run/media/rodimus/CybertronD/Games/Mame/artwork")
SKIP = ("/verilator/", "/Useful Information/", "/sim/", "/.git/")

clones = {}
for line in open(sys.argv[1]):
    p = line.split()
    if len(p) == 2 and p[0] != "Name:":
        clones[p[0]] = p[1]

cores = {}
for d in sorted(pathlib.Path("/Work/Projects").glob("Arcade-*")):
    if not d.is_dir():
        continue
    sets = {}
    for m in d.rglob("*.mra"):
        if any(s in str(m) for s in SKIP):
            continue
        try:
            t = m.read_text(errors="replace")
        except OSError:
            continue
        sn = re.search(r"<setname>\s*([^<\s]+)\s*</setname>", t)
        if not sn:
            continue
        rot = re.search(r"<rotation>\s*([^<]*?)\s*</rotation>", t)
        sets.setdefault(sn.group(1), rot.group(1) if rot else "")
    cores[d.name] = sets

def analyse(zipname):
    z = zipfile.ZipFile(ARCH / f"{zipname}.zip")
    names = z.namelist()
    lay_name = next((n for n in names if n.lower() == "default.lay"), None) or next((n for n in names if n.lower().endswith(".lay")), None)
    info = {"zip": zipname, "issues": [], "views": [], "png_bytes": 0, "max_png": [0, 0]}
    if lay_name is None:
        info["issues"].append("no .lay (legacy .art?)")
        return info
    try:
        lay = ET.fromstring(z.read(lay_name))
    except ET.ParseError as e:
        info["issues"].append(f"layout parse error: {e}")
        return info
    elems = {e.get("name"): e for e in lay.findall("element")}
    lamp = text = disk = other = 0
    used_imgs = set()
    for n, e in elems.items():
        kids = list(e)
        imgs = [k for k in kids if k.tag == "image"]
        tags = {k.tag for k in kids}
        if "text" in tags or "simplecounter" in tags or "led7seg" in tags or "led14seg" in tags or "dotmatrix" in tags:
            text += 1
        if "disk" in tags:
            disk += 1
        if tags - {"image", "rect", "disk", "text", "bounds", "color", "simplecounter", "led7seg", "led14seg", "led16seg", "dotmatrix", "reel"}:
            other += 1
        ds = e.get("defstate", "0")
        if imgs and not [i for i in imgs if i.get("state", ds) == ds]:
            lamp += 1
        elif any(k.get("state") is not None for k in kids):
            lamp += 1
        for i in imgs:
            for a in ("file", "alphafile"):
                if i.get(a):
                    used_imgs.add(i.get(a))
    if lamp: info["issues"].append(f"{lamp} lamp/state elements (need outputs)")
    if text: info["issues"].append(f"{text} text/LED elements")
    if disk: info["issues"].append(f"{disk} disk elements")
    if other: info["issues"].append(f"{other} elements with other primitives")
    modern = rot = groups = multiscreen = 0
    for v in lay.findall("view"):
        scr = [i for i in v.iter("screen")]
        if len(scr) != 1:
            multiscreen += 1
        for it in v.iter():
            if it.tag == "element" and it.get("ref"): modern += 1
            if it.tag in ("group", "repeat", "param", "collection"): groups += 1
            if it.tag == "orientation" and (it.get("rotate", "0") != "0" or it.get("flipy") == "yes"): rot += 1
        info["views"].append(v.get("name"))
    if modern: info["issues"].append("modern <element ref> syntax")
    if groups: info["issues"].append("groups/repeat/param")
    if rot: info["issues"].append(f"{rot} rotated/flipy items")
    if multiscreen: info["issues"].append(f"{multiscreen} views without exactly one screen")
    if len(info["views"]) > 15: info["issues"].append(f"{len(info['views'])} views (>15)")
    if len(used_imgs) > 32: info["issues"].append(f"{len(used_imgs)} images (>32)")
    for n in used_imgs:
        if n not in names:
            info["issues"].append(f"missing {n}")
            continue
        d = z.read(n)
        info["png_bytes"] += len(d)
        if d[:8] == b"\x89PNG\r\n\x1a\n":
            w, h, bd, ct, _, _, il = struct.unpack(">IIBBBBB", d[16:29])
            info["max_png"] = [max(info["max_png"][0], w), max(info["max_png"][1], h)]
            if bd != 8: info["issues"].append(f"{n}: {bd}-bit PNG")
            if il: info["issues"].append(f"{n}: interlaced PNG")
            if w > 8191 or h > 8191: info["issues"].append(f"{n}: {w}x{h} (>8191)")
        else:
            info["issues"].append(f"{n}: not PNG")
    if info["png_bytes"] > 32 * 1024 * 1024 - 2048:
        info["issues"].append(f"blob {info['png_bytes'] / 1e6:.1f} MB (>32 MB)")
    return info

zips = {p.stem for p in ARCH.glob("*.zip")}
out, cache = {}, {}
for core, sets in cores.items():
    rows = []
    for s, rot in sorted(sets.items()):
        par = clones.get(s)
        src = s if s in zips else (par if par in zips else None)
        if not src:
            continue
        if src not in cache:
            try:
                cache[src] = analyse(src)
            except Exception as e:
                cache[src] = {"zip": src, "issues": [f"error {e}"], "views": [], "png_bytes": 0, "max_png": [0, 0]}
        rows.append({"set": s, "parent": par, "rot": rot, "zip": src, "via": "own" if src == s else "parent"})
    out[core] = {"nsets": len(sets), "rows": rows}
json.dump({"cores": out, "zips": cache}, open(sys.argv[2], "w"), indent=1)
for core, c in out.items():
    zs = sorted({r["zip"] for r in c["rows"]})
    print(f"{core:34} sets {c['nsets']:4}  with art {len(c['rows']):4}  zips {len(zs):3}  {' '.join(zs)[:150]}")
