#!/usr/bin/env python3
"""Classify unreachable spawns/items from a nav_harness conn run.

Reads the per-map logs in the harness OUTDIR, pulls every
"Nav: CONNECTIVITY unreachable" line, then cross-references the map's
BSP entity lump (parsed straight out of the pak files) to guess WHY
each one is unreachable:

  SECRET      - inside/near a func_door_secret brush
  KEY_DOOR    - near a func_door requiring a silver/gold key
  TRIG_DOOR   - near a func_door that only opens by trigger (targetname)
  ON_MOVER    - inside a func_train / func_plat brush bbox (no static floor)
  UNCLASSIFIED- none of the above: candidate real navmesh bug

Heuristic triage, not proof -- it tells you which log lines are worth
chasing as navmesh-generation bugs and which are SP map design.

Usage: src/tools/nav_triage.py [logdir]   (default ~/quake/navruns/harness)
"""

import os
import re
import struct
import sys
from pathlib import Path

QUAKE_ID1 = Path(os.environ.get("QUAKE_ID1", Path.home() / "quake" / "id1"))
LOGDIR = Path(sys.argv[1] if len(sys.argv) > 1 else Path.home() / "quake" / "navruns" / "harness")

DOOR_SILVER_KEY = 8
DOOR_GOLD_KEY = 16
NEAR = 160.0  # how close a gate must be to an item to plausibly explain it


def pak_files():
    for pak in sorted(QUAKE_ID1.glob("pak*.pak")):
        with open(pak, "rb") as f:
            magic, diroff, dirlen = struct.unpack("<4sii", f.read(12))
            assert magic == b"PACK", pak
            f.seek(diroff)
            for i in range(dirlen // 64):
                name, off, size = struct.unpack("<56sii", f.read(64))
                yield pak, name.split(b"\0")[0].decode(), off, size


PAK_INDEX = {}


def read_pak_file(name):
    if not PAK_INDEX:
        for pak, fname, off, size in pak_files():
            PAK_INDEX[fname] = (pak, off, size)  # later paks override earlier
    pak, off, size = PAK_INDEX[name]
    with open(pak, "rb") as f:
        f.seek(off)
        return f.read(size)


def parse_bsp(mapname):
    """Return (entities, model_bboxes). entities = list of key/value dicts;
    model_bboxes[i] = (mins, maxs) for brush model *i."""
    data = read_pak_file(f"maps/{mapname}.bsp")
    version = struct.unpack_from("<i", data, 0)[0]
    assert version == 29, (mapname, version)
    lumps = [struct.unpack_from("<ii", data, 4 + i * 8) for i in range(15)]

    ent_off, ent_len = lumps[0]
    ents = []
    for block in re.findall(r"\{(.*?)\}", data[ent_off:ent_off + ent_len].decode("ascii", "replace"), re.S):
        ent = dict(re.findall(r'"([^"]+)"\s+"([^"]*)"', block))
        if ent:
            ents.append(ent)

    mod_off, mod_len = lumps[14]
    bboxes = []
    for i in range(mod_len // 64):
        vals = struct.unpack_from("<9f", data, mod_off + i * 64)
        bboxes.append((vals[0:3], vals[3:6]))
    return ents, bboxes


def vec(s):
    return tuple(float(x) for x in s.split())


def bbox_dist(pos, mins, maxs):
    d = 0.0
    for i in range(3):
        if pos[i] < mins[i]:
            d += (mins[i] - pos[i]) ** 2
        elif pos[i] > maxs[i]:
            d += (pos[i] - maxs[i]) ** 2
    return d ** 0.5


def classify(mapname, pos):
    ents, bboxes = BSP_CACHE.setdefault(mapname, parse_bsp(mapname))
    hits = []
    for ent in ents:
        cn = ent.get("classname", "")
        model = ent.get("model", "")
        if not model.startswith("*"):
            continue
        mins, maxs = bboxes[int(model[1:])]
        d = bbox_dist(pos, mins, maxs)
        if cn == "func_door_secret" and d <= NEAR:
            hits.append((d, "SECRET", cn))
        elif cn == "func_door" and d <= NEAR:
            flags = int(ent.get("spawnflags", "0") or "0")
            if flags & (DOOR_SILVER_KEY | DOOR_GOLD_KEY):
                hits.append((d, "KEY_DOOR", cn))
            elif ent.get("targetname"):
                hits.append((d, "TRIG_DOOR", cn))
        elif cn in ("func_train", "func_plat") and d <= 16.0:
            hits.append((d, "ON_MOVER", cn))
    if not hits:
        return "UNCLASSIFIED", ""
    hits.sort()
    d, tag, cn = hits[0]
    return tag, f"{cn} @{d:.0f}u"


BSP_CACHE = {}
LINE_RE = re.compile(
    r"Nav: CONNECTIVITY unreachable (\S+) at \((-?\d+) (-?\d+) (-?\d+)\)(?::)? (.*)")

STOP_RE = re.compile(r"stopped at (-?\d+) (-?\d+) (-?\d+)")
CLUSTER_DIST = 256.0


def cluster_key(clusters, stop):
    """Merge stops within CLUSTER_DIST into one gap-site."""
    for c in clusters:
        if sum((a - b) ** 2 for a, b in zip(c, stop)) ** 0.5 <= CLUSTER_DIST:
            return c
    clusters.append(stop)
    return stop


counts = {}
gap_sites = 0
for log in sorted(LOGDIR.glob("*.log")):
    mapname = log.stem
    rows = []
    for line in log.read_text(errors="replace").splitlines():
        m = LINE_RE.search(line)
        if not m:
            continue
        cn, x, y, z, why = m.groups()
        pos = (float(x), float(y), float(z))
        sm = STOP_RE.search(why)
        stop = tuple(float(v) for v in sm.groups()) if sm else None
        if "No floor polygon" in why:
            tag, detail = "NO_FLOOR_POLY", ""
            # still try the mover check -- items riding a train have no poly
            t2, d2 = classify(mapname, pos)
            if t2 == "ON_MOVER":
                tag, detail = t2, d2
        elif "snapped" in why:
            tag, detail = "SNAP_HOLE", ""
        elif cn.startswith("info_player"):
            tag, detail = "SPAWN_ISLAND", ""
        else:
            tag, detail = classify(mapname, pos)
        rows.append((tag, cn, pos, detail, stop))
        counts[tag] = counts.get(tag, 0) + 1

    # Group by path dead-end: everything that stops at the same spot is
    # ONE navmesh gap, however many items sit behind it.
    clusters = []
    grouped = {}
    for tag, cn, pos, detail, stop in rows:
        key = cluster_key(clusters, stop) if stop else None
        grouped.setdefault(key, []).append((tag, cn, pos, detail))
    for key, group in grouped.items():
        if key is not None:
            gap_sites += 1
            print(f"{mapname}: GAP at ({key[0]:.0f} {key[1]:.0f} {key[2]:.0f}) blocks {len(group)}:")
        for tag, cn, pos, detail in group:
            prefix = "    " if key is not None else f"{mapname:6s} "
            print(f"{prefix}{tag:13s} {cn:34s} ({pos[0]:.0f} {pos[1]:.0f} {pos[2]:.0f}) {detail}")

print("---")
for tag in sorted(counts, key=counts.get, reverse=True):
    print(f"{counts[tag]:4d} {tag}")
print(f"{gap_sites:4d} distinct gap sites (stop-point clusters)")
