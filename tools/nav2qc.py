#!/usr/bin/env python3
"""
nav2qc.py — Convert Quake Enhanced .navjson files to FrikBot QC waypoint files.

Reads the .navjson format used by jpiolho's Quake Nav Depot and outputs .qc
files compatible with FrikBot's make_way() compiled-in waypoint system.

Usage:
    python nav2qc.py <input.navjson> [output.qc]
    python nav2qc.py --batch <nav_dir> [qc_dir]
"""

import json
import sys
import os
import glob
import argparse


# .nav link types (from KEX engine)
LINK_WALK = 0
LINK_LONGJUMP = 1
LINK_BARRICADE = 2
LINK_JUMP = 3
LINK_LADDER = 4
LINK_FALL = 5
LINK_DOOR = 6
LINK_TELEPORT = 7
LINK_ELEVATOR = 8
LINK_TRAIN = 9

# FrikBot AI flags (from bot.qc / bot_way.qc)
AI_TELELINK = 1
AI_JUMP = 2
AI_DOOR = 4
AI_PRECISION = 16
AI_PLAT_BOTTOM = 64
AI_RIDE_TRAIN = 128
AI_SUPER_JUMP = 256
AI_SNIPER = 512
AI_AMBUSH = 1024

# Map .nav link types to FrikBot AI flag hints.
NAV_LINK_TO_AIFLAGS = {
    LINK_JUMP: AI_JUMP,
    LINK_LONGJUMP: AI_JUMP | AI_PRECISION,
    LINK_DOOR: AI_DOOR,
    LINK_TELEPORT: AI_TELELINK,
    LINK_ELEVATOR: AI_PLAT_BOTTOM,
    LINK_TRAIN: AI_RIDE_TRAIN,
    LINK_FALL: AI_PRECISION,
}

# Maximum links per waypoint in FrikBot
MAX_LINKS = 4


def read_navjson(filepath):
    """Parse a .navjson file. Returns list of dicts with origin, links, aiflags."""
    with open(filepath, 'r') as f:
        data = json.load(f)

    json_nodes = data.get('Nodes', [])

    # Build coordinate -> index lookup
    def coord_key(origin):
        return (round(origin[0], 3), round(origin[1], 3), round(origin[2], 3))

    coord_to_index = {}
    for i, jn in enumerate(json_nodes):
        key = coord_key(jn['Origin'])
        coord_to_index[key] = i

    nodes = []
    for i, jn in enumerate(json_nodes):
        node = {
            'origin': tuple(jn['Origin']),
            'links': [],
            'aiflags': 0,
        }

        for jl in jn.get('Links', []):
            target_key = coord_key(jl['Target'])
            dest_index = coord_to_index.get(target_key)
            if dest_index is None:
                continue
            link_type = jl.get('Type', 0)
            node['links'].append((dest_index, link_type))

        nodes.append(node)

    # Accumulate link-type AI flags onto destination nodes
    for node in nodes:
        for dest_index, link_type in node['links']:
            if link_type in NAV_LINK_TO_AIFLAGS:
                nodes[dest_index]['aiflags'] |= NAV_LINK_TO_AIFLAGS[link_type]

    return nodes


def fmt_coord(v):
    """Format a float as a coordinate string, using .1 precision."""
    s = f"{v:.1f}"
    return s


def nodes_to_qc(nodes, mapname):
    """Convert nodes to a FrikBot QC waypoint file string.

    make_way(origin, links_1_2_3, link_4, aiflags)
      - origin: vector
      - links_1_2_3: vector packing link indices 1-3 (1-based, 0=none)
      - link_4: float, 4th link index (1-based, 0=none)
      - aiflags: float, AI behavior flags
    """
    lines = []
    lines.append(f"/* QC Waypoint Data - map_{mapname}.qc")
    lines.append(f"   Converted from {mapname}.navjson by nav2qc.py")
    lines.append(f"   Source: jpiolho's Quake Nav Depot */")
    lines.append("")
    lines.append("void(vector org, vector bit1, float bit4, float flargs) make_way;")
    lines.append("")
    lines.append(f"void() map_{mapname} =")
    lines.append("{")

    for i, node in enumerate(nodes):
        ox = fmt_coord(node['origin'][0])
        oy = fmt_coord(node['origin'][1])
        oz = fmt_coord(node['origin'][2])

        # Get up to 4 link targets (1-based, 0=none)
        link_targets = [0, 0, 0, 0]
        for li, (dest, ltype) in enumerate(node['links'][:MAX_LINKS]):
            link_targets[li] = dest + 1  # 0-based to 1-based

        l1, l2, l3, l4 = link_targets
        aiflags = node['aiflags']

        lines.append(f"\tmake_way('{ox} {oy} {oz}', '{l1} {l2} {l3}', {l4}, {aiflags});")

    lines.append("};")
    lines.append("")

    return "\n".join(lines)


def convert(navjson_path, qc_path=None):
    """Convert a single .navjson file to a .qc waypoint file."""
    mapname = os.path.splitext(os.path.basename(navjson_path))[0]

    if qc_path is None:
        qc_dir = os.path.dirname(navjson_path)
        qc_path = os.path.join(qc_dir, f"map_{mapname}.qc")

    nodes = read_navjson(navjson_path)

    over_linked = sum(1 for n in nodes if len(n['links']) > MAX_LINKS)
    print(f"  {mapname}: {len(nodes)} nodes, "
          f"{sum(len(n['links']) for n in nodes)} links", end="")
    if over_linked:
        print(f" ({over_linked} nodes truncated to {MAX_LINKS} links)", end="")
    print()

    qc_content = nodes_to_qc(nodes, mapname)
    with open(qc_path, 'w') as f:
        f.write(qc_content)

    print(f"  wrote {qc_path}")
    return mapname


def main():
    parser = argparse.ArgumentParser(
        description="Convert .navjson files to FrikBot QC waypoint files")
    parser.add_argument('input',
                        help=".navjson file or directory (with --batch)")
    parser.add_argument('output', nargs='?', default=None,
                        help=".qc output file or directory (with --batch)")
    parser.add_argument('--batch', action='store_true',
                        help="Convert all .navjson files in input directory")
    parser.add_argument('--filter', default=None,
                        help="Glob filter for --batch (e.g. 'e*m*.navjson')")
    args = parser.parse_args()

    if args.batch:
        nav_dir = args.input
        qc_dir = args.output or nav_dir

        if not os.path.isdir(nav_dir):
            print(f"Error: {nav_dir} is not a directory", file=sys.stderr)
            sys.exit(1)

        os.makedirs(qc_dir, exist_ok=True)

        pattern = args.filter or "*.navjson"
        nav_files = sorted(glob.glob(os.path.join(nav_dir, pattern)))
        if not nav_files:
            print(f"No files matching {pattern} in {nav_dir}", file=sys.stderr)
            sys.exit(1)

        print(f"Converting {len(nav_files)} navjson files...")
        maps = []
        for nav_path in nav_files:
            try:
                mapname = convert(nav_path,
                                  os.path.join(qc_dir,
                                               f"map_{os.path.splitext(os.path.basename(nav_path))[0]}.qc"))
                maps.append(mapname)
            except Exception as e:
                print(f"  ERROR converting {nav_path}: {e}", file=sys.stderr)

        print(f"\nDone. Converted {len(maps)} maps.")
    else:
        if not os.path.isfile(args.input):
            print(f"Error: {args.input} not found", file=sys.stderr)
            sys.exit(1)
        convert(args.input, args.output)


if __name__ == '__main__':
    main()
