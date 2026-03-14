#!/usr/bin/env python3
"""Convert FrikBot .way waypoint files to compiled-in .qc format.

The .way format uses Quake console cvar assignments (saved1/2/3 for position,
scratch1/2/3/4 for links, saved4 for flags+state). CVars persist across records;
'wait' triggers loading each waypoint.

saved4 encoding: (b_aiflags * 4) + state_bit
  state_bit 1 = first waypoint, 2 = subsequent, 3 = end marker
"""

import re
import sys
import os
import math
from pathlib import Path


def parse_way_lines(lines, waypoints, state):
    """Parse .way format lines, appending to waypoints list.

    state dict holds persistent cvar values across files (exec chaining).
    Returns list of exec'd filenames encountered.
    """
    exec_files = []

    for line in lines:
        line = line.strip()
        if not line or line.startswith('//'):
            continue

        # Handle exec commands (chain to .wa1, .wa2 files)
        if line.startswith('exec '):
            exec_files.append(line[5:].strip())
            continue

        # Parse all cvar assignments on this line
        parts = line.split(';')
        has_wait = False

        for part in parts:
            part = part.strip()
            if part == 'wait':
                has_wait = True
                continue
            if not part:
                continue

            tokens = part.split()
            if len(tokens) >= 2:
                name = tokens[0]
                try:
                    val = float(tokens[1])
                except ValueError:
                    continue
                if name in state:
                    state[name] = val

        if has_wait:
            b_aiflags = int(math.floor(state['saved4'] / 4))

            waypoints.append({
                'x': state['saved1'],
                'y': state['saved2'],
                'z': state['saved3'],
                'link1': int(state['scratch1']),
                'link2': int(state['scratch2']),
                'link3': int(state['scratch3']),
                'link4': int(state['scratch4']),
                'flags': b_aiflags,
            })

    return exec_files


def parse_way_file(filepath):
    """Parse a .way file (and chained .wa* files) and return list of waypoint dicts."""
    waypoints = []
    state = {
        'saved1': 0.0, 'saved2': 0.0, 'saved3': 0.0,
        'scratch1': 0.0, 'scratch2': 0.0, 'scratch3': 0.0, 'scratch4': 0.0,
        'saved4': 0.0,
    }
    way_dir = os.path.dirname(filepath)

    # Process main .way file and any chained exec files
    files_to_process = [filepath]
    while files_to_process:
        current = files_to_process.pop(0)
        with open(current, 'r', encoding='utf-8', errors='replace') as f:
            lines = f.readlines()
        exec_refs = parse_way_lines(lines, waypoints, state)

        # Resolve exec paths: "maps/e1m2.wa1" -> look in our waypoints dir
        for ref in exec_refs:
            # exec refs use "maps/foo.wa1" but our files are in waypoints/
            basename = os.path.basename(ref)
            local_path = os.path.join(way_dir, basename)
            if os.path.exists(local_path):
                files_to_process.append(local_path)
            else:
                print(f"  WARNING: exec'd file not found: {ref} (looked for {local_path})")

    return waypoints


def format_coord(v):
    """Format a coordinate value: use integer if whole, else one decimal."""
    if v == int(v):
        return f"{int(v)}.0"
    else:
        return f"{v:.1f}"


def waypoints_to_qc(waypoints, mapname):
    """Convert waypoint list to QC source string."""
    lines = []
    lines.append(f"/* QC Waypoint Data - {mapname}")
    lines.append("   Converted from .way format (whipowill/quake-mod-frikbot-waypoints) */")
    lines.append("")
    lines.append("void(vector org, vector bit1, float bit4, float flargs) make_way;")
    lines.append("")
    lines.append(f"void() map_{mapname} =")
    lines.append("{")

    for wp in waypoints:
        x = format_coord(wp['x'])
        y = format_coord(wp['y'])
        z = format_coord(wp['z'])
        org = f"'{x} {y} {z}'"

        l1 = wp['link1']
        l2 = wp['link2']
        l3 = wp['link3']
        l4 = wp['link4']
        flags = wp['flags']

        bit1 = f"'{l1} {l2} {l3}'"
        bit4 = str(l4)

        lines.append(f"\tmake_way({org}, {bit1}, {bit4}, {flags});")

    lines.append("};")
    lines.append("")
    return "\n".join(lines)


def convert_file(way_path, output_dir):
    """Convert a single .way file to .qc."""
    stem = Path(way_path).stem  # e.g. "e1m1"
    mapname = stem
    qc_filename = f"map_{mapname}.qc"

    waypoints = parse_way_file(way_path)
    if not waypoints:
        print(f"  WARNING: No waypoints found in {way_path}, skipping")
        return None

    qc_source = waypoints_to_qc(waypoints, mapname)
    output_path = os.path.join(output_dir, qc_filename)

    with open(output_path, 'w', encoding='utf-8') as f:
        f.write(qc_source)

    print(f"  {os.path.basename(way_path)} -> {qc_filename} ({len(waypoints)} waypoints)")
    return qc_filename


def main():
    if len(sys.argv) < 3:
        print(f"Usage: {sys.argv[0]} <waypoints_dir> <output_dir>")
        print(f"  Converts all .way files in waypoints_dir to .qc files in output_dir")
        sys.exit(1)

    way_dir = sys.argv[1]
    out_dir = sys.argv[2]
    os.makedirs(out_dir, exist_ok=True)

    way_files = sorted(Path(way_dir).glob("*.way"))
    if not way_files:
        print(f"No .way files found in {way_dir}")
        sys.exit(1)

    print(f"Converting {len(way_files)} .way files...")
    converted = []
    for wf in way_files:
        result = convert_file(str(wf), out_dir)
        if result:
            converted.append(result)

    print(f"\nDone: {len(converted)} files converted to {out_dir}/")


if __name__ == "__main__":
    main()
