#!/usr/bin/env python3
"""
nav2way.py — Convert Quake Enhanced .nav files to FrikBot .way files.

Reads the binary .nav format (version 14/15) used by the Quake 2021 Re-Release
KEX engine and outputs .way text files compatible with FrikBot X's waypoint
loader (exec-based cvar pipeline).

Usage:
    python nav2way.py <input.nav> [output.way]
    python nav2way.py --batch <nav_dir> [way_dir]

The .nav format spec is from jpiolho's reverse engineering:
https://steamcommunity.com/sharedfiles/filedetails/?id=2584757297
"""

import struct
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
# These are per-link flags; we accumulate them on the destination node
# since FrikBot flags are per-node, not per-edge.
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

# .way file byte budget before splitting into .wa1, .wa2, etc.
WAY_SPLIT_BYTES = 7500


class NavNode:
    __slots__ = ('flags', 'conn_count', 'conn_start', 'radius',
                 'origin', 'links', 'aiflags')

    def __init__(self):
        self.flags = 0
        self.conn_count = 0
        self.conn_start = 0
        self.radius = 0
        self.origin = (0.0, 0.0, 0.0)
        self.links = []     # list of (dest_index, link_type)
        self.aiflags = 0


class NavLink:
    __slots__ = ('destination', 'link_type', 'traversal_index')

    def __init__(self, dest, ltype, trav):
        self.destination = dest
        self.link_type = ltype
        self.traversal_index = trav


def read_nav(filepath):
    """Parse a .nav binary file. Returns list of NavNode with links resolved."""
    with open(filepath, 'rb') as f:
        data = f.read()

    pos = 0

    # Header: magic(4) + version(4) + node_count(4) + link_count(4) + traversal_count(4)
    magic = data[pos:pos+4]
    pos += 4
    if magic != b'NAV2':
        raise ValueError(f"Bad magic: {magic!r}, expected b'NAV2'")

    version, node_count, link_count, traversal_count = struct.unpack_from('<IIII', data, pos)
    pos += 16

    if version not in (14, 15):
        raise ValueError(f"Unsupported nav version {version}, expected 14 or 15")

    # Nodes: 8 bytes each (flags:u16, conn_count:u16, conn_start:u16, radius:u16)
    nodes = []
    for i in range(node_count):
        flags, conn_count, conn_start, radius = struct.unpack_from('<HHHH', data, pos)
        pos += 8
        n = NavNode()
        n.flags = flags
        n.conn_count = conn_count
        n.conn_start = conn_start
        n.radius = radius
        nodes.append(n)

    # Node Origins: 12 bytes each (x:f32, y:f32, z:f32)
    for i in range(node_count):
        x, y, z = struct.unpack_from('<fff', data, pos)
        pos += 12
        nodes[i].origin = (x, y, z)

    # Links: 6 bytes each (destination:u16, type:u16, traversal_index:u16)
    links = []
    for i in range(link_count):
        dest, ltype, trav = struct.unpack_from('<HHH', data, pos)
        pos += 6
        links.append(NavLink(dest, ltype, trav))

    # Resolve links onto nodes
    for node in nodes:
        for li in range(node.conn_start, node.conn_start + node.conn_count):
            if li < len(links):
                link = links[li]
                node.links.append((link.destination, link.link_type))
                # Accumulate link-type flags onto the destination node
                if link.link_type in NAV_LINK_TO_AIFLAGS:
                    nodes[link.destination].aiflags |= NAV_LINK_TO_AIFLAGS[link.link_type]

    # We skip traversals and edicts — not needed for FrikBot waypoints

    return nodes


def nodes_to_way(nodes, mapname="unknown"):
    """
    Convert NavNodes to .way file content (list of strings, one per output file).

    FrikBot .way format:
    - Console commands setting cvars: saved1/saved2/saved3 (XYZ),
      scratch1-scratch4 (link indices, 1-based), saved4 (aiflags*4 + control).
    - Delta-compressed: only changed values are written.
    - 'wait' after each waypoint for frame-based loading.
    - saved4 & 3 == 1 for first waypoint (signals ClearAllWays).
    - saved4 & 3 == 2 for subsequent waypoints.
    - saved4 & 3 == 3 for end marker (signals FixWaypoints).
    - File splits at ~7500 bytes to avoid engine exec buffer limits.

    FrikBot waypoint indices are 1-based (.count starts at 1).
    .nav node indices are 0-based. We add 1 when writing link targets.
    Unlinked slots are 0.
    """
    if not nodes:
        return ["// no waypoints\nsaved4 3; wait\n"]

    files = []
    lines = []
    bytecounter = 0

    # Preamble
    header = f"// {mapname} - converted from .nav by nav2way\n"
    lines.append(header)
    bytecounter += len(header)

    # Track previous values for delta compression
    prev_x = None
    prev_y = None
    prev_z = None
    prev_s1 = None
    prev_s2 = None
    prev_s3 = None
    prev_s4 = None

    for i, node in enumerate(nodes):
        way_index = i + 1  # FrikBot is 1-based
        is_first = (i == 0)

        x = int(round(node.origin[0]))
        y = int(round(node.origin[1]))
        z = int(round(node.origin[2]))

        # Get up to 4 link targets (1-based indices, 0 = no link)
        link_targets = [0, 0, 0, 0]
        for li, (dest, ltype) in enumerate(node.links[:MAX_LINKS]):
            link_targets[li] = dest + 1  # convert 0-based to 1-based

        s1, s2, s3, s4 = link_targets

        # Check if we need to split files
        if bytecounter > WAY_SPLIT_BYTES:
            lines.append("\n// **** break here ****\n")
            files.append("".join(lines))
            lines = []
            bytecounter = 26
            # After a split, force all values to be written
            prev_x = None
            prev_y = None
            prev_z = None
            prev_s1 = None
            prev_s2 = None
            prev_s3 = None
            prev_s4 = None

        # Write coordinates (delta compressed)
        coord_parts = []
        if x != prev_x or is_first:
            coord_parts.append(f"saved1 {x}")
            prev_x = x
        if y != prev_y or is_first:
            coord_parts.append(f"saved2 {y}")
            prev_y = y
        if z != prev_z or is_first:
            coord_parts.append(f"saved3 {z}")
            prev_z = z

        if coord_parts:
            coord_line = "; ".join(coord_parts) + "\n"
            lines.append(coord_line)
            bytecounter += len(coord_line)

        # Write link targets (delta compressed)
        link_parts = []
        if s1 != prev_s1 or is_first:
            link_parts.append(f"scratch1 {s1}")
            prev_s1 = s1
        if s2 != prev_s2 or is_first:
            link_parts.append(f"scratch2 {s2}")
            prev_s2 = s2
        if s3 != prev_s3 or is_first:
            link_parts.append(f"scratch3 {s3}")
            prev_s3 = s3
        if s4 != prev_s4 or is_first:
            link_parts.append(f"scratch4 {s4}")
            prev_s4 = s4

        if link_parts:
            link_line = "; ".join(link_parts) + "\n"
            lines.append(link_line)
            bytecounter += len(link_line)

        # Write aiflags + control bits
        control = 1 if is_first else 2
        flag_val = node.aiflags * 4 + control
        flag_line = f"saved4 {flag_val}; wait\n"
        lines.append(flag_line)
        bytecounter += len(flag_line)

    # End marker
    lines.append("saved4 3\n")
    files.append("".join(lines))

    return files


def convert_nav_to_way(nav_path, way_path=None):
    """Convert a single .nav file to .way file(s)."""
    if way_path is None:
        way_path = os.path.splitext(nav_path)[0] + ".way"

    mapname = os.path.splitext(os.path.basename(nav_path))[0]
    nodes = read_nav(nav_path)

    print(f"  {mapname}: {len(nodes)} nodes, "
          f"{sum(len(n.links) for n in nodes)} links")

    # Warn about nodes with >4 links (FrikBot limit)
    over_linked = sum(1 for n in nodes if len(n.links) > MAX_LINKS)
    if over_linked:
        print(f"  warning: {over_linked} nodes have >{MAX_LINKS} links, "
              f"extras will be dropped")

    files = nodes_to_way(nodes, mapname)

    # Write main .way file and overflow files (.wa1, .wa2, ...)
    base, _ = os.path.splitext(way_path)
    for i, content in enumerate(files):
        if i == 0:
            out_path = way_path
        else:
            out_path = f"{base}.wa{i}"
        with open(out_path, 'w') as f:
            f.write(content)
        print(f"  wrote {out_path} ({len(content)} bytes)")


def main():
    parser = argparse.ArgumentParser(
        description="Convert Quake Enhanced .nav files to FrikBot .way files")
    parser.add_argument('input', help=".nav file or directory (with --batch)")
    parser.add_argument('output', nargs='?', default=None,
                        help=".way output file or directory (with --batch)")
    parser.add_argument('--batch', action='store_true',
                        help="Convert all .nav files in input directory")
    args = parser.parse_args()

    if args.batch:
        nav_dir = args.input
        way_dir = args.output or nav_dir

        if not os.path.isdir(nav_dir):
            print(f"Error: {nav_dir} is not a directory", file=sys.stderr)
            sys.exit(1)

        os.makedirs(way_dir, exist_ok=True)

        nav_files = sorted(glob.glob(os.path.join(nav_dir, "*.nav")))
        if not nav_files:
            print(f"No .nav files found in {nav_dir}", file=sys.stderr)
            sys.exit(1)

        print(f"Converting {len(nav_files)} .nav files...")
        for nav_path in nav_files:
            mapname = os.path.splitext(os.path.basename(nav_path))[0]
            way_path = os.path.join(way_dir, mapname + ".way")
            try:
                convert_nav_to_way(nav_path, way_path)
            except Exception as e:
                print(f"  ERROR converting {nav_path}: {e}", file=sys.stderr)

        print("Done.")
    else:
        if not os.path.isfile(args.input):
            print(f"Error: {args.input} not found", file=sys.stderr)
            sys.exit(1)

        convert_nav_to_way(args.input, args.output)


if __name__ == '__main__':
    main()
