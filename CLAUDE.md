# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

FrikBotNex is a QuakeC bot AI system for Quake 1 deathmatch, evolved from FrikBot X (v0.10.2). It runs on the CleanFixedQuakeC base (Jason2Brownlee's cleaned GPL release of the original id Software game code).

## Build

QuakeC has no package manager or dependency system. Compile with any QuakeC compiler from the `src/qc/` directory:

```sh
cd src/qc
fteqcc    # or: gmqcc progs.src  /  qcc
```

The compilation manifest is `src/qc/progs.src`. Output goes directly to the repo root: `progs.dat` (game bytecode) and `progs.lno` (debug line numbers).

There are no automated tests — testing requires running in a Quake engine.

## Architecture

### Three source areas

1. **Base game code** (`src/qc/`) — Standard Quake game logic (monsters, weapons, items, physics, client). `defs.qc` must be compiled first (global declarations).

2. **Bot AI system** (`src/frikbot/`) — The core of this project:
   - `bot.qc` — Bot entity lifecycle, main loop, initialization
   - `bot_ai.qc` — Target/enemy selection, goal management, aim pipeline, per-frame think
   - `bot_fight.qc` — Combat movement state machine (fight/retreat/pressure/flank), opponent modeling
   - `bot_goal.qc` — Item goal scoring: utility-weighted pickup selection, resource pools, personality preferences
   - `bot_move.qc` — Movement command generation, bunny hopping, off-mesh link traversal (jump/drop/plat/train/door/teleport)
   - `bot_misc.qc` — Utility functions

   There is no more hand-placed waypoint system — `bot_way.qc`, `bot_ed.qc`, `bot_sense.qc`,
   `bot_think.qc`, `bot_shoot.qc`, `bot_phys.qc`, `bot_qw.qc`, and `src/waypoints/` were all
   removed when navigation moved to a runtime navmesh (see below).

3. **Navmesh navigation** (`src/server/`) — C++ engine code, built alongside the game server:
   - `nav_hull.cpp`/`nav_mesh.cpp` — Extracts hull-1 collision geometry from the loaded BSP
     (worldmodel + func_wall/func_plat/func_train/func_door) and builds a Recast/Detour navmesh
   - `nav_bot.cpp` — Off-mesh link detection (jump, drop, rocket jump, teleporter, platform,
     train, door) and QC-facing builtins for path queries and link traversal
   - `net_bot.c` — Fake network driver so bots run through real engine physics, not QC emulation
   - `nav_val.cpp` — Waypoint/connectivity validation oracle used by the test harness
   - `src/tools/nav_harness.sh` + `nav_triage.py` — Full-map connectivity gate: every spawn must
     reach every item on every stock map (currently 37/37 id1, 31/31 mission-pack, 0 unreachable)

### Bot integration into game loop

The bot system hooks into the base game at four points in `src/qc/world.qc` and `src/qc/client.qc`:
- `BotInit()` in `WorldSpawn()` — spawns physics entities for each client slot
- `BotFrame()` in `StartFrame()` — per-frame AI processing (pathfinding, physics, decisions)
- `BotPreFrame()` in `PlayerPreThink()` — early bot checks, skips human-player logic for bots
- `BotPostFrame()` in `PlayerPostThink()` — late-stage updates after engine processing

### Key entity fields (bot-specific)

- `b_skill` (0–3) — Bot difficulty level, affects aim, dodge timing, decision-making
- `b_aiflags` — Bitfield of `AI_*` constants controlling off-mesh link and behavior types (telelinks, doors, jumps, platforms, sniper spots, ambush points — 18 flags total)
- `target1`–`target4` — Tracked entities (enemies, items, goals)
- `phys_obj` — Linked physics prediction entity (separate from game physics)

### Compilation order matters

`progs.src` defines strict compile order. Bot modules come before base game files (except `defs.qc`). New files must be added to `progs.src` in the correct position.

The engine (`src/server/`) is a separate build — see `src/build/build-server.sh` — and must
be rebuilt whenever `nav_bot.cpp`, `nav_mesh.cpp`/`.h`, `nav_hull.cpp`, or `net_bot.c`/`.h`
change. QC-only changes don't need an engine rebuild.

## Licensing

Base game code is GPL v2 (id Software). FrikBot code is public domain (Ryan "FrikaC" Smith).
