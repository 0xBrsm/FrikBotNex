# Changelog

## 0.3.0 — Detour Best Practice

Rewrote navmesh integration to follow Detour best practices.

- **dtPathCorridor** for path following — replaces manual waypoint tracking
- **Single-pass build** — jump links detected from contour edges mid-pipeline
- **Item poly cache** — cached findNearestPoly with 2u extents, direct findPath
- **Goal arrival** — corridor pos vs target distance check fixes end-of-path detection
- **Door blocking** — entity→poly mapping, post-path rejection, QC nav_block/nav_unblock
- **Unified look** — bot_aim + bot_look with skill-scaled flick decay for combat and nav
- **Gradient wall probe** — downward traces with frame-to-frame floor comparison
- **C++ nav_bot** — removed wrapper functions, exposed runtime struct directly

## 0.2.0 — Navmesh Navigation

Replaced hand-placed waypoints with automatic Recast/Detour navmesh. Bots navigate any Quake map without pre-authored data.

- **Engine-native bot physics** via `net_bot` network driver — bots run through real engine physics instead of QC emulation
- **Navmesh from BSP** — automatic extraction of worldmodel + brush entity (func_wall, func_plat, func_train, func_door) surfaces
- **Per-bot cached pathfinding** — path computed once per goal, followed frame-by-frame. Replaced per-frame findPath calls
- **Off-mesh link detection** — automatic jump, drop, rocket jump, teleporter, platform, train, and door links from boundary edge scanning and entity inspection
- **Link metadata + area costs** — FrikBot AI_* type flags per link, Detour cost weighting (jumps 3x, platforms 5x, rocket jumps 10x). Pathfinder prefers walking
- **QC link traversal** — bot branches on link type: jump, rocket jump, swim up, ride platform, trigger door, walk through teleporter
- **Removed waypoint system** — deleted bot_way.qc, bot_ed.qc, 37 waypoint files. Net -3,400 lines
- **Self-contained build** — NexQuake + Recast/Detour as git submodules, CI workflow for ARM64

## 0.1.0 — Initial Release

FrikBotNex: QuakeC bot AI for Quake 1 deathmatch, evolved from FrikBot X++ (v0.10.2).

- **Human emulation** — aim flick-and-settle, tracking drift, bhop fumbles, reaction delay, weapon commitment, navigation jitter. All scale with skill level
- **Spatial awareness** — unified `bot_sense` pass: ground state, forward obstacles, enemy visibility, friendly fire. Computed once per frame, read everywhere
- **Item goal system** — utility-scored item selection with resource pools, respawn anticipation, loadout priorities, death zone avoidance, personality-weighted preferences
- **Personality system** — 4 types (balanced, aggressive, defensive, camper) affecting weapon choice, retreat threshold, movement style, ambush patience
- **Combat AI** — 4-state utility machine (fight/retreat/pressure/flank), opponent modeling, team coordination, RL→LG combo
- **Engine integration** — bot hooks in world.qc and client.qc for BotInit/BotFrame/BotPreFrame/BotPostFrame
