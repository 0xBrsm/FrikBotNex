# FrikBotNex

A QuakeC bot AI for Quake 1 deathmatch, evolved from FrikBot X (v0.10.2) by Ryan "FrikaC" Smith. The focus is on **human emulation** -- bots that look and feel like real players, not perfect aim-machines.

## What's Different

FrikBotNex builds on the original FrikBot with:

- **Combat state machine** -- utility-scored decisions (fight, retreat, pressure, flank) replace binary fight-or-run logic. Skill 2+ bots evaluate situational scores with hysteresis to avoid jitter.
- **Opponent modeling** -- skill 2+ bots track enemy weapon, aggression, strafe tendency, and preferred distance using exponential moving averages. Used for counter-strafing, weapon counter-picks, and retreat direction.
- **Human emulation** across three axes:
  - *Aim*: flick-then-settle on target acquisition, low-frequency tracking drift via sine wave, overcorrection modeling with anti-correlated jitter
  - *Movement*: navigation path jitter, bunny hop fumbles, skill-scaled strafe timing
  - *Decisions*: reaction delay before engaging new enemies, weapon commitment locks, RL->LG combo with human-plausible success rate (40%)
- **Team coordination** -- bots read ally entity state to focus fire, rush enemies chasing retreating teammates, and bias retreat toward allies for crossfire
- **Personality system** -- balanced, aggressive, defensive, and camper archetypes that influence weapon preference, strafe speed, retreat threshold, and utility scoring
- **Skill scaling** -- skill 0 bots panic and fumble; skill 3 bots counter-strafe, combo weapons, and seek health while retreating. Each skill level unlocks features rather than just tightening aim.
- **Navmesh navigation** -- a Recast/Detour navmesh is built at map load from the BSP's hull-1 collision geometry, so bots navigate any map with no pre-authored waypoints. Off-mesh links cover everything the raw mesh can't express: jumps, drops, rocket jumps, teleporters, platforms, trains, and doors. A connectivity harness validates every item and spawn is reachable on every stock map.

## Building

Compile with any QuakeC compiler from `src/qc/`:

```sh
cd src/qc
fteqcc
```

Or use `gmqcc progs.src` or `qcc`. Output goes to the repo root: `progs.dat` (game bytecode). A pre-compiled `progs.dat` is checked in for convenience.

Bot navigation runs in a custom engine build (`src/server/`, NetQuake + Recast/Detour as git
submodules) rather than a generic Quake client/server -- see `src/build/build-server.sh`.
Rebuild it whenever `src/server/*.cpp`/`*.h`/`net_bot.c` change; QC-only changes don't need it.

There are no automated tests -- testing requires running bots in this engine build.

## Source Layout

```
src/
  qc/           Base Quake game code (GPL v2, id Software)
    defs.qc       Global declarations (must compile first)
    progs.src     Compilation manifest
    ...           Standard game modules (weapons, items, monsters, etc.)

  frikbot/      Bot AI system (public domain, Ryan "FrikaC" Smith + contributors)
    bot.qc        Entity lifecycle, declarations, main loop hooks
    bot_ai.qc     Target/enemy selection, goal management, aim pipeline, per-frame think
    bot_fight.qc  Combat movement state machine, opponent modeling
    bot_goal.qc   Item goal scoring: utility-weighted pickups, resource pools, personality
    bot_move.qc   Movement command generation, bunny hopping, off-mesh link traversal
    bot_misc.qc   Utility functions

  server/       Navmesh navigation (C++, built with the engine, not the QC progs)
    nav_hull.cpp  Extracts hull-1 BSP collision geometry
    nav_mesh.cpp  Recast/Detour navmesh build
    nav_bot.cpp   Off-mesh link detection + QC-facing path/link builtins
    net_bot.c     Fake network driver -- bots run through real engine physics

  tools/        nav_harness.sh + nav_triage.py -- full-map connectivity test gate
```

## Installation

To integrate into a QuakeC mod, add the bot files to `progs.src` after `defs.qc`, then hook into the game loop at four points:

1. `BotInit()` in `WorldSpawn()` -- spawns physics entities
2. `BotFrame()` in `StartFrame()` -- per-frame AI processing
3. `BotPreFrame()` in `PlayerPreThink()` -- early bot checks
4. `BotPostFrame()` in `PlayerPostThink()` -- late-stage updates

See the detailed instructions at the top of `src/frikbot/bot.qc`.

## Skill Levels

| Skill | Aim | Movement | Decisions |
|-------|-----|----------|-----------|
| 0 | High error, slow tracking, 0.3-0.7s reaction delay | Sluggish strafe, frequent bhop fumbles, nav jitter | Panic retreat, weapon commitment 2.5-4.5s, no weapon switching in combat |
| 1 | Moderate error, flick overshoot, 0.15-0.4s reaction | Faster strafe, occasional fumbles | Binary fight/retreat, team retreat bias, weapon switching |
| 2 | Lower error, tracking drift, 0.05-0.15s reaction | Quick strafe with fakes, angular retreat | Full utility state machine (fight/retreat/pressure/flank), opponent model |
| 3 | Minimal error, instant reaction | Counter-strafe from opponent model, no fumbles | RL->LG combos, health-seeking retreat, counter-picks |

## License

Base game code: GPL v2 (id Software). See `src/qc/LICENSE.txt`.
FrikBot code: Public domain (Ryan "FrikaC" Smith).
FrikBotNex additions: Public domain.

See [ATTRIBUTIONS](ATTRIBUTIONS.md) for full credits.
