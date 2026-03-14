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

## Building

Compile with any QuakeC compiler from `src/qc/`:

```sh
cd src/qc
fteqcc
```

Or use `gmqcc progs.src` or `qcc`. Output is `src/progs.dat`. A pre-compiled `progs.dat` is checked into the repo root for convenience.

There are no automated tests -- testing requires running in a Quake engine (Quakespasm, vkQuake, FTE, etc.).

## Source Layout

```
src/
  qc/           Base Quake game code (GPL v2, id Software)
    defs.qc       Global declarations (must compile first)
    progs.src     Compilation manifest
    ...           Standard game modules (weapons, items, monsters, etc.)

  frikbot/      Bot AI system (public domain, Ryan "FrikaC" Smith + contributors)
    bot.qc        Entity lifecycle, declarations, main loop hooks
    bot_way.qc    Waypoint navigation and pathfinding
    bot_think.qc  Opponent model, team coordination, utility scoring, threat assessment
    bot_shoot.qc  Weapon scoring, switching, fire control
    bot_fight.qc  Combat movement states and state machine
    bot_ai.qc     Target selection, aim pipeline, main per-frame think
    bot_misc.qc   Utility functions
    bot_phys.qc   Physics prediction and movement validation
    bot_move.qc   Movement command generation, bunny hopping
    bot_sense.qc  Unified spatial awareness pass (per-tick sensing)
    bot_ed.qc     In-game console/editor interface

  waypoints/    Per-map navigation graphs
    map_dm1.qc through map_dm6.qc
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
