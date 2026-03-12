# Bot Spatial Awareness System — Conceptual & Implementation Plan

## Goal

Replace FrikBotNex's scattered, redundant traceline calls with a unified **spatial awareness pass** (`bot_sense`) that runs once per bot per AI tick. This consolidation:

1. Eliminates redundant traces across `bot_move.qc`, `bot_phys.qc`, `bot_misc.qc`, `bot_ai.qc`, `bot_fight.qc`, and `bot_think.qc`
2. Creates a shared spatial model that all subsystems read from
3. Provides the foundation for **intra-region local navigation** — the ability for the bot to move intelligently within a map region using only traceline-based spatial reasoning, without dense waypoint breadcrumbs
4. Produces a spatial representation that could also feed an external ML navigation system

---

## Background: Current Traceline Usage

### Inventory of All 29 Existing Tracelines

#### bot_move.qc — Obstacle & Hazard Detection (12 traces)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 84 | `frik_recognize_plat()` | Down 64u from origin | Detect platform entity below | Every frame + dynamic waypoint |
| 246 | `frik_check_hazard()` | Down 64u from 48u ahead (center) | Detect lava/slime ahead | Every frame on ground |
| 263 | `frik_check_hazard()` | Down 64u from 48u ahead + 24u left | Detect lava/slime left-ahead | Every frame on ground |
| 276 | `frik_check_hazard()` | Down 64u from 48u ahead - 24u right | Detect lava/slime right-ahead | Every frame on ground |
| 310 | `frik_obstacles()` | Down 256u from 32u ahead at maxs_z | Measure obstacle/gap height | Every frame on ground |
| 326 | `frik_obstacles()` | Forward 256u from 8u below origin | Measure gap distance | When gap detected |
| 333 | `frik_obstacles()` | Left-right 20u sweep | Check gap width | When gap detected |
| 362 | `frik_obstacles()` | Forward (test+20)u | Check for ledge blocking path | When jump candidate |
| 714 | `frik_bot_roam()` | Random forward 2300u | Find wall for roam waypoint | Rare (roaming only) |
| 720 | `frik_bot_roam()` | Along wall normal 2300u | Follow wall for placement | Rare |
| 724 | `frik_bot_roam()` | From candidate back to self | Verify waypoint visibility | Rare |
| 729 | `frik_bot_roam()` | Down 48u from candidate | Verify ground at waypoint | Rare |

#### bot_phys.qc — Physics Simulation (1 trace)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 156 | `SV_UserFriction()` | Down 34u from leading edge | Edge friction detection | Every physics frame |

#### bot_ai.qc — Pathfinding & Aim (2 traces)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 585 | `bot_checkroute()` | Between waypoint origins | Detect doors/func_wall blocking path | Per waypoint transition |
| 1091 | `BotAim()` | From enemy to predicted impact point | Grenade bounce simulation | When using GL |

#### bot_way.qc — Waypoint Visibility (4 traces)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 800-818 | `WaypointThink()` | Self to target1/2/3/4 origins | Test waypoint LOS for pathfinding | When AI_TRACE_TEST flag set |

#### bot_misc.qc — Visibility Functions (8 traces)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 591 | `fisible()` | View to target origin | BSP entity visibility | Constant (enemy/item checks) |
| 615 | `fisible()` | View to target+mins | Water body visibility | When target in water |
| 632 | `fisible()` | View to target+maxs | Cross-water visibility | When in different medium |
| 644 | `fisible()` | View to target origin | Primary LOS | Every visibility check |
| 649 | `fisible()` | View to target+maxs | Top corner check | After primary fails |
| 654 | `fisible()` | View to target+mins | Bottom corner check | After top fails |
| 686 | `wisible()` | Between two entity origins | Visibility through movers | Dynamic waypoint linking |
| 710 | `sisible()` | Self to target origin | Simple direct LOS | Waypoint searches |

#### bot_fight.qc — Combat (1 trace)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 448 | `bot_fight_style()` | Forward along aim vector | Friendly fire check | Combat frames |

#### bot_think.qc — Tactical Analysis (1 trace)

| Line | Function | Direction | Purpose | Frequency |
|------|----------|-----------|---------|-----------|
| 749 | `bot_enemy_near_wall()` | Forward 100u from enemy | Wall proximity for splash damage | Combat frames (cached) |

### Redundancy Analysis

These traces share significant overlap:

- **Downward traces**: `frik_recognize_plat()`, `frik_check_hazard()` (×3), `frik_obstacles()`, and `SV_UserFriction()` all trace downward from near the bot's origin. A single downward probe at the bot's position plus 2-3 ahead/lateral probes could replace all six.
- **Forward probes**: `frik_obstacles()` traces forward multiple times to measure gaps, heights, and ledges. These are sequential refinements of the same question: "what's ahead of me?"
- **Visibility**: `fisible()` can burn up to 3 traces for one target. Combat LOS (`bot_fight_style`) and enemy wall proximity (`bot_enemy_near_wall`) are further traces in the same direction. A cached per-frame visibility result eliminates re-tracing.

---

## Architecture: The `bot_sense` System

### Design Principles

1. **Compute once, read everywhere.** The sense pass populates entity fields. All subsystems read fields instead of tracing.
2. **Amortize across frames.** Not everything needs updating every frame. Ground state changes fast; lateral clearance can update every 2-3 frames.
3. **Entity fields as the "struct."** QuakeC has no structs or arrays. All spatial awareness data lives as `.sense_*` fields on the bot entity, following the existing pattern (`.b_wall_near`, `.b_strafe_dir`, etc.).
4. **Graceful degradation.** Subsystems that currently trace can fall back to direct tracing if sense data is stale. This allows incremental migration.

### New File: `src/frikbot/bot_sense.qc`

This file contains:
- Entity field declarations for spatial awareness state
- `bot_sense()` — the main per-tick sensing function
- `bot_sense_ground()` — ground state probing
- `bot_sense_forward()` — forward obstacle/gap analysis
- `bot_sense_lateral()` — left/right clearance (new capability)
- `bot_sense_visibility()` — target and enemy LOS caching
- `bot_sense_local_nav()` — directional fan traces for intra-region movement (new capability)

### Entity Fields

```c
// --- Sense system metadata ---
.float sense_time;              // last full update time (avoid redundant runs)
.float sense_ground_time;       // last ground probe time
.float sense_lateral_time;      // last lateral probe time
.float sense_nav_time;          // last local nav probe time

// --- Ground state ---
// Replaces: frik_recognize_plat() trace, frik_check_hazard() center trace,
//           SV_UserFriction() edge trace
.float sense_ground_dist;       // distance to floor directly below (0-64)
.float sense_ground_type;       // CONTENT_SOLID, CONTENT_LAVA, CONTENT_SLIME, CONTENT_WATER
.entity sense_ground_ent;       // entity below (platform/train) or world
.float sense_on_edge;           // true if leading edge is over a dropoff (edge friction)

// --- Forward probe ---
// Replaces: frik_obstacles() traces (lines 310, 326, 333, 362),
//           frik_check_hazard() center ahead trace
.float sense_fwd_wall_dist;     // distance to wall/obstacle ahead (0-256)
.float sense_fwd_height;        // height differential of ground ahead vs current (-256 to +256)
.float sense_fwd_ground_type;   // ground type ahead (solid/lava/slime/void)
.float sense_fwd_gap_width;     // gap width ahead (0 = no gap)
.float sense_fwd_jumpable;      // TRUE if gap/step is jumpable
.vector sense_fwd_normal;       // wall surface normal (for deflection/wall-following)

// --- Hazard probes (ahead + lateral) ---
// Replaces: frik_check_hazard() left/right traces
.float sense_hazard_left;       // TRUE if lava/slime to the left-ahead
.float sense_hazard_right;      // TRUE if lava/slime to the right-ahead

// --- Lateral clearance (NEW — enables local navigation) ---
.float sense_left_dist;         // distance to wall on left (0-256)
.float sense_right_dist;        // distance to wall on right (0-256)
.vector sense_left_normal;      // left wall normal
.vector sense_right_normal;     // right wall normal

// --- Target/enemy visibility cache ---
// Replaces: fisible() multi-trace calls, bot_fight_style() LOS check,
//           bot_enemy_near_wall() trace
.float sense_target_vis;        // TRUE if target1 is visible
.float sense_enemy_vis;         // TRUE if enemy is visible
.float sense_enemy_dist;        // distance to enemy
.float sense_enemy_wall;        // TRUE if enemy has wall behind them (splash potential)
.float sense_friendly_block;    // TRUE if teammate in line of fire

// --- Local navigation fan (NEW — enables waypoint-free intra-region movement) ---
// 8-direction clearance distances at 45-degree intervals
// Indexed by compass direction relative to current velocity/facing
.float sense_nav_n;             // forward clearance
.float sense_nav_ne;            // forward-right clearance
.float sense_nav_e;             // right clearance
.float sense_nav_se;            // back-right clearance
.float sense_nav_s;             // back clearance
.float sense_nav_sw;            // back-left clearance
.float sense_nav_w;             // left clearance
.float sense_nav_nw;            // forward-left clearance
.float sense_nav_best_dir;      // computed best direction toward goal (yaw angle)
.float sense_nav_openness;      // 0.0 = tight corridor, 1.0 = open room
```

Note: QuakeC has no arrays. The 8 directional fields (`sense_nav_n` through `sense_nav_nw`) are the pragmatic substitute. This is ugly but it's the only option.

### Function Specifications

#### `bot_sense()` — Main Entry Point

Called once per bot per AI tick from `BotAI()`, before any decision-making.

```
void() bot_sense =
{
    if (self.sense_time == time) return;     // already ran this frame
    self.sense_time = time;

    bot_sense_ground();                      // every frame
    bot_sense_forward();                     // every frame

    if (time > self.sense_lateral_time)      // every 2nd frame
    {
        bot_sense_lateral();
        self.sense_lateral_time = time + 0.028;  // ~2 frames at 72fps
    }

    bot_sense_visibility();                  // every frame (combat-critical)

    if (time > self.sense_nav_time)          // every 3rd frame
    {
        bot_sense_local_nav();
        self.sense_nav_time = time + 0.042;  // ~3 frames at 72fps
    }
};
```

#### `bot_sense_ground()` — Ground State

**Traces: 2** (replaces 5-6 scattered traces)

1. Down 64u from origin → `sense_ground_dist`, `sense_ground_type`, `sense_ground_ent`
2. Down 34u from leading velocity edge → `sense_on_edge`

The platform detection (`frik_recognize_plat`) comes for free from trace 1: if `sense_ground_ent != world`, there's a platform below.

Edge friction detection (`SV_UserFriction`) comes from trace 2: if `trace_fraction == 1`, we're on an edge.

#### `bot_sense_forward()` — Forward Obstacle Analysis

**Traces: 3-5** (replaces 6-8 scattered traces)

1. Forward 32u at maxs_z, then down 256u → `sense_fwd_height`, initial gap detection
2. Forward 256u at feet level → `sense_fwd_wall_dist`, `sense_fwd_normal`, `sense_fwd_gap_width`
3. Down from 48u ahead (center) → `sense_fwd_ground_type`
4. Down from 48u ahead + 24u left → `sense_hazard_left` (conditional: only on ground)
5. Down from 48u ahead - 24u right → `sense_hazard_right` (conditional: only on ground)

Traces 4-5 can be skipped when airborne. The gap jumpability calculation (`sense_fwd_jumpable`) is pure math from `sense_fwd_gap_width` and current velocity — no extra trace needed.

#### `bot_sense_lateral()` — Left/Right Clearance

**Traces: 2** (new capability)

1. Left 256u perpendicular to velocity → `sense_left_dist`, `sense_left_normal`
2. Right 256u perpendicular to velocity → `sense_right_dist`, `sense_right_normal`

This gives corridor awareness. A bot in a narrow hallway sees `sense_left_dist=40, sense_right_dist=40`. In an open room, both are 256. This directly feeds local navigation decisions and dodge maneuvering.

#### `bot_sense_visibility()` — Target & Enemy LOS

**Traces: 2-4** (replaces 6-10 scattered traces)

1. View origin to `self.target1` origin → `sense_target_vis`
2. View origin to `self.enemy` origin → `sense_enemy_vis`
3. Forward along aim × weapon range → `sense_friendly_block` (only in combat)
4. Forward 100u from enemy position → `sense_enemy_wall` (only in combat)

Key optimization: The current `fisible()` traces up to 3 times per call (origin, +maxs, +mins). For the sense pass, we trace once to center. If critical (combat), we do the corner check. Most callers only need the boolean anyway.

Subsystems that need the full multi-corner `fisible()` check (e.g. item scanning during `bot_look_for_crap`) can still call `fisible()` directly — the sense cache covers the hot path (enemy and current navigation target).

#### `bot_sense_local_nav()` — Directional Fan Probes

**Traces: 8** (new capability — foundation for intra-region navigation)

Cast 8 rays at 45-degree intervals from the bot's origin, 256 units each, in the horizontal plane. Store distance-to-wall in `sense_nav_n` through `sense_nav_nw`.

From these 8 distances, compute:
- `sense_nav_openness`: average of all 8 distances, normalized to 0-1. Corridors score low, rooms score high.
- `sense_nav_best_dir`: given a goal position, find the fan direction that is both (a) closest to the goal direction and (b) has sufficient clearance. This is a simple weighted score across the 8 directions.

The fan traces use the **world frame** (N=+Y, E=+X, etc.), not relative to facing. This keeps them stable across frames even as the bot turns, and makes the openness calculation consistent.

---

## Trace Budget Comparison

### Before: Scattered System

| Subsystem | Traces/frame/bot | Notes |
|-----------|-----------------|-------|
| Ground/platform | 1 | `frik_recognize_plat` |
| Hazard check | 3 | `frik_check_hazard` ×3 |
| Obstacle detection | 2-4 | `frik_obstacles` (variable) |
| Edge friction | 1 | `SV_UserFriction` |
| Visibility (fisible) | 3-6 | Up to 3 per target, called multiple times |
| Combat LOS | 1-2 | `bot_fight_style` + `bot_enemy_near_wall` |
| Waypoint LOS | 0-4 | `WaypointThink` (when AI_TRACE_TEST set) |
| Path blockage | 0-1 | `bot_checkroute` |
| **Total** | **~12-22** | **Redundant, uncoordinated** |

### After: Unified Sense Pass

| Sense function | Traces/tick | Update rate | Effective traces/frame |
|---------------|-------------|-------------|----------------------|
| `bot_sense_ground` | 2 | every frame | 2.0 |
| `bot_sense_forward` | 3-5 | every frame | 3-5 |
| `bot_sense_lateral` | 2 | every 2nd frame | 1.0 |
| `bot_sense_visibility` | 2-4 | every frame | 2-4 |
| `bot_sense_local_nav` | 8 | every 3rd frame | 2.7 |
| **Total** | **17-21** | **amortized** | **~11-15** |

Net result: **slightly fewer traces per frame** while gaining two entirely new capabilities (lateral clearance + 8-direction nav fan). With 16 bots at 72fps, worst case is ~17,000 traces/sec — well under 10ms/sec of CPU time on modern hardware.

---

## Integration: Replacing Existing Callers

### Phase 1 — Drop-in field reads (low risk)

These replacements are mechanical — swap a traceline call for a field read.

#### `frik_recognize_plat()` → `sense_ground_ent`
```c
// Before:
traceline(self.origin, self.origin - '0 0 64', TRUE, self);
if (trace_ent != world) { ... }

// After:
if (self.sense_ground_ent != world) { ... }
```

#### `frik_check_hazard()` → `sense_fwd_ground_type`, `sense_hazard_left`, `sense_hazard_right`
```c
// Before: 3 traces (center, left, right) checking pointcontents
// After:
if (self.sense_fwd_ground_type < -3) { frik_obstructed(ang, TRUE); return TRUE; }
if (self.sense_hazard_left) { frik_obstructed(ang + side, TRUE); return TRUE; }
if (self.sense_hazard_right) { frik_obstructed(ang - side, TRUE); return TRUE; }
```

#### `SV_UserFriction()` → `sense_on_edge`
```c
// Before:
traceline(start, stop, TRUE, self);
if (trace_fraction == 1) friction = sv_friction * 2;

// After:
if (self.sense_on_edge) friction = sv_friction * 2;
```

**Note on `SV_UserFriction()`**: This function runs on the `phys_obj` entity (the physics prediction shadow), not on the bot entity itself. The sense pass runs on the bot. Two options:
- Copy `sense_on_edge` to `self.phys_obj.sense_on_edge` at sense time
- Keep the direct traceline in `SV_UserFriction` since it runs on the phys entity with a different origin

Recommendation: Keep the direct trace in `SV_UserFriction`. It's one trace, it runs at the phys_obj's position (not the bot's), and coupling it to the sense system would complicate the phys prediction loop. Mark it as a known exception.

#### `frik_obstacles()` → `sense_fwd_*` fields
```c
// Before: Complex multi-trace obstacle/gap detection
// After: Read sense_fwd_height, sense_fwd_gap_width, sense_fwd_jumpable
if (self.sense_fwd_height > 18) { bot_jump(); return; }
if (self.sense_fwd_height >= 0) return;
if (self.sense_fwd_gap_width < 20) return;  // walkable gap
// ... etc, translating the existing logic to field reads
```

#### `bot_enemy_near_wall()` → `sense_enemy_wall`
```c
// Before: traceline from enemy forward, cached per-frame
// After: already cached in sense pass
return self.sense_enemy_wall;
```

### Phase 2 — Visibility consolidation (moderate risk)

#### Hot-path `fisible()` calls → `sense_target_vis`, `sense_enemy_vis`

Most `fisible()` calls are checking the current enemy or navigation target. These are cached in the sense pass. Replace the hot-path calls:

```c
// In bot_ai.qc, combat checks:
// Before: if (fisible(self.enemy)) { ... }
// After:  if (self.sense_enemy_vis) { ... }

// In navigation, checking target1:
// Before: if (fisible(self.target1)) { ... }
// After:  if (self.sense_target_vis) { ... }
```

**Keep `fisible()` as-is** for non-cached targets (item scanning in `bot_look_for_crap`, etc.). The sense pass only caches the two most-checked entities. Other visibility checks are infrequent enough that direct tracing is fine.

#### `bot_fight_style()` friendly fire check → `sense_friendly_block`
```c
// Before: traceline along aim vector, check trace_ent
// After:  if (self.sense_friendly_block) { /* don't shoot */ }
```

### Phase 3 — Waypoint + pathfinding traces (keep separate)

These traces should **not** be absorbed into the sense pass:

- **`WaypointThink()` lines 800-818**: These trace between *waypoint* entities, not from the bot. They run during the global pathfinding calculation, not per-bot. Leave them alone.
- **`bot_checkroute()` line 585**: Traces between waypoints to detect blockage. Runs per waypoint transition, not per frame. Leave as-is.
- **`BotAim()` grenade bounce (line 1091)**: Specialized physics simulation. Not spatial awareness. Leave as-is.
- **`frik_bot_roam()` traces (lines 714-729)**: Roaming/dynamic waypoint generation. Runs rarely, uses unique trace patterns. Leave as-is.

### Phase 4 — Local navigation integration (new capability)

Once the sense system is in place, `frik_movetogoal()` can be extended:

```c
void() frik_movetogoal =
{
    // ... existing target direction calculation ...

    // NEW: If no immediate waypoint but we have a goal direction,
    // use the nav fan to find the best traversable direction
    if (!self.target1 || (vlen(self.origin - self.target1.origin) > 512))
    {
        // target is far or nonexistent — use spatial awareness
        local float best_yaw;
        best_yaw = self.sense_nav_best_dir;
        if (best_yaw >= 0)
        {
            // navigate by spatial awareness instead of waypoint chain
            local vector dir;
            makevectors('0 1 0' * best_yaw);
            dir = v_forward;
            self.keys = frik_KeysForDir(dir);
            return;
        }
    }

    // ... fall through to existing waypoint-based navigation ...
};
```

This is the bridge to the regional navigation paradigm: gateway waypoints get the bot to the right region, then `sense_nav_best_dir` guides it toward items/exits/enemies within the region using only spatial probing.

---

## Implementation Order

### Step 1: Create `bot_sense.qc` with field declarations and stub functions

Add file to `progs.src` after `bot.qc` (field declarations must come before usage):

```
../frikbot/bot.qc
../frikbot/bot_sense.qc    <-- NEW
../frikbot/bot_way.qc
...
```

Move new `.sense_*` field declarations into `bot_sense.qc`. Implement `bot_sense()` calling all sub-functions. Initially, the sub-functions just do their traces and populate fields — no callers are changed yet.

### Step 2: Wire `bot_sense()` into the bot loop

In `BotAI()` (bot_ai.qc), add `bot_sense()` call at the top, before any decision-making. Verify in-engine that the sense fields are being populated correctly (use `dprint()` or the editor overlay).

### Step 3: Migrate ground/obstacle callers (Phase 1)

Replace `frik_recognize_plat()` internals, `frik_check_hazard()`, and the `frik_obstacles()` trace cascade with reads from `sense_*` fields. Test extensively — obstacle avoidance is the most breakage-prone area.

### Step 4: Migrate visibility callers (Phase 2)

Replace hot-path `fisible()` calls with `sense_enemy_vis` / `sense_target_vis` reads. Keep `fisible()` for non-cached targets.

### Step 5: Implement and tune local navigation fan (Phase 4)

Add the 8-direction fan traces. Implement `sense_nav_best_dir` scoring. Extend `frik_movetogoal()` with the local-nav fallback. Test on maps with sparse waypoints (manually delete intra-region waypoints from a dm3/dm6 file and see if the bot can still navigate).

### Step 6: Iterative waypoint thinning

With local nav working, progressively remove intra-region waypoints:
1. Remove straight-corridor waypoints first (bot can walk a straight line)
2. Remove mid-room waypoints (bot can navigate open spaces)
3. Keep corner/doorway/elevation waypoints (these encode non-obvious geometry)
4. Keep all inter-region gateway waypoints

Test after each removal pass. The goal is to find the minimum waypoint density where the bot still navigates reliably.

---

## Risks and Mitigations

### Timing sensitivity
The sense pass runs at the start of the AI tick. If the bot moves significantly during the frame (bunny-hopping at 500+ u/s), sense data could be stale by the time movement code reads it. **Mitigation**: Ground and forward probes run every frame. At 72fps, even at 500 u/s the bot moves ~7 units/frame — well within the 32-64 unit probe distances.

### `SV_UserFriction` runs on `phys_obj`
The physics prediction entity has a different origin than the bot. The sense pass traces from the bot's position. **Mitigation**: Keep the direct traceline in `SV_UserFriction`. It's one trace and the position difference matters. Documented as a known exception above.

### `fisible()` corner checks matter for small targets
The sense pass only traces to target center. Small items behind partial cover might be missed. **Mitigation**: Keep `fisible()` for item scanning (`bot_look_for_crap`). The sense cache is only for the current enemy and navigation target, which are typically player-sized.

### Local nav getting stuck in concave geometry
The 8-direction fan can't see around U-shaped dead ends — all directions show walls, but there's an exit behind the bot. **Mitigation**: Stuck detection + fallback to waypoint-based navigation. The bot already has obstruction detection (`AI_OBSTRUCTED`). If stuck time exceeds a threshold, abandon local nav and find the nearest waypoint.

### QuakeC entity field limits
The Quake engine has a hard limit on entity field count (roughly 256 fields in vanilla engines, more in modern source ports). We're adding ~30 fields. **Mitigation**: Check the current field count in `defs.qc` + bot fields. Modern engines (FTE, QuakeSpasm-Spiked, vkQuake) support extended field counts. Document the minimum engine requirement.

---

## Relation to External ML Navigation System

The sense system produces a spatial model that maps cleanly to an ML observation space:

| Sense field(s) | ML observation |
|----------------|---------------|
| `sense_nav_n` through `sense_nav_nw` | 8-dimensional distance vector (local geometry) |
| `sense_nav_openness` | Scalar room/corridor classification |
| `sense_ground_type`, `sense_fwd_ground_type` | Hazard flags |
| `sense_enemy_vis`, `sense_enemy_dist` | Threat detection |
| `sense_target_vis` | Goal visibility |
| `sense_left_dist`, `sense_right_dist` | Corridor width |

If the ML model outputs a desired movement direction (yaw angle), the QuakeC bot can execute it directly through `frik_KeysForDir()`. The sense fields are the shared vocabulary between the two systems — the ML model would learn to interpret the same spatial representation that the bot's hand-coded logic reads.

For the regional navigation paradigm specifically:
- **Inter-region routing**: ML model (or existing pathfinder) picks which region to go to
- **Intra-region movement**: `sense_nav_best_dir` scoring (hand-coded) or ML policy reading `sense_nav_*` fields
- **Shared region definitions**: Both systems reference the same region boundary waypoints

---

## File Summary

| File | Action |
|------|--------|
| `src/frikbot/bot_sense.qc` | **NEW** — All sensing logic and field declarations |
| `src/qc/progs.src` | Add `../frikbot/bot_sense.qc` after `bot.qc` |
| `src/frikbot/bot.qc` | Remove field declarations that move to `bot_sense.qc` (if any) |
| `src/frikbot/bot_ai.qc` | Add `bot_sense()` call in `BotAI()`. Replace `fisible()` hot-path calls |
| `src/frikbot/bot_move.qc` | Replace traces in `frik_obstacles()`, `frik_check_hazard()`, `frik_recognize_plat()` with field reads. Extend `frik_movetogoal()` with local nav |
| `src/frikbot/bot_phys.qc` | `SV_UserFriction()` — keep direct trace (documented exception) |
| `src/frikbot/bot_misc.qc` | Keep `fisible()`/`sisible()`/`wisible()` for non-cached callers |
| `src/frikbot/bot_fight.qc` | Replace friendly-fire trace with `sense_friendly_block` read |
| `src/frikbot/bot_think.qc` | Replace `bot_enemy_near_wall()` trace with `sense_enemy_wall` read |
