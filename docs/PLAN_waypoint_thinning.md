# Waypoint Thinning via Spatial Navigation

## Problem

DM maps average ~75 waypoints, many of which are corridor breadcrumbs — 3-5
waypoints in a straight line just to keep the bot walking forward. Each waypoint
is an entity that `FindWayPoint()` scans with O(n) brute force. Dense chains
also make bot movement robotic: they walk point-to-point with no local
flexibility.

The sense system (`bot_sense.qc`) already computes 8-direction clearance,
best-dir scoring, and openness per tick. This data can replace intermediate
waypoints.

## Goal

Remove 30-50% of waypoints from DM maps by letting the sense nav fan guide bots
through open/corridor sections. Keep waypoints only at decision points.

## Waypoints That Must Stay

- Junctions (2+ outbound links to distinct areas)
- Elevation changes (stairs, drops, jumps — AI_JUMP, AI_SUPER_JUMP)
- Teleporters and telelink endpoints (AI_TELELINK_*)
- Doors and narrow passages (AI_DOORFLAG, AI_PRECISION)
- Platforms and elevators (AI_RIDE_TRAIN)
- Item spawns (mega health, RL, LG, armor)
- Sniper/ambush points (AI_SNIPER, AI_AMBUSH)
- Any waypoint with behavioral AI flags

## Waypoints That Can Be Removed

- Straight corridor waypoints where adjacent waypoints have LOS to each other
- Open room breadcrumbs (openness > 0.5 in the region)
- Redundant fan-out points near items (if the item waypoint is reachable from
  the previous junction)

## Implementation Phases

### Phase 1: Adaptive waypoint skipping in bot_path()

**No map data changes.** Purely runtime: when the bot reaches a waypoint,
evaluate whether the *next* waypoint can be skipped.

In `bot_path()` (bot_ai.qc ~line 542), after `FindRoute()` returns the next
waypoint:

```
// candidate: the waypoint after next
skip = FindRoute(next_way);
if (skip != world && can_skip_waypoint(next_way, skip))
    // target skip instead of next_way
```

`can_skip_waypoint(from, to)` checks:
1. `from` has no behavioral AI flags (AI_POINT_TYPES == 0)
2. `from` has exactly 1-2 outbound links (not a junction)
3. Bot can see `to` (traceline from self.origin to to.origin)
4. `sense_openness > 0.4` (not a tight squeeze)
5. Distance to `to` < 512 units (don't skip over huge gaps)

If all pass, skip the intermediate waypoint. The sense nav fan handles local
steering via the existing `frik_movetogoal()` override.

**Risk:** Low. Falls back to normal waypoint-by-waypoint if any check fails.
Can be gated behind `b_skill >= 1` initially.

### Phase 2: Extend sense nav to non-obstructed movement

Currently `frik_movetogoal()` only uses `sense_nav_best_dir` when
`AI_OBSTRUCTED`. For thinned paths with larger gaps between waypoints, enable
it proactively:

```
// in frik_movetogoal(), after computing way direction
if (sense_openness > 0.5 && sense_nav_best_dir >= 0)
{
    // blend waypoint direction with sense nav for smoother pathing
    // weight: 70% waypoint goal, 30% sense best-dir
}
```

This smooths movement between distant waypoints without requiring dense chains.

### Phase 3: Increase reach thresholds for flagless waypoints

`bot_check_lost()` currently uses 32 units. For waypoints with no AI flags in
a high-openness region, increase to 48-64 units. Bots don't need to hit the
exact spot when the next waypoint is visible and the area is open.

### Phase 4: Thin map data files

After phases 1-3 are tested and stable, manually thin `src/waypoints/map_dm*.qc`:

For each map:
1. Identify corridor chains (sequential waypoints with 1-2 links, no flags)
2. Remove every other waypoint in straight-line chains
3. Relink remaining waypoints
4. Test in-engine: bot must still reach all items/areas

Start with dm3 (relatively open) as proof of concept, then dm2 and dm6.

**Estimated reduction per map:**
- dm1: ~10 waypoints removable (short corridors)
- dm2: ~25 waypoints removable (many corridor chains)
- dm3: ~20 waypoints removable (open areas)
- dm4: ~15 waypoints removable
- dm5: ~10 waypoints removable (vertical, harder to thin)
- dm6: ~20 waypoints removable (long hallways)

### Phase 5: FindWayPoint() optimization (optional)

If waypoint count drops significantly, the O(n) scan becomes less of an issue.
But if we want to go further: bucket waypoints into a coarse spatial grid
(256-unit cells) so `FindWayPoint()` only scans nearby buckets. This is
independent of thinning but benefits from it.

## Metrics

- Waypoint count per map (before/after)
- Bot navigation success rate (reaches all major items in a timed run)
- `FindWayPoint()` call frequency and entity scan count
- Qualitative: does movement look natural or does the bot get stuck?

## Risks

- **Tight corners:** Bot might cut corners and get stuck on geometry. Mitigated
  by the openness threshold (< 0.4 = don't skip).
- **Platform timing:** Skipping a waypoint near a platform could break elevator
  logic. Mitigated by preserving all AI_RIDE_TRAIN waypoints.
- **Telelinks:** Must never skip telelink waypoints. Check is explicit.
- **Regression in combat:** Bot navigating with sense during combat might
  behave differently. Phase 1 can be disabled when `self.enemy` is set.
