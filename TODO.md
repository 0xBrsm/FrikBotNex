# FrikBotNex TODO

## Combat AI
- [x] Situational weapon selection (ammo, enemy health, splash near walls)
- [x] Strafe/dodge patterns during fights
- [x] Retreat-when-weak behavior
- [x] Ambush positioning
- [x] Human-like aim variance (interpolated tracking styles per skill level)

## Movement
- [x] Bunny hopping
- [x] Strafe jumping
- [x] Rocket jumping (via AI_SUPER_JUMP waypoints)
- [x] Better obstacle/ledge detection
- [x] Hazard avoidance (lava, slime)
- [x] Elevator/platform timing

## Strategic Awareness
- [x] Item respawn timers (mega health, red armor, quad)
- [x] Map control / positional advantage
- [x] Chokepoint and high-ground awareness
- [x] Smarter item prioritization based on current loadout
- [x] Track recent death locations as danger zones

## Waypoint System
- [ ] Auto-waypointing from BSP geometry (needs decision: engine extensions vs external tool)
- [ ] Weighted route selection (danger zones, enemy positions)
- [ ] Flanking behavior via alternate route scoring

## Personality / Unpredictability
- [ ] Per-bot personality traits (aggressive, defensive, etc.)
- [ ] Weapon preferences per personality
- [ ] Varied playstyle at same skill level
