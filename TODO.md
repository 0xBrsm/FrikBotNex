# FrikBotNex TODO

## Combat AI
- [x] Situational weapon selection (ammo, enemy health, splash near walls)
- [x] Strafe/dodge patterns during fights
- [x] Retreat-when-weak behavior
- [x] Ambush positioning
- [x] Human-like aim variance (flick, drift, overcorrection per skill level)
- [x] Opponent modeling (weapon, aggression, strafe tendency — skill 2+)
- [x] Combat state machine (fight/retreat/pressure/flank with utility scoring)
- [x] RL->LG combo (skill 3)

## Movement
- [x] Bunny hopping (skill 2+, with fumble chance)
- [x] Strafe jumping (air acceleration during bhop)
- [x] Rocket jumping (via AI_SUPER_JUMP waypoints)
- [x] Hazard avoidance (lava, slime — 3-probe ahead/left/right)
- [x] Elevator/platform timing

## Strategic Awareness
- [x] Item respawn timer anticipation (skill 2+)
- [x] Map control / positional advantage (skill 2+)
- [x] High-ground preference
- [x] Track recent death locations as danger zones (skill 1+)
- [ ] Chokepoint awareness (height exists, no explicit chokepoint scoring)

## Waypoint System
- [ ] Auto-waypointing from BSP geometry (needs decision: engine extensions vs external tool)
- [ ] Weighted route selection (death zone penalty exists, no full route cost system)
- [x] Flanking via combat state machine (bot_flank_move + utility scoring)

## Personality / Unpredictability
- [x] Per-bot personality traits (balanced, aggressive, defensive, camper)
- [x] Weapon preferences per personality
- [x] Varied playstyle at same skill level
- [x] Idle behavior: glancing, recreational jumps, personality-scaled
- [x] Movement jitter, look frequency, jump rate per personality
- [ ] Grudge system: track nemesis player, prioritize targeting
- [ ] Post-kill BM: brief pause + look at corpse, weapon flex
- [ ] Personality-flavored chat: branch chat strings by personality type
- [ ] Dynamic personality drift: tilt on kill/death streaks
- [ ] Sound bait: skill 3 campers fire bait shots to draw enemies

## Spatial Awareness System (planned, disabled)
- [x] Design document and field declarations (bot_sense.qc)
- [ ] Wire bot_sense() into AI loop (disabled pending entity field limit testing)
- [ ] Replace scattered tracelines with cached sense reads
- [ ] 8-direction nav fan for intra-region navigation
- [ ] Waypoint thinning using local spatial navigation
