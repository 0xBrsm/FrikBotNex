/*
 * nav_physics.h -- Quake player-movement kinematics, single source of truth.
 *
 * These mirror engine/game physics the navmesh must agree with; they are
 * NOT tunables.  Jump reach, fall time, and run distance computed anywhere
 * in the nav code must come from these values so a change to one (e.g. a
 * mod altering sv_gravity) has exactly one place to land.
 */
#ifndef NAV_PHYSICS_H
#define NAV_PHYSICS_H

#define NAV_PHYS_GRAVITY       800.0f  /* sv_gravity default */
#define NAV_PHYS_JUMP_IMPULSE  270.0f  /* jump up-velocity (SV_ClientThink) */
#define NAV_PHYS_RUN_SPEED     320.0f  /* ground run speed (sv_maxspeed) */
#define NAV_PHYS_STEP_HEIGHT    18.0f  /* max stair step (SV_WalkMove STEPSIZE) */
#define NAV_PHYS_FLOOR_OFFSET   24.0f  /* player origin sits 24u above feet (mins_z = -24) */
/* Jump apex height.  True value is v0^2/2g = 270^2/1600 = 45.5625u; the nav
   code has always used the rounded 45.0f, kept verbatim -- deriving it here
   would change bake output ([DIFF], not code motion). */
#define NAV_PHYS_JUMP_APEX      45.0f

/* Rocket-jump lift, from the game QC (verified in this mod, vanilla values):
   a rocket's radius damage is 120 (weapons.qc T_MissileTouch), self-damage
   is halved (combat.qc T_RadiusDamage), and knockback velocity is damage*8
   (combat.qc T_Damage).  A point-blank feet shot therefore boosts up to
   (120 * 0.5) * 8 = 480 u/s; stacked on a jump, v0 = 270 + 480 = 750 and
   the physical apex is v0^2/2g ~ 351.6u. */
#define NAV_PHYS_RJ_BOOST      (120.0f * 0.5f * 8.0f)
#define NAV_PHYS_RJ_APEX       ((NAV_PHYS_JUMP_IMPULSE + NAV_PHYS_RJ_BOOST) \
	* (NAV_PHYS_JUMP_IMPULSE + NAV_PHYS_RJ_BOOST) / (2.0f * NAV_PHYS_GRAVITY))

/* Link envelopes (policy bounded by the physics above).  The candidate scans
   in nav_mesh.cpp and the physics validator in nav_bot.cpp MUST share these:
   they were once copy-pasted literals, and an envelope tighter than the
   validator silently loses links while a looser one changes which candidate
   gets validated.  The validator is authoritative for acceptance. */
#define NAV_JUMP_HEIGHT_MAX         48.0f  /* max jump-up height in Quake */
/* RJ stays a near-vertical last resort well inside the physical apex --
   explosion offset, imperfect timing and self-damage argue for margin. */
#define NAV_RJ_HEIGHT_MAX          256.0f  /* max single-rocket-jump up height */
#define NAV_RJ_HORIZ_MAX           128.0f  /* max horizontal while RJ-ing up */

/* Fall damage (client.qc PlayerPostThink, verified vanilla in this mod):
   landing on dry ground faster than 650 u/s costs a FLAT 5HP -- falls are
   never lethal from any reasonable health, and a water landing negates the
   damage entirely.  650 u/s is reached after v^2/2g = 264u of free fall.
   Every drop cap below is therefore WILLINGNESS or SCAN-RANGE policy, not
   survival physics. */
#define NAV_PHYS_FALL_DMG_SPEED    650.0f
#define NAV_PHYS_FALL_DMG_HEIGHT   (NAV_PHYS_FALL_DMG_SPEED \
	* NAV_PHYS_FALL_DMG_SPEED / (2.0f * NAV_PHYS_GRAVITY))

/* Boundary-drop tier (nav_link_callback edge detector): common, cheap
   links a bot takes casually -- so keep them damage-free.  The dry cap
   sits under NAV_PHYS_FALL_DMG_HEIGHT (asserted below); water negates
   fall damage, so a water landing may be deeper, paired with an
   AI_SURFACE swim-out so it is never a pit-trap.  The min stays above
   contour simplification error so an edge can't "drop" onto itself. */
#define NAV_DROP_HEIGHT_MIN          6.0f
#define NAV_DROP_HEIGHT_MAX        192.0f  /* dry land, damage-free */
#define NAV_WATER_DROP_HEIGHT_MAX  400.0f  /* into water (no fall damage) */

/* Deep-drop tier (nav_mesh_compute_deep_drops): last-resort links, added
   only when the landing can already path back out.  The bot accepts the
   5HP toll past 264u; the dry cap is willingness, chosen well past every
   drop-in region on the stock maps (a flat raise to 320 once dropped bots
   into exitless dm3 pits, so depth alone is never the gate -- the
   pathback + fall-column validation is).  For a water landing the cap
   applies to the DRY portion only (ledge to water surface); after
   splashdown the bot swims, budgeted by the wet-leg cap.  The scan max
   bounds the candidate search and must cover dry reach + wet reach
   (asserted below); the validator stays authoritative past it. */
#define NAV_DEEP_DROP_HEIGHT_MIN    48.0f  /* below this, walk/jump passes own it */
#define NAV_DEEP_DROP_HEIGHT_MAX   700.0f  /* dry-portion willingness cap */
#define NAV_DEEP_DROP_MAX_SPEED    300.0f  /* launch speed budget (full run ~320) */
#define NAV_DEEP_DROP_WET_LEG      600.0f  /* swim-out budget after splashdown */
#define NAV_DEEP_DROP_SCAN_MAX    1400.0f  /* candidate scan range (total dz) */

#ifdef __cplusplus
static_assert(NAV_RJ_HEIGHT_MAX < NAV_PHYS_RJ_APEX,
	"RJ link cap must stay under the physical rocket-jump apex");
static_assert(NAV_PHYS_JUMP_APEX < NAV_JUMP_HEIGHT_MAX,
	"run-jump reach cap must cover the jump apex");
static_assert(NAV_DROP_HEIGHT_MAX < NAV_PHYS_FALL_DMG_HEIGHT,
	"casual boundary drops must stay damage-free");
static_assert(NAV_DEEP_DROP_SCAN_MAX >=
	NAV_DEEP_DROP_HEIGHT_MAX + NAV_DEEP_DROP_WET_LEG,
	"deep-drop scan range must cover dry reach plus the wet leg");
#endif

#endif /* NAV_PHYSICS_H */
