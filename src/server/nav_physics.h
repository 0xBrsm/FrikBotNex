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

#endif /* NAV_PHYSICS_H */
