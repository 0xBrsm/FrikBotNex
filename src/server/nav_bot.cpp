/*
 * nav_bot.c -- Bot navmesh: BSP extraction, Recast build, Detour builtins
 *
 * Extracts BSP geometry, builds a Recast navmesh with Detour
 * off-mesh connections for teleporters, and provides QC builtins
 * for bot pathfinding.
 *
 * QC builtins:
 *   #80  float nav_ready()
 *   #81  vector nav_move(vector pos, vector target)
 *   #82  float nav_path_start(vector goal)
 *   #83  float nav_route_cost(vector start, vector goal)
 *   #84  vector nav_path_steer(vector pos)
 */
// C++ compatible Quake header inclusion
extern "C" {
#include "quakedef.h"
}

#include "nav_bot.h"
#include "nav_hull.h"
#include "nav_mesh.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include "Recast.h"

#include <stdlib.h>
#include <string.h>
#include <vector>

#include <math.h>

static float nav_frand(void) { return (float)rand() / (float)RAND_MAX; }

/* NAV_DD_DEBUG opts into verbose deep-drop/swim validator logging. */
static int nav_dd_debug_enabled(void)
{
	static int dbg = -1;
	if (dbg < 0) dbg = getenv("NAV_DD_DEBUG") != NULL;
	return dbg;
}

extern "C" {
extern builtin_t *pr_builtins;
extern int pr_numbuiltins;
extern ddef_t *ED_FindField(char *name);
extern ddef_t *ED_FindGlobal(char *name);
}

/* ---- Recast config ---- */

/* Geometry comes from clip hull 1 (see nav_hull.cpp), which qbsp
   pre-expanded by the player box.  The agent is therefore a POINT:
   walkable_radius 0 (no erosion), and walkable_height is the hull-gap
   left after expansion (real gap minus 56), not the player height. */
#define NAV_CELL_SIZE                 4.0f
#define NAV_CELL_HEIGHT               2.0f
#define NAV_WALKABLE_SLOPE_ANGLE     45.0f
#define NAV_WALKABLE_HEIGHT           8.0f
#define NAV_WALKABLE_CLIMB           18.0f
#define NAV_WALKABLE_RADIUS           0.0f
#define NAV_MAX_EDGE_LEN            192.0f
#define NAV_MAX_SIMPLIFICATION_ERROR  0.1f
#define NAV_MIN_REGION_SIZE           2
#define NAV_MERGE_REGION_SIZE        20
#define NAV_MAX_VERTS_PER_POLY        6
#define NAV_DETAIL_SAMPLE_DISTANCE    6.0f
#define NAV_DETAIL_SAMPLE_MAX_ERROR   1.0f


/* Jump/drop link detection */
#define NAV_JUMP_IMPULSE            270.0f  /* Quake jump velocity (SV_ClientThink) */
#define NAV_JUMP_HEIGHT_MIN         18.0f  /* below this, walkableClimb handles it */
#define NAV_JUMP_HEIGHT_MAX         48.0f  /* max jump-up height in Quake */
/* Drops have no walkableClimb floor: a contour boundary edge means the
   surfaces did NOT connect, so even a small clear fall needs a link (dm4
   GL pocket: 8u drop over an unwalkable hull-bevel ridge).  Keep above
   contour simplification error so an edge can't "drop" onto itself. */
#define NAV_DROP_HEIGHT_MIN          6.0f
/* Quake players survive long falls (damage only past ~265u, death past
   ~800u), and whole lower regions are reached by dropping in.  Capping at
   128u orphaned them from the mesh.  Reach deep enough to link those drops;
   the fall-column hull-truth + lane checks still gate each candidate. */
#define NAV_DROP_HEIGHT_MAX        192.0f  /* max drop-down height (dry land) */
/* Water negates fall damage, so a deep plunge into a pool is survivable and
   common (dm3's -370 pool holds items).  Allow a much deeper drop ONLY when
   the landing is underwater, and always pair it with an AI_SURFACE swim-out
   so the bot can climb back to the ledge -- a deep drop-in with no exit is
   the pit-trap that a blanket cap raise caused before. */
#define NAV_WATER_DROP_HEIGHT_MAX  400.0f  /* max drop-down into water */
/* Deep dry drops past NAV_DROP_HEIGHT_MAX are handled by a separate
   post-build pass (nav_mesh_compute_deep_drops) that only links landings
   which can already path back out -- a flat cap raise to 320 dropped bots
   into exitless dm3 pits, so depth alone can't be the gate.  Falls kill
   past ~800u; stay under that. */
#define NAV_DEEP_DROP_HEIGHT_MIN    48.0f  /* below this, walk/jump passes own it */
#define NAV_DEEP_DROP_HEIGHT_MAX   700.0f
#define NAV_DEEP_DROP_MAX_SPEED    300.0f  /* launch speed budget (full run ~320) */
/* Rocket jump: the bot fires an RL at its feet while jumping for a big
   upward boost a normal run-jump can't reach.  Only used for orphan ledges
   above normal jump height; a single RJ clears ~250u up.  Horizontal reach
   while gaining that height is modest -- keep it tight so RJ stays a
   near-vertical last resort, never a substitute for a run-jump across. */
#define NAV_RJ_HEIGHT_MAX          256.0f  /* max single-rocket-jump up height */
#define NAV_RJ_HORIZ_MAX           128.0f  /* max horizontal while RJ-ing up */
#define NAV_JUMP_PROBE_DIST         48.0f  /* how far to project from edge */
#define NAV_JUMP_LINK_RADIUS        16.0f  /* agent radius */
#define NAV_START_SNAP_MAX_DIST     24.0f
#define NAV_JUMP_LAND_SNAP_MAX_DIST 24.0f
#define NAV_PLAYER_FLOOR_OFFSET     24.0f
#define NAV_DEBUG_MARKER_LIFT       24.0f

/* ---- Per-bot path corridor ---- */

static nav_corridor_t *nav_bot_corridors[MAX_SCOREBOARD];
static nav_mesh_runtime_t *nav_mesh;
static void nav_build_block_map(void);

/* ---- Entity blocking filter ---- */

static nav_blocked_polys nav_blocked;

/* Entity → poly ref mapping for block/unblock.
   Built once after navmesh construction. */
#define NAV_MAX_BLOCK_ENTITIES 64
#define NAV_MAX_ENTITY_POLYS   16
static struct {
	edict_t *ent;
	dtPolyRef polys[NAV_MAX_ENTITY_POLYS];
	int poly_count;
	int is_blocked;
} nav_block_map[NAV_MAX_BLOCK_ENTITIES];
static int nav_block_map_count = 0;

static int nav_bot_slot(void)
{
	edict_t *e = PROG_TO_EDICT(pr_global_struct->self);
	int num = NUM_FOR_EDICT(e);
	if (num < 1 || num > svs.maxclients) return -1;
	return num - 1;
}

static float nav_xy_dist_sq(const float *a, const float *b)
{
	float dx = a[0] - b[0];
	float dy = a[1] - b[1];
	return dx * dx + dy * dy;
}

static float nav_player_floor_z(float origin_z)
{
	return origin_z - NAV_PLAYER_FLOOR_OFFSET;
}

static float nav_debug_marker_z(float surface_z)
{
	return surface_z + NAV_DEBUG_MARKER_LIFT;
}

/* Trace vertically from top_z down to bot_z at a given XY position.
   Returns 1 if clear (no solid geometry in between), 0 if blocked. */
static int nav_trace_clear_vertical(const float *xy, float top_z, float bot_z)
{
	vec3_t trace_start, trace_end, zero = {0, 0, 0};
	trace_t trace;

	trace_start[0] = xy[0]; trace_start[1] = xy[1]; trace_start[2] = top_z;
	trace_end[0] = xy[0]; trace_end[1] = xy[1]; trace_end[2] = bot_z;
	trace = SV_Move(trace_start, zero, zero, trace_end, MOVE_NOMONSTERS, NULL);
	return !trace.allsolid && !trace.startsolid && trace.fraction >= 0.99f;
}

static int nav_trace_clear_at_height(const float *start, const float *end, float z, edict_t *passedict)
{
	vec3_t trace_start;
	vec3_t trace_end;
	vec3_t zero = {0, 0, 0};
	trace_t trace;

	VectorCopy(start, trace_start);
	VectorCopy(end, trace_end);
	trace_start[2] = z;
	trace_end[2] = z;
	trace = SV_Move(trace_start, zero, zero, trace_end, MOVE_NOMONSTERS, passedict);
	return !trace.allsolid && !trace.startsolid && trace.fraction >= 1.0f;
}

/* Decide how a player could traverse from 'from' to 'to' (Quake FOOT points,
   navmesh poly centroids), for nav_mesh_compute_orphan_jumps.  Returns the link
   type, or 0 if none:
     AI_WALK -- continuous floor the mesh failed to LINK (a player hull walks +
                steps straight across); bidirectional, the safest connection.
     AI_JUMP -- run-jump (up / across / across-and-down), bidirectional.
   Endpoints are walkable by construction, so no standability re-check. */
static int nav_link_validate(const float *from, const float *to, void *user)
{
	vec3_t pmins = {-16, -16, -24}, pmaxs = {16, 16, 32}, zero = {0, 0, 0};
	vec3_t ts, te;
	trace_t tr;
	float dz, adz, dx, dy, hd, disc, airtime;
	const float g = 800.0f;                 /* sv_gravity */
	const float v0 = NAV_JUMP_IMPULSE;       /* 270, jump up-velocity */
	const float maxspeed = 320.0f;           /* ground run speed */
	(void)user;

	dz = to[2] - from[2];
	dx = to[0] - from[0]; dy = to[1] - from[1];
	hd = sqrt(dx * dx + dy * dy);
	if (hd < 8.0f)
		return 0;
	adz = dz < 0 ? -dz : dz;

	/* WALK: a player box sweeps level from 'from' to 'to' (lifted one step) and
	   reaches it -> the floor is continuous and only the mesh adjacency is
	   missing.  Step height bounds the climb. */
	if (adz <= NAV_JUMP_HEIGHT_MIN)
	{
		ts[0] = from[0]; ts[1] = from[1]; ts[2] = from[2] + 24 + 18;
		te[0] = to[0]; te[1] = to[1]; te[2] = to[2] + 24 + 18;
		tr = SV_Move(ts, pmins, pmaxs, te, MOVE_NOMONSTERS, NULL);
		if (!tr.startsolid && tr.fraction > 0.97f)
			return AI_WALK;

		/* Low-passage fallback: the 18u step-lift above demands 74u of
		   headroom, vetoing flat runs through short doorways (hip2m4's
		   62u crypt doorway: lintel at +62 fails both the lifted sweep
		   and the jump apex arc).  Drop each end to its hull-truth floor
		   -- raster quantization inflates centroid z by up to a cell or
		   two, which alone can eat the clearance margin -- and sweep an
		   UNLIFTED player box.  No step allowance means any 18u+ bump
		   mid-path still rejects, so this only accepts genuinely flat,
		   low-ceiling runs a real player walks through. */
		{
			vec3_t zero2 = {0, 0, 0};
			vec3_t ds, de;
			float fz1 = from[2], fz2 = to[2];
			trace_t dtr;
			ds[0] = from[0]; ds[1] = from[1]; ds[2] = from[2] + 4;
			de[0] = from[0]; de[1] = from[1]; de[2] = from[2] - 40;
			dtr = SV_Move(ds, zero2, zero2, de, MOVE_NOMONSTERS, NULL);
			if (!dtr.startsolid && dtr.fraction < 1.0f)
				fz1 = dtr.endpos[2];
			ds[0] = to[0]; ds[1] = to[1]; ds[2] = to[2] + 4;
			de[0] = to[0]; de[1] = to[1]; de[2] = to[2] - 40;
			dtr = SV_Move(ds, zero2, zero2, de, MOVE_NOMONSTERS, NULL);
			if (!dtr.startsolid && dtr.fraction < 1.0f)
				fz2 = dtr.endpos[2];
			if (fabs(fz1 - fz2) <= NAV_JUMP_HEIGHT_MIN)
			{
				/* +1: the down-trace endpos rests ON the floor plane; a box
				   whose bottom sits exactly there reports startsolid. */
				ts[0] = from[0]; ts[1] = from[1]; ts[2] = fz1 + 24 + 1;
				te[0] = to[0]; te[1] = to[1]; te[2] = fz2 + 24 + 1;
				tr = SV_Move(ts, pmins, pmaxs, te, MOVE_NOMONSTERS, NULL);
				if (!tr.startsolid && tr.fraction > 0.97f)
					return AI_WALK;
			}
		}
	}

	/* JUMP: clear dz against gravity (apex v0^2/2g ~45u up), run distance <=
	   maxspeed * air time, apex arc wall-free.  Use SIGNED dz, not |dz| --
	   a downward target gets a LONGER airtime (falls further before landing),
	   never shorter.  disc can only go negative when dz is positive (a rise
	   past the jump's max height); it's never negative for dz<=0. */
	disc = v0 * v0 - 2.0f * g * dz;
	if (disc < 0.0f)
	{
		/* Over a normal jump's reach.  A rocket jump can still get UP to a
		   ledge (orphan higher than here) within the RJ envelope: the manual
		   graphs tag exactly these edges AI_SUPER_JUMP.  Near-vertical only --
		   keep horizontal tight so RJ never replaces a run-jump across.  The
		   caller pairs this with a drop-out so the ledge isn't a one-way trap. */
		if (dz > NAV_JUMP_HEIGHT_MAX && dz <= NAV_RJ_HEIGHT_MAX && hd <= NAV_RJ_HORIZ_MAX)
		{
			float topz = to[2] + 24.0f + 18.0f;
			ts[0] = from[0]; ts[1] = from[1]; ts[2] = topz;
			te[0] = to[0]; te[1] = to[1]; te[2] = topz;
			tr = SV_Move(ts, zero, zero, te, MOVE_NOMONSTERS, NULL);
			if (tr.fraction >= 1.0f)
				return AI_SUPER_JUMP;
		}
		return 0;
	}
	airtime = (v0 + sqrt(disc)) / g;
	if (hd / airtime > maxspeed)
		return 0;
	{
		float apexz = (to[2] > from[2] ? to[2] : from[2]) + 45.0f + 24.0f;
		ts[0] = from[0]; ts[1] = from[1]; ts[2] = apexz;
		te[0] = to[0]; te[1] = to[1]; te[2] = apexz;
		tr = SV_Move(ts, zero, zero, te, MOVE_NOMONSTERS, NULL);
		if (tr.fraction < 1.0f)
			return 0;
	}
	return AI_JUMP;
}

/* Validator for nav_mesh_compute_deep_drops: is a walk-off fall from 'from'
   down to 'to' physically clean?  Mirrors the boundary drop detector's
   gates (walk-off line, hull-truth fall column, no lava/slime landing) on
   centroid pairs.  Reachability/no-trap gating happens in the pass itself. */
static int nav_deep_drop_validate(const float *from, const float *to, void *user)
{
	float drop = from[2] - to[2];
	float dx = to[0] - from[0], dy = to[1] - from[1];
	float hd = sqrtf(dx * dx + dy * dy);
#define DDFAIL(stage) do { if (nav_dd_debug_enabled()) fprintf(stderr, \
	"DDVAL (%.0f %.0f %.0f)->(%.0f %.0f %.0f): %s\n", \
	from[0], from[1], from[2], to[0], to[1], to[2], stage); return 0; } while (0)
	(void)user;

	if (drop <= NAV_DEEP_DROP_HEIGHT_MIN)
		DDFAIL("shallow");
	if (hd < 8.0f)
		DDFAIL("speed");

	/* Underwater landing: the fall becomes "drop into the water body, then
	   swim to the floor" -- the landing poly needn't be plumb below the
	   ledge (e3m5's hole floor is 824u under its rim platform behind an
	   overhanging slope, but water catches the fall after ~210u).  The
	   lethal-fall cap applies to the DRY portion only. */
	{
		vec3_t wp;
		wp[0] = to[0]; wp[1] = to[1]; wp[2] = to[2] + 8.0f;
		if (SV_PointContents(wp) != CONTENTS_WATER)
		{
			/* Water-surface polys sit ~24u above the water plane, so the
			   probe above them reads air; check just below the plane too. */
			wp[2] = to[2] - 32.0f;
			if (SV_PointContents(wp) != CONTENTS_WATER)
				goto dry_fall;
		}
	}
	{
		/* Entry-column search: walk-off direction is from->to horizontally;
		   probe outward past the rim for a spot with a hull-clear plunge
		   into the water body.  The landing poly's own xy is often plumb
		   under an overhang (e3m5's hole floor tucks beneath its rim
		   platform), so each probe finds its own local water surface. */
		static const float kOff[] = { 16, 32, 64, 128, 192 };
		float dirh[2];
		int oi;
		char miss[160];
		miss[0] = 0;
#define DDOFF(why) do { if (nav_dd_debug_enabled()) { size_t l = strlen(miss); \
	snprintf(miss + l, sizeof(miss) - l, " o%.0f=%s", kOff[oi], why); } } while (0)
		dirh[0] = dx / (hd > 8.0f ? hd : 8.0f);
		dirh[1] = dy / (hd > 8.0f ? hd : 8.0f);
		for (oi = 0; oi < (int)(sizeof(kOff)/sizeof(kOff[0])); oi++)
		{
			vec3_t entry, fs, fe, hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			trace_t tr;
			float surface, z, ts;
			int found = 0;
			entry[0] = from[0] + dirh[0] * kOff[oi];
			entry[1] = from[1] + dirh[1] * kOff[oi];
			/* Local water surface below the ledge at this xy.  Scan a bit
			   past the landing z: a surface poly hovers ~24u above the
			   water plane it stands for. */
			surface = 0;
			for (z = from[2] - 8.0f; z >= to[2] - 40.0f; z -= 8.0f)
			{
				int c;
				entry[2] = z;
				c = SV_PointContents(entry);
				if (c == CONTENTS_WATER) { surface = z; found = 1; break; }
				if (c != CONTENTS_EMPTY) break; /* dry floor/solid first: no plunge here */
			}
			if (!found)
				{ DDOFF("nosurf"); continue; }
			if (from[2] - surface > NAV_DEEP_DROP_HEIGHT_MAX)
				{ DDOFF("deepdry"); continue; }
			/* Horizontal drift during the dry fall is physics-capped. */
			ts = sqrtf(2.0f * (from[2] - surface > 16.0f ? from[2] - surface : 16.0f) / 800.0f);
			if (kOff[oi] > NAV_DEEP_DROP_MAX_SPEED * ts)
				{ DDOFF("drift"); break; }
			if (!nav_trace_clear_at_height(from, entry, from[2] + 24.0f, NULL))
				{ DDOFF("walkoff"); continue; }
			fs[0] = entry[0]; fs[1] = entry[1]; fs[2] = from[2] + 26.0f;
			fe[0] = entry[0]; fe[1] = entry[1]; fe[2] = surface - 24.0f;
			tr = SV_Move(fs, hmins, hmaxs, fe, MOVE_NOMONSTERS, NULL);
			if (tr.startsolid || tr.allsolid || tr.endpos[2] > surface - 8.0f)
				{ DDOFF("column"); continue; }
			/* Cushion: enough water under the entry point to absorb the
			   plunge. */
			fs[2] = tr.endpos[2];
			fe[2] = fs[2] - 64.0f;
			tr = SV_Move(fs, hmins, hmaxs, fe, MOVE_NOMONSTERS, NULL);
			if (tr.fraction < 0.5f)
				{ DDOFF("cushion"); continue; }
			/* Wet leg: swim from the entry point to the landing through
			   open water.  Sink vertically in the entry column first,
			   then go straight -- a single straight chord often clips
			   underwater slopes that swim physics simply follows around. */
			{
				vec3_t goal, dir;
				float len, s;
				int wet = 1;
				entry[2] = surface - 24.0f;
				goal[0] = to[0]; goal[1] = to[1]; goal[2] = to[2] + 24.0f;
				if (SV_PointContents(goal) != CONTENTS_WATER)
					goal[2] = to[2] - 32.0f; /* surface poly: aim just under the plane */
				while (entry[2] - 16.0f >= goal[2])
				{
					vec3_t sink;
					sink[0] = entry[0]; sink[1] = entry[1]; sink[2] = entry[2] - 16.0f;
					if (SV_PointContents(sink) != CONTENTS_WATER)
						break;
					entry[2] -= 16.0f;
				}
				dir[0] = goal[0] - entry[0]; dir[1] = goal[1] - entry[1]; dir[2] = goal[2] - entry[2];
				len = sqrtf(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
				if (len > 600.0f) /* swim-reach cap, as in the swim pass */
					{ DDOFF("wetlen"); continue; }
				for (s = 16.0f; s < len && wet; s += 16.0f)
				{
					vec3_t sp;
					sp[0] = entry[0] + dir[0] * (s / len);
					sp[1] = entry[1] + dir[1] * (s / len);
					sp[2] = entry[2] + dir[2] * (s / len);
					if (SV_PointContents(sp) != CONTENTS_WATER)
						wet = 0;
				}
				if (!wet)
					{ DDOFF("wet"); continue; }
			}
			return AI_DROP;
		}
#undef DDOFF
		/* No clean plunge found -- the landing may still be plumb below
		   the ledge with a clear column; let the dry path decide. */
		if (nav_dd_debug_enabled())
			fprintf(stderr, "DDVAL (%.0f %.0f %.0f)->(%.0f %.0f %.0f): water-entry-miss%s\n",
				from[0], from[1], from[2], to[0], to[1], to[2], miss);
	}

dry_fall:
	if (drop > NAV_DEEP_DROP_HEIGHT_MAX)
		DDFAIL("deep-dry");
	/* Horizontal reach is physics-limited, not a fixed radius: the launch
	   speed needed to cover hd during the fall must fit inside a full run.
	   Dry landings only -- a water landing swims its horizontal remainder,
	   which the entry-column search caps per offset. */
	if (hd > NAV_DEEP_DROP_MAX_SPEED * sqrtf(2.0f * drop / 800.0f))
		DDFAIL("speed");

	/* Fall-column search: the landing rep point is often tucked under the
	   take-off slab or within hull width of a wall, where the plumb hull
	   column reads solid even though a player falling just short of (or
	   past) that point lands on the same floor and walks the remainder.
	   Probe columns at offsets along the walk-off line. */
	{
		static const float kRel[] = { 0, -16, 16, -32, 32, -48, 48 };
		static const float kLat[] = { 0, -16, 16, -32, 32, -48, 48, -64, 64, -96, 96 };
		float dirh[2], ts;
		int oi, li, ok = 0;
		char miss[200];
		miss[0] = 0;
#define DDOFF(why) do { if (nav_dd_debug_enabled() && li == 0) { size_t l = strlen(miss); \
	snprintf(miss + l, sizeof(miss) - l, " r%+.0f=%s", kRel[oi], why); } } while (0)
		ts = sqrtf(2.0f * drop / 800.0f);
		dirh[0] = dx / (hd > 8.0f ? hd : 8.0f);
		dirh[1] = dy / (hd > 8.0f ? hd : 8.0f);
		for (oi = 0; oi < (int)(sizeof(kRel)/sizeof(kRel[0])) && !ok; oi++)
		for (li = 0; li < (int)(sizeof(kLat)/sizeof(kLat[0])) && !ok; li++)
		{
			vec3_t p, fs, fe, hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			trace_t tr;
			float o = hd + kRel[oi];
			if (o < 0.0f)
				continue;
			/* Drift during the fall is physics-capped: air-steering covers
			   both the along-line and the sideways component. */
			if (sqrtf(o * o + kLat[li] * kLat[li]) > NAV_DEEP_DROP_MAX_SPEED * ts)
				{ DDOFF("drift"); continue; }
			p[0] = from[0] + dirh[0] * o - dirh[1] * kLat[li];
			p[1] = from[1] + dirh[1] * o + dirh[0] * kLat[li];
			p[2] = to[2];
			/* Walk-off line at start height out to above the column. */
			if (!nav_trace_clear_at_height(from, p, from[2] + 24.0f, NULL))
				{ DDOFF("walkoff"); continue; }
			fs[0] = p[0]; fs[1] = p[1]; fs[2] = from[2] + 26.0f;
			fe[0] = p[0]; fe[1] = p[1]; fe[2] = to[2] + 24.0f;
			tr = SV_Move(fs, hmins, hmaxs, fe, MOVE_NOMONSTERS, NULL);
			if (tr.startsolid || tr.allsolid || tr.endpos[2] > to[2] + 36.0f)
				{ DDOFF("column"); continue; }
			/* Landed off the rep point: the remainder is a level walk.
			   Mid-point floor probe guards against the level trace sailing
			   over a trench between the column and the rep point. */
			if (kRel[oi] != 0.0f || kLat[li] != 0.0f)
			{
				vec3_t land, mid, mlow, zero = {0, 0, 0};
				trace_t mtr;
				land[0] = p[0]; land[1] = p[1]; land[2] = tr.endpos[2];
				if (!nav_trace_clear_at_height(land, to, tr.endpos[2] + 24.0f, NULL))
					{ DDOFF("walkback"); continue; }
				mid[0] = (land[0] + to[0]) * 0.5f;
				mid[1] = (land[1] + to[1]) * 0.5f;
				mid[2] = tr.endpos[2] + 24.0f;
				mlow[0] = mid[0]; mlow[1] = mid[1]; mlow[2] = to[2] - 40.0f;
				mtr = SV_Move(mid, zero, zero, mlow, MOVE_NOMONSTERS, NULL);
				if (mtr.fraction >= 1.0f)
					{ DDOFF("midgap"); continue; }
			}
			ok = 1;
		}
#undef DDOFF
		if (!ok)
		{
			if (nav_dd_debug_enabled())
				fprintf(stderr, "DDVAL (%.0f %.0f %.0f)->(%.0f %.0f %.0f): column-miss%s\n",
					from[0], from[1], from[2], to[0], to[1], to[2], miss);
			return 0;
		}
	}

	/* Suicide-chute gate: never a drop that lands in lava or slime. */
	{
		vec3_t lc;
		int lcont;
		lc[0] = to[0]; lc[1] = to[1]; lc[2] = to[2] + 8.0f;
		lcont = SV_PointContents(lc);
		if (lcont == CONTENTS_LAVA || lcont == CONTENTS_SLIME)
			DDFAIL("lava");
	}
	return AI_DROP;
}

/* Validator for nav_mesh_compute_swim_links: can a player swim the straight
   segment from 'from' to 'to' (floor points, probed at swim height +24)?
   Submersion is what makes the link safely bidirectional: no falls, no
   damage, symmetric traversal.  Deliberately NOT a full-corridor hull sweep:
   tight shafts (end's exit well is barely hull-wide and skewed) have no
   straight hull-clear line, yet swim physics slides along the walls just
   fine.  Instead: every sample along the line must be open water (a small
   air allowance right at an endpoint covers surface pop-outs), the point
   trace must not cross solid, and the player hull must actually fit at
   both endpoints. */
/* One straight swim segment between two swim-height points: every sample
   open water (small air allowance right at the ends covers surface
   pop-outs), and a point trace to catch thin walls the sampling straddled. */
static char nav_swim_seg_why[96];

static int nav_swim_seg_wet(const float *fs, const float *fe)
{
	float len;
	int i, steps, water_samples = 0;
	float dx = fe[0]-fs[0], dy = fe[1]-fs[1], dz = fe[2]-fs[2];
	len = sqrtf(dx*dx + dy*dy + dz*dz);
	nav_swim_seg_why[0] = 0;
	if (len < 8.0f)
		{ snprintf(nav_swim_seg_why, sizeof(nav_swim_seg_why), "short"); return 0; }
	steps = (int)(len / 16.0f) + 1;
	for (i = 0; i <= steps; i++)
	{
		vec3_t p;
		float f = (float)i / (float)steps;
		float edge_dist = (f < 0.5f ? f : 1.0f - f) * len;
		int c;
		p[0] = fs[0] + dx * f;
		p[1] = fs[1] + dy * f;
		p[2] = fs[2] + dz * f;
		c = SV_PointContents(p);
		if (c == CONTENTS_WATER)
			water_samples++;
		else if (c != CONTENTS_EMPTY || edge_dist > 56.0f)
		{
			/* solid/lava/slime anywhere, or air away from the rims */
			snprintf(nav_swim_seg_why, sizeof(nav_swim_seg_why),
				"c%d@(%.0f %.0f %.0f)", c, p[0], p[1], p[2]);
			return 0;
		}
	}
	/* The middle must be genuinely submerged, not a mostly-air hop. */
	if (water_samples * 2 < steps + 1)
		{ snprintf(nav_swim_seg_why, sizeof(nav_swim_seg_why), "majority"); return 0; }
	{
		vec3_t zero3 = {0, 0, 0}, a, b;
		trace_t tr;
		a[0]=fs[0]; a[1]=fs[1]; a[2]=fs[2];
		b[0]=fe[0]; b[1]=fe[1]; b[2]=fe[2];
		tr = SV_Move(a, zero3, zero3, b, MOVE_NOMONSTERS, NULL);
		if (tr.startsolid || tr.allsolid || tr.fraction < 1.0f)
			{ snprintf(nav_swim_seg_why, sizeof(nav_swim_seg_why), "trace"); return 0; }
	}
	return 1;
}

static int nav_swim_link_validate(const float *from, const float *to, void *user)
{
	vec3_t fs, fe;
	int i, wet;
	(void)user;

	for (i = 0; i < 3; i++)
	{
		fs[i] = from[i];
		fe[i] = to[i];
	}
	fs[2] += 24.0f;
	fe[2] += 24.0f;

	/* Waterline polys hover a varying 14-30u above the water plane; their
	   swim point is just below the plane, not above the poly.  Scan down
	   for the local surface instead of trusting one fixed offset. */
	for (i = 0; i < 2; i++)
	{
		float *pt = i ? fe : fs;
		if (SV_PointContents(pt) != CONTENTS_WATER)
		{
			vec3_t sub;
			float z;
			sub[0] = pt[0]; sub[1] = pt[1];
			for (z = pt[2] - 8.0f; z >= pt[2] - 96.0f; z -= 8.0f)
			{
				int c;
				sub[2] = z;
				c = SV_PointContents(sub);
				if (c == CONTENTS_WATER) { pt[2] = z - 8.0f; break; }
				if (c != CONTENTS_EMPTY) break; /* floor first: no water here */
			}
		}
	}

	{
		wet = nav_swim_seg_wet(fs, fe);
		if (!wet && nav_dd_debug_enabled())
			fprintf(stderr, "SWVAL (%.0f %.0f %.0f)->(%.0f %.0f %.0f): straight %s\n",
				fs[0], fs[1], fs[2], fe[0], fe[1], fe[2], nav_swim_seg_why);
	}

	/* Swim paths bend: a well/hole floor rises through its own water
	   column, then crosses over -- the straight chord clips the shaft
	   wall or an overhang (e3m5's hole under its donut platform).  Try
	   an L: climb vertically inside the water at either endpoint, then
	   go straight from that pivot. */
	if (!wet)
	{
		int dn;
		for (dn = 0; dn < 2 && !wet; dn++)
		for (i = 0; i < 2 && !wet; i++)
		{
			/* Rise pivots first (surface pop-outs), then sink pivots --
			   diving under a wall that splits two pools above depth is a
			   standard Quake move (e3m5's divider is solid -140..-660 but
			   open water beneath). */
			const float *s = i ? fe : fs;
			const float *o = i ? fs : fe;
			vec3_t pivot;
			float step = dn ? -32.0f : 32.0f;
			float rise;
			pivot[0] = s[0]; pivot[1] = s[1];
			for (rise = step; rise <= 512.0f && rise >= -512.0f && !wet; rise += step)
			{
				pivot[2] = s[2] + rise;
				if (SV_PointContents(pivot) != CONTENTS_WATER)
					break; /* left the water column: no farther pivot here */
				if (nav_swim_seg_wet(pivot, o))
					wet = 1;
			}
		}
	}
	if (!wet)
		return 0;

	/* The player hull must fit at each endpoint -- at swim height, or a
	   little higher for shelves right under the surface (the bot floats). */
	for (i = 0; i < 2; i++)
	{
		const float *pt = i ? to : from;
		vec3_t hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
		int fits = 0;
		for (float up = 24.0f; up <= 40.0f; up += 8.0f)
		{
			vec3_t hp;
			trace_t tr;
			hp[0] = pt[0]; hp[1] = pt[1]; hp[2] = pt[2] + up;
			tr = SV_Move(hp, hmins, hmaxs, hp, MOVE_NOMONSTERS, NULL);
			if (!tr.startsolid && !tr.allsolid)
			{
				fits = 1;
				break;
			}
		}
		if (!fits)
		{
			if (nav_dd_debug_enabled())
				fprintf(stderr, "SWVAL (%.0f %.0f %.0f)->(%.0f %.0f %.0f): hullfit@%d\n",
					from[0], from[1], from[2], to[0], to[1], to[2], i);
			return 0;
		}
	}
	return AI_DROP;
}

static int nav_find_bot_poly(dtNavMeshQuery *query, edict_t *bot, const float *qpos, dtPolyRef *out_ref, float *out_nearest)
{
	dtQueryFilter filter;
	float rc_pos[3];
	float test[3];
	bool over_poly = false;

	if (query == NULL || nav_mesh == NULL || qpos == NULL || out_ref == NULL || out_nearest == NULL)
		return 0;

	nav_mesh_setup_filter(&filter);
	nav_quake_to_recast(qpos, rc_pos);
	*out_ref = 0;
	if (!nav_mesh_actor_floor_snap(nav_mesh, &filter, rc_pos, out_ref, out_nearest, &over_poly))
		return 0;

	nav_recast_to_quake(out_nearest, test);
	if (nav_xy_dist_sq(qpos, test) > NAV_START_SNAP_MAX_DIST * NAV_START_SNAP_MAX_DIST)
		return 0;
	if (!over_poly && nav_xy_dist_sq(qpos, test) > 8.0f * 8.0f)
		return 0;
	if (!nav_trace_clear_at_height(qpos, test, test[2] + NAV_PLAYER_FLOOR_OFFSET, bot))
		return 0;

	return 1;
}

/* ---- Item poly ref cache ---- */
#define NAV_MAX_ITEMS 256
static struct {
	edict_t *ent;
	dtPolyRef poly_ref;
	float nav_pos[3]; /* position snapped to navmesh (Recast coords) */
} nav_item_cache[NAV_MAX_ITEMS];
static int nav_item_count = 0;
static func_t nav_bot_want_func = 0;

static void nav_ent_pos(edict_t *ent, float *pos);
static int nav_build_attempted = 0;
static struct model_s *nav_built_for_model = NULL;
static cvar_t nav_enabled_cvar = {"nav_enabled", "0"};
static cvar_t nav_jump_links_cvar = {"nav_jump_links", "1"};
static cvar_t nav_directed_links_cvar = {"nav_directed_links", "1"};
static cvar_t nav_gap_jumps_cvar = {"nav_gap_jumps", "1"};
static cvar_t nav_rocket_jumps_cvar = {"nav_rocket_jumps", "1"};
static cvar_t nav_deep_drops_cvar = {"nav_deep_drops", "1"};
static cvar_t nav_swim_links_cvar = {"nav_swim_links", "1"};
static cvar_t nav_debug_cvar = {"nav_debug", "0"};

/* debug visualization state */
static nav_mesh_poly_record_t *nav_debug_polys = NULL;
static int nav_debug_poly_count = 0;
static int nav_debug_cursor = 0;

static void nav_default_config(nav_mesh_build_config_t *config)
{
	memset(config, 0, sizeof(*config));
	config->cell_size             = NAV_CELL_SIZE;
	config->cell_height           = NAV_CELL_HEIGHT;
	config->walkable_slope_angle  = NAV_WALKABLE_SLOPE_ANGLE;
	config->walkable_height       = NAV_WALKABLE_HEIGHT;
	config->walkable_climb        = NAV_WALKABLE_CLIMB;
	config->walkable_radius       = NAV_WALKABLE_RADIUS;
	config->max_edge_len          = NAV_MAX_EDGE_LEN;
	config->max_simplification_error = NAV_MAX_SIMPLIFICATION_ERROR;
	config->min_region_size       = NAV_MIN_REGION_SIZE;
	config->merge_region_size     = NAV_MERGE_REGION_SIZE;
	config->max_verts_per_poly    = NAV_MAX_VERTS_PER_POLY;
	config->detail_sample_distance   = NAV_DETAIL_SAMPLE_DISTANCE;
	config->detail_sample_max_error  = NAV_DETAIL_SAMPLE_MAX_ERROR;
}

/* ---- BSP geometry extraction ---- */

/* Brush entities whose clip hulls belong in the navmesh. */
/* QC spawn code renames brush entities (doors.qc/plats.qc):
   func_door & func_door_secret -> "door", func_plat -> "plat",
   func_train -> "train".  Nav builds after spawn, so match the
   renamed forms.

   Doors are special: thin horizontal slabs (dm2 water covers, dm6
   secret-door platform) are floors bots stand on and must be in the
   mesh.  Tall doors block passages bots path through (they open on
   touch), so those stay out.

   Plats resting at the BOTTOM are excluded: baking the body at that
   position leaves a column standing in the lower room, fragmenting
   the surrounding mesh (dm3 lost edges) -- traversal is the off-mesh
   link system's job.  Plats parked at the TOP (targetname'd plats
   wait at pos1 until triggered, per plats.qc) are the opposite case:
   the raised body IS the floor the map intends players to walk on,
   and omitting it leaves a shaft-deep pit in the mesh (e4m8's
   1008-unit elevator).  Bake those at their resting position; the
   plat link still spans top<->bottom for traversal. */
#define NAV_DOOR_FLOOR_MAX_THICKNESS 32.0f

static int nav_plat_rests_at_top(edict_t *e)
{
	eval_t *pos1 = GetEdictFieldValue(e, "pos1");
	if (!pos1) return 0;
	return e->v.origin[0] == pos1->vector[0]
		&& e->v.origin[1] == pos1->vector[1]
		&& e->v.origin[2] == pos1->vector[2];
}

/* Can a player actually STAND on this brush's top face?  A START_OPEN
   floor-door parked in a slot under the real floor (e2m2's extending
   bridge) has no headroom anywhere on its top; baking it creates a
   phantom walkable layer inside the closed cavity that captures
   item/spawn snapping.  Sample a grid over the top face and require
   player-height clearance at one of them before the brush qualifies. */
static int nav_top_face_has_clearance(edict_t *e)
{
	vec3_t zero = {0, 0, 0}, start, end;
	int ix, iy;
	for (ix = 0; ix < 3; ix++)
	{
		for (iy = 0; iy < 3; iy++)
		{
			trace_t tr;
			start[0] = e->v.absmin[0] + (e->v.absmax[0] - e->v.absmin[0]) * (0.25f + 0.25f * ix);
			start[1] = e->v.absmin[1] + (e->v.absmax[1] - e->v.absmin[1]) * (0.25f + 0.25f * iy);
			start[2] = e->v.absmax[2] + 2.0f;
			end[0] = start[0];
			end[1] = start[1];
			end[2] = start[2] + 56.0f;
			tr = SV_Move(start, zero, zero, end, MOVE_NOMONSTERS, e);
			if (!tr.startsolid && tr.fraction == 1.0f)
				return 1;
		}
	}
	return 0;
}

/* A spawn point or item resting on a door's top face is map-author
   evidence the door is meant to be stood on -- bake doors like that
   even when they're too narrow for the room-sized footprint test
   (hip2m6's 110u-wide pit lid has a DM spawn standing on it). */
static int nav_door_top_supports_entity(edict_t *e)
{
	int i;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *o = EDICT_NUM(i);
		const char *cn;
		if (o->free || o == e) continue;
		cn = pr_strings + (int)o->v.classname;
		if (strcmp(cn, "info_player_deathmatch") && strcmp(cn, "info_player_start")
			&& strncmp(cn, "item_", 5) && strncmp(cn, "weapon_", 7))
			continue;
		if (o->v.origin[0] < e->v.absmin[0] - 16.0f || o->v.origin[0] > e->v.absmax[0] + 16.0f) continue;
		if (o->v.origin[1] < e->v.absmin[1] - 16.0f || o->v.origin[1] > e->v.absmax[1] + 16.0f) continue;
		if (o->v.origin[2] < e->v.absmax[2] - 4.0f || o->v.origin[2] > e->v.absmax[2] + 40.0f) continue;
		return 1;
	}
	return 0;
}

/* A func_door acting as a room-sized collapsing floor (e2m6's oubliette):
   opens downward, footprint wide enough to stand on, parked closed at its
   top position, with standing room above.  These get baked as floor AND
   get a plat-style ride link, since the descent is the intended route. */
static int nav_is_floor_collapse_door(edict_t *e)
{
	eval_t *pos1 = GetEdictFieldValue(e, "pos1");
	eval_t *pos2 = GetEdictFieldValue(e, "pos2");
	float sx = e->v.absmax[0] - e->v.absmin[0];
	float sy = e->v.absmax[1] - e->v.absmin[1];
	float min_horiz = sx < sy ? sx : sy;
	return pos1 && pos2
		&& pos2->vector[2] < pos1->vector[2] - 32.0f
		&& e->v.origin[2] == pos1->vector[2]
		&& (min_horiz >= 128.0f
			|| (min_horiz >= 48.0f && nav_door_top_supports_entity(e)))
		&& nav_top_face_has_clearance(e);
}

static int nav_is_brush_entity(edict_t *e)
{
	char *classname = pr_strings + (int)e->v.classname;
	if (!strcasecmp(classname, "door"))
	{
		if ((e->v.absmax[2] - e->v.absmin[2]) <= NAV_DOOR_FLOOR_MAX_THICKNESS)
			return nav_top_face_has_clearance(e);
		/* Tall doors are usually passages -- never bake those.  But a
		   room-sized collapsing floor (see nav_is_floor_collapse_door)
		   is the floor the mesh needs. */
		return nav_is_floor_collapse_door(e);
	}
	if (!strcasecmp(classname, "plat"))
		return nav_plat_rests_at_top(e);
	/* Trains you can stand on are floor; a floor-to-ceiling train is a
	   triggered barrier (e2m6's corridor bars) -- don't seal the passage. */
	if (!strcasecmp(classname, "train"))
		return nav_top_face_has_clearance(e);
	return !strncasecmp(classname, "func_wall", 9)
		|| !strncasecmp(classname, "func_episodegate", 16)
		|| !strncasecmp(classname, "func_bossgate", 13);
}

/* Openable doors are SOLID_BSP, so every build-time SV_Move validation
   trace (jump apex arcs, walk sweeps, drop fall columns) hits them
   CLOSED and vetoes links through doorways the door will vacate in
   play (e1m4 armor sill: a 6u-thick triggered door across the only
   jump-up).  Hold every non-baked door at its open position (pos2)
   for the duration of the link passes, then put them back.  Baked
   doors stay closed: the raster has them closed, and traces must
   match the mesh. */
#define NAV_MAX_OPEN_DOORS 128
static struct { edict_t *e; vec3_t org; } nav_opened_doors[NAV_MAX_OPEN_DOORS];
static int nav_opened_door_count;

static void nav_doors_open_for_build(void)
{
	int i;

	nav_opened_door_count = 0;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		eval_t *pos2;
		vec3_t open_pos;
		if (e->free) continue;
		if (strcasecmp(pr_strings + (int)e->v.classname, "door")) continue;
		if (nav_is_brush_entity(e)) continue;
		pos2 = GetEdictFieldValue(e, "pos2");
		if (pos2 == NULL) continue;
		if (nav_opened_door_count >= NAV_MAX_OPEN_DOORS) break;
		VectorCopy(pos2->vector, open_pos);
		/* START_OPEN func_doors swapped pos1/pos2 at spawn: they REST at
		   their open position and pos2 is the authored CLOSED spot, so
		   "hold at pos2" would wrongly assemble them shut (hipend's boss
		   dais meshed as a floor no DM game ever has).  They're already
		   open -- hold them right where they are.  (movedir!=0 keeps
		   fd_secret out: its spawnflag 1 means open-once, not start-open.) */
		if (((int)e->v.spawnflags & 1)
			&& (e->v.movedir[0] != 0.0f || e->v.movedir[1] != 0.0f
				|| e->v.movedir[2] != 0.0f))
			VectorCopy(e->v.origin, open_pos);
		if (e->v.movedir[0] == 0.0f && e->v.movedir[1] == 0.0f
			&& e->v.movedir[2] == 0.0f)
		{
			/* No movedir means fd_secret, not func_door (secrets never
			   call SetMovedir): pos2 is computed lazily on first use,
			   still zero here.  Replicate fd_secret_use's dest2.  (A
			   zero pos2 alone is NOT a secret tell -- START_OPEN doors
			   swap pos1/pos2 at spawn, so their open offset IS zero.) */
			eval_t *mangle = GetEdictFieldValue(e, "mangle");
			eval_t *tw = GetEdictFieldValue(e, "t_width");
			eval_t *tl = GetEdictFieldValue(e, "t_length");
			vec3_t fwd, right, up;
			float width, length, temp;
			if (mangle == NULL) continue;
			AngleVectors(mangle->vector, fwd, right, up);
			temp = ((int)e->v.spawnflags & 2) ? -1.0f : 1.0f;
			if (tw != NULL && tw->_float != 0.0f)
				width = tw->_float;
			else if ((int)e->v.spawnflags & 4)
				width = fabsf(DotProduct(up, e->v.size));
			else
				width = fabsf(DotProduct(right, e->v.size));
			if (tl != NULL && tl->_float != 0.0f)
				length = tl->_float;
			else
				length = fabsf(DotProduct(fwd, e->v.size));
			if ((int)e->v.spawnflags & 4)
				VectorMA(e->v.origin, -width, up, open_pos);
			else
				VectorMA(e->v.origin, width * temp, right, open_pos);
			VectorMA(open_pos, length, fwd, open_pos);
		}
		nav_opened_doors[nav_opened_door_count].e = e;
		VectorCopy(e->v.origin, nav_opened_doors[nav_opened_door_count].org);
		nav_opened_door_count++;
		VectorCopy(open_pos, e->v.origin);
		SV_LinkEdict(e, false);
	}
	if (nav_opened_door_count > 0)
		fprintf(stderr, "Nav: %d doors held open for link validation\n", nav_opened_door_count);
}

static void nav_doors_restore(void)
{
	int i;

	for (i = 0; i < nav_opened_door_count; i++)
	{
		edict_t *e = nav_opened_doors[i].e;
		VectorCopy(nav_opened_doors[i].org, e->v.origin);
		SV_LinkEdict(e, false);
	}
	nav_opened_door_count = 0;
}

static int nav_door_held_open(edict_t *e)
{
	int i;

	for (i = 0; i < nav_opened_door_count; i++)
		if (nav_opened_doors[i].e == e)
			return 1;
	return 0;
}

/* Polygonize clip hull 1 of the world plus static brush entities.
   See nav_hull.cpp for why hull geometry instead of render faces. */
static int nav_extract_bsp(model_t *worldmodel,
	float **out_verts, int *out_vert_count,
	int **out_tris, int *out_tri_count)
{
	int i;

	*out_verts = NULL; *out_vert_count = 0;
	*out_tris = NULL;  *out_tri_count = 0;
	if (!worldmodel) return 0;

	nav_hull_begin();
	nav_hull_add_model(worldmodel, NULL);

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		model_t *m;
		if (e->free) continue;
		m = sv.models[(int)e->v.modelindex];
		if (!m || m == worldmodel) continue;
		if (!nav_is_brush_entity(e) && !nav_door_held_open(e)) continue;
		if (getenv("NAV_DUMP_BAKE") != NULL)
			fprintf(stderr, "Nav: BAKE %s %s org=(%.0f %.0f %.0f) abs=(%.0f %.0f %.0f)-(%.0f %.0f %.0f)\n",
				pr_strings + (int)e->v.classname, sv.model_precache[(int)e->v.modelindex],
				e->v.origin[0], e->v.origin[1], e->v.origin[2],
				e->v.absmin[0], e->v.absmin[1], e->v.absmin[2],
				e->v.absmax[0], e->v.absmax[1], e->v.absmax[2]);
		nav_hull_add_model(m, e->v.origin);
	}

	{
		int ok = nav_hull_end(out_verts, out_vert_count, out_tris, out_tri_count);
		const char *box = getenv("NAV_DUMP_TRIS");
		if (ok && box)
		{
			float x0, y0, x1, y1;
			if (sscanf(box, "%f %f %f %f", &x0, &y0, &x1, &y1) == 4)
			{
				int t;
				for (t = 0; t < *out_tri_count; t++)
				{
					float *a = *out_verts + (*out_tris)[t * 3 + 0] * 3;
					float *b = *out_verts + (*out_tris)[t * 3 + 1] * 3;
					float *c = *out_verts + (*out_tris)[t * 3 + 2] * 3;
					float cx = (a[0] + b[0] + c[0]) / 3.0f;
					float cy = (a[1] + b[1] + c[1]) / 3.0f;
					if (cx < x0 || cx > x1 || cy < y0 || cy > y1) continue;
					fprintf(stderr, "Nav: TRIDUMP %d (%.0f %.0f %.0f) (%.0f %.0f %.0f) (%.0f %.0f %.0f)\n",
						t, a[0], a[1], a[2], b[0], b[1], b[2], c[0], c[1], c[2]);
				}
			}
		}
		return ok;
	}
}

/* Grow a link array by doubling once it's full. */
static void nav_link_ensure_cap(nav_off_mesh_link_t **links, int n, int *cap)
{
	if (n >= *cap)
	{
		*cap *= 2;
		*links = (nav_off_mesh_link_t *)realloc(*links, (size_t)*cap * sizeof(**links));
	}
}

/* ---- Teleporter off-mesh links ---- */

static int nav_collect_teleporters(nav_off_mesh_link_t **out_links)
{
	int i, j, count, n;
	nav_off_mesh_link_t *links;

	count = 0;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		if (!e->free && !strcasecmp(pr_strings + (int)e->v.classname, "trigger_teleport"))
			count++;
	}
	if (count == 0) { *out_links = NULL; return 0; }

	links = (nav_off_mesh_link_t *)calloc(count, sizeof(*links));
	n = 0;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *src = EDICT_NUM(i);
		const char *tgt;
		if (src->free) continue;
		if (strcasecmp(pr_strings + (int)src->v.classname, "trigger_teleport")) continue;
		tgt = src->v.target ? pr_strings + (int)src->v.target : "";
		if (!tgt[0]) continue;

		for (j = 1; j < sv.num_edicts; j++)
		{
			edict_t *dst = EDICT_NUM(j);
			const char *tn;
			if (dst->free) continue;
			if (strcasecmp(pr_strings + (int)dst->v.classname, "info_teleport_destination"))
				continue;
			tn = dst->v.targetname ? pr_strings + (int)dst->v.targetname : "";
			if (strcmp(tgt, tn)) continue;

			links[n].start[0] = (src->v.absmin[0] + src->v.absmax[0]) * 0.5f;
			links[n].start[1] = (src->v.absmin[1] + src->v.absmax[1]) * 0.5f;
			links[n].start[2] = src->v.absmin[2];
			links[n].end[0] = dst->v.origin[0];
			links[n].end[1] = dst->v.origin[1];
			links[n].end[2] = dst->v.origin[2];
			links[n].radius = 128.0f;
			links[n].bidirectional = 0;
			links[n].link_type = AI_TELELINK;
			links[n].required_speed = 0;
			links[n].height_delta = 0;
			n++;
			break;
		}
	}
	*out_links = links;
	return n;
}

/* ---- Wind tunnel (trigger_push) links ---- */

/* Simulate a player carried by trigger_push volumes (e3m5 wind tunnels).
   QC sets velocity = movedir * speed * 10 EVERY frame the player touches
   the volume; outside, plain ballistic gravity.  Chained tunnels (vertical
   shaft feeding a horizontal blower) fall out naturally by checking every
   push volume each step.  One one-way link per tunnel: enter at the bottom,
   land wherever the ride ends. */
static int nav_collect_push_links(nav_off_mesh_link_t **out_links)
{
	int i, n = 0, cap = 8;
	nav_off_mesh_link_t *links;
	edict_t *pushers[64];
	int npush = 0;

	*out_links = NULL;
	for (i = 1; i < sv.num_edicts && npush < 64; i++)
	{
		edict_t *e = EDICT_NUM(i);
		if (e->free) continue;
		if (strcasecmp(pr_strings + (int)e->v.classname, "trigger_push")) continue;
		if ((int)e->v.spawnflags & 1) continue; /* PUSH_ONCE: gone after one use */
		pushers[npush++] = e;
	}
	if (npush == 0) { fprintf(stderr, "Nav: push: 0 pushers\n"); return 0; }

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));

	for (i = 0; i < npush; i++)
	{
		edict_t *src = pushers[i];
		eval_t *spd = GetEdictFieldValue(src, "speed");
		float speed = (spd && spd->_float > 0) ? spd->_float : 1000.0f;
		vec3_t start, pos, vel;
		float t, dt = 0.05f;
		int landed = 0, stuck = 0, rode_up = 0;
		(void)speed;

		/* Entry: bottom center of the volume, where a bot walks in --
		   ground-snapped, since the volume can hang well above the floor
		   (e3m5's shaft bottoms float 70u+ up) and Detour's off-mesh
		   stitching only tolerates ~walkableClimb of vertical error. */
		start[0] = (src->v.absmin[0] + src->v.absmax[0]) * 0.5f;
		start[1] = (src->v.absmin[1] + src->v.absmax[1]) * 0.5f;
		start[2] = src->v.absmin[2];

		/* Simulate from inside the volume bottom (a rider is grabbed there),
		   but the LINK start must be the floor beneath it: the volume can
		   hang 70u+ up (e3m5's shaft bottoms) and Detour's off-mesh stitch
		   only tolerates ~walkableClimb of vertical error. */
		pos[0] = start[0]; pos[1] = start[1]; pos[2] = start[2] + 25.0f;
		{
			vec3_t gs, ge, hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			trace_t gtr;
			gs[0] = start[0]; gs[1] = start[1]; gs[2] = start[2] + 25.0f;
			ge[0] = start[0]; ge[1] = start[1]; ge[2] = start[2] - 512.0f;
			gtr = SV_Move(gs, hmins, hmaxs, ge, MOVE_NOMONSTERS, NULL);
			if (!gtr.startsolid && !gtr.allsolid && gtr.fraction < 1.0f)
				start[2] = gtr.endpos[2] - 24.0f;
		}
		vel[0] = vel[1] = vel[2] = 0;

		for (t = 0; t < 15.0f && !landed && !stuck; t += dt)
		{
			vec3_t next, hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			trace_t tr;
			int j, inside = 0;

			/* Touch check mirrors the engine: player bbox vs trigger bbox. */
			for (j = 0; j < npush; j++)
			{
				edict_t *p = pushers[j];
				if (pos[0] + hmaxs[0] < p->v.absmin[0] || pos[0] + hmins[0] > p->v.absmax[0]) continue;
				if (pos[1] + hmaxs[1] < p->v.absmin[1] || pos[1] + hmins[1] > p->v.absmax[1]) continue;
				if (pos[2] + hmaxs[2] < p->v.absmin[2] || pos[2] + hmins[2] > p->v.absmax[2]) continue;
				eval_t *ps = GetEdictFieldValue(p, "speed");
				float psp = (ps && ps->_float > 0) ? ps->_float : 1000.0f;
				vel[0] = p->v.movedir[0] * psp * 10.0f;
				vel[1] = p->v.movedir[1] * psp * 10.0f;
				vel[2] = p->v.movedir[2] * psp * 10.0f;
				inside = 1;
				break;
			}
			if (inside && vel[2] > 200.0f)
				rode_up = 1;
			if (!inside)
				vel[2] -= 800.0f * dt;

			/* Vertical shaft mouth: with neutral input the rider oscillates
			   at the apex forever (rises out, falls back in, re-grabbed) --
			   a real player air-steps onto the rim.  When an upward ride
			   stalls with no horizontal carry, look for a rim ledge. */
			if (!inside && rode_up && vel[2] <= 0.0f
				&& fabsf(vel[0]) < 100.0f && fabsf(vel[1]) < 100.0f)
			{
				static const float dirs[8][2] = {
					{1,0},{-1,0},{0,1},{0,-1},
					{0.707f,0.707f},{0.707f,-0.707f},{-0.707f,0.707f},{-0.707f,-0.707f}};
				int d;
				for (d = 0; d < 8 && !landed; d++)
				{
					float rr;
					for (rr = 32.0f; rr <= 128.0f && !landed; rr += 32.0f)
					{
						vec3_t hp, dp;
						trace_t htr, dtr;
						hp[0] = pos[0] + dirs[d][0] * rr;
						hp[1] = pos[1] + dirs[d][1] * rr;
						hp[2] = pos[2];
						htr = SV_Move(pos, hmins, hmaxs, hp, MOVE_NOMONSTERS, NULL);
						if (htr.startsolid || htr.allsolid)
							break;
						dp[0] = htr.endpos[0]; dp[1] = htr.endpos[1];
						dp[2] = htr.endpos[2] - 384.0f;
						dtr = SV_Move(htr.endpos, hmins, hmaxs, dp, MOVE_NOMONSTERS, NULL);
						if (dtr.fraction >= 1.0f || dtr.plane.normal[2] <= 0.7f)
							continue;
						/* The rim must be out of every push volume, or the
						   link just re-enters the shaft. */
						for (j = 0; j < npush; j++)
						{
							edict_t *p = pushers[j];
							if (dtr.endpos[0] + hmaxs[0] < p->v.absmin[0] || dtr.endpos[0] + hmins[0] > p->v.absmax[0]) continue;
							if (dtr.endpos[1] + hmaxs[1] < p->v.absmin[1] || dtr.endpos[1] + hmins[1] > p->v.absmax[1]) continue;
							if (dtr.endpos[2] + hmaxs[2] < p->v.absmin[2] || dtr.endpos[2] + hmins[2] > p->v.absmax[2]) continue;
							break;
						}
						if (j < npush)
							continue;
						pos[0] = dtr.endpos[0];
						pos[1] = dtr.endpos[1];
						pos[2] = dtr.endpos[2];
						landed = 1;
					}
				}
				if (landed)
					break;
			}

			next[0] = pos[0] + vel[0] * dt;
			next[1] = pos[1] + vel[1] * dt;
			next[2] = pos[2] + vel[2] * dt;
			tr = SV_Move(pos, hmins, hmaxs, next, MOVE_NOMONSTERS, NULL);
			if (tr.startsolid || tr.allsolid)
			{
				stuck = 1;
				break;
			}
			if (tr.fraction < 1.0f)
			{
				if (tr.plane.normal[2] > 0.7f && vel[2] <= 0 && !inside)
				{
					landed = 1;
				}
				else
				{
					/* Slide along the surface like the engine does. */
					float bo = vel[0]*tr.plane.normal[0] + vel[1]*tr.plane.normal[1] + vel[2]*tr.plane.normal[2];
					vel[0] -= tr.plane.normal[0] * bo;
					vel[1] -= tr.plane.normal[1] * bo;
					vel[2] -= tr.plane.normal[2] * bo;
					/* Grounded against a floor with no push and no speed:
					   the ride is over even if the last hit was a wall. */
					if (!inside && fabsf(vel[0]) < 1 && fabsf(vel[1]) < 1 && fabsf(vel[2]) < 1)
						landed = 1;
				}
			}
			pos[0] = tr.endpos[0]; pos[1] = tr.endpos[1]; pos[2] = tr.endpos[2];
		}

		if (!landed)
		{
			fprintf(stderr, "Nav: push: no link for pusher at (%.0f %.0f %.0f): %s at (%.0f %.0f %.0f)\n",
				start[0], start[1], start[2], stuck ? "stuck" : "timeout",
				pos[0], pos[1], pos[2]);
			continue;
		}

		/* Suicide-chute gate, same as drops. */
		{
			vec3_t lc;
			int lcont;
			lc[0] = pos[0]; lc[1] = pos[1]; lc[2] = pos[2] - 16.0f;
			lcont = SV_PointContents(lc);
			if (lcont == CONTENTS_LAVA || lcont == CONTENTS_SLIME)
				continue;
		}

		nav_link_ensure_cap(&links, n, &cap);
		links[n].start[0] = start[0];
		links[n].start[1] = start[1];
		links[n].start[2] = start[2];
		links[n].end[0] = pos[0];
		links[n].end[1] = pos[1];
		links[n].end[2] = pos[2] - 24.0f; /* origin -> feet */
		/* Teleporter-width snap: the neutral-input sim lands short of where
		   a steering player would, so give Detour slack to find the pad. */
		links[n].radius = 128.0f;
		links[n].bidirectional = 0;
		links[n].link_type = AI_DROP;
		links[n].required_speed = 0;
		links[n].height_delta = links[n].end[2] - links[n].start[2];
		fprintf(stderr, "Nav: LINK PUSH start=(%.0f %.0f %.0f) end=(%.0f %.0f %.0f)\n",
			start[0], start[1], start[2], links[n].end[0], links[n].end[1], links[n].end[2]);
		n++;
	}

	fprintf(stderr, "Nav: push: %d pushers, %d links\n", npush, n);
	if (n == 0) { free(links); return 0; }
	*out_links = links;
	return n;
}

/* ---- Platform link detection ---- */

/* Scan plat entities (func_plat renames itself "plat" at spawn).
   Create bidirectional links between top and bottom standing surfaces.
   Bot rides the platform to traverse. */
static int nav_collect_platform_links(nav_off_mesh_link_t **out_links)
{
	int i, n = 0, cap = 16;
	nav_off_mesh_link_t *links;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));
	*out_links = links;

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		eval_t *pos1, *pos2, *spd;
		float top_z, bot_z, speed, travel;
		if (e->free) continue;
		if (strcasecmp(pr_strings + (int)e->v.classname, "plat")
			&& !(!strcasecmp(pr_strings + (int)e->v.classname, "door")
				&& nav_is_floor_collapse_door(e))) continue;

		/* pos1 = top, pos2 = bottom (QC fields, set by plat spawn code;
		   same ordering for a collapse-floor door, which opens down).
		   Link endpoints are where the bot STANDS: brush top surface
		   (pos z + maxs z), which is flush with the floor at each stop. */
		pos1 = GetEdictFieldValue(e, "pos1");
		pos2 = GetEdictFieldValue(e, "pos2");
		if (!pos1 || !pos2) continue;
		top_z = pos1->vector[2] + e->v.maxs[2];
		bot_z = pos2->vector[2] + e->v.maxs[2];

		spd = GetEdictFieldValue(e, "speed");
		speed = (spd && spd->_float > 0) ? spd->_float : 150.0f;
		travel = (top_z - bot_z) / speed;

		nav_link_ensure_cap(&links, n, &cap);

		/* Center of platform XY */
		links[n].start[0] = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
		links[n].start[1] = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
		links[n].start[2] = bot_z;
		links[n].end[0] = links[n].start[0];
		links[n].end[1] = links[n].start[1];
		links[n].end[2] = top_z;
		/* The radius is Detour's horizontal snap extent when tying each
		   endpoint to a ground poly.  The plat body itself isn't meshed,
		   so the endpoint must reach PAST the brush edge to the boarding
		   floor around it -- a fixed radius strands any plat wider than
		   2x that (e4m8's 190x206 exit lift never linked). */
		{
			float half_x = (e->v.absmax[0] - e->v.absmin[0]) * 0.5f;
			float half_y = (e->v.absmax[1] - e->v.absmin[1]) * 0.5f;
			float rad = (half_x > half_y ? half_x : half_y) + 24.0f;
			links[n].radius = rad > 64.0f ? rad : 64.0f;
		}
		links[n].bidirectional = 1;
		links[n].link_type = AI_PLAT_BOTTOM;
		links[n].height_delta = top_z - bot_z;
		links[n].wait_time = travel;
		links[n].required_speed = 0;
		if (nav_debug_cvar.value)
			Con_Printf("Nav: plat link (%.0f %.0f) z %.0f -> %.0f spd %.0f\n",
				links[n].start[0], links[n].start[1], bot_z, top_z, speed);
		n++;

		/* A collapse-floor door is baked CLOSED, so its raster walls
		   fence off the floor beneath it from the surrounding rooms.
		   When it's fully lowered its top face sits flush with the
		   adjacent floor, and the way out is a step over its rim.  Emit
		   a short exit link across each side at the lowered top level;
		   sides that face solid walls simply fail to snap and drop out. */
		if (!strcasecmp(pr_strings + (int)e->v.classname, "door"))
		{
			int side;
			float cx = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
			float cy = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
			for (side = 0; side < 4; side++)
			{
				float in_x = cx, in_y = cy, out_x = cx, out_y = cy;
				switch (side)
				{
				case 0: in_x = e->v.absmin[0] + 24; out_x = e->v.absmin[0] - 32; break;
				case 1: in_x = e->v.absmax[0] - 24; out_x = e->v.absmax[0] + 32; break;
				case 2: in_y = e->v.absmin[1] + 24; out_y = e->v.absmin[1] - 32; break;
				case 3: in_y = e->v.absmax[1] - 24; out_y = e->v.absmax[1] + 32; break;
				}
				nav_link_ensure_cap(&links, n, &cap);
				links[n].start[0] = in_x;
				links[n].start[1] = in_y;
				links[n].start[2] = bot_z;
				links[n].end[0] = out_x;
				links[n].end[1] = out_y;
				links[n].end[2] = bot_z;
				links[n].radius = 32.0f;
				links[n].bidirectional = 1;
				links[n].link_type = AI_PLAT_BOTTOM;
				links[n].height_delta = 0;
				links[n].wait_time = 0;
				links[n].required_speed = 0;
				n++;
			}
		}
	}

	*out_links = links;
	return n;
}

/* ---- Train link detection ---- */

/* Scan func_train entities.  Create links between consecutive path_corner
   stops.  Trains follow path_corner chains. */
static int nav_collect_train_links(nav_off_mesh_link_t **out_links)
{
	int i, j, n = 0, cap = 16;
	nav_off_mesh_link_t *links;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));
	*out_links = links;

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		const char *tgt;
		if (e->free) continue;
		if (strcasecmp(pr_strings + (int)e->v.classname, "train")) continue;

		/* Walk the path_corner chain */
		tgt = e->v.target ? pr_strings + (int)e->v.target : "";
		if (!tgt[0]) continue;

		/* Find first path_corner */
		for (j = 1; j < sv.num_edicts; j++)
		{
			edict_t *pc = EDICT_NUM(j);
			edict_t *next_pc;
			const char *pcname, *pctgt;
			int k;
			if (pc->free) continue;
			if (strcasecmp(pr_strings + (int)pc->v.classname, "path_corner")) continue;
			pcname = pc->v.targetname ? pr_strings + (int)pc->v.targetname : "";
			if (strcmp(tgt, pcname)) continue;

			/* Found start — follow chain, create links between stops */
			pctgt = pc->v.target ? pr_strings + (int)pc->v.target : "";
			if (!pctgt[0]) break;

			for (k = 1; k < sv.num_edicts; k++)
			{
				next_pc = EDICT_NUM(k);
				const char *nname;
				float dist;
				if (next_pc->free) continue;
				if (strcasecmp(pr_strings + (int)next_pc->v.classname, "path_corner")) continue;
				nname = next_pc->v.targetname ? pr_strings + (int)next_pc->v.targetname : "";
				if (strcmp(pctgt, nname)) continue;

				/* Create link between this path_corner and next.
				   Trains move so their MINS corner sits at the path_corner
				   (func_train_find: origin = corner - mins), so the bot
				   stands at corner + size/2 XY, corner z + size z. */
				nav_link_ensure_cap(&links, n, &cap);
				links[n].start[0] = pc->v.origin[0] + e->v.size[0] * 0.5f;
				links[n].start[1] = pc->v.origin[1] + e->v.size[1] * 0.5f;
				links[n].start[2] = pc->v.origin[2] + e->v.size[2];
				links[n].end[0] = next_pc->v.origin[0] + e->v.size[0] * 0.5f;
				links[n].end[1] = next_pc->v.origin[1] + e->v.size[1] * 0.5f;
				links[n].end[2] = next_pc->v.origin[2] + e->v.size[2];
				dist = sqrt((links[n].end[0]-links[n].start[0])*(links[n].end[0]-links[n].start[0])
					+ (links[n].end[1]-links[n].start[1])*(links[n].end[1]-links[n].start[1])
					+ (links[n].end[2]-links[n].start[2])*(links[n].end[2]-links[n].start[2]));
				links[n].radius = 64.0f;
				links[n].bidirectional = 0;
				links[n].link_type = AI_RIDE_TRAIN;
				links[n].height_delta = next_pc->v.origin[2] - pc->v.origin[2];
				links[n].wait_time = dist / (100.0f);
				links[n].required_speed = 0;
				n++;
				break;
			}
			break; /* only process first path_corner match */
		}
	}

	*out_links = links;
	return n;
}

/* ---- Door link detection ---- */

/* ---- Physics-based link detection ---- */

extern cvar_t sv_gravity;
extern cvar_t sv_maxspeed;

/* Helper: add a link, growing the array if needed. */
static void nav_link_push(nav_off_mesh_link_t **links, int *n, int *cap,
	const float *start, const float *end, int type, float speed, float dz)
{
	if (*n >= *cap)
	{
		*cap *= 2;
		*links = (nav_off_mesh_link_t *)realloc(*links, (size_t)*cap * sizeof(**links));
	}
	nav_off_mesh_link_t *l = &(*links)[*n];
	l->start[0] = start[0]; l->start[1] = start[1]; l->start[2] = start[2];
	l->end[0] = end[0]; l->end[1] = end[1]; l->end[2] = end[2];
	l->radius = NAV_JUMP_LINK_RADIUS;
	l->bidirectional = 0;
	l->link_type = type;
	l->height_delta = dz;
	l->required_speed = speed;
	l->wait_time = 0;
	fprintf(stderr, "Nav: LINK %s start=(%.0f %.0f %.0f) end=(%.0f %.0f %.0f) dz=%.0f spd=%.0f\n",
		type == AI_JUMP ? "JUMP" : type == AI_DROP ? "DROP" : type == AI_SUPER_JUMP ? "RJ" :
			type == AI_SURFACE ? "SURF" : type == AI_WALK ? "WALK" : "???",
		start[0], start[1], start[2], end[0], end[1], end[2], dz, speed);
	(*n)++;
}

/* Scan outward from edge along normal in cell_size steps to find
   where a floor at target_z begins.  Returns horizontal distance,
   or 0 if the floor is directly below the edge. */
static float nav_find_horizontal_gap(const nav_heightfield_t *hf,
	const float *edge, const float *normal, float target_z, float max_dist)
{
	float step = 4.0f; /* cell_size */
	float probe[3];

	for (float d = step; d <= max_dist; d += step)
	{
		probe[0] = edge[0] + normal[0] * d;
		probe[1] = edge[1] + normal[1] * d;
		probe[2] = edge[2];

		float found_z;
		if (nav_heightfield_floor_z(hf, probe, target_z, &found_z))
		{
			if (fabsf(found_z - target_z) < 8.0f)
				return d;
		}
	}
	return 0;
}

/* Callback for nav_mesh_build: detect drop/jump links from boundary edges.
   Uses Quake physics to compute landing positions and required approach speeds.
   One edge can produce multiple links (one per reachable floor below). */
static int nav_link_callback(
	const nav_mesh_boundary_edge_t *edges, int edge_count,
	const nav_heightfield_t *hf,
	nav_off_mesh_link_t **out_links,
	void *user_data)
{
	(void)user_data;
	int n = 0, cap = 128, i;
	nav_off_mesh_link_t *links = NULL;
	float gravity = sv_gravity.value;
	float maxspeed = sv_maxspeed.value;

	if (gravity < 1.0f) gravity = 800.0f;
	if (maxspeed < 1.0f) maxspeed = 320.0f;
	float peak = NAV_JUMP_IMPULSE * NAV_JUMP_IMPULSE / (2.0f * gravity);

	*out_links = NULL;
	if (edge_count == 0) return 0;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));

	for (i = 0; i < edge_count; i++)
	{
		const float *mid = edges[i].midpoint;
		const float *norm = edges[i].normal;
		float short_probe[3];

		/* Short probe outward (8u) to check if there's a wall right at the edge */
		short_probe[0] = mid[0] + norm[0] * 8.0f;
		short_probe[1] = mid[1] + norm[1] * 8.0f;
		short_probe[2] = mid[2];
		if (!nav_trace_clear_at_height(mid, short_probe, mid[2] + 24.0f, NULL))
		{
			continue;
		}

		/* Hull standability: hull-1 extraction emits bevel faces that
		   rasterize as thin phantom shelves (e1m3 z=-134: 4u thick, 70u
		   above the real floor) — mesh and links form on floor that
		   cannot support a player.  Landings are fall-column traced
		   already; starts are not.  Sweep the PLAYER HULL down at the
		   edge: real hull-1 floor catches it (including legit 16u brush
		   overhang), a phantom shelf lets it fall through.  startsolid
		   is inconclusive (cramped rims under stairs, dm4 pocket) —
		   only a CLEAN miss proves there is no floor. */
		{
			vec3_t ds, de, pmins = {-16, -16, -24}, pmaxs = {16, 16, 32};
			trace_t tr;
			ds[0] = mid[0]; ds[1] = mid[1]; ds[2] = mid[2] + 26.0f;
			de[0] = mid[0]; de[1] = mid[1]; de[2] = mid[2];
			tr = SV_Move(ds, pmins, pmaxs, de, MOVE_NOMONSTERS, NULL);
			if (!tr.startsolid && !tr.allsolid && tr.fraction >= 1.0f)
			{
				continue;
			}
		}

		/* ---- Drops: find all floors below ---- */
		{
			float floors[8];
			/* Search as deep as a water plunge allows; each floor past the
			   dry cap is kept only if it's underwater (gated below). */
			float min_z = mid[2] - NAV_WATER_DROP_HEIGHT_MAX;
			float max_z = mid[2] - NAV_DROP_HEIGHT_MIN;
			int nfloors = 0;
			/* Probe outward too: a thin unwalkable ridge at the boundary
			   (hull bevel artifact) hides the landing from the edge column. */
			const float probe_offs[3] = {0.0f, 12.0f, 20.0f};
			for (int pi = 0; pi < 3; pi++)
			{
				float pp[3], pf[8];
				int pn, ti;
				pp[0] = mid[0] + norm[0] * probe_offs[pi];
				pp[1] = mid[1] + norm[1] * probe_offs[pi];
				pp[2] = mid[2];
				pn = nav_heightfield_floors_below(hf, pp, max_z, min_z, pf, 8);
				for (ti = 0; ti < pn && nfloors < 8; ti++)
				{
					int dup = 0;
					for (int di = 0; di < nfloors; di++)
						if (fabsf(floors[di] - pf[ti]) < 2.0f) { dup = 1; break; }
					if (!dup)
						floors[nfloors++] = pf[ti];
				}
			}

			for (int fi = 0; fi < nfloors; fi++)
			{
				float drop_height = mid[2] - floors[fi];
				if (drop_height < NAV_DROP_HEIGHT_MIN)
					continue;

				/* Past the dry-land cap, only a water landing is survivable
				   (and escapable via the surface link); anything else is a
				   killing fall or a dry pit-trap -- leave it unlinked. */
				int deep_water_drop = 0;
				if (drop_height > NAV_DROP_HEIGHT_MAX)
				{
					vec3_t wc;
					wc[0] = mid[0]; wc[1] = mid[1]; wc[2] = floors[fi] + 24.0f;
					if (SV_PointContents(wc) != CONTENTS_WATER)
						continue;
					deep_water_drop = 1;
				}

				/* Verify drop is physically possible: check that there's no solid
				   span blocking the fall at a point outward from the edge.
				   Uses heightfield (no BSP height offset issues). */
				{
					float probe_xy[3];
					probe_xy[0] = mid[0] + norm[0] * 16.0f;
					probe_xy[1] = mid[1] + norm[1] * 16.0f;
					probe_xy[2] = 0;
					if (nav_heightfield_is_blocked(hf, probe_xy, mid[2]))
					{
						continue; /* wall at edge height blocks the drop */
					}
					if (nav_heightfield_is_blocked(hf, probe_xy, floors[fi]))
					{
						continue; /* solid at landing height */
					}
				}

				/* Lane step check: the bot RUNS from the edge to the fall
				   point — anything taller than a step (18u) in the lane
				   stops it on the ground (point traces at +24 sail over
				   20u parapets).  Solid spans reaching above mid+18 at any
				   lane sample mean the run is impossible. */
				{
					int lane_blocked = 0;
					for (float loff = 0.0f; loff <= 16.0f; loff += 4.0f)
					{
						float lp[3];
						lp[0] = mid[0] + norm[0] * loff;
						lp[1] = mid[1] + norm[1] * loff;
						lp[2] = 0;
						if (nav_heightfield_is_blocked(hf, lp, mid[2] + 18.0f))
						{
							lane_blocked = 1;
							break;
						}
					}
					if (lane_blocked)
					{
						continue;
					}
				}

				/* Approach footing: the bot runs up to the edge from
				   behind, so there must be REAL floor there.  The
				   heightfield can't tell — phantom extraction shelves
				   live in it (e1m3 (36,-388,-134): bot walks the shelf
				   toward the rim, falls through into a 64u pit it can't
				   path out of).  Sweep the player hull down at each
				   approach point: real hull-1 floor catches it, a
				   phantom shelf is a clean miss.  startsolid alone is
				   inconclusive (cramped rim, dm4 pocket) and counts as
				   footing; allsolid means the approach corridor is
				   inside hull-1 wall — the bot can never stand there
				   (e1m3 trap rim: allsolid at every offset, yet the
				   heightfield shows a walkable shelf). */
				{
					int footing = 0;
					for (float boff = 12.0f; boff <= 28.0f; boff += 8.0f)
					{
						vec3_t ds, de, pmins = {-16, -16, -24}, pmaxs = {16, 16, 32};
						trace_t tr;
						ds[0] = mid[0] - norm[0] * boff;
						ds[1] = mid[1] - norm[1] * boff;
						ds[2] = mid[2] + 26.0f;
						de[0] = ds[0]; de[1] = ds[1]; de[2] = mid[2];
						tr = SV_Move(ds, pmins, pmaxs, de, MOVE_NOMONSTERS, NULL);
						if (!tr.allsolid && (tr.startsolid || tr.fraction < 1.0f))
						{
							footing = 1;
							break;
						}
					}
					if (!footing)
					{
						continue;
					}
				}

				/* Fall time from physics: t = sqrt(2h / g) */
				float fall_time = sqrtf(2.0f * drop_height / gravity);

				/* Find horizontal gap to the landing surface */
				float gap = nav_find_horizontal_gap(hf, mid, norm, floors[fi], 128.0f);

				/* Required speed to clear the gap */
				float speed;
				if (gap < 4.0f)
					speed = 10.0f; /* step-off: minimal speed */
				else
					speed = gap / fall_time;

				/* Half run speed, not max: the steering corner sits AT the
				   edge, so the bot may arrive slow with no runway — a gap
				   needing a flat-out sprint drops it into the chasm instead
				   of the landing (dm1 (142,1558): 108u gap, 212 u/s). */
				if (speed > maxspeed * 0.5f)
					continue; /* needs more run-up than traversal guarantees */

				/* Landing position: edge + normal * landing_dist */
				float land_dist = speed * fall_time;
				/* Add small margin (8u) past the gap edge */
				if (gap > 4.0f)
					land_dist = gap + 8.0f;

				float end[3];
				end[0] = mid[0] + norm[0] * land_dist;
				end[1] = mid[1] + norm[1] * land_dist;
				end[2] = floors[fi];

				/* Verify landing is clear */
				if (hf && nav_heightfield_is_blocked(hf, end, floors[fi]))
				{
					continue;
				}

				/* Full-length BSP traces.  The 16u heightfield probes above
				   miss thick walls (dm4 x=192 wall: link probed through it)
				   and landings tucked under the start floor. */
				if (!nav_trace_clear_at_height(mid, end, mid[2] + 24.0f, NULL))
				{
					continue;
				}
				{
					vec3_t fs, fe, zero3 = {0, 0, 0};
					trace_t tr;
					fs[0] = end[0]; fs[1] = end[1]; fs[2] = mid[2] + 24.0f;
					fe[0] = end[0]; fe[1] = end[1]; fe[2] = floors[fi] + 4.0f;
					tr = SV_Move(fs, zero3, zero3, fe, MOVE_NOMONSTERS, NULL);
					if (tr.startsolid || tr.allsolid || tr.fraction < 1.0f)
					{
						continue; /* fall column obstructed */
					}
				}

				/* Hull-truth the fall column.  The point/voxel checks above
				   pass through slots the heightfield sees as open but a
				   player can't fit (e1m1 pool ledge: sub-hull gap between
				   walkway and wall spawned a phantom drop that pinned bots
				   on the lip).  Sweep the real player hull down; it can't
				   sit closer than its half-width to the drop face, so clamp
				   the column at least 18u out — step-off landings end up
				   there anyway once the hull clips the wall. */
				{
					vec3_t fs, fe, hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
					trace_t tr;
					float col_dist = land_dist;
					if (col_dist < 18.0f)
						col_dist = 18.0f;
					fs[0] = mid[0] + norm[0] * col_dist;
					fs[1] = mid[1] + norm[1] * col_dist;
					fs[2] = mid[2] + 26.0f;
					fe[0] = fs[0]; fe[1] = fs[1]; fe[2] = floors[fi] + 24.0f;
					tr = SV_Move(fs, hmins, hmaxs, fe, MOVE_NOMONSTERS, NULL);
					if (tr.startsolid || tr.allsolid
						|| tr.endpos[2] > floors[fi] + 36.0f)
					{
						continue; /* player hull can't ride the column down */
					}
				}

				/* Never link a drop that lands in lava or slime: it's a suicide
				   chute (dm4 green-armor ledge dropped bots into the lava once the
				   192u cap reached it).  Water is survivable, gate only the deadly. */
				{
					vec3_t lc;
					int lcont;
					lc[0] = end[0]; lc[1] = end[1]; lc[2] = floors[fi] + 8.0f;
					lcont = SV_PointContents(lc);
					if (lcont == CONTENTS_LAVA || lcont == CONTENTS_SLIME)
						continue;
				}

				nav_link_push(&links, &n, &cap, mid, end, AI_DROP, speed, -drop_height);

				/* Deep water plunge: pair it with an AI_SURFACE swim-out so the
				   pool isn't a one-way grave.  Only when the ledge sits nearly
				   straight above the landing (the bot swims up and steps off);
				   a far ledge would just nose the lip underwater. */
				if (deep_water_drop && land_dist <= 48.0f)
					nav_link_push(&links, &n, &cap, end, mid, AI_SURFACE, 0.0f, drop_height);

				/* Reverse: if drop height is within jump reach, also create
				   a jump link from the landing floor back up to the edge.
				   The bot jumps from below the ledge up to the top.
				   No reverse jump out of liquid: jump impulse doesn't
				   apply while swimming, so bots just nose the lip (dm5
				   pool, 20 stalls/run).  Surface links handle water exit. */
				int land_in_liquid;
				{
					vec3_t lc;
					lc[0] = end[0]; lc[1] = end[1]; lc[2] = floors[fi] + 24.0f;
					land_in_liquid = (SV_PointContents(lc) <= CONTENTS_WATER);
				}
				if (!land_in_liquid &&
					drop_height >= NAV_JUMP_HEIGHT_MIN && drop_height <= peak)
				{
					float disc = NAV_JUMP_IMPULSE * NAV_JUMP_IMPULSE - 2.0f * gravity * drop_height;
					if (disc >= 0)
					{
						float time_up = (NAV_JUMP_IMPULSE - sqrtf(disc)) / gravity;
						float jspeed = land_dist / time_up;
						if (jspeed <= maxspeed)
						{
							if (jspeed < 10.0f) jspeed = 10.0f;
							/* Jump: start at landing, end at edge */
							nav_link_push(&links, &n, &cap, end, mid, AI_JUMP, jspeed, drop_height);
						}
					}
				}
				/* Micro-drop (under step height): not a ledge, a STAIR the
				   raster fused shut -- the engine walks a player back up an
				   18u step for free.  When the landing floor (nearly) butts
				   the edge, pair the drop with a reverse WALK so the step
				   works both ways.  Without it a step-tall sill severed by
				   a 1-cell raster wall is enterable only outbound (end's
				   doorway threshold: the quad pen's only entrance).  The
				   32u-wide hull spans a crevice up to half its width while
				   stepping, and the raster wall itself reads as a 4-8u
				   "gap" here, so allow up to 16u.  Liquid landings keep
				   using surface links. */
				else if (!land_in_liquid && drop_height < NAV_JUMP_HEIGHT_MIN
					&& gap <= 16.0f)
				{
					nav_link_push(&links, &n, &cap, end, mid, AI_WALK, 10.0f, drop_height);
				}
			}
		}

		/* ---- Jumps: scan outward for floors above within jump reach ----
		   Disabled for now: inverted edge normals kept this scan inert
		   since it was written (0 links on every map), so enabling it
		   alongside the normal fix is an untested behavior change —
		   first suite with it live regressed e1m2/e1m3.  Re-enable as
		   its own change with its own suite run. */
#if 0
		{
			float step;

			/* Probe at multiple distances outward (8u to 64u in cell_size steps) */
			for (step = 8.0f; step <= 64.0f; step += 4.0f)
			{
				float probe[3];
				probe[0] = mid[0] + norm[0] * step;
				probe[1] = mid[1] + norm[1] * step;
				probe[2] = mid[2];

				/* Check for wall at jump apex height — the bot jumps OVER the edge */
				if (!nav_trace_clear_at_height(mid, probe, mid[2] + peak + NAV_PLAYER_FLOOR_OFFSET, NULL))
					break; /* wall at jump height — no point probing further */

				/* Find a floor ABOVE the edge at the probe point. */
				float land_z;
				if (!nav_heightfield_floor_above(hf, probe,
						mid[2] + NAV_JUMP_HEIGHT_MIN, mid[2] + peak, &land_z))
					continue;

				float jump_height = land_z - mid[2];
				if (jump_height < NAV_JUMP_HEIGHT_MIN || jump_height > peak)
					continue;

				/* Time to reach jump_height:
				   h = v0*t - 0.5*g*t^2  →  t = (v0 - sqrt(v0^2 - 2*g*h)) / g */
				float disc = NAV_JUMP_IMPULSE * NAV_JUMP_IMPULSE - 2.0f * gravity * jump_height;
				if (disc < 0) continue;
				float time_up = (NAV_JUMP_IMPULSE - sqrtf(disc)) / gravity;
				float speed = step / time_up;

				if (speed > maxspeed)
					continue;
				if (speed < 10.0f) speed = 10.0f;

				float end[3];
				end[0] = probe[0];
				end[1] = probe[1];
				end[2] = land_z;

				nav_link_push(&links, &n, &cap, mid, end, AI_JUMP, speed, jump_height);
				break; /* found a jump at this edge, don't create duplicates */
			}
		}
#endif
	}

	/* Log wall rejection stats for lower corridor */
	{
		int lower_total = 0, lower_wall = 0;
		for (int ei = 0; ei < edge_count; ei++)
		{
			if (edges[ei].midpoint[2] < -100 && edges[ei].midpoint[2] > -160)
			{
				lower_total++;
				float sp[3];
				sp[0] = edges[ei].midpoint[0] + edges[ei].normal[0] * 8.0f;
				sp[1] = edges[ei].midpoint[1] + edges[ei].normal[1] * 8.0f;
				sp[2] = edges[ei].midpoint[2];
				if (!nav_trace_clear_at_height(edges[ei].midpoint, sp, edges[ei].midpoint[2] + 24.0f, NULL))
					lower_wall++;
			}
		}
		fprintf(stderr, "Nav: lower corridor edges: %d total, %d wall-rejected, %d pass\n",
			lower_total, lower_wall, lower_total - lower_wall);
	}

	/* Log edge height distribution */
	{
		int z_hist[20];
		memset(z_hist, 0, sizeof(z_hist));
		for (int ei = 0; ei < edge_count; ei++)
		{
			int bucket = (int)((edges[ei].midpoint[2] + 400) / 50);
			if (bucket >= 0 && bucket < 20) z_hist[bucket]++;
		}
		fprintf(stderr, "Nav: edge Z distribution (%d edges):", edge_count);
		for (int bi = 0; bi < 20; bi++)
			if (z_hist[bi]) fprintf(stderr, " [%.0f]=% d", bi * 50.0f - 400, z_hist[bi]);
		fprintf(stderr, "\n");
	}

	*out_links = links;
	{
		int nj = 0, nd = 0, nr = 0, li;
		for (li = 0; li < n; li++)
		{
			if (links[li].link_type == AI_JUMP) nj++;
			else if (links[li].link_type == AI_DROP) nd++;
			else if (links[li].link_type == AI_SUPER_JUMP) nr++;
		}
		fprintf(stderr, "Nav: detected %d links from %d edges (%d jump, %d drop, %d rj)\n",
			n, edge_count, nj, nd, nr);
	}
	return n;
}

/* Cache item poly refs — called once after navmesh build.
   Uses findNearestPoly with tight 2u extents so each item maps
   to exactly the poly it's standing on. */
static void nav_cache_item_polys(void)
{
	dtNavMeshQuery *query = nav_mesh->query;
	dtQueryFilter filter;
	float tight[3] = {2.0f, 56.0f, 2.0f}; /* 2u XZ, walkable_height Y */
	int i;

	nav_item_count = 0;
	if (!query) return;

	nav_mesh_setup_filter(&filter);

	for (i = 1; i < sv.num_edicts && nav_item_count < NAV_MAX_ITEMS; i++)
	{
		edict_t *e = EDICT_NUM(i);
		if (e->free) continue;

		eval_t *val = GetEdictFieldValue(e, "item_res");
		if (!val || val->_float < 1) continue;

		float qpos[3], rpos[3];
		nav_ent_pos(e, qpos);
		nav_quake_to_recast(qpos, rpos);

		dtPolyRef ref = 0;
		float nearest[3];
		query->findNearestPoly(rpos, tight, &filter, &ref, nearest);

		if (ref != 0)
		{
			nav_item_cache[nav_item_count].ent = e;
			nav_item_cache[nav_item_count].poly_ref = ref;
			memcpy(nav_item_cache[nav_item_count].nav_pos, nearest, sizeof(float) * 3);
			nav_item_count++;
		}
	}
	if (nav_item_count >= NAV_MAX_ITEMS)
		Con_Printf("Nav: WARNING item cache full (%d), some items skipped\n", NAV_MAX_ITEMS);
	Con_Printf("Nav: cached %d item poly refs\n", nav_item_count);
}

/* Merge multiple link arrays into one. */
static int nav_merge_links(
	nav_off_mesh_link_t *a, int a_count,
	nav_off_mesh_link_t *b, int b_count,
	nav_off_mesh_link_t **out)
{
	int total = a_count + b_count;
	nav_off_mesh_link_t *merged;
	if (total == 0) { *out = NULL; return 0; }
	merged = (nav_off_mesh_link_t *)calloc(total, sizeof(*merged));
	if (a_count > 0) memcpy(merged, a, a_count * sizeof(*merged));
	if (b_count > 0) memcpy(merged + a_count, b, b_count * sizeof(*merged));
	*out = merged;
	return total;
}

/* ---- Build ---- */

void Nav_BuildForMap(void)
{
	nav_mesh_build_config_t config;
	nav_mesh_summary_t summary;
	nav_off_mesh_link_t *entity_links;
	int entity_count;
	float *verts = NULL;
	int vert_count = 0;
	int *tris = NULL;
	int tri_count = 0;
	char error[256];
	double t_start, t_done;

	Nav_Shutdown();
	if (sv.worldmodel == NULL) return;

	t_start = Sys_FloatTime();

	/* Doors are held open before extraction so their bodies rasterize at
	   the OPEN position: a passage door's open body tucks into its wall
	   slot (no mesh change), while a step/bridge door's open body IS the
	   walk surface players use (e2m3's shoot-button step).  Traces during
	   link validation then match the raster exactly. */
	nav_doors_open_for_build();

	if (!nav_extract_bsp(sv.worldmodel, &verts, &vert_count, &tris, &tri_count))
	{
		Con_Printf("Nav: BSP extraction failed\n");
		nav_doors_restore();
		return;
	}
	Con_Printf("Nav: BSP extracted %d verts, %d tris\n", vert_count, tri_count);

	/* Log BSP bounds */
	{
		int bi;
		float bmin[3] = {999999, 999999, 999999};
		float bmax[3] = {-999999, -999999, -999999};
		for (bi = 0; bi < vert_count; bi++)
		{
			if (verts[bi*3+0] < bmin[0]) bmin[0] = verts[bi*3+0];
			if (verts[bi*3+1] < bmin[1]) bmin[1] = verts[bi*3+1];
			if (verts[bi*3+2] < bmin[2]) bmin[2] = verts[bi*3+2];
			if (verts[bi*3+0] > bmax[0]) bmax[0] = verts[bi*3+0];
			if (verts[bi*3+1] > bmax[1]) bmax[1] = verts[bi*3+1];
			if (verts[bi*3+2] > bmax[2]) bmax[2] = verts[bi*3+2];
		}
		Con_Printf("Nav: BSP bounds (%.0f %.0f %.0f) - (%.0f %.0f %.0f)\n",
			bmin[0], bmin[1], bmin[2], bmax[0], bmax[1], bmax[2]);
	}

	nav_default_config(&config);

	/* Collect all entity-based links */
	entity_count = 0;
	entity_links = NULL;

	entity_count = nav_collect_teleporters(&entity_links);
	{
		nav_off_mesh_link_t *plat_links = NULL, *train_links = NULL, *push_links = NULL;
		int plat_count = nav_collect_platform_links(&plat_links);
		int train_count = nav_collect_train_links(&train_links);
		int push_count = nav_collect_push_links(&push_links);
		if (plat_count + train_count + push_count > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + plat_count + train_count + push_count) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, plat_links,
				(size_t)plat_count * sizeof(*entity_links));
			memcpy(entity_links + entity_count + plat_count, train_links,
				(size_t)train_count * sizeof(*entity_links));
			memcpy(entity_links + entity_count + plat_count + train_count, push_links,
				(size_t)push_count * sizeof(*entity_links));
		}
		free(plat_links);
		free(train_links);
		free(push_links);
		Con_Printf("Nav: %d teleporter, %d plat, %d train, %d push links\n",
			entity_count, plat_count, train_count, push_count);
		entity_count += plat_count + train_count + push_count;
	}

	/* Level exits: their component is escapable by definition (you leave
	   the level), so the deep-drop no-trap gate must accept it. */
	{
		float exits[32][3];
		int nexits = 0, ei;
		for (ei = 1; ei < sv.num_edicts && nexits < 32; ei++)
		{
			edict_t *e = EDICT_NUM(ei);
			if (e->free) continue;
			if (strcasecmp(pr_strings + (int)e->v.classname, "trigger_changelevel")) continue;
			exits[nexits][0] = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
			exits[nexits][1] = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
			exits[nexits][2] = (e->v.absmin[2] + e->v.absmax[2]) * 0.5f;
			nexits++;
		}
		nav_mesh_set_exit_points(&exits[0][0], nexits);
		if (nexits > 0)
			fprintf(stderr, "Nav: %d level-exit points registered as escapable\n", nexits);
	}

	/* Single-pass build: entity links provided upfront, jump/drop links
	   detected mid-build via callback after contours are ready. */
	memset(&summary, 0, sizeof(summary));
	memset(error, 0, sizeof(error));
	nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
		&config, entity_links, entity_count, &summary,
		nav_jump_links_cvar.value ? nav_link_callback : NULL, NULL,
		error, sizeof(error));

	if (nav_mesh == NULL)
	{
		Con_Printf("Nav: build failed: %s\n", error);
		nav_doors_restore();
		free(verts); free(tris); free(entity_links);
		return;
	}

	/* Second pass: reconnect areas stranded by a jump-up the contour scan
	   can't reliably find (dm4 quad shelf).  Generate hull-validated jump
	   links only where they reconnect an orphan, then rebuild with them.
	   Targeted, so unlike a broad edge scan it can't spray false jumps. */
	if (nav_jump_links_cvar.value)
	{
		nav_off_mesh_link_t *ojumps = NULL;
		int noj = nav_mesh_compute_orphan_jumps(nav_mesh, nav_link_validate, NULL, &ojumps);
		if (noj > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + noj) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, ojumps, (size_t)noj * sizeof(*entity_links));
			entity_count += noj;
			free(ojumps);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	/* Third pass: complete one-way connectivity.  Runs on the mesh that
	   already has teleport/plat/orphan links, so it can see which areas can
	   only be entered or only exited, and add the missing direction. */
	if (nav_mesh != NULL && nav_directed_links_cvar.value)
	{
		nav_off_mesh_link_t *dlinks = NULL;
		int nd = nav_mesh_compute_directed_links(nav_mesh, nav_link_validate, NULL, &dlinks);
		if (nd > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + nd) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, dlinks, (size_t)nd * sizeof(*entity_links));
			entity_count += nd;
			free(dlinks);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	/* Fourth pass: bridge local connectivity gaps.  Runs last, on the mesh
	   with every other link in place, so its findPath gate sees the real
	   graph and only adds a run-jump where two ledges still have no route
	   between them (dm3 wp62->wp63). */
	if (nav_mesh != NULL && nav_gap_jumps_cvar.value)
	{
		nav_off_mesh_link_t *glinks = NULL;
		int ng = nav_mesh_compute_gap_jumps(nav_mesh, nav_link_validate, NULL, &glinks);
		if (ng > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + ng) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, glinks, (size_t)ng * sizeof(*entity_links));
			entity_count += ng;
			free(glinks);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	/* Fifth pass: rocket-jump links to high ledges out of run-jump reach,
	   gated so the high end can already get back down (no launcher-less
	   trap).  Runs last so its reachability gate sees every other link. */
	if (nav_mesh != NULL && nav_rocket_jumps_cvar.value)
	{
		nav_off_mesh_link_t *rlinks = NULL;
		int nr = nav_mesh_compute_rocket_jumps(nav_mesh, nav_link_validate, NULL, &rlinks);
		if (nr > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + nr) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, rlinks, (size_t)nr * sizeof(*entity_links));
			entity_count += nr;
			free(rlinks);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	/* Sixth pass: bidirectional swim links across components whose rims
	   connect through fully-submerged water (e4m8 canal tunnel, end's
	   underwater ledge).  Runs before the deep drops: a water landing's
	   only way OUT is often a swim (e3m5's hole floor swims to the wind
	   tunnel), and the drop pass's no-trap gate must see that route. */
	if (nav_mesh != NULL && nav_swim_links_cvar.value)
	{
		nav_off_mesh_link_t *swlinks = NULL;
		int nsw = nav_mesh_compute_swim_links(nav_mesh, nav_swim_link_validate, NULL, &swlinks);
		if (nsw > 0)
		{
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + nsw) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, swlinks, (size_t)nsw * sizeof(*entity_links));
			entity_count += nsw;
			free(swlinks);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	/* Seventh pass: deep one-way drops (past the boundary detector's 192u
	   cap) into lower regions with no other way in, gated on the landing
	   already having a way back OUT.  Runs after every other link pass so
	   both its no-access and no-trap findPath gates see the real graph. */
	if (nav_mesh != NULL && nav_deep_drops_cvar.value)
	{
		/* Iterate to fixpoint: a pit whose only exit is itself a deep drop
		   (e3m5's ogre platform over the hole) fails the no-trap gate on the
		   first round -- its escape link doesn't exist yet.  Once round 1
		   adds the outbound drop, round 2 can accept the inbound one.  The
		   no-access gate skips already-linked pairs, so rounds never
		   duplicate. */
		int ddround;
		for (ddround = 0; ddround < 3; ddround++)
		{
			nav_off_mesh_link_t *ddlinks = NULL;
			int ndd = nav_mesh_compute_deep_drops(nav_mesh, nav_deep_drop_validate, NULL, &ddlinks);
			if (ndd <= 0)
				break;
			entity_links = (nav_off_mesh_link_t *)realloc(entity_links,
				(size_t)(entity_count + ndd) * sizeof(*entity_links));
			memcpy(entity_links + entity_count, ddlinks, (size_t)ndd * sizeof(*entity_links));
			entity_count += ndd;
			free(ddlinks);

			nav_mesh_destroy(nav_mesh);
			memset(&summary, 0, sizeof(summary));
			memset(error, 0, sizeof(error));
			nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
				&config, entity_links, entity_count, &summary,
				nav_link_callback, NULL, error, sizeof(error));
			if (nav_mesh == NULL)
			{
				Con_Printf("Nav: rebuild failed: %s\n", error);
				nav_doors_restore();
				free(verts); free(tris); free(entity_links);
				return;
			}
		}
	}

	nav_doors_restore();

	free(verts);
	free(tris);
	free(entity_links);

	nav_build_block_map();

	t_done = Sys_FloatTime();
	Con_Printf("Nav: %.3fs, %d polys, %d entity links\n",
		t_done - t_start, summary.polygon_count, entity_count);

	/* Validate navmesh against hand-crafted waypoints */
	if (nav_mesh != NULL)
		Nav_Validate(nav_mesh, sv.name);

	/* ---- Height diagnostic: compare player-origin, floor, and navmesh surface ---- */
	if (nav_mesh != NULL)
	{
		int si;
		for (si = 1; si < sv.num_edicts; si++)
		{
			edict_t *e = EDICT_NUM(si);
			if (e->free) continue;
			const char *cn = pr_strings + (int)e->v.classname;
			if (strcmp(cn, "info_player_deathmatch") != 0 &&
				strcmp(cn, "info_player_start") != 0)
				continue;

			/* BSP floor trace: point trace down from spawn origin */
			vec3_t ts, te, zero = {0, 0, 0};
			trace_t trace;
			float entity_floor = nav_player_floor_z(e->v.origin[2]);
			ts[0] = e->v.origin[0]; ts[1] = e->v.origin[1]; ts[2] = e->v.origin[2] + 8;
			te[0] = e->v.origin[0]; te[1] = e->v.origin[1]; te[2] = e->v.origin[2] - 128;
			trace = SV_Move(ts, zero, zero, te, MOVE_NOMONSTERS, NULL);
			float bsp_floor = trace.endpos[2];

			/* Also trace with player hull to see hull floor */
			vec3_t hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			ts[2] = e->v.origin[2] + 32;
			trace = SV_Move(ts, hmins, hmaxs, te, MOVE_NOMONSTERS, NULL);
			float hull_floor = trace.endpos[2];

			/* Navmesh surface at the player floor, not at the authored origin. */
			nav_mesh_nearest_result_t nr;
			char nerr[64];
			vec3_t floor_probe;
			floor_probe[0] = e->v.origin[0];
			floor_probe[1] = e->v.origin[1];
			floor_probe[2] = entity_floor;
			if (nav_mesh_find_nearest(nav_mesh, floor_probe, &nr, nerr, sizeof(nerr)))
			{
				fprintf(stderr, "Nav: SURFACE %s at (%.0f %.0f) origin_z=%.1f player_floor=%.1f bsp_floor=%.1f hull_floor=%.1f nav_surface=%.1f origin_delta=%.1f floor_delta=%.1f hull_delta=%.1f\n",
					cn, e->v.origin[0], e->v.origin[1],
					e->v.origin[2], entity_floor, bsp_floor, hull_floor, nr.nearest_point[2],
					e->v.origin[2] - nr.nearest_point[2],
					entity_floor - nr.nearest_point[2],
					hull_floor - nr.nearest_point[2]);
			}
			if (si > 20) break; /* limit output */
		}
	}

	/* Triage probe: NAV_PATH_TEST="x1 y1 z1 x2 y2 z2[;...]" runs findPath
	   between arbitrary points on the final mesh and prints the outcome. */
	if (nav_mesh != NULL)
	{
		const char *pt = getenv("NAV_PATH_TEST");
		while (pt != NULL && *pt)
		{
			vec3_t a, b;
			if (sscanf(pt, "%f %f %f %f %f %f", &a[0], &a[1], &a[2], &b[0], &b[1], &b[2]) == 6)
			{
				nav_mesh_path_result_t res;
				char perr[128] = "";
				int ok = nav_mesh_find_path(nav_mesh, a, b, &res, perr, sizeof(perr));
				fprintf(stderr, "Nav: PATHTEST (%.0f %.0f %.0f)->(%.0f %.0f %.0f): %s (%s) end=(%.0f %.0f %.0f)\n",
					a[0], a[1], a[2], b[0], b[1], b[2],
					ok ? "OK" : "FAIL", perr,
					res.end_point[0], res.end_point[1], res.end_point[2]);
				if (getenv("NAV_PATH_TEST_VERBOSE") != NULL)
				{
					int pi;
					for (pi = 0; pi < res.path_ref_count; pi++)
					{
						float c[3];
						int pk = nav_mesh_poly_center_by_ref(nav_mesh, res.path_refs[pi], c);
						if (pk)
							fprintf(stderr, "Nav: PATHTEST   [%d] (%.0f %.0f %.0f)%s\n",
								pi, c[0], c[1], c[2], pk == 2 ? " OFFMESH" : "");
					}
				}
			}
			pt = strchr(pt, ';');
			if (pt) pt++;
		}
	}

	/* ---- Oracle-free connectivity check: reachability from spawns to
	   every other spawn and every item, via the same pathfinder bots use.
	   No hand-authored waypoints required, so this runs on any map. ---- */
	if (nav_mesh != NULL)
	{
		struct nav_conn_entry { const char *cn; float pos[3]; };
		std::vector<nav_conn_entry> spawns;
		std::vector<nav_conn_entry> items;
		int si;

		for (si = 1; si < sv.num_edicts; si++)
		{
			edict_t *e = EDICT_NUM(si);
			if (e->free) continue;
			const char *cn = pr_strings + (int)e->v.classname;
			int is_spawn = (strcmp(cn, "info_player_deathmatch") == 0 ||
				strcmp(cn, "info_player_start") == 0);
			int is_item = (strncmp(cn, "item_", 5) == 0 || strncmp(cn, "weapon_", 7) == 0);
			nav_conn_entry ent;
			ent.cn = cn;
			VectorCopy(e->v.origin, ent.pos);
			if (is_spawn)
			{
				/* Spawn points are often placed in mid-air (e4m6 hangs
				   them 480-650u up) -- the player falls to the floor on
				   spawn.  Path from where the player LANDS, not from the
				   floating origin, which has no floor poly under it.
				   The trace can START inside hull-1 solid when the spawn
				   hangs just under a ceiling (hull expansion swallows it:
				   hip2m3's deck spawns) -- step the start down until it
				   exits solid, like gravity does with a stuck spawner. */
				vec3_t ts, te, pmins = {-16, -16, -24}, pmaxs = {16, 16, 32};
				trace_t tr;
				int nudge;
				for (nudge = 0; nudge <= 64; nudge += 8)
				{
					VectorCopy(ent.pos, ts);
					ts[2] -= nudge;
					VectorCopy(ts, te);
					te[2] = ent.pos[2] - 2048;
					tr = SV_Move(ts, pmins, pmaxs, te, MOVE_NOMONSTERS, NULL);
					if (!tr.startsolid)
						break;
				}
				if (!tr.startsolid && !tr.allsolid && tr.fraction < 1.0f)
					ent.pos[2] = tr.endpos[2];
				spawns.push_back(ent);
			}
			else if (is_item)
			{
				/* Some stock maps embed items in world solid, outside the
				   playable space (e2m5: two item_health in the void above
				   the great hall's ceiling).  droptofloor's trace exits the
				   solid so they survive spawn, but no player can EVER pick
				   them up -- exclude them from the oracle rather than chase
				   a navmesh route that cannot exist.  Check 1u above the
				   origin: a legitimately-dropped item rests ON its floor,
				   so origin+1 is always open space. */
				vec3_t probe;
				VectorCopy(ent.pos, probe);
				probe[2] += 1;
				if (SV_PointContents(probe) == CONTENTS_SOLID)
				{
					fprintf(stderr, "Nav: CONNECTIVITY: skipping %s at (%.0f %.0f %.0f): embedded in world solid (unobtainable by design)\n",
						cn, ent.pos[0], ent.pos[1], ent.pos[2]);
					continue;
				}
				items.push_back(ent);
			}
		}

		/* A bot can respawn at ANY deathmatch spawn, so an item or spawn is
		   only genuinely unreachable if it can't be reached from ANY of
		   them. A single fixed reference spawn gives false mass-unreachable
		   reports whenever that one spawn happens to sit on a small or
		   one-way-linked pocket of the mesh (confirmed on e4m7/e2m4/e3m5:
		   picking a *different*, still-valid reference spawn swung the
		   unreachable count from ~5% to ~90%, i.e. the old single-reference
		   design was measuring the reference's pocket, not the map). Union
		   across all spawns instead -- this also naturally survives spawns
		   that don't resolve to a floor poly at all (they just contribute
		   nothing, rather than poisoning every query). */
		int usable_spawns = 0;
		for (size_t i = 0; i < spawns.size(); i++)
		{
			nav_mesh_path_result_t self_check;
			char perr[64];
			if (nav_mesh_find_path(nav_mesh, spawns[i].pos, spawns[i].pos, &self_check, perr, sizeof(perr)))
				usable_spawns++;
		}

		if (usable_spawns > 0)
		{
			int spawn_unreachable = 0, item_unreachable = 0;

			fprintf(stderr, "Nav: CONNECTIVITY: %d/%d spawns resolve to a navmesh floor poly\n",
				usable_spawns, (int)spawns.size());

			/* A bot respawns AT a spawn point -- it never walks to one, so
			   inbound reachability (can other spawns path TO this one) is
			   the wrong question and flags legitimate dead-end spawns
			   (drop-in landings, dm6's rocket-jump ledge) that only need
			   an exit, not an entrance. Check outbound instead: can this
			   spawn get OUT to the rest of the map at all -- any other
			   spawn or any item, not just other spawns (a dead-end spawn
			   with only a drop-link out still lands somewhere with items,
			   even if no other spawn happens to be reachable from it). */
			for (size_t i = 0; i < spawns.size(); i++)
			{
				int reached = 0;
				char lasterr[128] = "";
				for (size_t j = 0; j < spawns.size() && !reached; j++)
				{
					if (j == i) continue;
					nav_mesh_path_result_t path_result;
					char perr[128];
					if (nav_mesh_find_path(nav_mesh, spawns[i].pos, spawns[j].pos, &path_result, perr, sizeof(perr)))
						reached = 1;
					else
						memcpy(lasterr, perr, sizeof(lasterr));
				}
				for (size_t j = 0; j < items.size() && !reached; j++)
				{
					nav_mesh_path_result_t path_result;
					char perr[128];
					if (nav_mesh_find_path(nav_mesh, spawns[i].pos, items[j].pos, &path_result, perr, sizeof(perr)))
						reached = 1;
					else
						memcpy(lasterr, perr, sizeof(lasterr));
				}
				if (!reached)
				{
					spawn_unreachable++;
					/* Wide-extents probe: says whether the mesh is merely
					   out of the actor snap's reach or absent entirely. */
					char near_note[96] = "no poly within wide extents";
					nav_mesh_nearest_result_t nr;
					char nerr[64];
					if (nav_mesh_find_nearest(nav_mesh, spawns[i].pos, &nr, nerr, sizeof(nerr)) && nr.found)
						snprintf(near_note, sizeof(near_note), "nearest poly point (%.0f %.0f %.0f)",
							nr.nearest_point[0], nr.nearest_point[1], nr.nearest_point[2]);
					fprintf(stderr, "Nav: CONNECTIVITY unreachable %s at (%.0f %.0f %.0f): can't reach any other spawn or item (%s; %s)\n",
						spawns[i].cn, spawns[i].pos[0], spawns[i].pos[1], spawns[i].pos[2], lasterr, near_note);
				}
			}
			for (size_t i = 0; i < items.size(); i++)
			{
				int reached = 0;
				/* 128, not 64: the partial-path error now embeds a "stopped
				   at x y z" position and 64 truncates it. */
				char lasterr[128] = "";
				for (size_t j = 0; j < spawns.size(); j++)
				{
					nav_mesh_path_result_t path_result;
					char perr[128];
					if (nav_mesh_find_path(nav_mesh, spawns[j].pos, items[i].pos, &path_result, perr, sizeof(perr)))
					{
						reached = 1;
						break;
					}
					strncpy(lasterr, perr, sizeof(lasterr) - 1);
				}
				if (!reached)
				{
					/* Pinpoint the physical gap: closest pair between the
					   spawn-reachable poly set and the item's island. */
					std::vector<float> flat;
					for (size_t s = 0; s < spawns.size(); s++)
					{
						flat.push_back(spawns[s].pos[0]);
						flat.push_back(spawns[s].pos[1]);
						flat.push_back(spawns[s].pos[2]);
					}
					float gfrom[3], gto[3];
					char gerr[64];
					int have_gap = nav_mesh_gap_probe(nav_mesh, flat.data(), (int)spawns.size(),
						items[i].pos, gfrom, gto, gerr, sizeof(gerr));
					/* If even the CLOSEST approach to the island is a taller
					   ascent than any movement primitive covers -- 256u is one
					   rocket's lift, the tallest climb in the whole link
					   arsenal -- no player gets there without a game event the
					   map never fires in deathmatch (hipend's boss dais
					   power-ups 384u up on pillars, hip1m2's exit walkway 272u
					   up its rotate-lift tower).  Unobtainable by design, not
					   a navmesh gap. */
					if (have_gap && gto[2] - gfrom[2] > 256.0f)
					{
						fprintf(stderr, "Nav: CONNECTIVITY: skipping %s at (%.0f %.0f %.0f): island %.0fu above all reachable mesh (event-gated, unobtainable by design)\n",
							items[i].cn, items[i].pos[0], items[i].pos[1], items[i].pos[2],
							gto[2] - gfrom[2]);
						continue;
					}
					item_unreachable++;
					fprintf(stderr, "Nav: CONNECTIVITY unreachable %s at (%.0f %.0f %.0f): unreachable from every spawn (%s)\n",
						items[i].cn, items[i].pos[0], items[i].pos[1], items[i].pos[2], lasterr);
					if (have_gap)
						fprintf(stderr, "Nav: CONNECTIVITY gap: reach ends (%.0f %.0f %.0f), island starts (%.0f %.0f %.0f), dz=%.0f hd=%.0f\n",
							gfrom[0], gfrom[1], gfrom[2], gto[0], gto[1], gto[2],
							gfrom[2] - gto[2],
							sqrtf((gfrom[0]-gto[0])*(gfrom[0]-gto[0]) + (gfrom[1]-gto[1])*(gfrom[1]-gto[1])));
				}
			}

			fprintf(stderr, "Nav: CONNECTIVITY: %d/%d spawns unreachable, %d/%d items unreachable\n",
				spawn_unreachable, (int)spawns.size(), item_unreachable, (int)items.size());
		}
		else if (!spawns.empty())
			fprintf(stderr, "Nav: CONNECTIVITY: no spawn resolves to a navmesh floor poly, skipped\n");
		else
			fprintf(stderr, "Nav: CONNECTIVITY: no spawn points found, skipped\n");
	}

	/* Inline diagnostics removed — see Nav_Validate() in nav_val.cpp */

	/* ---- DM4 stairway connectivity probe — REMOVED ---- */
#if 0 /* inline diagnostics moved to nav_val.cpp */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm4"))
	{
		/* Probe from wp33 (772,-471,-82) toward wp1 (776,-808,-210)
		   in steps to find where the navmesh breaks */
		float probe[3];
		float start[3] = {772, -471, -82};
		float end[3] = {776, -808, -210};
		int steps = 20;
		fprintf(stderr, "Nav: STAIR PROBE from (%.0f %.0f %.0f) to (%.0f %.0f %.0f)\n",
			start[0], start[1], start[2], end[0], end[1], end[2]);
		for (int s = 0; s <= steps; s++)
		{
			float t = (float)s / steps;
			probe[0] = start[0] + (end[0] - start[0]) * t;
			probe[1] = start[1] + (end[1] - start[1]) * t;
			probe[2] = start[2] + (end[2] - start[2]) * t;
			nav_mesh_nearest_result_t nr;
			char nerr[64];
			int found = nav_mesh_find_nearest(nav_mesh, probe, &nr, nerr, sizeof(nerr));
			fprintf(stderr, "  t=%.2f pos=(%.0f %.0f %.0f) %s dist=%.0f\n",
				t, probe[0], probe[1], probe[2],
				found ? "HIT" : "MISS",
				found ? sqrtf((probe[0]-nr.nearest_point[0])*(probe[0]-nr.nearest_point[0]) +
							  (probe[1]-nr.nearest_point[1])*(probe[1]-nr.nearest_point[1]) +
							  (probe[2]-nr.nearest_point[2])*(probe[2]-nr.nearest_point[2])) : 0.0f);
		}
	}

	/* ---- DM4 fine probe along wp14→wp53 to find the exact break ---- */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm4"))
	{
		fprintf(stderr, "Nav: FINE PROBE wp14(512,-97,-82)→wp53(224,-59,-82):\n");
		dtPolyRef prev_ref = 0;
		for (int step = 0; step <= 40; step++)
		{
			float t = step / 40.0f;
			float qpos[3] = {512 + (224-512)*t, -97 + (-59+97)*t, -82};
			nav_mesh_nearest_result_t nr;
			char nerr[64];
			int found = nav_mesh_find_nearest(nav_mesh, qpos, &nr, nerr, sizeof(nerr));
			float snap = found ? sqrtf(
				(qpos[0]-nr.nearest_point[0])*(qpos[0]-nr.nearest_point[0]) +
				(qpos[1]-nr.nearest_point[1])*(qpos[1]-nr.nearest_point[1]) +
				(qpos[2]-nr.nearest_point[2])*(qpos[2]-nr.nearest_point[2])) : 0;

			/* Also BSP floor trace */
			vec3_t ts = {qpos[0], qpos[1], 100}, te = {qpos[0], qpos[1], -200}, zero = {0,0,0};
			trace_t tr = SV_Move(ts, zero, zero, te, MOVE_NOMONSTERS, NULL);

			if (!found || nr.poly_ref != prev_ref || step == 0 || step == 40)
			{
				fprintf(stderr, "  t=%.2f (%.0f,%.0f) %s ref=%llu snap=%.0f bsp_z=%.1f nav_z=%.1f\n",
					t, qpos[0], qpos[1],
					found ? (snap < 50 ? "HIT" : "FAR") : "MISS",
					found ? nr.poly_ref : 0ULL, snap, tr.endpos[2],
					found ? nr.nearest_point[2] : 0.0f);
				if (found) prev_ref = nr.poly_ref;
			}
		}
	}

	/* ---- DM4 full waypoint edge validation ---- */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm4"))
	{
		static float dm4_wps_v[][3] = {
			{776,-808,-210},{272,-952,46},{112,-1136,-82},{784,-176,46},
			{-64,-232,-50},{-64,512,-274},{428,-966,-82},{134,-820,-82},
			{426,-790,-82},{336,-732,-82},{354,-471,-86},{188,-408,-82},
			{505,-318,-82},{512,-97,-82},{631,-386,-82},{337,-1206,-2},
			{334,-1016,46},{337,-863,42},{337,-737,46},{337,-508,46},
			{516,-468,46},{610,-180,46},{120,-478,46},{143,-303,-34},
			{160,-183,-50},{199,76,-82},{-120,-518,-50},{868,-176,46},
			{847,-578,46},{1121,-593,46},{983,-457,-82},{993,-594,-82},
			{772,-471,-82},{368,-92,-274},{375,-239,-274},{-49,-234,-274},
			{-56,75,-274},{69,70,-274},{620,-242,-274},{624,-42,-274},
			{514,-240,-274},{206,-245,-274},{626,58,-274},{369,65,-266},
			{701,66,-274},{-60,334,-274},{752,-502,-82},{856,-391,46},
			{553,66,-274},{557,-45,46},{605,-113,46},{173,-60,-82},
			{224,-59,-82},{335,-431,-82},{434,-390,-82},{-183,-472,-50},
			{314,-1027,-82},{336,-801,-82},{-61,573,-274},
		};
		/* edges: from(0-based) → to(0-based) */
		static int dm4_edges_v[][2] = {
			{0,32},{0,33},{0,46},{1,17},{1,16},{2,56},{2,15},{2,7},
			{3,21},{3,27},{4,23},{4,26},{5,45},{5,58},{6,56},{6,8},
			{6,7},{6,57},{7,2},{7,56},{7,57},{7,6},{8,57},{8,56},
			{8,6},{8,2},{9,57},{9,53},{9,10},{10,54},{11,53},{11,10},
			{12,53},{12,14},{12,13},{13,12},{13,14},{13,52},{13,34},
			{14,12},{14,13},{14,34},{15,2},{15,16},{16,15},{16,17},
			{17,16},{17,18},{17,42},{18,17},{18,19},{19,18},{19,20},
			{19,22},{19,12},{20,19},{20,53},{20,12},{20,21},{21,20},
			{21,3},{21,50},{21,13},{22,19},{22,23},{22,53},{23,22},
			{23,24},{23,4},{24,23},{24,51},{25,51},{26,4},{26,27},
			{26,55},{27,47},{27,3},{28,47},{28,29},{28,30},{28,0},
			{29,28},{30,28},{31,32},{31,30},{32,31},{32,0},{33,34},
			{33,43},{34,33},{34,40},{34,35},{34,27},{35,34},{35,36},
			{35,41},{36,35},{36,37},{36,45},{37,36},{38,40},{38,39},
			{39,38},{39,44},{39,48},{39,42},{40,34},{40,38},{41,13},
			{42,48},{42,17},{42,44},{43,33},{44,39},{44,48},{44,42},
			{45,36},{45,5},{46,47},{47,27},{47,28},{47,32},{47,30},
			{48,42},{48,44},{48,39},{49,50},{49,13},{50,21},{50,49},
			{51,24},{51,25},{52,25},{53,34},{53,12},{53,11},{53,9},
			{54,21},{55,26},{56,57},{56,2},{56,6},{56,8},{57,9},
			{57,8},{57,7},{57,56},{58,5},
		};
		int nwps = sizeof(dm4_wps_v) / sizeof(dm4_wps_v[0]);
		int nedges = sizeof(dm4_edges_v) / sizeof(dm4_edges_v[0]);
		dtQueryFilter filter;
		nav_mesh_setup_filter(&filter);

		int ok_count = 0, partial_count = 0, fail_count = 0, miss_count = 0;
		fprintf(stderr, "Nav: DM4 WAYPOINT VALIDATION (%d edges):\n", nedges);
		for (int ei = 0; ei < nedges; ei++)
		{
			int ai = dm4_edges_v[ei][0], bi = dm4_edges_v[ei][1];
			if (ai < 0 || ai >= nwps || bi < 0 || bi >= nwps) continue;

			nav_mesh_nearest_result_t nra, nrb;
			char nerr[64];
			int fa = nav_mesh_find_nearest(nav_mesh, dm4_wps_v[ai], &nra, nerr, sizeof(nerr));
			int fb = nav_mesh_find_nearest(nav_mesh, dm4_wps_v[bi], &nrb, nerr, sizeof(nerr));

			if (!fa || !fb) { miss_count++; continue; }

			float rca[3], rcb[3];
			rca[0]=nra.nearest_point[0]; rca[1]=nra.nearest_point[2]; rca[2]=nra.nearest_point[1];
			rcb[0]=nrb.nearest_point[0]; rcb[1]=nrb.nearest_point[2]; rcb[2]=nrb.nearest_point[1];
			dtPolyRef path[512]; int pc = 0;
			dtStatus st = nav_mesh->query->findPath(
				(dtPolyRef)nra.poly_ref, (dtPolyRef)nrb.poly_ref,
				rca, rcb, &filter, path, &pc, 512);
			int partial = dtStatusDetail(st, DT_PARTIAL_RESULT) ? 1 : 0;
			int failed = dtStatusFailed(st) ? 1 : 0;

			if (failed) { fail_count++; fprintf(stderr, "  FAIL wp%d→wp%d\n", ai+1, bi+1); }
			else if (partial)
			{
				partial_count++;
				float dz = dm4_wps_v[bi][2] - dm4_wps_v[ai][2];
				fprintf(stderr, "  PARTIAL wp%d→wp%d (%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f) dz=%.0f polys=%d\n",
					ai+1, bi+1,
					dm4_wps_v[ai][0], dm4_wps_v[ai][1], dm4_wps_v[ai][2],
					dm4_wps_v[bi][0], dm4_wps_v[bi][1], dm4_wps_v[bi][2],
					dz, pc);
			}
			else ok_count++;
		}
		fprintf(stderr, "Nav: DM4 WAYPOINT VALIDATION: %d OK, %d PARTIAL, %d FAIL, %d MISS / %d total\n",
			ok_count, partial_count, fail_count, miss_count, nedges);
	}

	/* ---- DM1 full waypoint edge validation ---- */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm1"))
	{
		static float dm1_wps_v[][3] = {
			{640,716,46},{1066,733,94},{1054,1321,94},{783,1313,46},
			{585,883,46},{901,1060,46},{448,1313,42},{436,1562,46},
			{238,1593,46},{-122,1583,-98},{-127,1414,-98},{113,1416,-98},
			{109,1287,-98},{417,1258,-98},{157,916,-98},{-292,866,-98},
			{397,1595,-98},{-143,1418,46},{-385,1407,46},{607,608,46},
			{77,614,46},{-501,770,46},{-534,988,46},{-655,992,46},
			{-651,1227,46},{-524,1592,46},{443,1145,46},{220,1236,46},
			{178,1027,46},{28,1157,46},{153,1154,-98},{-307,1219,94},
			{-292,1100,94},{464,823,46},{435,1451,-98},{-300,1432,46},
			{600,1309,46},{-322,1007,-98},{-543,1222,46},{1000,941,94},
			{-156,1049,-98},{104,1392,46},{280,1416,-98},{280,1416,46},
			{-656,1417,46},{-263,723,-98},{-203,769,46},{82,763,46},
			{348,758,46},{141,764,46},{488,607,46},{482,683,54},
		};
		static int dm1_edges_v[][2] = {
			{0,1},{0,19},{1,0},{1,2},{1,39},{2,1},{2,3},{3,5},{3,36},
			{3,2},{3,4},{4,3},{4,5},{4,36},{5,3},{5,4},{5,36},{6,36},
			{6,26},{6,7},{7,6},{7,8},{8,7},{8,9},{8,43},{9,8},{9,10},
			{10,9},{10,11},{11,10},{11,12},{11,42},{12,11},{12,13},
			{12,40},{12,30},{13,12},{13,34},{13,30},{13,14},{14,45},
			{14,15},{14,30},{14,13},{15,45},{15,37},{15,40},{15,14},
			{16,34},{16,6},{17,43},{17,18},{18,35},{18,17},{19,0},
			{19,50},{20,50},{20,47},{21,46},{21,22},{22,21},{22,23},
			{23,22},{23,24},{24,23},{24,25},{24,31},{24,44},{25,24},
			{25,9},{25,38},{25,44},{26,6},{26,27},{26,28},{27,26},
			{27,29},{27,30},{28,26},{28,29},{28,30},{29,28},{29,27},
			{29,30},{30,12},{30,14},{30,40},{30,13},{31,24},{31,32},
			{31,38},{32,31},{32,15},{32,37},{33,51},{34,16},{35,9},
			{36,3},{36,4},{36,6},{36,5},{37,15},{37,40},{38,25},
			{38,31},{38,44},{39,5},{40,37},{40,30},{40,15},{40,12},
			{41,43},{41,10},{42,11},{43,17},{43,8},{43,41},{44,24},
			{44,25},{44,38},{45,14},{45,15},{46,47},{46,21},{46,37},
			{47,20},{47,46},{47,49},{47,14},{48,33},{49,48},{50,51},
			{50,19},{50,20},{51,50},{51,33},
		};
		int nwps = sizeof(dm1_wps_v) / sizeof(dm1_wps_v[0]);
		int nedges = sizeof(dm1_edges_v) / sizeof(dm1_edges_v[0]);
		dtQueryFilter filter;
		nav_mesh_setup_filter(&filter);

		int ok_count = 0, partial_count = 0, fail_count = 0, miss_count = 0;
		fprintf(stderr, "Nav: DM1 WAYPOINT VALIDATION (%d edges):\n", nedges);
		for (int ei = 0; ei < nedges; ei++)
		{
			int ai = dm1_edges_v[ei][0], bi = dm1_edges_v[ei][1];
			if (ai < 0 || ai >= nwps || bi < 0 || bi >= nwps) continue;
			nav_mesh_nearest_result_t nra, nrb;
			char nerr[64];
			int fa = nav_mesh_find_nearest(nav_mesh, dm1_wps_v[ai], &nra, nerr, sizeof(nerr));
			int fb = nav_mesh_find_nearest(nav_mesh, dm1_wps_v[bi], &nrb, nerr, sizeof(nerr));
			if (!fa || !fb)
			{
				miss_count++;
				fprintf(stderr, "  MISS wp%d→wp%d%s%s\n", ai+1, bi+1,
					fa ? "" : " (start off mesh)", fb ? "" : " (end off mesh)");
				continue;
			}
			float rca[3], rcb[3];
			rca[0]=nra.nearest_point[0]; rca[1]=nra.nearest_point[2]; rca[2]=nra.nearest_point[1];
			rcb[0]=nrb.nearest_point[0]; rcb[1]=nrb.nearest_point[2]; rcb[2]=nrb.nearest_point[1];
			dtPolyRef path[512]; int pc = 0;
			dtStatus st = nav_mesh->query->findPath(
				(dtPolyRef)nra.poly_ref, (dtPolyRef)nrb.poly_ref,
				rca, rcb, &filter, path, &pc, 512);
			int partial = dtStatusDetail(st, DT_PARTIAL_RESULT) ? 1 : 0;
			int failed = dtStatusFailed(st) ? 1 : 0;
			if (failed) { fail_count++; fprintf(stderr, "  FAIL wp%d→wp%d\n", ai+1, bi+1); }
			else if (partial)
			{
				partial_count++;
				float dz = dm1_wps_v[bi][2] - dm1_wps_v[ai][2];
				fprintf(stderr, "  PARTIAL wp%d→wp%d (%.0f,%.0f,%.0f)→(%.0f,%.0f,%.0f) dz=%.0f polys=%d\n",
					ai+1, bi+1,
					dm1_wps_v[ai][0], dm1_wps_v[ai][1], dm1_wps_v[ai][2],
					dm1_wps_v[bi][0], dm1_wps_v[bi][1], dm1_wps_v[bi][2],
					dz, pc);
			}
			else ok_count++;
		}
		fprintf(stderr, "Nav: DM1 WAYPOINT VALIDATION: %d OK, %d PARTIAL, %d FAIL, %d MISS / %d total\n",
			ok_count, partial_count, fail_count, miss_count, nedges);
	}

	/* ---- DM4 grid probe: find all poly refs on LOWER CORRIDOR level (Z≈-82) ---- */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm4"))
	{
		/* Grid probe at Z=46 (upper walkway) across the map.
		   Log unique poly refs and their centers to find islands. */
		dtQueryFilter filter;
		nav_mesh_setup_filter(&filter);
		unsigned long long seen_refs[256];
		float seen_centers[256][3];
		int seen_count = 0;
		int x, y;
		for (x = -200; x <= 1200; x += 32)
		{
			for (y = -1200; y <= 600; y += 32)
			{
				float p[3] = {(float)x, (float)y, -82.0f};
				nav_mesh_nearest_result_t nr;
				char nerr[64];
				if (!nav_mesh_find_nearest(nav_mesh, p, &nr, nerr, sizeof(nerr)))
					continue;
				/* Only consider polys near the upper level (Z within 50u) */
				if (nr.nearest_point[2] < -140 || nr.nearest_point[2] > -100)
					continue;
				/* Check if already seen */
				int found = 0;
				for (int si = 0; si < seen_count; si++)
					if (seen_refs[si] == nr.poly_ref) { found = 1; break; }
				if (!found && seen_count < 256)
				{
					seen_refs[seen_count] = nr.poly_ref;
					seen_centers[seen_count][0] = nr.nearest_point[0];
					seen_centers[seen_count][1] = nr.nearest_point[1];
					seen_centers[seen_count][2] = nr.nearest_point[2];
					seen_count++;
				}
			}
		}
		fprintf(stderr, "Nav: GRID found %d unique upper-level polys\n", seen_count);

		/* For each pair, test pathfinding to find connected components */
		/* Simpler: test all polys against ref=358 (known main island) */
		float rc_start[3];
		rc_start[0] = seen_centers[0][0]; rc_start[1] = seen_centers[0][2]; rc_start[2] = seen_centers[0][1];
		int connected_to_first = 0;
		for (int si = 0; si < seen_count; si++)
		{
			float rc_end[3];
			rc_end[0] = seen_centers[si][0]; rc_end[1] = seen_centers[si][2]; rc_end[2] = seen_centers[si][1];
			dtPolyRef path[512];
			int pc = 0;
			dtStatus st = nav_mesh->query->findPath(
				(dtPolyRef)seen_refs[0], (dtPolyRef)seen_refs[si],
				rc_start, rc_end, &filter, path, &pc, 512);
			int ok = !dtStatusFailed(st) && !dtStatusDetail(st, DT_PARTIAL_RESULT);
			if (ok) connected_to_first++;
			else
				fprintf(stderr, "  DISCONNECTED ref=%llu at (%.0f %.0f %.0f)\n",
					seen_refs[si], seen_centers[si][0], seen_centers[si][1], seen_centers[si][2]);
		}
		fprintf(stderr, "Nav: GRID %d/%d upper polys connected to first poly (ref=%llu)\n",
			connected_to_first, seen_count, seen_refs[0]);

		/* Find the graph break: closest connected poly to closest disconnected poly */
		{
			float best_dist = 999999;
			int best_conn = -1, best_disc = -1;
			for (int ci = 0; ci < seen_count; ci++)
			{
				/* check if connected */
				float rc_e[3];
				rc_e[0] = seen_centers[ci][0]; rc_e[1] = seen_centers[ci][2]; rc_e[2] = seen_centers[ci][1];
				dtPolyRef tp[512]; int tpc = 0;
				dtStatus ts = nav_mesh->query->findPath((dtPolyRef)seen_refs[0], (dtPolyRef)seen_refs[ci],
					rc_start, rc_e, &filter, tp, &tpc, 512);
				int conn = !dtStatusFailed(ts) && !dtStatusDetail(ts, DT_PARTIAL_RESULT);
				if (!conn) continue;

				for (int di = 0; di < seen_count; di++)
				{
					float rc_d[3];
					rc_d[0] = seen_centers[di][0]; rc_d[1] = seen_centers[di][2]; rc_d[2] = seen_centers[di][1];
					dtPolyRef dp[512]; int dpc = 0;
					dtStatus ds = nav_mesh->query->findPath((dtPolyRef)seen_refs[0], (dtPolyRef)seen_refs[di],
						rc_start, rc_d, &filter, dp, &dpc, 512);
					int disc = dtStatusFailed(ds) || dtStatusDetail(ds, DT_PARTIAL_RESULT);
					if (!disc) continue;

					float dx = seen_centers[ci][0] - seen_centers[di][0];
					float dy = seen_centers[ci][1] - seen_centers[di][1];
					float d = sqrtf(dx*dx + dy*dy);
					if (d < best_dist)
					{
						best_dist = d;
						best_conn = ci;
						best_disc = di;
					}
				}
			}
			if (best_conn >= 0 && best_disc >= 0)
			{
				fprintf(stderr, "Nav: BREAK closest connected=(%.0f %.0f) ref=%llu ↔ disconnected=(%.0f %.0f) ref=%llu dist=%.0f\n",
					seen_centers[best_conn][0], seen_centers[best_conn][1], seen_refs[best_conn],
					seen_centers[best_disc][0], seen_centers[best_disc][1], seen_refs[best_disc],
					best_dist);
			}
		}
	}

#endif /* inline diagnostics */

}

void Nav_Shutdown(void)
{
	nav_item_count = 0;
	nav_bot_want_func = 0;
	nav_block_map_count = 0;
	nav_blocked.count = 0;
	if (nav_mesh != NULL)
	{
		nav_mesh_destroy(nav_mesh);
		nav_mesh = NULL;
	}
	nav_build_attempted = 0;
	{
		int ci;
		for (ci = 0; ci < MAX_SCOREBOARD; ci++)
		{
			if (nav_bot_corridors[ci] != NULL)
			{
				nav_corridor_destroy(nav_bot_corridors[ci]);
				nav_bot_corridors[ci] = NULL;
			}
		}
	}
	if (nav_debug_polys != NULL)
	{
		nav_mesh_free_poly_records(nav_debug_polys);
		nav_debug_polys = NULL;
		nav_debug_poly_count = 0;
		nav_debug_cursor = 0;
	}
}

/* ---- QC Builtins ---- */

static void Nav_EnsureBuilt(void)
{
	/* Rebuild if the worldmodel changed (changelevel). */
	if (sv.worldmodel != nav_built_for_model)
	{
		Nav_Shutdown();
		nav_built_for_model = sv.worldmodel;
	}
	if (nav_mesh != NULL || nav_build_attempted)
		return;
	nav_build_attempted = 1;
	Nav_BuildForMap();
}

extern "C" void SV_StartParticle(vec3_t org, vec3_t dir, int color, int count);

extern "C" void MSG_WriteByte(sizebuf_t *sb, int c);
extern "C" void MSG_WriteCoord(sizebuf_t *sb, float f);

static double nav_debug_last_time = 0;

static void Nav_DebugDraw(void)
{
	char error[128];
	int i, batch, count;
	edict_t *player;
	float px, py, pz, dx, dy, dist_sq;
	float range_sq = 512.0f * 512.0f;

	if (!nav_debug_cvar.value || nav_mesh == NULL)
		return;

	/* cache poly data on first call */
	if (nav_debug_polys == NULL)
	{
		if (!nav_mesh_collect_polys(nav_mesh, &nav_debug_polys,
			&nav_debug_poly_count, error, sizeof(error)))
			return;
		Con_Printf("Nav: debug draw active, %d polys\n", nav_debug_poly_count);

		/* Dump elevated debug markers to OBJ so they are visible above the floor.
		   These are marker crosses, not raw navmesh vertices. */
		{
			FILE *fp = fopen("navmesh_debug_markers.obj", "w");
			if (fp)
			{
				int vi = 1;
				for (i = 0; i < nav_debug_poly_count; i++)
				{
					float cx = nav_debug_polys[i].center[0];
					float cy = nav_debug_polys[i].center[1];
					float cz = nav_debug_marker_z(nav_debug_polys[i].center[2]);
					fprintf(fp, "v %f %f %f\n", cx - 4, cy, cz);
					fprintf(fp, "v %f %f %f\n", cx + 4, cy, cz);
					fprintf(fp, "v %f %f %f\n", cx, cy - 4, cz);
					fprintf(fp, "v %f %f %f\n", cx, cy + 4, cz);
					fprintf(fp, "f %d %d %d\n", vi, vi+1, vi+2);
					fprintf(fp, "f %d %d %d\n", vi, vi+2, vi+3);
					vi += 4;
				}
				fclose(fp);
				Con_Printf("Nav: wrote navmesh_debug_markers.obj (%d polys, +%.0fu marker lift)\n",
					nav_debug_poly_count, NAV_DEBUG_MARKER_LIFT);
			}
		}
	}

	/* throttle to 2x per second */
	if (sv.time - nav_debug_last_time < 0.5)
		return;
	nav_debug_last_time = sv.time;
}

static void PF_nav_ready(void)
{
	Nav_EnsureBuilt();
	Nav_DebugDraw();
	G_FLOAT(OFS_RETURN) = (nav_mesh != NULL) ? 1.0f : 0.0f;
}

/* Stubs for legacy builtin slots #82 and #83 */
static void PF_nav_stub(void)
{
	G_FLOAT(OFS_RETURN) = -1.0f;
}

/* A bot may be routed onto a rocket-jump link only if it can actually pay
   for the launch: owns the launcher, has a rocket loaded, and enough health
   to survive the self-damage (>75).  Otherwise RJ links are excluded from
   its pathing and it takes the normal route the safety gate guarantees. */
#define NAV_IT_ROCKET_LAUNCHER 32
static int nav_bot_can_rj(edict_t *bot)
{
	if (bot == NULL)
		return 0;
	return ((int)bot->v.items & NAV_IT_ROCKET_LAUNCHER)
		&& bot->v.ammo_rockets > 0.0f
		&& bot->v.health > 75.0f;
}

/* vector nav_path_steer(vector pos) = #84
   Uses dtPathCorridor to get next steering corner.
   Returns corner position. Off-mesh links encoded in Z (10000 + type).
   Returns '0 0 0' if corridor is empty. */
static void PF_nav_path_steer(void)
{
	float *pos;
	int slot;
	float corner[3];
	unsigned char flags;
	unsigned long long ref;

	G_FLOAT(OFS_RETURN + 0) = 0.0f;
	G_FLOAT(OFS_RETURN + 1) = 0.0f;
	G_FLOAT(OFS_RETURN + 2) = 0.0f;

	pos = G_VECTOR(OFS_PARM0);

	slot = nav_bot_slot();
	if (slot < 0) return;

	if (nav_bot_corridors[slot] == NULL) return;
	if (nav_mesh == NULL) return;

	/* Keep the corridor's RJ gate in step with the bot's live state (health
	   or rockets may have changed since the goal was planned). */
	nav_corridor_set_rj(nav_bot_corridors[slot],
		nav_bot_can_rj(PROG_TO_EDICT(pr_global_struct->self)));

	if (!navigate(nav_bot_corridors[slot], nav_mesh, pos,
		corner, &flags, &ref))
		return;

	G_FLOAT(OFS_RETURN + 0) = corner[0];
	G_FLOAT(OFS_RETURN + 1) = corner[1];
	G_FLOAT(OFS_RETURN + 2) = corner[2];

	/* encode off-mesh link type in corner z for the QC side */
	if (flags & 0x04) /* DT_STRAIGHTPATH_OFFMESH_CONNECTION */
	{
		int lt = nav_mesh_get_link_type(nav_mesh, ref);
		if (lt > 0)
		{
			/* Multiplicative encoding so QC can recover both type and
			   corner z: z' = z + 10000*(1+type).  Additive (+10000+type)
			   was ambiguous — type and z can't be separated. */
			G_FLOAT(OFS_RETURN + 2) = corner[2] + 10000.0f * (1.0f + (float)lt);
		}
	}
}


/* vector nav_path_debug(entity bot, float index) = #85
   Return position of waypoint N in bot's cached path.
   Returns '0 0 0' if no valid path or index out of range.
   index == -1 returns (point_count, current_index, valid). */
static void PF_nav_path_debug(void)
{
	edict_t *e;
	int idx;

	G_FLOAT(OFS_RETURN + 0) = 0.0f;
	G_FLOAT(OFS_RETURN + 1) = 0.0f;
	G_FLOAT(OFS_RETURN + 2) = 0.0f;

	e = G_EDICT(OFS_PARM0);
	idx = (int)G_FLOAT(OFS_PARM1);

	/* world entity: return elevated debug marker positions for navmesh polys.
	   index == -1 -> (poly_count, 0, 0). index >= 0 -> poly center lifted above
	   the surface so QC particle markers do not sit inside the floor. */
	if (e == sv.edicts)
	{
		char error[128];
		if (nav_debug_polys == NULL && nav_mesh != NULL)
			nav_mesh_collect_polys(nav_mesh, &nav_debug_polys,
				&nav_debug_poly_count, error, sizeof(error));
		if (idx == -1)
		{
			G_FLOAT(OFS_RETURN + 0) = (float)nav_debug_poly_count;
			return;
		}
		if (idx >= 0 && idx < nav_debug_poly_count && nav_debug_polys)
		{
			G_FLOAT(OFS_RETURN + 0) = nav_debug_polys[idx].center[0];
			G_FLOAT(OFS_RETURN + 1) = nav_debug_polys[idx].center[1];
			G_FLOAT(OFS_RETURN + 2) = nav_debug_marker_z(nav_debug_polys[idx].center[2]);
		}
		return;
	}

	/* bot entity: corridor doesn't support random access, but idx == -1
	   is used by QC as a coverage proxy (detect poly-to-poly movement),
	   so answer that one case with the bot's current poly ref. */
	if (idx == -1 && nav_mesh != NULL)
	{
		float pos[3], nearest[3];
		dtPolyRef ref = 0;
		VectorCopy(e->v.origin, pos);
		if (nav_find_bot_poly(nav_mesh->query, e, pos, &ref, nearest))
			G_FLOAT(OFS_RETURN + 0) = (float)(ref & 0xFFFFFFu);
	}
}

/* ---- nav_ent_pos: entity position for navmesh queries ---- */

static void nav_ent_pos(edict_t *ent, float *pos)
{
	pos[0] = ent->v.origin[0];
	pos[1] = ent->v.origin[1];
	pos[2] = ent->v.origin[2];
	if (ent->v.solid == SOLID_TRIGGER)
		pos[2] = ent->v.absmin[2];
}

/* Classify a path against the blocked-door table for a specific bot:
   0 = clear, 1 = blocked by something the bot can't open, 2 = blocked
   only by doors the bot can open itself.  Shootable doors (health > 0,
   which includes secret doors) open to gunfire; key doors open on touch
   when the bot carries the key.  Doors waiting on a button or trigger
   elsewhere report the blocking door via blocker_out so the goal picker
   can chase the opener instead. */
static int nav_path_block_class(const dtPolyRef *path, int path_count,
	edict_t *bot, edict_t **blocker_out)
{
	int cls = 0;
	if (blocker_out) *blocker_out = NULL;
	for (int i = 0; i < nav_block_map_count; i++)
	{
		edict_t *e;
		int hit = 0;
		if (!nav_block_map[i].is_blocked) continue;
		for (int p = 0; p < nav_block_map[i].poly_count && !hit; p++)
			for (int q = 0; q < path_count; q++)
				if (nav_block_map[i].polys[p] == path[q]) { hit = 1; break; }
		if (!hit) continue;
		e = nav_block_map[i].ent;
		if (e->v.health > 0)
			cls = 2;
		else if ((int)e->v.items != 0
			&& ((int)bot->v.items & (int)e->v.items) == (int)e->v.items)
			cls = 2;
		else
		{
			if (blocker_out) *blocker_out = e;
			return 1;
		}
	}
	return cls;
}

/* ---- Opener chains: who do I press to open this door? ---- */

static void nav_brush_center(edict_t *ent, float *pos)
{
	pos[0] = (ent->v.absmin[0] + ent->v.absmax[0]) * 0.5f;
	pos[1] = (ent->v.absmin[1] + ent->v.absmax[1]) * 0.5f;
	pos[2] = (ent->v.absmin[2] + ent->v.absmax[2]) * 0.5f;
}

/* Trigger brushes a bot fires just by stepping into them.  Counters and
   relays are use-only — they have no touch and must be climbed instead.
   trigger_onlyregistered/changelevel are level-flow gates, NOT door openers:
   chasing them sent bots across the start hub to "open" the registration gate
   guarding the episode alcoves, where they pinned forever (and stepping into
   the changelevel beyond would END the match).  Leave such gated items to
   dead-end as unreachable so the picker rejects them. */
static int nav_opener_walkthrough(const char *cn)
{
	return !strcmp(cn, "trigger_once")
		|| !strcmp(cn, "trigger_multiple")
		|| !strcmp(cn, "trigger_secret");
}

/* Climb the targetname chain from a blocked door to something a bot can
   actually act on: an unpressed func_button (touch or shoot) or a
   walk-through trigger brush.  Counters and relays recurse upward, so a
   3-button trigger_counter hands out its buttons one unpressed press at
   a time.  Monster-fired chains dead-end naturally (monsters aren't
   buttons and have nothing targeting them). */
static edict_t *nav_door_opener(edict_t *ent, int depth)
{
	const char *tn;
	if (depth > 4) return NULL;
	if (!ent->v.targetname) return NULL;
	tn = pr_strings + (int)ent->v.targetname;
	if (!tn[0]) return NULL;

	for (int i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		const char *cn;
		if (e->free || !e->v.target) continue;
		if (strcmp(pr_strings + (int)e->v.target, tn)) continue;
		cn = pr_strings + (int)e->v.classname;
		if (!strcmp(cn, "func_button"))
		{
			/* pressable only while at rest at the bottom; a wait -1
			   button parked at the top already gave its one press */
			eval_t *st = GetEdictFieldValue(e, "state");
			if (st && st->_float == 1 /* STATE_BOTTOM */)
				return e;
			continue;
		}
		if (nav_opener_walkthrough(cn))
			return e;
		if (!strncmp(cn, "trigger_", 8))
		{
			edict_t *up = nav_door_opener(e, depth + 1);
			if (up) return up;
		}
	}
	return NULL;
}

/* ---- nav_find_goal: pick best item, pathfind, cache path ---- */

extern "C" dfunction_t *ED_FindFunction(char *name);
extern "C" void PR_ExecuteProgram(func_t fnum);

/* entity nav_find_goal() = #86
   Iterates all items, calls QC bot_want for each, pathfinds
   candidates, caches the winning path. Returns winning entity. */
static void PF_nav_find_goal(void)
{
	edict_t *bot, *it, *best;
	int slot, i;
	float pos[3];
	float want, dist, cost, bestcost;
	int best_path_count = 0;
	dtPolyRef best_path[NAV_MESH_MAX_PATH_REFS];
	float best_goal_rc[3] = {0};

	G_INT(OFS_RETURN) = EDICT_TO_PROG(sv.edicts);

	Nav_EnsureBuilt();
	if (nav_mesh == NULL) return;

	/* Lazy item cache — wait until QC has classified items (item_res set) */
	if (nav_item_count == 0)
		nav_cache_item_polys();

	slot = nav_bot_slot();
	if (slot < 0) return;

	bot = PROG_TO_EDICT(pr_global_struct->self);
	VectorCopy(bot->v.origin, pos);

	/* cache QC function and field offsets once */
	if (!nav_bot_want_func)
	{
		dfunction_t *f = ED_FindFunction("bot_want");
		if (!f) return;
		nav_bot_want_func = (func_t)(f - pr_functions);
	}

	dtNavMeshQuery *query = nav_mesh->query;
	dtQueryFilter plain_filter;
	nav_mesh_setup_filter(&plain_filter);
	/* Plan around RJ links unless this bot can rocket-jump, so the goal it
	   picks and the path it commits to never depend on a launch it can't make. */
	if (!nav_bot_can_rj(bot))
		plain_filter.setExcludeFlags(NAV_POLYFLAG_RJ);

	/* Find bot's current poly (once for all candidates) */
	float bot_nearest[3];
	dtPolyRef bot_ref = 0;
	if (!nav_find_bot_poly(query, bot, pos, &bot_ref, bot_nearest)) return;

	bestcost = 999999.0f;
	best = sv.edicts;

	/* Failed-goal cooldown: QC marks the goal it stalled on; skip it so
	   the deterministic scorer can't immediately re-pick it and recreate
	   the same jam. */
	edict_t *failed_goal = NULL;
	{
		eval_t *fg = GetEdictFieldValue(bot, "_failed_goal");
		eval_t *fgt = GetEdictFieldValue(bot, "_failed_goal_time");
		if (fg && fgt && fgt->_float > sv.time && fg->edict)
			failed_goal = PROG_TO_EDICT(fg->edict);
	}

	int dbg_avail = 0, dbg_wanted = 0, dbg_pathed = 0, dbg_blocked = 0;

	for (i = 0; i < nav_item_count; i++)
	{
		it = nav_item_cache[i].ent;
		if (it->free) continue;
		if (it == failed_goal) continue;

		if ((int)it->v.flags & FL_ITEM)
			if (!it->v.model) continue;

		dbg_avail++;

		/* call QC bot_want */
		pr_global_struct->self = EDICT_TO_PROG(bot);
		G_INT(OFS_PARM0) = EDICT_TO_PROG(it);
		PR_ExecuteProgram(nav_bot_want_func);
		want = G_FLOAT(OFS_RETURN);
		if (want < 0.01f) continue;

		dbg_wanted++;

		dtPolyRef path[NAV_MESH_MAX_PATH_REFS];
		int path_count = 0;
		dtStatus status = query->findPath(
			bot_ref, nav_item_cache[i].poly_ref,
			bot_nearest, nav_item_cache[i].nav_pos,
			&plain_filter, path, &path_count, NAV_MESH_MAX_PATH_REFS);

		if (dtStatusFailed(status) || path_count < 1)
		{
			if (nav_debug_cvar.value)
				fprintf(stderr, "  findPath FAIL to %s\n", pr_strings + (int)it->v.classname);
			continue;
		}
		if (dtStatusDetail(status, DT_PARTIAL_RESULT))
		{
			if (path[path_count - 1] != nav_item_cache[i].poly_ref)
			{
				if (nav_debug_cvar.value)
				{
					/* Log where the partial path ends */
					float last_pos[3] = {0};
					nav_mesh->query->closestPointOnPoly(path[path_count-1], bot_nearest, last_pos, NULL);
					float lq[3];
					lq[0] = last_pos[0]; lq[1] = last_pos[2]; lq[2] = last_pos[1]; /* recast→quake */
					fprintf(stderr, "  PARTIAL to %s (%d polys) ends=(%.0f %.0f %.0f)\n",
						pr_strings + (int)it->v.classname, path_count, lq[0], lq[1], lq[2]);
				}
				continue;
			}
		}

		dbg_pathed++;

		{
			edict_t *blocker = NULL;
			int bc = nav_path_block_class(path, path_count, bot, &blocker);
			if (bc == 1)
			{
				/* Door somebody has to open: chase the opener instead.
				   The button inherits the prize's want — pressing it IS
				   progress toward the item — plus a detour surcharge.
				   Once the door opens, the block clears and the next
				   goal pass routes straight through. */
				edict_t *opener = blocker ? nav_door_opener(blocker, 0) : NULL;
				int routed = 0;
				if (opener && opener != failed_goal)
				{
					float oq[3], orc[3], onear[3];
					float oext[3] = {64.0f, 128.0f, 64.0f};
					dtPolyRef oref = 0;
					nav_brush_center(opener, oq);
					nav_quake_to_recast(oq, orc);
					query->findNearestPoly(orc, oext, &plain_filter, &oref, onear);
					if (oref != 0)
					{
						dtPolyRef opath[NAV_MESH_MAX_PATH_REFS];
						int ocount = 0;
						dtStatus os = query->findPath(
							bot_ref, oref, bot_nearest, onear,
							&plain_filter, opath, &ocount, NAV_MESH_MAX_PATH_REFS);
						edict_t *oblk = NULL;
						int ook = 0;
						if (!dtStatusFailed(os) && ocount > 0
							&& !(dtStatusDetail(os, DT_PARTIAL_RESULT)
								&& opath[ocount - 1] != oref))
						{
							int obc = nav_path_block_class(opath, ocount, bot, &oblk);
							if (obc != 1)
								ook = 1;
							else if (oblk == blocker)
							{
								/* A walk-through trigger can sit in the
								   doorway itself: its path is "blocked" by
								   the very door it opens — fine, it fires
								   from the near side. But only if its
								   volume overlaps the door's. A trigger
								   entirely BEYOND the door (start gate:
								   door x -207..-185, trigger x -167..-153)
								   is unreachable from this side — picking
								   it parks the bot grinding the door face. */
								ook = 1;
								for (int ax = 0; ax < 3; ax++)
									if (opener->v.absmin[ax] > blocker->v.absmax[ax]
										|| opener->v.absmax[ax] < blocker->v.absmin[ax])
										ook = 0;
							}
						}
						if (ook)
						{
							dist = (float)ocount * 48.0f;
							cost = (1.0f - want) * dist + 400.0f;
							if (cost < bestcost)
							{
								bestcost = cost;
								best = opener;
								best_path_count = ocount;
								memcpy(best_path, opath, (size_t)ocount * sizeof(dtPolyRef));
								memcpy(best_goal_rc, onear, sizeof(float) * 3);
							}
							routed = 1;
							if (nav_debug_cvar.value)
								fprintf(stderr, "  GOAL %s via opener %s polys=%d cost=%.0f\n",
									pr_strings + (int)it->v.classname,
									pr_strings + (int)opener->v.classname, ocount, cost);
						}
					}
				}
				if (!routed)
					dbg_blocked++;
				continue;
			}
			dist = (float)path_count * 48.0f;
			cost = (1.0f - want) * dist;
			if (bc == 2)
				cost += 200.0f; /* opening the door costs a moment */
		}

		/* Log each reachable item's cost breakdown */
		if (nav_debug_cvar.value)
		{
			float qdist = sqrtf(
				(pos[0]-it->v.origin[0])*(pos[0]-it->v.origin[0]) +
				(pos[1]-it->v.origin[1])*(pos[1]-it->v.origin[1]) +
				(pos[2]-it->v.origin[2])*(pos[2]-it->v.origin[2]));
			fprintf(stderr, "  GOAL %s want=%.2f polys=%d cost=%.0f qdist=%.0f%s\n",
				pr_strings + (int)it->v.classname, want, path_count, cost, qdist,
				(cost < bestcost) ? " *BEST*" : "");
		}

		if (cost < bestcost)
		{
			bestcost = cost;
			best = it;
			best_path_count = path_count;
			memcpy(best_path, path, (size_t)path_count * sizeof(dtPolyRef));
			memcpy(best_goal_rc, nav_item_cache[i].nav_pos, sizeof(float) * 3);
		}
	}

	/* Count reachable polys from bot's position via flood fill.
	   Debug-only: the flood is O(polys) and this path runs every call
	   that finds no goal, so an idle/stuck bot would otherwise re-run it
	   (and print) every frame -- the same unthrottled-spam class fixed
	   for this function's other two Con_Printf calls. */
	if (nav_debug_cvar.value)
	{
		const dtNavMesh *nm = nav_mesh->navmesh;
		const dtMeshTile *tile = nm->getTile(0);
		int total_polys = tile ? tile->header->polyCount : 0;
		int reachable = 0;
		if (total_polys > 0 && dbg_pathed == 0)
		{
			dtPolyRef flood[512];
			int flood_count = 0;
			flood[flood_count++] = bot_ref;
			for (int fi = 0; fi < flood_count && flood_count < 512; fi++)
			{
				const dtMeshTile *ft; const dtPoly *fp;
				if (dtStatusFailed(nm->getTileAndPolyByRef(flood[fi], &ft, &fp)))
					continue;
				for (unsigned int li = fp->firstLink; li != DT_NULL_LINK; li = ft->links[li].next)
				{
					dtPolyRef nb = ft->links[li].ref;
					int found = 0;
					for (int k = 0; k < flood_count; k++)
						if (flood[k] == nb) { found = 1; break; }
					if (!found && flood_count < 512)
						flood[flood_count++] = nb;
				}
			}
			reachable = flood_count;
			fprintf(stderr, "nav_find_goal[%d]: %d cached, %d avail, %d wanted, %d pathed, %d blocked | reachable=%d/%d\n",
				slot, nav_item_count, dbg_avail, dbg_wanted, dbg_pathed, dbg_blocked, reachable, total_polys);
		}
		else
			fprintf(stderr, "nav_find_goal[%d]: %d cached, %d avail, %d wanted, %d pathed, %d blocked\n",
				slot, nav_item_count, dbg_avail, dbg_wanted, dbg_pathed, dbg_blocked);
	}

	/* No item found — roam: pick a random reachable point within 800u */
	if (best == sv.edicts)
	{
		dtPolyRef roam_ref = 0;
		float roam_rc[3];
		dtStatus rs = query->findRandomPointAroundCircle(
			bot_ref, bot_nearest, 800.0f, &plain_filter,
			nav_frand, &roam_ref, roam_rc);
		if (dtStatusSucceed(rs) && roam_ref != 0)
		{
			dtPolyRef path[NAV_MESH_MAX_PATH_REFS];
			int path_count = 0;
			dtStatus ps = query->findPath(
				bot_ref, roam_ref, bot_nearest, roam_rc,
				&plain_filter, path, &path_count, NAV_MESH_MAX_PATH_REFS);
			if (dtStatusSucceed(ps) && path_count > 0
				&& nav_path_block_class(path, path_count, bot, NULL) != 1)
			{
				if (nav_debug_cvar.value)
				{
					float rdx = roam_rc[0] - bot_nearest[0];
					float rdz = roam_rc[2] - bot_nearest[2];
					fprintf(stderr, "nav_find_goal[%d]: ROAM dist=%.0f polys=%d\n",
						slot, sqrtf(rdx*rdx + rdz*rdz), path_count);
				}
				best_path_count = path_count;
				memcpy(best_path, path, (size_t)path_count * sizeof(dtPolyRef));
				memcpy(best_goal_rc, roam_rc, sizeof(float) * 3);
				best = bot; /* sentinel: have a corridor, no item */
			}
		}
	}

	/* Load winning path into corridor */
	if (best != sv.edicts && best_path_count > 0)
	{
		float bot_start_q[3];
		float best_goal_q[3];
		nav_recast_to_quake(bot_nearest, bot_start_q);
		nav_recast_to_quake(best_goal_rc, best_goal_q);

		unsigned long long refs[NAV_MESH_MAX_PATH_REFS];
		for (i = 0; i < best_path_count; i++)
			refs[i] = (unsigned long long)best_path[i];

		if (nav_bot_corridors[slot] == NULL)
			nav_bot_corridors[slot] = nav_corridor_create(NAV_MESH_MAX_PATH_REFS);
		if (nav_bot_corridors[slot] != NULL)
			nav_corridor_set(nav_bot_corridors[slot], nav_mesh,
				bot_start_q, best_goal_q, refs, best_path_count);
	}

	pr_global_struct->self = EDICT_TO_PROG(bot);
	G_INT(OFS_RETURN) = EDICT_TO_PROG(best);
}


/* ---- Entity blocking ---- */

/* Does this door start in a blocked state for navigation?
   Touch-doors (no key, no targetname, no health) open automatically
   when the bot walks into them — leave those unblocked.
   Everything else (key doors, trigger doors, secret doors) starts blocked. */
static int Nav_DoorStartsBlocked(edict_t *e)
{
	/* Triage aid: measure how much unreachability is door-gating. */
	static int all_open = -1;
	if (all_open < 0) all_open = getenv("NAV_ALL_DOORS_OPEN") != NULL;
	if (all_open) return 0;

	if ((int)e->v.items != 0)    return 1; /* key door */
	if (e->v.targetname)         return 1; /* trigger door */
	if (e->v.health > 0)         return 1; /* shootable / secret */
	return 0;                              /* touch door */
}

/* Build entity→poly mapping for all blockable entities (doors).
   Called once after navmesh build. */
static void nav_build_block_map(void)
{
	dtNavMeshQuery *query = nav_mesh->query;
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	nav_block_map_count = 0;
	for (int i = 1; i < sv.num_edicts && nav_block_map_count < NAV_MAX_BLOCK_ENTITIES; i++)
	{
		edict_t *e = EDICT_NUM(i);
		if (e->free) continue;
		const char *cn = pr_strings + (int)e->v.classname;
		if (strcmp(cn, "door") && strncasecmp(cn, "func_door", 9))
			continue;
		if (!Nav_DoorStartsBlocked(e))
			continue;

		/* Find polys in the door's footprint.  Use the full bbox vertical
		   span (+16u pad): door brushes often extend well below the
		   walkable floor (e1m1 t10 bottoms out 112u under it), so a thin
		   slice at absmin misses every floor poly.  Polys on unrelated
		   floors can't match — the AABB only reaches what the door
		   actually spans. */
		float center[3], rc_center[3], extents[3];
		center[0] = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
		center[1] = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
		center[2] = (e->v.absmin[2] + e->v.absmax[2]) * 0.5f;
		nav_quake_to_recast(center, rc_center);

		extents[0] = (e->v.absmax[0] - e->v.absmin[0]) * 0.5f;
		extents[1] = (e->v.absmax[2] - e->v.absmin[2]) * 0.5f + 16.0f; /* Recast Y = Quake Z */
		extents[2] = (e->v.absmax[1] - e->v.absmin[1]) * 0.5f;

		dtPolyRef polys[NAV_MAX_ENTITY_POLYS];
		int poly_count = 0;
		query->queryPolygons(rc_center, extents, &filter, polys, &poly_count, NAV_MAX_ENTITY_POLYS);

		if (nav_debug_cvar.value)
			Con_Printf("Nav: door %s (%.0f %.0f %.0f)-(%.0f %.0f %.0f) -> %d polys\n",
				e->v.targetname ? pr_strings + (int)e->v.targetname : "-",
				e->v.absmin[0], e->v.absmin[1], e->v.absmin[2],
				e->v.absmax[0], e->v.absmax[1], e->v.absmax[2], poly_count);

		if (poly_count > 0)
		{
			int idx = nav_block_map_count++;
			nav_block_map[idx].ent = e;
			nav_block_map[idx].poly_count = poly_count;
			memcpy(nav_block_map[idx].polys, polys, (size_t)poly_count * sizeof(dtPolyRef));
			nav_block_map[idx].is_blocked = 1; /* doors start closed */
			nav_blocked.block(polys, poly_count);
		}
	}

	Con_Printf("Nav: mapped %d doors, %d blocked polys\n", nav_block_map_count, nav_blocked.count);
}

/* void nav_block(entity e) = #89
   QC calls this when an entity becomes impassable (door closes). */
static void PF_nav_block(void)
{
	edict_t *e = G_EDICT(OFS_PARM0);
	for (int i = 0; i < nav_block_map_count; i++)
	{
		if (nav_block_map[i].ent == e && !nav_block_map[i].is_blocked)
		{
			nav_blocked.block(nav_block_map[i].polys, nav_block_map[i].poly_count);
			nav_block_map[i].is_blocked = 1;
			return;
		}
	}
}

/* void nav_unblock(entity e) = #90
   QC calls this when an entity becomes passable (door opens). */
static void PF_nav_unblock(void)
{
	edict_t *e = G_EDICT(OFS_PARM0);
	for (int i = 0; i < nav_block_map_count; i++)
	{
		if (nav_block_map[i].ent == e && nav_block_map[i].is_blocked)
		{
			nav_blocked.unblock(nav_block_map[i].polys, nav_block_map[i].poly_count);
			nav_block_map[i].is_blocked = 0;
			return;
		}
	}
}

/* void nav_report_stats(vector core, vector env, vector lava_entries, string tgt, float tgt_dist) = #92
   Single-line, machine-parseable periodic nav-quality report for the
   calling bot. Replaces the old bprint()-per-token sequence in QC: bprint
   broadcasts each call individually through the server's print hook, so a
   multi-call report fragments into one log line per token (and can
   interleave with other bots' reports under concurrent load). Packing
   into vectors keeps this well within the 8-param VM limit:
     core         = (stuck_pct, no_target_pct, roam_idle_pct)
     env          = (lava_pct, coverage_polys, lava_entries_roam)
     lava_entries = (lava_entries_nav, lava_entries_beeline, lava_entries_combat)
   lava_entries_* are raw counts (not percentages): how many times the bot
   crossed into lava/slime this window while in each movement mode -- the
   QC side already tracked these but never reported them. */
static void PF_nav_report_stats(void)
{
	edict_t *bot = PROG_TO_EDICT(pr_global_struct->self);
	float *core = G_VECTOR(OFS_PARM0);
	float *env = G_VECTOR(OFS_PARM1);
	float *lava_entries = G_VECTOR(OFS_PARM2);
	const char *tgt = G_STRING(OFS_PARM3);
	float tgt_dist = G_FLOAT(OFS_PARM4);

	Con_Printf("NAVSTAT bot=%s stk=%.0f notgt=%.0f roam_idle=%.0f lava=%.0f cov=%.0f "
		"le_nav=%.0f le_beeline=%.0f le_combat=%.0f le_roam=%.0f tgt=%s dist=%.0f\n",
		pr_strings + (int)bot->v.netname,
		core[0], core[1], core[2],
		env[0], env[1], env[2],
		lava_entries[0], lava_entries[1], lava_entries[2],
		tgt, tgt_dist);
}

/* void nav_debug_event(float streak, string tgt, float tgt_dist) = #93
   One-shot forensic dump for a bot that has been continuously "stuck"
   (wants to move, hasn't) for `streak` frames. Everything besides the
   QC-side counters is read straight from the bot's own entvars_t --
   origin/velocity/flags/movetype/waterlevel/groundentity are all
   engine-physics fields, no QC round-trip needed. Investigating dm1/dm5
   bots that go stk=100 cov=0 for 50+s straight: this pins down whether
   it's a wedge (zero velocity, blocked on all sides) or something else
   (e.g. stuck in air, no ground). */
static void PF_nav_debug_event(void)
{
	edict_t *bot = PROG_TO_EDICT(pr_global_struct->self);
	float streak = G_FLOAT(OFS_PARM0);
	const char *tgt = G_STRING(OFS_PARM1);
	float tgt_dist = G_FLOAT(OFS_PARM2);
	const char *ground_cn = "none";
	vec3_t down_start, down_end, fwd_start, fwd_end, hdir;
	trace_t down_tr, fwd_tr;
	const char *down_cn = "none", *fwd_cn = "none";
	float hlen;

	if (bot->v.groundentity)
	{
		edict_t *ge = PROG_TO_EDICT(bot->v.groundentity);
		if (ge != NULL && !ge->free)
			ground_cn = pr_strings + (int)ge->v.classname;
	}

	/* Straight-down probe: how far to the nearest floor, and what plane
	   is it (normal.z tells us if it's walkable vs a slope too steep to
	   ever set FL_ONGROUND). */
	VectorCopy(bot->v.origin, down_start);
	VectorCopy(bot->v.origin, down_end);
	down_end[2] -= 64;
	down_tr = SV_Move(down_start, bot->v.mins, bot->v.maxs, down_end, MOVE_NORMAL, bot);
	if (down_tr.ent != NULL && !down_tr.ent->free)
		down_cn = pr_strings + (int)down_tr.ent->v.classname;

	/* Horizontal-velocity-direction probe: what's directly ahead of the
	   bot's current travel heading, if it has any horizontal velocity. */
	VectorCopy(bot->v.velocity, hdir);
	hdir[2] = 0;
	hlen = Length(hdir);
	VectorCopy(bot->v.origin, fwd_start);
	VectorCopy(bot->v.origin, fwd_end);
	if (hlen > 1)
	{
		VectorScale(hdir, 48.0f / hlen, hdir);
		VectorAdd(fwd_end, hdir, fwd_end);
	}
	fwd_tr = SV_Move(fwd_start, bot->v.mins, bot->v.maxs, fwd_end, MOVE_NORMAL, bot);
	if (fwd_tr.ent != NULL && !fwd_tr.ent->free)
		fwd_cn = pr_strings + (int)fwd_tr.ent->v.classname;

	Con_Printf("NAVSTUCK bot=%s streak=%.0f pos=(%.0f %.0f %.0f) vel=(%.0f %.0f %.0f) "
		"flags=%.0f onground=%d movetype=%.0f water=%.0f ground=%s tgt=%s dist=%.0f "
		"down_frac=%.2f down_norm=(%.2f %.2f %.2f) down_ent=%s "
		"fwd_frac=%.2f fwd_norm=(%.2f %.2f %.2f) fwd_ent=%s\n",
		pr_strings + (int)bot->v.netname, streak,
		bot->v.origin[0], bot->v.origin[1], bot->v.origin[2],
		bot->v.velocity[0], bot->v.velocity[1], bot->v.velocity[2],
		bot->v.flags, ((int)bot->v.flags & FL_ONGROUND) != 0, bot->v.movetype, bot->v.waterlevel,
		ground_cn, tgt, tgt_dist,
		down_tr.fraction, down_tr.plane.normal[0], down_tr.plane.normal[1], down_tr.plane.normal[2], down_cn,
		fwd_tr.fraction, fwd_tr.plane.normal[0], fwd_tr.plane.normal[1], fwd_tr.plane.normal[2], fwd_cn);
}

/* ---- Registration ---- */

#define NAV_BUILTIN_BASE  80
#define NAV_BUILTIN_COUNT 14
#define NAV_BUILTIN_MAX   (NAV_BUILTIN_BASE + NAV_BUILTIN_COUNT)

static builtin_t nav_extended_builtins[NAV_BUILTIN_MAX];

void Nav_RegisterBuiltins(void)
{
	int i;
	if (pr_numbuiltins >= NAV_BUILTIN_MAX) return;

	for (i = 0; i < pr_numbuiltins; i++)
		nav_extended_builtins[i] = pr_builtins[i];
	for (i = pr_numbuiltins; i < NAV_BUILTIN_MAX; i++)
		nav_extended_builtins[i] = pr_builtins[0];

	nav_extended_builtins[NAV_BUILTIN_BASE + 0] = PF_nav_ready;
	nav_extended_builtins[NAV_BUILTIN_BASE + 1] = PF_nav_stub;      /* was nav_move */
	nav_extended_builtins[NAV_BUILTIN_BASE + 2] = PF_nav_stub;      /* was nav_path_start */
	nav_extended_builtins[NAV_BUILTIN_BASE + 3] = PF_nav_stub;      /* was nav_route_cost */
	nav_extended_builtins[NAV_BUILTIN_BASE + 4] = PF_nav_path_steer;
	nav_extended_builtins[NAV_BUILTIN_BASE + 5] = PF_nav_path_debug;
	nav_extended_builtins[NAV_BUILTIN_BASE + 6] = PF_nav_find_goal;
	nav_extended_builtins[NAV_BUILTIN_BASE + 7] = PF_nav_stub;      /* was nav_wp_count */
	nav_extended_builtins[NAV_BUILTIN_BASE + 8] = PF_nav_stub;      /* was nav_wp_pos */
	nav_extended_builtins[NAV_BUILTIN_BASE + 9] = PF_nav_block;
	nav_extended_builtins[NAV_BUILTIN_BASE + 10] = PF_nav_unblock;
	nav_extended_builtins[NAV_BUILTIN_BASE + 11] = PF_nav_stub;     /* was nav_link_info */
	nav_extended_builtins[NAV_BUILTIN_BASE + 12] = PF_nav_report_stats;
	nav_extended_builtins[NAV_BUILTIN_BASE + 13] = PF_nav_debug_event;

	pr_builtins = nav_extended_builtins;
	pr_numbuiltins = NAV_BUILTIN_MAX;

	Cvar_RegisterVariable(&nav_enabled_cvar);
	Cvar_RegisterVariable(&nav_jump_links_cvar);
	Cvar_RegisterVariable(&nav_directed_links_cvar);
	Cvar_RegisterVariable(&nav_gap_jumps_cvar);
	Cvar_RegisterVariable(&nav_rocket_jumps_cvar);
	Cvar_RegisterVariable(&nav_deep_drops_cvar);
	Cvar_RegisterVariable(&nav_swim_links_cvar);
	Cvar_RegisterVariable(&nav_debug_cvar);
	Cvar_SetValue("nav_enabled", 1);
}
