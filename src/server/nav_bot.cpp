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
#include "nav_mesh.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include "Recast.h"

#include <stdlib.h>
#include <string.h>

/* Quake↔Recast coordinate conversion (Quake: X,Y,Z-up  Recast: X,Z,Y-up) */
static inline void nav_q2r(const float *q, float *r) { r[0]=q[0]; r[1]=q[2]; r[2]=q[1]; }
static inline void nav_r2q(const float *r, float *q) { q[0]=r[0]; q[1]=r[2]; q[2]=r[1]; }

#include <math.h>

static float nav_frand(void) { return (float)rand() / (float)RAND_MAX; }

extern "C" {
extern builtin_t *pr_builtins;
extern int pr_numbuiltins;
extern ddef_t *ED_FindField(char *name);
extern ddef_t *ED_FindGlobal(char *name);
}

/* ---- Recast config ---- */

/* cell_size=4 per Recast author (agentRadius / 3..4 for indoor).
   Quake agent radius is 16, so cs=4 gives 4-cell erosion. */
#define NAV_CELL_SIZE                 4.0f
#define NAV_CELL_HEIGHT               2.0f
#define NAV_WALKABLE_SLOPE_ANGLE     45.0f
#define NAV_WALKABLE_HEIGHT          56.0f
#define NAV_WALKABLE_CLIMB           18.0f
#define NAV_WALKABLE_RADIUS          16.0f
#define NAV_MAX_EDGE_LEN            192.0f
#define NAV_MAX_SIMPLIFICATION_ERROR  1.3f
#define NAV_MIN_REGION_SIZE           2
#define NAV_MERGE_REGION_SIZE        20
#define NAV_MAX_VERTS_PER_POLY        6
#define NAV_DETAIL_SAMPLE_DISTANCE    6.0f
#define NAV_DETAIL_SAMPLE_MAX_ERROR   1.0f


/* Jump/drop link detection */
#define NAV_JUMP_HEIGHT_MIN         18.0f  /* below this, walkableClimb handles it */
#define NAV_JUMP_HEIGHT_MAX         48.0f  /* max jump-up height in Quake */
#define NAV_DROP_HEIGHT_MAX        128.0f  /* max drop-down height */
#define NAV_JUMP_PROBE_DIST         48.0f  /* how far to project from edge */
#define NAV_JUMP_LINK_RADIUS        16.0f  /* agent radius */
#define NAV_START_SNAP_MAX_DIST     24.0f
#define NAV_JUMP_LAND_SNAP_MAX_DIST 24.0f

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

static int nav_find_bot_poly(dtNavMeshQuery *query, edict_t *bot, const float *qpos, dtPolyRef *out_ref, float *out_nearest)
{
	dtQueryFilter filter;
	float tight[3];
	float rc_pos[3];
	float test[3];
	bool over_poly = false;
	dtStatus status;

	if (query == NULL || nav_mesh == NULL || qpos == NULL || out_ref == NULL || out_nearest == NULL)
		return 0;

	nav_mesh_setup_filter(&filter);
	memcpy(tight, nav_mesh->query_half_extents_tight, sizeof(tight));
	nav_q2r(qpos, rc_pos);
	*out_ref = 0;
	status = query->findNearestPoly(rc_pos, tight, &filter, out_ref, out_nearest, &over_poly);
	if (dtStatusFailed(status) || *out_ref == 0)
		return 0;

	nav_r2q(out_nearest, test);
	if (nav_xy_dist_sq(qpos, test) > NAV_START_SNAP_MAX_DIST * NAV_START_SNAP_MAX_DIST)
		return 0;
	if (!over_poly && nav_xy_dist_sq(qpos, test) > 8.0f * 8.0f)
		return 0;
	if (!nav_trace_clear_at_height(qpos, test, test[2] + 24.0f, bot))
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
static cvar_t nav_debug_cvar = {"nav_debug", "1"};

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

static int nav_count_face_tris(model_t *m);

/* Static brush entities whose surfaces belong in the navmesh.
   Moving entities (func_plat, func_train, func_door) are excluded —
   they get off-mesh links instead (or nothing, for doors). */
static int nav_is_brush_entity(char *classname)
{
	return !strncasecmp(classname, "func_wall", 9)
		|| !strncasecmp(classname, "func_episodegate", 16)
		|| !strncasecmp(classname, "func_bossgate", 13);
}

/* Count triangles from static brush entity submodels (func_wall etc.)
   These share the worldmodel vertex pool — just different surface ranges. */
static int nav_count_brush_entity_tris(model_t *worldmodel)
{
	int i, total = 0;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		model_t *m;
		if (e->free) continue;
		if (!nav_is_brush_entity(pr_strings + (int)e->v.classname)) continue;
		m = sv.models[(int)e->v.modelindex];
		if (!m || m == worldmodel) continue;
		total += nav_count_face_tris(m);
	}
	return total;
}

/* Emit triangles from brush entity submodels.
   Uses the same vertex pool as worldmodel — indices are already valid. */
static void nav_emit_brush_entity_tris(
	model_t *worldmodel, int *tris, int *tri_write)
{
	int i, si, ei, ec, fe;
	int *fv = NULL;
	int fc = 0;

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		model_t *m;
		int first, num;

		if (e->free) continue;
		if (!nav_is_brush_entity(pr_strings + (int)e->v.classname)) continue;
		m = sv.models[(int)e->v.modelindex];
		if (!m || m == worldmodel) continue;

		first = m->firstmodelsurface;
		num = m->nummodelsurfaces;

		for (si = 0; si < num; si++)
		{
			msurface_t *s = &m->surfaces[first + si];
			if (s->texinfo && (s->texinfo->flags & TEX_SPECIAL)) continue;
			ec = s->numedges;
			if (ec < 3) continue;
			if (ec > fc) { fv = (int *)realloc(fv, (size_t)ec * sizeof(int)); fc = ec; }

			fe = s->firstedge;
			for (ei = 0; ei < ec; ei++)
			{
				int se = worldmodel->surfedges[fe + ei];
				int v = (se >= 0) ? worldmodel->edges[se].v[0]
						  : worldmodel->edges[-se].v[1];
				fv[ei] = v;
			}
			for (ei = 1; ei < ec - 1; ei++)
			{
				tris[(*tri_write)++] = fv[0];
				tris[(*tri_write)++] = fv[ei];
				tris[(*tri_write)++] = fv[ei + 1];
			}
		}
	}
	free(fv);
}

static int nav_count_face_tris(model_t *m)
{
	int first = m->firstmodelsurface, num = m->nummodelsurfaces;
	int i, total = 0;
	for (i = 0; i < num; i++)
	{
		msurface_t *s = &m->surfaces[first + i];
		if (s->texinfo && (s->texinfo->flags & TEX_SPECIAL)) continue;
		if (s->numedges < 3) continue;
		total += s->numedges - 2;
	}
	return total;
}

static int nav_extract_bsp(model_t *worldmodel,
	float **out_verts, int *out_vert_count,
	int **out_tris, int *out_tri_count)
{
	int world_verts, brush_tris, total_verts, total_tris;
	float *verts;
	int *tris, *fv;
	int fc, first, num, si, vi, wi;

	*out_verts = NULL; *out_vert_count = 0;
	*out_tris = NULL;  *out_tri_count = 0;
	if (!worldmodel) return 0;

	world_verts = worldmodel->numvertexes;
	if (world_verts <= 0) return 0;

	brush_tris = nav_count_brush_entity_tris(worldmodel);
	total_verts = world_verts;
	total_tris = nav_count_face_tris(worldmodel) + brush_tris;
	if (total_tris <= 0) return 0;

	verts = (float *)malloc((size_t)total_verts * 3 * sizeof(float));
	tris = (int *)malloc((size_t)total_tris * 3 * sizeof(int));
	if (!verts || !tris) { free(verts); free(tris); return 0; }

	for (vi = 0; vi < world_verts; vi++)
	{
		verts[vi*3+0] = worldmodel->vertexes[vi].position[0];
		verts[vi*3+1] = worldmodel->vertexes[vi].position[1];
		verts[vi*3+2] = worldmodel->vertexes[vi].position[2];
	}

	first = worldmodel->firstmodelsurface;
	num = worldmodel->nummodelsurfaces;
	fv = NULL; fc = 0; wi = 0;

	for (si = 0; si < num; si++)
	{
		msurface_t *s = &worldmodel->surfaces[first + si];
		int ec, fe, ei;

		if (s->texinfo && (s->texinfo->flags & TEX_SPECIAL)) continue;
		/* Skip lava/slime surfaces — bots must not walk on them */
		if (s->texinfo && s->texinfo->texture &&
			(s->texinfo->texture->name[0] == '*') &&
			(!strncasecmp(s->texinfo->texture->name, "*lava", 5) ||
			 !strncasecmp(s->texinfo->texture->name, "*slime", 6)))
			continue;
		ec = s->numedges;
		if (ec < 3) continue;
		if (ec > fc) { fv = (int *)realloc(fv, (size_t)ec * sizeof(int)); fc = ec; }

		fe = s->firstedge;
		for (ei = 0; ei < ec; ei++)
		{
			int se = worldmodel->surfedges[fe + ei];
			int v = (se >= 0) ? worldmodel->edges[se].v[0]
					  : worldmodel->edges[-se].v[1];
			if (v < 0 || v >= world_verts) { free(fv); free(verts); free(tris); return 0; }
			fv[ei] = v;
		}
		for (ei = 1; ei < ec - 1; ei++)
		{
			tris[wi++] = fv[0];
			tris[wi++] = fv[ei];
			tris[wi++] = fv[ei + 1];
		}
	}

	nav_emit_brush_entity_tris(worldmodel, tris, &wi);

	free(fv);
	*out_verts = verts;  *out_vert_count = total_verts;
	*out_tris = tris;    *out_tri_count = wi / 3;
	return 1;
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

/* ---- Platform link detection ---- */

/* Scan func_plat entities.  Create bidirectional links between top and
   bottom positions.  Bot rides the platform to traverse. */
static int nav_collect_platform_links(nav_off_mesh_link_t **out_links)
{
	int i, n = 0, cap = 16;
	nav_off_mesh_link_t *links;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));
	*out_links = links;

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		float top_z, bot_z, travel;
		if (e->free) continue;
		if (strcasecmp(pr_strings + (int)e->v.classname, "func_plat")) continue;

		/* pos1 = top, pos2 = bottom (set by plat spawn code) */
		top_z = e->v.origin[2]; /* plats start at top */
		if (e->v.size[2] > 8)
			bot_z = top_z - e->v.size[2] + 8;
		else
			bot_z = top_z - 64;
		travel = (top_z - bot_z) / (150.0f);

		if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }

		/* Center of platform XY */
		links[n].start[0] = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
		links[n].start[1] = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
		links[n].start[2] = bot_z;
		links[n].end[0] = links[n].start[0];
		links[n].end[1] = links[n].start[1];
		links[n].end[2] = top_z;
		links[n].radius = 64.0f;
		links[n].bidirectional = 1;
		links[n].link_type = AI_PLAT_BOTTOM;
		links[n].height_delta = top_z - bot_z;
		links[n].wait_time = travel;
		links[n].required_speed = 0;
		n++;
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
		if (strcasecmp(pr_strings + (int)e->v.classname, "func_train")) continue;

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

				/* Create link between this path_corner and next */
				if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }
				links[n].start[0] = pc->v.origin[0];
				links[n].start[1] = pc->v.origin[1];
				links[n].start[2] = pc->v.origin[2];
				links[n].end[0] = next_pc->v.origin[0];
				links[n].end[1] = next_pc->v.origin[1];
				links[n].end[2] = next_pc->v.origin[2];
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

/* ---- Jump/drop link detection ---- */

/* Scan boundary edges of an existing navmesh.  For each edge, project
   outward and check if there's a navmesh polygon at a different height.
   If the height delta is in the jumpable/droppable range, create a link. */
/* Callback for nav_mesh_build: detect jump/drop links from contour boundary edges.
   Called mid-build after contours, before Detour finalization. */
static int nav_jump_link_callback(
	const nav_mesh_boundary_edge_t *edges, int edge_count,
	const nav_heightfield_t *hf,
	nav_off_mesh_link_t **out_links,
	void *user_data)
{
	(void)user_data;
	int n = 0, cap = 64, i;
	nav_off_mesh_link_t *links = NULL;

	*out_links = NULL;
	if (edge_count == 0) return 0;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));

	for (i = 0; i < edge_count; i++)
	{
		float test[3], dz;

		/* Project outward from edge midpoint */
		test[0] = edges[i].midpoint[0] + edges[i].normal[0] * NAV_JUMP_PROBE_DIST;
		test[1] = edges[i].midpoint[1] + edges[i].normal[1] * NAV_JUMP_PROBE_DIST;
		test[2] = edges[i].midpoint[2];

		/* Reject links that project straight into a wall. */
		if (!nav_trace_clear_at_height(edges[i].midpoint, test, edges[i].midpoint[2] + 24.0f, NULL))
			continue;

		/* Check heightfield: if the projected point is blocked by a wall, skip */
		if (hf && nav_heightfield_is_blocked(hf, test, edges[i].midpoint[2]))
			continue;

		/* Find walkable floor height at landing point */
		float land_z;
		if (!nav_heightfield_floor_z(hf, test, edges[i].midpoint[2], &land_z))
			continue;
		test[2] = land_z;

		/* Height delta: landing minus edge (Quake Z) */
		dz = land_z - edges[i].midpoint[2];

		/* Jump up: landing is higher */
		if (dz > NAV_JUMP_HEIGHT_MIN && dz < NAV_JUMP_HEIGHT_MAX)
		{
			if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }
			links[n].start[0] = edges[i].midpoint[0];
			links[n].start[1] = edges[i].midpoint[1];
			links[n].start[2] = edges[i].midpoint[2];
			links[n].end[0] = test[0];
			links[n].end[1] = test[1];
			links[n].end[2] = test[2];
			links[n].radius = NAV_JUMP_LINK_RADIUS;
			links[n].bidirectional = 0;
			links[n].link_type = AI_JUMP;
			links[n].height_delta = dz;
			links[n].required_speed = 0; /* TODO: compute from physics */
			n++;
		}
		/* Rocket jump up: landing is too high for normal jump */
		else if (dz > NAV_JUMP_HEIGHT_MAX && dz < NAV_DROP_HEIGHT_MAX)
		{
			if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }
			links[n].start[0] = edges[i].midpoint[0];
			links[n].start[1] = edges[i].midpoint[1];
			links[n].start[2] = edges[i].midpoint[2];
			links[n].end[0] = test[0];
			links[n].end[1] = test[1];
			links[n].end[2] = test[2];
			links[n].radius = NAV_JUMP_LINK_RADIUS;
			links[n].bidirectional = 0;
			links[n].link_type = AI_SUPER_JUMP;
			links[n].height_delta = dz;
			links[n].required_speed = 0;
			n++;
		}
		/* Drop down: landing is lower */
		else if (dz < -NAV_JUMP_HEIGHT_MIN && dz > -NAV_DROP_HEIGHT_MAX)
		{
			if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }
			links[n].start[0] = edges[i].midpoint[0];
			links[n].start[1] = edges[i].midpoint[1];
			links[n].start[2] = edges[i].midpoint[2];
			links[n].end[0] = test[0];
			links[n].end[1] = test[1];
			links[n].end[2] = test[2];
			links[n].radius = NAV_JUMP_LINK_RADIUS;
			links[n].bidirectional = 0;
			links[n].link_type = AI_DROP;
			links[n].height_delta = dz;
			links[n].required_speed = 0;
			n++;
		}
	}

	*out_links = links;
	{
		int nj = 0, nd = 0, nr = 0;
		int li;
		for (li = 0; li < n; li++)
		{
			if (links[li].link_type == AI_JUMP) nj++;
			else if (links[li].link_type == AI_DROP) nd++;
			else if (links[li].link_type == AI_SUPER_JUMP) nr++;
		}
		Con_Printf("Nav: detected %d links from %d edges (%d jump, %d drop, %d rj)\n",
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
		nav_q2r(qpos, rpos);

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

	if (!nav_extract_bsp(sv.worldmodel, &verts, &vert_count, &tris, &tri_count))
	{
		Con_Printf("Nav: BSP extraction failed\n");
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

	/* Only teleporters for now — plats/trains need runtime ride behavior */
	entity_count = nav_collect_teleporters(&entity_links);

	Con_Printf("Nav: %d teleporter links\n", entity_count);

	/* Single-pass build: entity links provided upfront, jump/drop links
	   detected mid-build via callback after contours are ready. */
	memset(&summary, 0, sizeof(summary));
	memset(error, 0, sizeof(error));
	nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
		&config, entity_links, entity_count, &summary,
		NULL, NULL, /* jump link callback disabled for now */
		error, sizeof(error));

	if (nav_mesh == NULL)
	{
		Con_Printf("Nav: build failed: %s\n", error);
		free(verts); free(tris); free(entity_links);
		return;
	}

	free(verts);
	free(tris);
	free(entity_links);

	nav_build_block_map();

	t_done = Sys_FloatTime();
	Con_Printf("Nav: %.3fs, %d polys, %d entity links\n",
		t_done - t_start, summary.polygon_count, entity_count);

	/* ---- DM4 waypoint edge diagnostic ---- */
	if (nav_mesh != NULL && !strcasecmp(sv.name, "dm4"))
	{
		typedef struct { int from, to, flags; } dm4_edge_t;
		static float dm4_wps[][3] = {
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
			{314,-1027,-82},{336,-801,-82},{-61,573,-274}
		};
		static dm4_edge_t dm4_edges[] = {
			{0,32,2},{0,33,2},{0,46,2},{1,17,0},{1,16,0},{2,56,0},{2,15,0},
			{2,7,0},{3,21,0},{3,27,0},{4,23,0},{4,26,0},{5,45,0},{5,58,0},
			{6,56,0},{6,8,0},{6,7,0},{6,57,0},{7,2,0},{7,56,0},{7,57,0},
			{7,6,0},{8,57,0},{8,56,0},{8,6,0},{8,2,0},{9,57,0},{9,53,0},
			{9,10,0},{10,54,0},{11,53,0},{11,10,0},{12,53,0},{12,14,0},
			{12,13,0},{13,12,0},{13,14,0},{13,52,0},{13,34,0},{14,12,4},
			{14,13,4},{14,34,4},{15,2,0},{15,16,0},{16,15,0},{16,17,0},
			{17,16,4},{17,18,4},{17,42,4},{18,17,0},{18,19,0},{19,18,0},
			{19,20,0},{19,22,0},{19,12,0},{20,19,0},{20,53,0},{20,12,0},
			{20,21,0},{21,20,0},{21,3,0},{21,50,0},{21,13,0},{22,19,0},
			{22,23,0},{22,53,0},{23,22,0},{23,24,0},{23,4,0},{24,23,384},
			{24,51,384},{25,51,0},{26,4,2},{26,27,2},{26,55,2},{27,47,0},
			{27,3,0},{28,47,0},{28,29,0},{28,30,0},{28,0,0},{29,28,0},
			{30,31,0},{30,32,0},{31,30,0},{31,32,0},{32,31,0},{32,30,0},
			{32,0,0},{33,34,0},{33,43,0},{34,33,128},{34,40,128},
			{34,35,128},{35,34,0},{35,36,0},{35,41,0},{36,35,0},{36,37,0},
			{36,45,0},{37,36,0},{38,40,0},{38,39,0},{39,38,0},{39,44,0},
			{39,48,0},{39,42,0},{40,34,0},{40,38,0},{41,13,4096},
			{42,48,2},{42,17,2},{42,44,2},{43,33,0},{44,39,0},{44,48,0},
			{44,42,0},{45,36,128},{45,5,128},{46,47,4096},{47,27,0},
			{47,28,0},{47,32,0},{47,30,0},{48,42,0},{48,44,0},{48,39,0},
			{49,50,0},{49,13,0},{50,21,4096},{50,49,4096},{51,24,384},
			{51,25,384},{52,25,0},{53,34,0},{53,12,0},{53,11,0},{53,9,0},
			{54,21,4096},{55,26,0},{56,57,0},{56,2,0},{56,6,0},{56,8,0},
			{57,9,0},{57,8,0},{57,7,0},{57,56,0},{58,5,0}
		};
		#define DM4_EDGE_COUNT 149
		static const char *flag_names[] = {
			"WALK","","TELE","","JUMP","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","","","","","","","","","","","","","","",
			"","","","","","SNIP"
		};
		nav_mesh_path_result_t res;
		char err[256];
		int ei, ok = 0, fail = 0;
		const char *fn;

		Con_Printf("Nav: DM4 waypoint edge diagnostic (%d edges)\n", DM4_EDGE_COUNT);
		for (ei = 0; ei < DM4_EDGE_COUNT; ei++)
		{
			int f = dm4_edges[ei].from;
			int t = dm4_edges[ei].to;
			int fl = dm4_edges[ei].flags;
			float dz = dm4_wps[t][2] - dm4_wps[f][2];

			memset(&res, 0, sizeof(res));
			memset(err, 0, sizeof(err));
			if (nav_mesh_find_path(nav_mesh, dm4_wps[f], dm4_wps[t], &res, err, sizeof(err))
				&& res.found)
			{
				ok++;
			}
			else
			{
				fn = (fl == 2) ? "TELE" : (fl == 4) ? "JUMP" :
					(fl == 4096) ? "RJ" : (fl == 128) ? "SNIP" :
					(fl == 384) ? "SJ" : "WALK";
				Con_Printf("  FAIL wp%d->wp%d [%s] dz=%.0f: %s\n",
					f+1, t+1, fn, dz, err);
				fail++;
			}
		}
		Con_Printf("Nav: DM4 edges: %d OK, %d FAIL / %d total\n", ok, fail, DM4_EDGE_COUNT);

		/* Waypoint coverage: find nearest poly for each waypoint with huge extents */
		Con_Printf("Nav: DM4 waypoint coverage:\n");
		{
			nav_mesh_nearest_result_t nr;
			char nerr[256];
			int wi, on_mesh = 0;
			for (wi = 0; wi < 59; wi++)
			{
				float dx, dy, dz, dist;
				memset(&nr, 0, sizeof(nr));
				memset(nerr, 0, sizeof(nerr));
				nav_mesh_find_nearest(nav_mesh, dm4_wps[wi], &nr, nerr, sizeof(nerr));
				if (nr.found)
				{
					dx = nr.nearest_point[0] - dm4_wps[wi][0];
					dy = nr.nearest_point[1] - dm4_wps[wi][1];
					dz = nr.nearest_point[2] - dm4_wps[wi][2];
					dist = sqrt(dx*dx + dy*dy + dz*dz);
					if (dist > 32)
						Con_Printf("  wp%d (%.0f %.0f %.0f) -> poly at (%.0f %.0f %.0f) dist=%.0f %s\n",
							wi+1, dm4_wps[wi][0], dm4_wps[wi][1], dm4_wps[wi][2],
							nr.nearest_point[0], nr.nearest_point[1], nr.nearest_point[2],
							dist, nr.is_over_poly ? "OVER" : "off");
					else
						on_mesh++;
				}
				else
					Con_Printf("  wp%d (%.0f %.0f %.0f) -> NO POLY FOUND\n",
						wi+1, dm4_wps[wi][0], dm4_wps[wi][1], dm4_wps[wi][2]);
			}
			Con_Printf("Nav: %d/59 waypoints within 32u of mesh\n", on_mesh);
		}

		#undef DM4_EDGE_COUNT
	}
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

		/* dump navmesh to OBJ file for external viewing */
		{
			FILE *fp = fopen("navmesh_debug.obj", "w");
			if (fp)
			{
				int vi = 1;
				for (i = 0; i < nav_debug_poly_count; i++)
				{
					float cx = nav_debug_polys[i].center[0];
					float cy = nav_debug_polys[i].center[1];
					float cz = nav_debug_polys[i].center[2] + 48;
					fprintf(fp, "v %f %f %f\n", cx - 4, cy, cz);
					fprintf(fp, "v %f %f %f\n", cx + 4, cy, cz);
					fprintf(fp, "v %f %f %f\n", cx, cy - 4, cz);
					fprintf(fp, "v %f %f %f\n", cx, cy + 4, cz);
					fprintf(fp, "f %d %d %d\n", vi, vi+1, vi+2);
					fprintf(fp, "f %d %d %d\n", vi, vi+2, vi+3);
					vi += 4;
				}
				fclose(fp);
				Con_Printf("Nav: wrote navmesh_debug.obj (%d polys)\n", nav_debug_poly_count);
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

	if (!navigate(nav_bot_corridors[slot], nav_mesh, pos,
		corner, &flags, &ref))
		return;

	G_FLOAT(OFS_RETURN + 0) = corner[0];
	G_FLOAT(OFS_RETURN + 1) = corner[1];
	G_FLOAT(OFS_RETURN + 2) = corner[2];

	/* encode off-mesh link type */
	if (flags & 0x04) /* DT_STRAIGHTPATH_OFFMESH_CONNECTION */
	{
		int lt = nav_mesh_get_link_type(nav_mesh, ref);
		if (lt > 0)
			G_FLOAT(OFS_RETURN + 2) = corner[2] + 10000.0f + (float)lt;
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

	/* world entity: return navmesh poly centers for debug visualization.
	   index == -1 → (poly_count, 0, 0).  index >= 0 → poly center + 48u Z. */
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
			G_FLOAT(OFS_RETURN + 2) = nav_debug_polys[idx].center[2] + 48;
		}
		return;
	}

	/* bot entity: corridor doesn't support random access. */
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

	/* Find bot's current poly (once for all candidates) */
	float bot_nearest[3];
	dtPolyRef bot_ref = 0;
	if (!nav_find_bot_poly(query, bot, pos, &bot_ref, bot_nearest)) return;

	bestcost = 999999.0f;
	best = sv.edicts;

	int dbg_avail = 0, dbg_wanted = 0, dbg_pathed = 0, dbg_blocked = 0;

	for (i = 0; i < nav_item_count; i++)
	{
		it = nav_item_cache[i].ent;
		if (it->free) continue;

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
			Con_Printf("  findPath FAIL to %s\n", pr_strings + (int)it->v.classname);
			continue;
		}
		if (dtStatusDetail(status, DT_PARTIAL_RESULT))
		{
			if (path[path_count - 1] != nav_item_cache[i].poly_ref)
			{
				Con_Printf("  PARTIAL to %s (%d polys)\n", pr_strings + (int)it->v.classname, path_count);
				continue;
			}
		}

		dbg_pathed++;

		if (nav_blocked.path_blocked(path, path_count))
		{
			dbg_blocked++;
			continue;
		}

		dist = (float)path_count * 48.0f;
		cost = (1.0f - want) * dist;
		if (cost < bestcost)
		{
			bestcost = cost;
			best = it;
			best_path_count = path_count;
			memcpy(best_path, path, (size_t)path_count * sizeof(dtPolyRef));
			memcpy(best_goal_rc, nav_item_cache[i].nav_pos, sizeof(float) * 3);
		}
	}

	/* Count reachable polys from bot's position via flood fill */
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
			Con_Printf("nav_find_goal[%d]: %d cached, %d avail, %d wanted, %d pathed, %d blocked | reachable=%d/%d\n",
				slot, nav_item_count, dbg_avail, dbg_wanted, dbg_pathed, dbg_blocked, reachable, total_polys);
		}
		else
			Con_Printf("nav_find_goal[%d]: %d cached, %d avail, %d wanted, %d pathed, %d blocked\n",
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
			if (dtStatusSucceed(ps) && path_count > 0)
			{
				{
					float rdx = roam_rc[0] - bot_nearest[0];
					float rdz = roam_rc[2] - bot_nearest[2];
					Con_Printf("nav_find_goal[%d]: ROAM dist=%.0f polys=%d\n",
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
		nav_r2q(bot_nearest, bot_start_q);
		nav_r2q(best_goal_rc, best_goal_q);

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

		/* Find polys in the door's floor footprint.
		   Use bottom of door bbox with thin vertical slice to avoid
		   matching polys on other floors above/below the doorway. */
		float center[3], rc_center[3], extents[3];
		center[0] = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
		center[1] = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
		center[2] = e->v.absmin[2]; /* floor of door */
		nav_q2r(center, rc_center);

		extents[0] = (e->v.absmax[0] - e->v.absmin[0]) * 0.5f;
		extents[1] = 32.0f; /* Recast Y = Quake Z, thin vertical slice */
		extents[2] = (e->v.absmax[1] - e->v.absmin[1]) * 0.5f;

		dtPolyRef polys[NAV_MAX_ENTITY_POLYS];
		int poly_count = 0;
		query->queryPolygons(rc_center, extents, &filter, polys, &poly_count, NAV_MAX_ENTITY_POLYS);

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

/* ---- Registration ---- */

#define NAV_BUILTIN_BASE  80
#define NAV_BUILTIN_COUNT 11
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

	pr_builtins = nav_extended_builtins;
	pr_numbuiltins = NAV_BUILTIN_MAX;

	Cvar_RegisterVariable(&nav_enabled_cvar);
	Cvar_RegisterVariable(&nav_jump_links_cvar);
	Cvar_RegisterVariable(&nav_debug_cvar);
	Cvar_SetValue("nav_enabled", 1);
}
