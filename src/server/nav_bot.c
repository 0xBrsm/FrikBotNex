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
#include "quakedef.h"
#include "nav_bot.h"
#include "nav_mesh.h"

#include <stdlib.h>
#include <string.h>
#include <math.h>

extern builtin_t *pr_builtins;
extern int pr_numbuiltins;

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

#define NAV_ADVANCE_RADIUS          24.0f
#define NAV_ADVANCE_HEIGHT          36.0f  /* max Z delta for waypoint advance */
#define NAV_STALE_TIME               8.0f

/* Jump/drop link detection */
#define NAV_JUMP_HEIGHT_MIN         18.0f  /* below this, walkableClimb handles it */
#define NAV_JUMP_HEIGHT_MAX         48.0f  /* max jump-up height in Quake */
#define NAV_DROP_HEIGHT_MAX        128.0f  /* max drop-down height */
#define NAV_JUMP_PROBE_DIST         48.0f  /* how far to project from edge */
#define NAV_JUMP_LINK_RADIUS        16.0f  /* agent radius */

/* ---- Per-bot path cache ---- */

typedef struct {
	float	goal[3];
	int	valid;
	float	points[NAV_MESH_MAX_STRAIGHT_POINTS * 3]; /* Quake coords */
	unsigned char flags[NAV_MESH_MAX_STRAIGHT_POINTS];
	unsigned long long refs[NAV_MESH_MAX_STRAIGHT_POINTS]; /* poly refs for link lookup */
	int	point_count;
	int	current_index;		/* next waypoint to steer toward */
	double	compute_time;
	double	last_advance_time;
	float	last_wp_dist_sq;	/* previous XY distance² to current waypoint */
} nav_bot_path_t;

static nav_bot_path_t nav_bot_paths[MAX_SCOREBOARD];

static int nav_bot_slot(void)
{
	edict_t *e = PROG_TO_EDICT(pr_global_struct->self);
	int num = NUM_FOR_EDICT(e);
	if (num < 1 || num > svs.maxclients) return -1;
	return num - 1;
}

static nav_mesh_runtime_t *nav_mesh = NULL;
static int nav_build_attempted = 0;
static struct model_s *nav_built_for_model = NULL;
static cvar_t nav_enabled_cvar = {"nav_enabled", "0"};
static cvar_t nav_jump_links_cvar = {"nav_jump_links", "1"};

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

static int nav_is_door(char *classname)
{
	return !Q_strncasecmp(classname, "func_door", 9);
}

/* Is this a solid brush entity whose surfaces should be in the navmesh? */
static int nav_is_brush_entity(char *classname)
{
	return !Q_strncasecmp(classname, "func_wall", 9)
		|| !Q_strncasecmp(classname, "func_plat", 9)
		|| !Q_strncasecmp(classname, "func_train", 10)
		|| !Q_strncasecmp(classname, "func_episodegate", 16)
		|| !Q_strncasecmp(classname, "func_bossgate", 13);
}

static int nav_count_door_quads(void)
{
	int i, count = 0;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		if (e->free) continue;
		if (!nav_is_door(pr_strings + (int)e->v.classname)) continue;
		if (e->v.absmin[0] >= e->v.absmax[0] || e->v.absmin[1] >= e->v.absmax[1])
			continue;
		count++;
	}
	return count;
}

static void nav_emit_door_quads(
	float *verts, int *tris, int *vert_count, int *tri_write)
{
	int i;
	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		float x0, y0, x1, y1, z;
		int base;

		if (e->free) continue;
		if (!nav_is_door(pr_strings + (int)e->v.classname)) continue;
		x0 = e->v.absmin[0]; y0 = e->v.absmin[1];
		x1 = e->v.absmax[0]; y1 = e->v.absmax[1];
		if (x0 >= x1 || y0 >= y1) continue;

		z = e->v.absmin[2];
		base = *vert_count;
		*vert_count += 4;

		verts[(base+0)*3+0]=x0; verts[(base+0)*3+1]=y0; verts[(base+0)*3+2]=z;
		verts[(base+1)*3+0]=x1; verts[(base+1)*3+1]=y0; verts[(base+1)*3+2]=z;
		verts[(base+2)*3+0]=x1; verts[(base+2)*3+1]=y1; verts[(base+2)*3+2]=z;
		verts[(base+3)*3+0]=x0; verts[(base+3)*3+1]=y1; verts[(base+3)*3+2]=z;

		tris[(*tri_write)++]=base;   tris[(*tri_write)++]=base+1; tris[(*tri_write)++]=base+2;
		tris[(*tri_write)++]=base;   tris[(*tri_write)++]=base+2; tris[(*tri_write)++]=base+3;
	}
}

/* Count triangles from brush entity submodels (func_wall, func_plat, etc.)
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
	int world_verts, door_quads, brush_tris, total_verts, total_tris;
	float *verts;
	int *tris, *fv;
	int fc, first, num, si, vi, wi;

	*out_verts = NULL; *out_vert_count = 0;
	*out_tris = NULL;  *out_tri_count = 0;
	if (!worldmodel) return 0;

	world_verts = worldmodel->numvertexes;
	if (world_verts <= 0) return 0;

	door_quads = nav_count_door_quads();
	brush_tris = nav_count_brush_entity_tris(worldmodel);
	total_verts = world_verts + door_quads * 4;
	total_tris = nav_count_face_tris(worldmodel) + door_quads * 2 + brush_tris;
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
			(!Q_strncasecmp(s->texinfo->texture->name, "*lava", 5) ||
			 !Q_strncasecmp(s->texinfo->texture->name, "*slime", 6)))
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

	total_verts = world_verts;
	nav_emit_door_quads(verts, tris, &total_verts, &wi);
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
		if (!e->free && !Q_strcasecmp(pr_strings + (int)e->v.classname, "trigger_teleport"))
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
		if (Q_strcasecmp(pr_strings + (int)src->v.classname, "trigger_teleport")) continue;
		tgt = src->v.target ? pr_strings + (int)src->v.target : "";
		if (!tgt[0]) continue;

		for (j = 1; j < sv.num_edicts; j++)
		{
			edict_t *dst = EDICT_NUM(j);
			const char *tn;
			if (dst->free) continue;
			if (Q_strcasecmp(pr_strings + (int)dst->v.classname, "info_teleport_destination"))
				continue;
			tn = dst->v.targetname ? pr_strings + (int)dst->v.targetname : "";
			if (Q_strcmp(tgt, tn)) continue;

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
		if (Q_strcasecmp(pr_strings + (int)e->v.classname, "func_plat")) continue;

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
		if (Q_strcasecmp(pr_strings + (int)e->v.classname, "func_train")) continue;

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
			if (Q_strcasecmp(pr_strings + (int)pc->v.classname, "path_corner")) continue;
			pcname = pc->v.targetname ? pr_strings + (int)pc->v.targetname : "";
			if (Q_strcmp(tgt, pcname)) continue;

			/* Found start — follow chain, create links between stops */
			pctgt = pc->v.target ? pr_strings + (int)pc->v.target : "";
			if (!pctgt[0]) break;

			for (k = 1; k < sv.num_edicts; k++)
			{
				next_pc = EDICT_NUM(k);
				const char *nname;
				float dist;
				if (next_pc->free) continue;
				if (Q_strcasecmp(pr_strings + (int)next_pc->v.classname, "path_corner")) continue;
				nname = next_pc->v.targetname ? pr_strings + (int)next_pc->v.targetname : "";
				if (Q_strcmp(pctgt, nname)) continue;

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

/* Scan func_door entities that block passages.  Create links through
   them so bots know to wait/trigger the door. */
static int nav_collect_door_links(nav_off_mesh_link_t **out_links)
{
	int i, n = 0, cap = 16;
	nav_off_mesh_link_t *links;

	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));
	*out_links = links;

	for (i = 1; i < sv.num_edicts; i++)
	{
		edict_t *e = EDICT_NUM(i);
		float cx, cy, cz, dx, dy;
		if (e->free) continue;
		if (Q_strncasecmp(pr_strings + (int)e->v.classname, "func_door", 9)) continue;
		if (e->v.absmin[0] >= e->v.absmax[0]) continue;

		cx = (e->v.absmin[0] + e->v.absmax[0]) * 0.5f;
		cy = (e->v.absmin[1] + e->v.absmax[1]) * 0.5f;
		cz = e->v.absmin[2]; /* floor of door */
		dx = e->v.absmax[0] - e->v.absmin[0];
		dy = e->v.absmax[1] - e->v.absmin[1];

		if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }

		/* Link through the door along its thinnest axis */
		if (dx < dy)
		{
			links[n].start[0] = cx - dx * 0.5f - 32;
			links[n].start[1] = cy;
			links[n].end[0] = cx + dx * 0.5f + 32;
			links[n].end[1] = cy;
		}
		else
		{
			links[n].start[0] = cx;
			links[n].start[1] = cy - dy * 0.5f - 32;
			links[n].end[0] = cx;
			links[n].end[1] = cy + dy * 0.5f + 32;
		}
		links[n].start[2] = cz;
		links[n].end[2] = cz;
		links[n].radius = 32.0f;
		links[n].bidirectional = 1;
		links[n].link_type = AI_DOORFLAG;
		links[n].height_delta = 0;
		links[n].wait_time = 4.0f;
		links[n].required_speed = 0;
		n++;
	}

	*out_links = links;
	return n;
}

/* ---- Jump/drop link detection ---- */

/* Scan boundary edges of an existing navmesh.  For each edge, project
   outward and check if there's a navmesh polygon at a different height.
   If the height delta is in the jumpable/droppable range, create a link. */
static int nav_collect_jump_links(
	nav_mesh_runtime_t *mesh,
	nav_off_mesh_link_t **out_links)
{
	nav_mesh_boundary_edge_t *edges = NULL;
	int edge_count = 0, n = 0, cap = 0, i;
	nav_off_mesh_link_t *links = NULL;
	nav_mesh_nearest_result_t probe;
	char error[256];

	*out_links = NULL;
	if (!nav_mesh_collect_boundary_edges(mesh, &edges, &edge_count) || edge_count == 0)
		return 0;

	cap = 64;
	links = (nav_off_mesh_link_t *)calloc(cap, sizeof(*links));

	for (i = 0; i < edge_count; i++)
	{
		float test[3], dz;

		/* Project outward from edge midpoint */
		test[0] = edges[i].midpoint[0] + edges[i].normal[0] * NAV_JUMP_PROBE_DIST;
		test[1] = edges[i].midpoint[1] + edges[i].normal[1] * NAV_JUMP_PROBE_DIST;
		test[2] = edges[i].midpoint[2]; /* same height — let findNearest snap vertically */

		/* Check if there's a navmesh polygon at the projected point */
		memset(&probe, 0, sizeof(probe));
		if (!nav_mesh_find_nearest(mesh, test, &probe, error, sizeof(error)))
			continue;
		if (!probe.found)
			continue;

		/* Height delta: landing minus edge (Quake Z) */
		dz = probe.nearest_point[2] - edges[i].midpoint[2];

		/* Jump up: landing is higher */
		if (dz > NAV_JUMP_HEIGHT_MIN && dz < NAV_JUMP_HEIGHT_MAX)
		{
			if (n >= cap) { cap *= 2; links = (nav_off_mesh_link_t *)realloc(links, cap * sizeof(*links)); }
			links[n].start[0] = edges[i].midpoint[0];
			links[n].start[1] = edges[i].midpoint[1];
			links[n].start[2] = edges[i].midpoint[2];
			links[n].end[0] = probe.nearest_point[0];
			links[n].end[1] = probe.nearest_point[1];
			links[n].end[2] = probe.nearest_point[2];
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
			links[n].end[0] = probe.nearest_point[0];
			links[n].end[1] = probe.nearest_point[1];
			links[n].end[2] = probe.nearest_point[2];
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
			links[n].end[0] = probe.nearest_point[0];
			links[n].end[1] = probe.nearest_point[1];
			links[n].end[2] = probe.nearest_point[2];
			links[n].radius = NAV_JUMP_LINK_RADIUS;
			links[n].bidirectional = 0;
			links[n].link_type = AI_DROP;
			links[n].height_delta = dz;
			links[n].required_speed = 0;
			n++;
		}
	}

	free(edges);
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
	nav_off_mesh_link_t *entity_links, *jump_links, *all_links;
	nav_off_mesh_link_t *tmp;
	int entity_count, jump_count, all_count, tmp_count;
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

	nav_default_config(&config);

	/* Collect all entity-based links */
	entity_count = 0;
	entity_links = NULL;

	tmp_count = nav_collect_teleporters(&tmp);
	if (tmp_count > 0)
	{
		all_count = nav_merge_links(entity_links, entity_count, tmp, tmp_count, &all_links);
		free(entity_links); free(tmp);
		entity_links = all_links; entity_count = all_count;
	}

	tmp_count = nav_collect_platform_links(&tmp);
	if (tmp_count > 0)
	{
		all_count = nav_merge_links(entity_links, entity_count, tmp, tmp_count, &all_links);
		free(entity_links); free(tmp);
		entity_links = all_links; entity_count = all_count;
	}

	tmp_count = nav_collect_train_links(&tmp);
	if (tmp_count > 0)
	{
		all_count = nav_merge_links(entity_links, entity_count, tmp, tmp_count, &all_links);
		free(entity_links); free(tmp);
		entity_links = all_links; entity_count = all_count;
	}

	tmp_count = nav_collect_door_links(&tmp);
	if (tmp_count > 0)
	{
		all_count = nav_merge_links(entity_links, entity_count, tmp, tmp_count, &all_links);
		free(entity_links); free(tmp);
		entity_links = all_links; entity_count = all_count;
	}

	Con_Printf("Nav: %d entity links (tele+plat+train+door)\n", entity_count);

	/* First pass: build with entity links */
	memset(&summary, 0, sizeof(summary));
	memset(error, 0, sizeof(error));
	nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
		&config, entity_links, entity_count, &summary, error, sizeof(error));

	if (nav_mesh == NULL)
	{
		Con_Printf("Nav: first pass failed: %s\n", error);
		free(verts); free(tris); free(entity_links);
		return;
	}

	/* Second pass: detect jump/drop links from boundary edges, rebuild */
	jump_count = 0;
	jump_links = NULL;
	if (!nav_jump_links_cvar.value)
		goto skip_jump_links;
	jump_count = nav_collect_jump_links(nav_mesh, &jump_links);
	if (jump_count > 0)
	{
		all_count = nav_merge_links(entity_links, entity_count,
			jump_links, jump_count, &all_links);

		nav_mesh_destroy(nav_mesh);
		nav_mesh = NULL;

		memset(&summary, 0, sizeof(summary));
		memset(error, 0, sizeof(error));
		nav_mesh = nav_mesh_build(verts, vert_count, tris, tri_count,
			&config, all_links, all_count, &summary, error, sizeof(error));
		free(all_links);

		if (nav_mesh == NULL)
		{
			Con_Printf("Nav: second pass failed: %s\n", error);
			free(verts); free(tris); free(entity_links); free(jump_links);
			return;
		}
	}

skip_jump_links:
	free(verts);
	free(tris);
	free(entity_links);
	free(jump_links);

	t_done = Sys_FloatTime();
	Con_Printf("Nav: %.3fs, %d polys, %d entity + %d jump links\n",
		t_done - t_start, summary.polygon_count, entity_count, jump_count);
}

void Nav_Shutdown(void)
{
	if (nav_mesh != NULL)
	{
		nav_mesh_destroy(nav_mesh);
		nav_mesh = NULL;
	}
	nav_build_attempted = 0;
	memset(nav_bot_paths, 0, sizeof(nav_bot_paths));
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

static void PF_nav_ready(void)
{
	Nav_EnsureBuilt();
	G_FLOAT(OFS_RETURN) = (nav_mesh != NULL) ? 1.0f : 0.0f;
}

static void PF_nav_route_cost(void)
{
	float *start, *goal;
	nav_mesh_path_result_t result;
	char error[256];

	start = G_VECTOR(OFS_PARM0);
	goal = G_VECTOR(OFS_PARM1);
	G_FLOAT(OFS_RETURN) = -1.0f;
	if (nav_mesh == NULL) return;

	memset(&result, 0, sizeof(result));
	if (nav_mesh_find_path(nav_mesh, start, goal, &result, error, sizeof(error))
		&& result.found)
		G_FLOAT(OFS_RETURN) = result.travel_distance;
}

/* float nav_path_start(vector goal) = #82
   Compute and cache a path from self's position to goal.
   Returns waypoint count on success, -1 on failure. */
static void PF_nav_path_start(void)
{
	float *goal;
	edict_t *e;
	float pos[3];
	int slot;
	nav_bot_path_t *bp;
	nav_mesh_path_result_t result;
	char error[256];

	G_FLOAT(OFS_RETURN) = -1.0f;
	goal = G_VECTOR(OFS_PARM0);

	slot = nav_bot_slot();
	if (slot < 0) return;
	bp = &nav_bot_paths[slot];

	if (nav_mesh == NULL) { bp->valid = 0; return; }

	e = PROG_TO_EDICT(pr_global_struct->self);
	pos[0] = e->v.origin[0];
	pos[1] = e->v.origin[1];
	pos[2] = e->v.origin[2];

	memset(&result, 0, sizeof(result));
	if (!nav_mesh_find_path(nav_mesh, pos, goal, &result, error, sizeof(error))
		|| !result.found || result.straight_point_count < 1)
	{
		Con_Printf("NAV_FAIL [%d]: %s\n", slot, error);
		bp->valid = 0;
		return;
	}

	/* Reject if start snapped too far vertically (wrong floor).
	   Relaxed from strict isOverPoly — allows bots slightly off-mesh
	   (on steps, lips) while still rejecting cross-floor snaps. */
	{
		float start_snap_dz = result.start_point[2] - pos[2];
		if (start_snap_dz < 0) start_snap_dz = -start_snap_dz;
		if (start_snap_dz > NAV_ADVANCE_HEIGHT)
		{
			bp->valid = 0;
			return;
		}
	}

	/* Cache the straight path */
	bp->point_count = result.straight_point_count;
	memcpy(bp->points, result.straight_points,
		(size_t)bp->point_count * 3 * sizeof(float));
	memcpy(bp->flags, result.straight_flags,
		(size_t)bp->point_count);
	memcpy(bp->refs, result.straight_refs,
		(size_t)bp->point_count * sizeof(unsigned long long));
	bp->current_index = 1; /* index 0 is start position */
	if (bp->current_index >= bp->point_count)
		bp->current_index = bp->point_count - 1;
	bp->goal[0] = goal[0];
	bp->goal[1] = goal[1];
	bp->goal[2] = goal[2];
	bp->valid = 1;
	bp->compute_time = sv.time;
	bp->last_advance_time = sv.time;
	bp->last_wp_dist_sq = 1e18f; /* force first progress check to succeed */

	/* Dump waypoints for debugging */
	{
		int i;
		Con_Printf("NAV_PATH [%d]: %d wps, start=(%.0f %.0f %.0f) goal=(%.0f %.0f %.0f)\n",
			slot, bp->point_count, pos[0], pos[1], pos[2],
			goal[0], goal[1], goal[2]);
		for (i = 0; i < bp->point_count && i < 8; i++)
		{
			float *wp = &bp->points[i * 3];
			Con_Printf("  wp%d: (%.0f %.0f %.0f) flags=0x%02x%s\n",
				i, wp[0], wp[1], wp[2], bp->flags[i],
				(bp->flags[i] & 0x04) ? " OFFMESH" :
				(bp->flags[i] & 0x02) ? " END" : "");
		}
	}

	G_FLOAT(OFS_RETURN) = (float)bp->point_count;
}

/* vector nav_path_steer(vector pos) = #84
   Return steering vector to the next cached waypoint.
   X,Y = normalized 2D direction.  Z = height delta to next waypoint
   (positive = step up, negative = drop).  QC uses Z to decide jumps.
   Returns '0 0 0' if no valid path, arrived, or stale. */
static void PF_nav_path_steer(void)
{
	float *pos;
	int slot;
	nav_bot_path_t *bp;
	float dx, dy, dz, len, dist_sq;
	float *wp;
	float advance_sq = NAV_ADVANCE_RADIUS * NAV_ADVANCE_RADIUS;

	G_FLOAT(OFS_RETURN + 0) = 0.0f;
	G_FLOAT(OFS_RETURN + 1) = 0.0f;
	G_FLOAT(OFS_RETURN + 2) = 0.0f;

	pos = G_VECTOR(OFS_PARM0);

	slot = nav_bot_slot();
	if (slot < 0) return;
	bp = &nav_bot_paths[slot];

	if (!bp->valid) return;

	/* Staleness: no waypoint advance in NAV_STALE_TIME seconds */
	if (sv.time - bp->last_advance_time > NAV_STALE_TIME)
	{
		bp->valid = 0;
		return;
	}

	/* Advance past reached waypoints (XY + Z check) */
	while (bp->current_index < bp->point_count)
	{
		wp = &bp->points[bp->current_index * 3];
		dx = wp[0] - pos[0];
		dy = wp[1] - pos[1];
		dz = wp[2] - pos[2];
		dist_sq = dx * dx + dy * dy;

		if (dist_sq > advance_sq)
			break; /* not close enough in XY */
		if (dz > NAV_ADVANCE_HEIGHT || dz < -NAV_ADVANCE_HEIGHT)
			break; /* wrong floor — don't skip past vertical transitions */

		bp->current_index++;
		bp->last_advance_time = sv.time;

		if (bp->current_index >= bp->point_count)
		{
			bp->valid = 0; /* arrived */
			return;
		}

		/* Overshoot check: if next waypoint is even closer, keep advancing */
	}

	/* Steer toward current waypoint */
	wp = &bp->points[bp->current_index * 3];
	dx = wp[0] - pos[0];
	dy = wp[1] - pos[1];
	dz = wp[2] - pos[2];
	dist_sq = dx * dx + dy * dy;

	/* Reset stale timer if making progress toward current waypoint */
	if (dist_sq < bp->last_wp_dist_sq)
	{
		bp->last_advance_time = sv.time;
		bp->last_wp_dist_sq = dist_sq;
	}

	len = (float)sqrt(dist_sq);
	if (len > 0.001f)
	{
		G_FLOAT(OFS_RETURN + 0) = dx / len;
		G_FLOAT(OFS_RETURN + 1) = dy / len;
	}

	/* Off-mesh link: encode link type in Z (1000 + AI_*).
	   QC checks steer_z >= 1000 to detect link traversal. */
	if (bp->flags[bp->current_index] & 0x04) /* DT_STRAIGHTPATH_OFFMESH_CONNECTION */
	{
		int lt = nav_mesh_get_link_type(nav_mesh, bp->refs[bp->current_index]);
		if (lt > 0)
			dz = 1000.0f + (float)lt;
	}
	G_FLOAT(OFS_RETURN + 2) = dz;
}

/* vector nav_move(vector pos, vector target) = #81
   Slides pos toward target along navmesh walls. Returns reachable position. */
static void PF_nav_move(void)
{
	float *pos, *target;
	float result[3];
	char error[256];

	pos = G_VECTOR(OFS_PARM0);
	target = G_VECTOR(OFS_PARM1);
	G_FLOAT(OFS_RETURN + 0) = pos[0];
	G_FLOAT(OFS_RETURN + 1) = pos[1];
	G_FLOAT(OFS_RETURN + 2) = pos[2];
	if (nav_mesh == NULL) return;

	if (nav_mesh_move_along_surface(nav_mesh, pos, target, result, error, sizeof(error)))
	{
		G_FLOAT(OFS_RETURN + 0) = result[0];
		G_FLOAT(OFS_RETURN + 1) = result[1];
		G_FLOAT(OFS_RETURN + 2) = result[2];
	}
}

/* vector nav_path_debug(entity bot, float index) = #85
   Return position of waypoint N in bot's cached path.
   Returns '0 0 0' if no valid path or index out of range.
   index == -1 returns (point_count, current_index, valid). */
static void PF_nav_path_debug(void)
{
	edict_t *e;
	int num, slot, idx;
	nav_bot_path_t *bp;

	G_FLOAT(OFS_RETURN + 0) = 0.0f;
	G_FLOAT(OFS_RETURN + 1) = 0.0f;
	G_FLOAT(OFS_RETURN + 2) = 0.0f;

	e = G_EDICT(OFS_PARM0);
	idx = (int)G_FLOAT(OFS_PARM1);
	num = NUM_FOR_EDICT(e);
	if (num < 1 || num > svs.maxclients) return;
	slot = num - 1;
	bp = &nav_bot_paths[slot];

	if (idx == -1)
	{
		/* meta query: point_count, current_index, valid */
		G_FLOAT(OFS_RETURN + 0) = (float)bp->point_count;
		G_FLOAT(OFS_RETURN + 1) = (float)bp->current_index;
		G_FLOAT(OFS_RETURN + 2) = (float)bp->valid;
		return;
	}

	if (!bp->valid || idx < 0 || idx >= bp->point_count) return;

	G_FLOAT(OFS_RETURN + 0) = bp->points[idx * 3 + 0];
	G_FLOAT(OFS_RETURN + 1) = bp->points[idx * 3 + 1];
	G_FLOAT(OFS_RETURN + 2) = bp->points[idx * 3 + 2];
}

/* ---- Registration ---- */

#define NAV_BUILTIN_BASE  80
#define NAV_BUILTIN_COUNT  6
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
	nav_extended_builtins[NAV_BUILTIN_BASE + 1] = PF_nav_move;
	nav_extended_builtins[NAV_BUILTIN_BASE + 2] = PF_nav_path_start;
	nav_extended_builtins[NAV_BUILTIN_BASE + 3] = PF_nav_route_cost;
	nav_extended_builtins[NAV_BUILTIN_BASE + 4] = PF_nav_path_steer;
	nav_extended_builtins[NAV_BUILTIN_BASE + 5] = PF_nav_path_debug;

	pr_builtins = nav_extended_builtins;
	pr_numbuiltins = NAV_BUILTIN_MAX;

	Cvar_RegisterVariable(&nav_enabled_cvar);
	Cvar_RegisterVariable(&nav_jump_links_cvar);
	Cvar_SetValue("nav_enabled", 1);
}
