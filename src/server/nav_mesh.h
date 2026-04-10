#ifndef NAV_MESH_H
#define NAV_MESH_H

#include <stddef.h>
#include <stdio.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct nav_mesh_runtime_s nav_mesh_runtime_t;

void nav_set_error(char *error, size_t error_size, const char *format, ...)
#ifdef __GNUC__
	__attribute__((format(printf, 3, 4)))
#endif
;

#define NAV_MESH_MAX_NEIGHBORS 16
#define NAV_MESH_MAX_PATH_REFS 512

/* Link types — FrikBot AI_ naming convention */
#define AI_TELELINK       1   /* teleporter: walk in, instant transport */
#define AI_JUMP           2   /* jump up: press jump on approach */
#define AI_DROP           3   /* drop down: walk off edge */
#define AI_PLAT_BOTTOM    4   /* platform: wait at bottom, ride up */
#define AI_RIDE_TRAIN     5   /* train/elevator: stand on, ride */
#define AI_DOORFLAG       6   /* door: wait or trigger */
#define AI_SUPER_JUMP     7   /* rocket jump: RL aim down fire+jump */
#define AI_SURFACE        8   /* water: swim up to surface */

/* Detour area types for cost weighting */
#define NAV_AREA_WALK      0   /* walking + teleporters (cost 1.0) */
#define NAV_AREA_JUMP      1   /* jump links (cost 3.0) */
#define NAV_AREA_DROP      2   /* drop links (cost 2.0) */
#define NAV_AREA_PLAT      3   /* platform/train (cost 5.0 — slow ride) */
#define NAV_AREA_DOOR      4   /* door (cost 2.0 — brief wait) */
#define NAV_AREA_RJ        5   /* rocket jump (cost 10.0 — expensive, risky) */
#define NAV_AREA_NEAR_WALL 6   /* within walkable_radius of wall (cost 3.0) */

typedef struct
{
	float	cell_size;
	float	cell_height;
	float	walkable_slope_angle;
	float	walkable_height;
	float	walkable_climb;
	float	walkable_radius;
	float	max_edge_len;
	float	max_simplification_error;
	int	min_region_size;
	int	merge_region_size;
	int	max_verts_per_poly;
	float	detail_sample_distance;
	float	detail_sample_max_error;
} nav_mesh_build_config_t;

/* Off-mesh connection (teleporter, jump, drop, etc.) */
typedef struct
{
	float	start[3];
	float	end[3];
	float	radius;
	int	bidirectional;
	int	link_type;		/* AI_TELELINK, AI_JUMP, AI_DROP, etc. */
	float	required_speed;		/* min velocity to clear (jumps) */
	float	height_delta;		/* vertical change start→end */
	float	wait_time;		/* seconds to wait (platforms, doors) */
} nav_off_mesh_link_t;

/* C++ only: close extern "C", include Detour, define struct */
#ifdef __cplusplus
} /* close extern "C" for Detour includes */
#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"
class dtQueryFilter;

struct nav_mesh_runtime_s
{
	dtNavMesh *navmesh;
	dtNavMeshQuery *query;
	float query_half_extents[3];       /* wide: for items/goals */
	float query_half_extents_actor_origin[3]; /* tight: actor origin -> surface snap */
	nav_off_mesh_link_t *links;
	int link_count;

	nav_mesh_runtime_s()
		: navmesh(nullptr), query(nullptr), links(nullptr), link_count(0)
	{
		query_half_extents[0] = 64.0f;
		query_half_extents[1] = 96.0f;
		query_half_extents[2] = 64.0f;
		query_half_extents_actor_origin[0] = 32.0f;
		query_half_extents_actor_origin[1] = 56.0f;
		query_half_extents_actor_origin[2] = 32.0f;
	}
};

void nav_mesh_setup_filter(dtQueryFilter *filter);

/* Blocked poly table: paths through these polys are rejected post-findPath.
   No virtual dispatch — just a flat array checked after pathfinding. */
#define NAV_MAX_BLOCKED_POLYS 256

struct nav_blocked_polys
{
	dtPolyRef polys[NAV_MAX_BLOCKED_POLYS];
	int count;

	nav_blocked_polys() : count(0) {}

	void block(const dtPolyRef *refs, int n)
	{
		for (int i = 0; i < n && count < NAV_MAX_BLOCKED_POLYS; i++)
			polys[count++] = refs[i];
	}

	void unblock(const dtPolyRef *refs, int n)
	{
		for (int i = 0; i < n; i++)
			for (int j = 0; j < count; j++)
				if (polys[j] == refs[i])
				{
					polys[j] = polys[--count];
					break;
				}
	}

	bool path_blocked(const dtPolyRef *path, int path_count) const
	{
		if (count == 0) return false;
		for (int pi = 0; pi < path_count; pi++)
			for (int bi = 0; bi < count; bi++)
				if (path[pi] == polys[bi])
					return true;
		return false;
	}
};

extern "C" { /* reopen for remaining C declarations */
#endif

typedef struct
{
	int	input_vertex_count;
	int	input_triangle_count;
	int	polygon_count;
	int	navmesh_vertex_count;
	int	detail_mesh_count;
	int	detail_vertex_count;
	int	detail_triangle_count;
} nav_mesh_summary_t;

typedef struct
{
	int	found;
	int	is_over_poly;
	unsigned long long poly_ref;
	float query_point[3];   /* caller-supplied sample point in Quake coords */
	float nearest_point[3]; /* nearest point on the navmesh surface in Quake coords */
	float poly_center[3];   /* polygon center on the navmesh surface in Quake coords */
	float wall_distance;
	int	neighbor_count;
	unsigned long long neighbor_refs[NAV_MESH_MAX_NEIGHBORS];
} nav_mesh_nearest_result_t;

typedef struct
{
	unsigned long long poly_ref;
	float	center[3]; /* polygon center on the navmesh surface in Quake coords */
	float	bounds_min[3];
	float	bounds_max[3];
	int	neighbor_count;
	unsigned long long neighbor_refs[NAV_MESH_MAX_NEIGHBORS];
} nav_mesh_poly_record_t;

typedef struct
{
	int	found;
	unsigned long long start_ref;
	unsigned long long end_ref;
	float start_point[3];
	float end_point[3];
	int	path_ref_count;
	unsigned long long path_refs[NAV_MESH_MAX_PATH_REFS];
	int	start_over_poly;
} nav_mesh_path_result_t;

/* Query link metadata by userId (index into build-time link array).
   Returns NULL if navmesh has no stored links or index is out of range. */
const nav_off_mesh_link_t *nav_mesh_get_link(
	const nav_mesh_runtime_t *navmesh, int link_index);

/* Look up link type for an off-mesh connection polygon.
   Returns the AI_* link type, or 0 if not an off-mesh connection. */
int nav_mesh_get_link_type(
	const nav_mesh_runtime_t *navmesh, unsigned long long poly_ref);

/* Heightfield probe: check if a point (Quake coords) is blocked by solid
   geometry. Returns 1 if there is a solid span (wall) between floor_z and
   floor_z + walkable_height at this XY. Returns 0 if clear/open. */
typedef struct nav_heightfield_s nav_heightfield_t;
int nav_heightfield_is_blocked(const nav_heightfield_t *hf, const float *point, float floor_z);
/* Find the nearest walkable floor Z at a point.  Returns 1 if found, 0 if no walkable floor. */
int nav_heightfield_floor_z(const nav_heightfield_t *hf, const float *point, float search_z, float *out_z);
/* Find all walkable floors below edge_z (and above min_z) at an XY position.
   Returns count of floors found.  out_floors[] sorted top to bottom (nearest landing first). */
int nav_heightfield_floors_below(const nav_heightfield_t *hf, const float *point,
	float edge_z, float min_z, float *out_floors, int max_floors);
/* Find the lowest walkable floor above min_z (and below max_z) at an XY position.
   Returns 1 if found, 0 if no floor in range. */
int nav_heightfield_floor_above(const nav_heightfield_t *hf, const float *point,
	float min_z, float max_z, float *out_z);
void nav_heightfield_free(nav_heightfield_t *hf);

/* Boundary edge: an edge of the navmesh with no neighbor polygon. */
typedef struct
{
	float	midpoint[3];	/* Quake coords */
	float	normal[3];	/* outward 2D normal (Quake coords, Z=0) */
} nav_mesh_boundary_edge_t;

/* Callback invoked mid-build after contours are ready.
   Receives boundary edges + heightfield; returns additional off-mesh links
   to include in the final Detour build (single pass, no rebuild).
   Return count of additional links, 0 for none.  Caller frees *out_links. */
typedef int (*nav_mesh_link_callback_t)(
	const nav_mesh_boundary_edge_t *edges, int edge_count,
	const nav_heightfield_t *hf,
	nav_off_mesh_link_t **out_links,
	void *user_data);

nav_mesh_runtime_t *nav_mesh_build(
	const float *verts, int vertex_count,
	const int *tris, int triangle_count,
	const nav_mesh_build_config_t *config,
	const nav_off_mesh_link_t *off_mesh_links, int off_mesh_link_count,
	nav_mesh_summary_t *summary,
	nav_mesh_link_callback_t link_callback, void *callback_data,
	char *error, size_t error_size);

int nav_mesh_find_nearest(
	const nav_mesh_runtime_t *navmesh,
	const float *point,
	nav_mesh_nearest_result_t *result,
	char *error, size_t error_size);

int nav_mesh_collect_polys(
	const nav_mesh_runtime_t *navmesh,
	nav_mesh_poly_record_t **records, int *record_count,
	char *error, size_t error_size);

int nav_mesh_find_path(
	const nav_mesh_runtime_t *navmesh,
	const float *start, const float *end,
	nav_mesh_path_result_t *result,
	char *error, size_t error_size);

void nav_mesh_free_poly_records(nav_mesh_poly_record_t *records);
void nav_mesh_destroy(nav_mesh_runtime_t *navmesh);

/* ---- Path corridor (dtPathCorridor wrapper) ---- */

typedef struct nav_corridor_s nav_corridor_t;

nav_corridor_t *nav_corridor_create(int max_path);
void nav_corridor_destroy(nav_corridor_t *c);

/* Load a computed path into the corridor. */
int nav_corridor_set(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	const float *start, const float *target,
	const unsigned long long *path_refs, int path_count);

/* Per-frame: find next corner to steer toward.
   Returns 1 if a corner was found, 0 if path is empty.
   corner_pos: Quake coords of the steering target.
   corner_flags: DT_STRAIGHTPATH_* flags (off-mesh, end, etc.)
   corner_ref: poly ref of the corner. */
int navigate(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	const float *agent_pos,
	float *corner_pos,
	unsigned char *corner_flags,
	unsigned long long *corner_ref);

/* Advance past an off-mesh connection. Returns landing position. */
int nav_corridor_offmesh(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	unsigned long long offmesh_ref,
	float *start_pos, float *end_pos);

/* Get corridor length (number of polys remaining). */
int nav_corridor_length(const nav_corridor_t *c);

/* Waypoint-vs-navmesh validation (nav_val.cpp) */
void Nav_Validate(const nav_mesh_runtime_t *mesh, const char *mapname);

#ifdef __cplusplus
}
#endif

#endif
