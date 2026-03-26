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
#define NAV_MESH_MAX_PATH_REFS 128
#define NAV_MESH_MAX_STRAIGHT_POINTS 64

/* Link types — FrikBot AI_ naming convention */
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
	float query_point[3];
	float nearest_point[3];
	float poly_center[3];
	float wall_distance;
	int	neighbor_count;
	unsigned long long neighbor_refs[NAV_MESH_MAX_NEIGHBORS];
} nav_mesh_nearest_result_t;

typedef struct
{
	unsigned long long poly_ref;
	float	center[3];
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
	float travel_distance;
	int	path_ref_count;
	unsigned long long path_refs[NAV_MESH_MAX_PATH_REFS];
	int	straight_point_count;
	float straight_points[NAV_MESH_MAX_STRAIGHT_POINTS * 3];
	unsigned char straight_flags[NAV_MESH_MAX_STRAIGHT_POINTS];
	unsigned long long straight_refs[NAV_MESH_MAX_STRAIGHT_POINTS];
	int	start_over_poly;	/* 1 if start is directly above a polygon */
} nav_mesh_path_result_t;

/* Query link metadata by userId (index into build-time link array).
   Returns NULL if navmesh has no stored links or index is out of range. */
const nav_off_mesh_link_t *nav_mesh_get_link(
	const nav_mesh_runtime_t *navmesh, int link_index);

/* Look up link type for an off-mesh connection polygon.
   Returns the AI_* link type, or 0 if not an off-mesh connection. */
int nav_mesh_get_link_type(
	const nav_mesh_runtime_t *navmesh, unsigned long long poly_ref);

nav_mesh_runtime_t *nav_mesh_build(
	const float *verts, int vertex_count,
	const int *tris, int triangle_count,
	const nav_mesh_build_config_t *config,
	const nav_off_mesh_link_t *off_mesh_links, int off_mesh_link_count,
	nav_mesh_summary_t *summary,
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

/* Move from start toward end, sliding along walls.
   Returns 1 and sets result_pos to the reachable position. */
int nav_mesh_move_along_surface(
	const nav_mesh_runtime_t *navmesh,
	const float *start, const float *end,
	float *result_pos,
	char *error, size_t error_size);

/* Boundary edge: an edge of the navmesh with no neighbor polygon. */
typedef struct
{
	float	midpoint[3];	/* Quake coords */
	float	normal[3];	/* outward 2D normal (Quake coords, Z=0) */
} nav_mesh_boundary_edge_t;

/* Collect all boundary edges from the navmesh.
   Caller must free(*out_edges) when done. */
int nav_mesh_collect_boundary_edges(
	const nav_mesh_runtime_t *navmesh,
	nav_mesh_boundary_edge_t **out_edges,
	int *out_count);

void nav_mesh_free_poly_records(nav_mesh_poly_record_t *records);
void nav_mesh_destroy(nav_mesh_runtime_t *navmesh);

#ifdef __cplusplus
}
#endif

#endif
