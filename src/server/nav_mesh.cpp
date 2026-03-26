#include "nav_mesh.h"

#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "Recast.h"

#define NAV_MESH_GOAL_SNAP_MAX_Z 128.0f

/* Shared error-formatting helper. */
extern "C" void nav_set_error(char *error, size_t error_size, const char *format, ...)
{
	va_list args;

	if (error == nullptr || error_size == 0)
		return;
	va_start(args, format);
	vsnprintf(error, error_size, format, args);
	va_end(args);
}

namespace {

constexpr unsigned char kAreaWalkable = 1;
constexpr unsigned short kPolyFlagWalk = 1;

struct NavRcContext : public rcContext
{
	explicit NavRcContext()
		: rcContext(true)
	{
	}

	std::string last_error;

protected:
	void doLog(const rcLogCategory category, const char *msg, const int len) override
	{
		if (category != RC_LOG_ERROR)
			return;
		last_error.assign(msg, static_cast<size_t>(len));
	}
};

static void nav_quake_to_recast(const float *quake, float *recast)
{
	recast[0] = quake[0];
	recast[1] = quake[2];
	recast[2] = quake[1];
}

static void nav_recast_to_quake(const float *recast, float *quake)
{
	quake[0] = recast[0];
	quake[1] = recast[2];
	quake[2] = recast[1];
}

struct RecastBuildGuard
{
	rcHeightfield *solid;
	rcCompactHeightfield *compact;
	rcContourSet *contours;
	rcPolyMesh *poly_mesh;
	rcPolyMeshDetail *detail_mesh;
	unsigned char *nav_data;
	nav_mesh_runtime_t *runtime;

	RecastBuildGuard()
		: solid(nullptr), compact(nullptr), contours(nullptr),
		  poly_mesh(nullptr), detail_mesh(nullptr), nav_data(nullptr),
		  runtime(nullptr)
	{
	}

	~RecastBuildGuard()
	{
		rcFreePolyMeshDetail(detail_mesh);
		rcFreePolyMesh(poly_mesh);
		rcFreeContourSet(contours);
		rcFreeCompactHeightfield(compact);
		rcFreeHeightField(solid);
		if (nav_data != nullptr)
			dtFree(nav_data);
		if (runtime != nullptr)
			nav_mesh_destroy(runtime);
	}

	RecastBuildGuard(const RecastBuildGuard &) = delete;
	RecastBuildGuard &operator=(const RecastBuildGuard &) = delete;
};

}  // namespace

struct nav_mesh_runtime_s
{
	dtNavMesh *navmesh;
	dtNavMeshQuery *query;
	float query_half_extents[3];       /* wide: for items/goals */
	float query_half_extents_tight[3]; /* tight: for agent position */
	nav_off_mesh_link_t *links;        /* persistent link metadata (userId indexes here) */
	int link_count;

	nav_mesh_runtime_s()
		: navmesh(nullptr), query(nullptr), links(nullptr), link_count(0)
	{
		query_half_extents[0] = 64.0f;
		query_half_extents[1] = 96.0f;
		query_half_extents[2] = 64.0f;
		query_half_extents_tight[0] = 32.0f;
		query_half_extents_tight[1] = 56.0f;
		query_half_extents_tight[2] = 32.0f;
	}
};

/* Helper: set up area costs on a query filter. */
static void nav_setup_filter(dtQueryFilter *filter)
{
	filter->setAreaCost(NAV_AREA_WALK, 1.0f);
	filter->setAreaCost(NAV_AREA_JUMP, 3.0f);
	filter->setAreaCost(NAV_AREA_DROP, 2.0f);
	filter->setAreaCost(NAV_AREA_PLAT, 5.0f);
	filter->setAreaCost(NAV_AREA_DOOR, 2.0f);
	filter->setAreaCost(NAV_AREA_RJ, 10.0f);
}

/* Helper: map link type to Detour area ID. */
static unsigned char nav_area_for_link(int link_type)
{
	switch (link_type)
	{
	case AI_JUMP:        return NAV_AREA_JUMP;
	case AI_DROP:        return NAV_AREA_DROP;
	case AI_PLAT_BOTTOM:
	case AI_RIDE_TRAIN:  return NAV_AREA_PLAT;
	case AI_DOORFLAG:    return NAV_AREA_DOOR;
	case AI_SUPER_JUMP:  return NAV_AREA_RJ;
	default:             return NAV_AREA_WALK;
	}
}

static void nav_mesh_poly_center(const dtMeshTile *tile, const dtPoly *poly, float *center)
{
	int i;

	center[0] = 0.0f;
	center[1] = 0.0f;
	center[2] = 0.0f;
	if (tile == nullptr || poly == nullptr || poly->vertCount <= 0)
		return;

	for (i = 0; i < poly->vertCount; ++i)
	{
		const float *vert;

		vert = &tile->verts[poly->verts[i] * 3];
		center[0] += vert[0];
		center[1] += vert[1];
		center[2] += vert[2];
	}

	center[0] /= (float)poly->vertCount;
	center[1] /= (float)poly->vertCount;
	center[2] /= (float)poly->vertCount;
}

static void nav_mesh_poly_bounds(const dtMeshTile *tile, const dtPoly *poly, float *mins, float *maxs)
{
	int i;

	mins[0] = mins[1] = mins[2] = 999999.0f;
	maxs[0] = maxs[1] = maxs[2] = -999999.0f;
	if (tile == nullptr || poly == nullptr || poly->vertCount <= 0)
	{
		mins[0] = mins[1] = mins[2] = 0.0f;
		maxs[0] = maxs[1] = maxs[2] = 0.0f;
		return;
	}

	for (i = 0; i < poly->vertCount; ++i)
	{
		const float *vert;

		vert = &tile->verts[poly->verts[i] * 3];
		mins[0] = fminf(mins[0], vert[0]);
		mins[1] = fminf(mins[1], vert[1]);
		mins[2] = fminf(mins[2], vert[2]);
		maxs[0] = fmaxf(maxs[0], vert[0]);
		maxs[1] = fmaxf(maxs[1], vert[1]);
		maxs[2] = fmaxf(maxs[2], vert[2]);
	}
}

static int nav_mesh_push_unique_ref(unsigned long long *refs, int count, int max_count, dtPolyRef ref)
{
	int i;
	unsigned long long value;

	value = static_cast<unsigned long long>(ref);
	for (i = 0; i < count; ++i)
	{
		if (refs[i] == value)
			return count;
	}
	if (count >= max_count)
		return count;
	refs[count] = value;
	return count + 1;
}

static int nav_mesh_collect_neighbors(const nav_mesh_runtime_t *navmesh, dtPolyRef ref, unsigned long long *refs, int max_refs)
{
	const dtMeshTile *tile;
	const dtPoly *poly;
	int link_index;
	int count;

	if (navmesh == nullptr || navmesh->navmesh == nullptr || refs == nullptr || max_refs <= 0)
		return 0;
	if (dtStatusFailed(navmesh->navmesh->getTileAndPolyByRef(ref, &tile, &poly)))
		return 0;

	count = 0;
	for (link_index = poly->firstLink; link_index != DT_NULL_LINK; link_index = tile->links[link_index].next)
	{
		dtPolyRef neighbor_ref;

		neighbor_ref = tile->links[link_index].ref;
		if (neighbor_ref == 0 || neighbor_ref == ref)
			continue;
		count = nav_mesh_push_unique_ref(refs, count, max_refs, neighbor_ref);
	}
	return count;
}

/* extents_override: if non-NULL, use these instead of the runtime defaults. */
static int nav_mesh_find_nearest_internal(
	const nav_mesh_runtime_t *navmesh,
	const float *point,
	dtPolyRef *nearest_ref,
	float *nearest_pt,
	bool *is_over_poly,
	const float *extents_override,
	char *error,
	size_t error_size)
{
	dtQueryFilter filter; nav_setup_filter(&filter);
	float recast_point[3];
	const float *extents;
	dtStatus status;

	if (navmesh == nullptr || navmesh->query == nullptr || navmesh->navmesh == nullptr)
	{
		nav_set_error(error, error_size, "Navmesh query requested before navmesh was initialized");
		return 0;
	}

	extents = extents_override ? extents_override : navmesh->query_half_extents;

	nav_quake_to_recast(point, recast_point);
	status = navmesh->query->findNearestPoly(
		recast_point,
		extents,
		&filter,
		nearest_ref,
		nearest_pt,
		is_over_poly);
	if (dtStatusFailed(status))
	{
		nav_set_error(error, error_size, "Detour findNearestPoly failed");
		return 0;
	}
	return *nearest_ref != 0 ? 1 : 0;
}

static float nav_mesh_path_distance(const float *points, int point_count)
{
	float total;
	int i;

	total = 0.0f;
	for (i = 1; i < point_count; ++i)
	{
		const float dx = points[i * 3 + 0] - points[(i - 1) * 3 + 0];
		const float dy = points[i * 3 + 1] - points[(i - 1) * 3 + 1];
		const float dz = points[i * 3 + 2] - points[(i - 1) * 3 + 2];
		total += sqrtf(dx * dx + dy * dy + dz * dz);
	}
	return total;
}

extern "C" nav_mesh_runtime_t *nav_mesh_build(
	const float *verts,
	int vertex_count,
	const int *tris,
	int triangle_count,
	const nav_mesh_build_config_t *config,
	const nav_off_mesh_link_t *off_mesh_links,
	int off_mesh_link_count,
	nav_mesh_summary_t *summary,
	char *error,
	size_t error_size)
{
	NavRcContext ctx;
	rcConfig rc_config;
	RecastBuildGuard guard;
	std::vector<float> recast_verts;
	std::vector<unsigned char> areas;
	int nav_data_size;
	dtNavMeshCreateParams params;
	dtStatus status;
	int i;

	if (summary != nullptr)
		memset(summary, 0, sizeof(*summary));
	if (verts == nullptr || tris == nullptr || config == nullptr)
	{
		nav_set_error(error, error_size, "Navmesh build requires non-null vertices, triangles, and config");
		return nullptr;
	}
	if (vertex_count < 3 || triangle_count < 1)
	{
		nav_set_error(error, error_size, "Navmesh build requires at least 3 vertices and 1 triangle");
		return nullptr;
	}

	memset(&rc_config, 0, sizeof(rc_config));
	recast_verts.resize(static_cast<size_t>(vertex_count) * 3u);
	for (i = 0; i < vertex_count; ++i)
		nav_quake_to_recast(&verts[i * 3], &recast_verts[static_cast<size_t>(i) * 3u]);

	rcCalcBounds(recast_verts.data(), vertex_count, rc_config.bmin, rc_config.bmax);
	rc_config.cs = config->cell_size;
	rc_config.ch = config->cell_height;
	rcCalcGridSize(rc_config.bmin, rc_config.bmax, rc_config.cs, &rc_config.width, &rc_config.height);
	rc_config.walkableSlopeAngle = config->walkable_slope_angle;
	rc_config.walkableHeight = (int)ceilf(config->walkable_height / rc_config.ch);
	rc_config.walkableClimb = (int)ceilf(config->walkable_climb / rc_config.ch);
	rc_config.walkableRadius = (int)ceilf(config->walkable_radius / rc_config.cs);
	rc_config.maxEdgeLen = (int)(config->max_edge_len / rc_config.cs);
	rc_config.maxSimplificationError = config->max_simplification_error;
	rc_config.minRegionArea = config->min_region_size * config->min_region_size;
	rc_config.mergeRegionArea = config->merge_region_size * config->merge_region_size;
	rc_config.maxVertsPerPoly = config->max_verts_per_poly;
	rc_config.detailSampleDist = config->detail_sample_distance < 0.9f ? 0.0f : rc_config.cs * config->detail_sample_distance;
	rc_config.detailSampleMaxError = rc_config.ch * config->detail_sample_max_error;

	nav_data_size = 0;

	guard.solid = rcAllocHeightfield();
	if (guard.solid == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate Recast heightfield");
		return nullptr;
	}
	if (!rcCreateHeightfield(&ctx, *guard.solid, rc_config.width, rc_config.height, rc_config.bmin, rc_config.bmax, rc_config.cs, rc_config.ch))
	{
		nav_set_error(error, error_size, "Failed to create Recast heightfield");
		return nullptr;
	}

	areas.assign(static_cast<size_t>(triangle_count), 0);
	rcMarkWalkableTriangles(&ctx, rc_config.walkableSlopeAngle, recast_verts.data(), vertex_count, tris, triangle_count, areas.data());
	if (!rcRasterizeTriangles(&ctx, recast_verts.data(), vertex_count, tris, areas.data(), triangle_count, *guard.solid, rc_config.walkableClimb))
	{
		nav_set_error(error, error_size, "Failed to rasterize triangles into heightfield");
		return nullptr;
	}

	rcFilterLowHangingWalkableObstacles(&ctx, rc_config.walkableClimb, *guard.solid);
	rcFilterLedgeSpans(&ctx, rc_config.walkableHeight, rc_config.walkableClimb, *guard.solid);
	rcFilterWalkableLowHeightSpans(&ctx, rc_config.walkableHeight, *guard.solid);

	guard.compact = rcAllocCompactHeightfield();
	if (guard.compact == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate compact heightfield");
		return nullptr;
	}
	if (!rcBuildCompactHeightfield(&ctx, rc_config.walkableHeight, rc_config.walkableClimb, *guard.solid, *guard.compact))
	{
		nav_set_error(error, error_size, "Failed to build compact heightfield");
		return nullptr;
	}
	if (!rcErodeWalkableArea(&ctx, rc_config.walkableRadius, *guard.compact))
	{
		nav_set_error(error, error_size, "Failed to erode walkable area");
		return nullptr;
	}
	if (!rcBuildDistanceField(&ctx, *guard.compact))
	{
		nav_set_error(error, error_size, "Failed to build distance field");
		return nullptr;
	}
	if (!rcBuildRegions(&ctx, *guard.compact, 0, rc_config.minRegionArea, rc_config.mergeRegionArea))
	{
		nav_set_error(error, error_size, "Failed to build navigation regions");
		return nullptr;
	}

	guard.contours = rcAllocContourSet();
	if (guard.contours == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate contour set");
		return nullptr;
	}
	if (!rcBuildContours(&ctx, *guard.compact, rc_config.maxSimplificationError, rc_config.maxEdgeLen, *guard.contours, RC_CONTOUR_TESS_WALL_EDGES))
	{
		nav_set_error(error, error_size, "Failed to build contours");
		return nullptr;
	}

	guard.poly_mesh = rcAllocPolyMesh();
	if (guard.poly_mesh == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate polygon mesh");
		return nullptr;
	}
	if (!rcBuildPolyMesh(&ctx, *guard.contours, rc_config.maxVertsPerPoly, *guard.poly_mesh))
	{
		nav_set_error(error, error_size, "Failed to build polygon mesh");
		return nullptr;
	}

	guard.detail_mesh = rcAllocPolyMeshDetail();
	if (guard.detail_mesh == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate detail mesh");
		return nullptr;
	}
	if (!rcBuildPolyMeshDetail(&ctx, *guard.poly_mesh, *guard.compact, rc_config.detailSampleDist, rc_config.detailSampleMaxError, *guard.detail_mesh))
	{
		nav_set_error(error, error_size, "Failed to build detail mesh");
		return nullptr;
	}
	if (guard.poly_mesh->npolys <= 0 || guard.poly_mesh->nverts <= 0)
	{
		nav_set_error(error, error_size, "Recast produced an empty navmesh");
		return nullptr;
	}

	for (i = 0; i < guard.poly_mesh->npolys; ++i)
	{
		if (guard.poly_mesh->areas[i] == RC_WALKABLE_AREA)
			guard.poly_mesh->areas[i] = kAreaWalkable;
		if (guard.poly_mesh->areas[i] == kAreaWalkable)
			guard.poly_mesh->flags[i] = kPolyFlagWalk;
	}

	memset(&params, 0, sizeof(params));
	params.verts = guard.poly_mesh->verts;
	params.vertCount = guard.poly_mesh->nverts;
	params.polys = guard.poly_mesh->polys;
	params.polyAreas = guard.poly_mesh->areas;
	params.polyFlags = guard.poly_mesh->flags;
	params.polyCount = guard.poly_mesh->npolys;
	params.nvp = guard.poly_mesh->nvp;
	params.detailMeshes = guard.detail_mesh->meshes;
	params.detailVerts = guard.detail_mesh->verts;
	params.detailVertsCount = guard.detail_mesh->nverts;
	params.detailTris = guard.detail_mesh->tris;
	params.detailTriCount = guard.detail_mesh->ntris;
	params.walkableHeight = config->walkable_height;
	params.walkableRadius = config->walkable_radius;
	params.walkableClimb = config->walkable_climb;
	rcVcopy(params.bmin, guard.poly_mesh->bmin);
	rcVcopy(params.bmax, guard.poly_mesh->bmax);
	params.cs = rc_config.cs;
	params.ch = rc_config.ch;
	params.buildBvTree = true;

	/* Off-mesh connections (teleporters, jump pads).
	   Detour wants parallel arrays: verts (ax,ay,az,bx,by,bz per link),
	   radii, flags, areas, directions, and user IDs. */
	std::vector<float> omc_verts;
	std::vector<float> omc_rad;
	std::vector<unsigned short> omc_flags;
	std::vector<unsigned char> omc_areas;
	std::vector<unsigned char> omc_dir;
	std::vector<unsigned int> omc_id;

	if (off_mesh_links != nullptr && off_mesh_link_count > 0)
	{
		float rc_start[3], rc_end[3];

		for (int li = 0; li < off_mesh_link_count; li++)
		{
			nav_quake_to_recast(off_mesh_links[li].start, rc_start);
			nav_quake_to_recast(off_mesh_links[li].end, rc_end);
			omc_verts.push_back(rc_start[0]);
			omc_verts.push_back(rc_start[1]);
			omc_verts.push_back(rc_start[2]);
			omc_verts.push_back(rc_end[0]);
			omc_verts.push_back(rc_end[1]);
			omc_verts.push_back(rc_end[2]);
			omc_rad.push_back(off_mesh_links[li].radius);
			omc_flags.push_back(kPolyFlagWalk);
			omc_areas.push_back(nav_area_for_link(off_mesh_links[li].link_type));
			omc_dir.push_back(off_mesh_links[li].bidirectional ? DT_OFFMESH_CON_BIDIR : 0);
			omc_id.push_back((unsigned int)li);
		}

		params.offMeshConVerts = omc_verts.data();
		params.offMeshConRad = omc_rad.data();
		params.offMeshConFlags = omc_flags.data();
		params.offMeshConAreas = omc_areas.data();
		params.offMeshConDir = omc_dir.data();
		params.offMeshConUserID = omc_id.data();
		params.offMeshConCount = off_mesh_link_count;
	}

	if (!dtCreateNavMeshData(&params, &guard.nav_data, &nav_data_size))
	{
		nav_set_error(error, error_size, "Failed to create Detour navmesh tile");
		return nullptr;
	}

	guard.runtime = new (std::nothrow) nav_mesh_runtime_t();
	if (guard.runtime == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate navmesh runtime");
		return nullptr;
	}

	guard.runtime->navmesh = dtAllocNavMesh();
	guard.runtime->query = dtAllocNavMeshQuery();
	if (guard.runtime->navmesh == nullptr || guard.runtime->query == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate Detour navmesh/query");
		return nullptr;
	}

	status = guard.runtime->navmesh->init(guard.nav_data, nav_data_size, DT_TILE_FREE_DATA);
	if (dtStatusFailed(status))
	{
		nav_set_error(error, error_size, "Failed to initialize Detour navmesh");
		return nullptr;
	}
	/* nav_data ownership transferred to Detour navmesh via DT_TILE_FREE_DATA */
	guard.nav_data = nullptr;

	status = guard.runtime->query->init(guard.runtime->navmesh, 2048);
	if (dtStatusFailed(status))
	{
		nav_set_error(error, error_size, "Failed to initialize Detour navmesh query");
		return nullptr;
	}

	/* Log how many off-mesh connections Detour actually stored. */
	{
		const dtNavMesh *nm = guard.runtime->navmesh;
		const dtMeshTile *tile = nm->getTile(0);
		if (tile && tile->header)
			fprintf(stderr, "Nav: Detour stored %d/%d off-mesh connections\n",
				tile->header->offMeshConCount, off_mesh_link_count);
	}

	/* Wide extents for goal/item snapping (items may float above floor). */
	guard.runtime->query_half_extents[0] = fmaxf(config->walkable_radius * 8.0f, 128.0f);
	guard.runtime->query_half_extents[1] = fmaxf(config->walkable_height * 4.0f, 256.0f);
	guard.runtime->query_half_extents[2] = fmaxf(config->walkable_radius * 8.0f, 128.0f);

	/* Tight extents for agent position (bot should be on or very near the mesh). */
	guard.runtime->query_half_extents_tight[0] = config->walkable_radius * 2.0f;
	guard.runtime->query_half_extents_tight[1] = config->walkable_height;
	guard.runtime->query_half_extents_tight[2] = config->walkable_radius * 2.0f;

	/* Store link metadata for userId lookup during path following. */
	if (off_mesh_link_count > 0 && off_mesh_links != nullptr)
	{
		size_t sz = (size_t)off_mesh_link_count * sizeof(nav_off_mesh_link_t);
		guard.runtime->links = static_cast<nav_off_mesh_link_t *>(malloc(sz));
		memcpy(guard.runtime->links, off_mesh_links, sz);
		guard.runtime->link_count = off_mesh_link_count;
	}

	if (summary != nullptr)
	{
		summary->input_vertex_count = vertex_count;
		summary->input_triangle_count = triangle_count;
		summary->polygon_count = guard.poly_mesh->npolys;
		summary->navmesh_vertex_count = guard.poly_mesh->nverts;
		summary->detail_mesh_count = guard.detail_mesh->nmeshes;
		summary->detail_vertex_count = guard.detail_mesh->nverts;
		summary->detail_triangle_count = guard.detail_mesh->ntris;
	}

	/* Success: release runtime from the guard so it is not destroyed */
	nav_mesh_runtime_t *result = guard.runtime;
	guard.runtime = nullptr;
	return result;
}

extern "C" int nav_mesh_find_nearest(
	const nav_mesh_runtime_t *navmesh,
	const float *point,
	nav_mesh_nearest_result_t *result,
	char *error,
	size_t error_size)
{
	dtPolyRef nearest_ref;
	float nearest_pt[3];
	float poly_center[3];
	float hit_pos[3];
	float hit_normal[3];
	float wall_distance;
	bool is_over_poly;
	const dtMeshTile *tile;
	const dtPoly *poly;
	dtQueryFilter filter; nav_setup_filter(&filter);
	dtStatus status;

	if (result == nullptr || point == nullptr)
	{
		nav_set_error(error, error_size, "Nearest navmesh query requires a result buffer and point");
		return 0;
	}
	memset(result, 0, sizeof(*result));
	memcpy(result->query_point, point, sizeof(result->query_point));

	nearest_ref = 0;
	memset(nearest_pt, 0, sizeof(nearest_pt));
	is_over_poly = false;
	if (!nav_mesh_find_nearest_internal(navmesh, point, &nearest_ref, nearest_pt, &is_over_poly, NULL, error, error_size))
		return 0;

	if (dtStatusFailed(navmesh->navmesh->getTileAndPolyByRef(nearest_ref, &tile, &poly)))
	{
		nav_set_error(error, error_size, "Detour could not resolve the nearest polygon reference");
		return 0;
	}

	nav_mesh_poly_center(tile, poly, poly_center);
	wall_distance = 0.0f;
	memset(hit_pos, 0, sizeof(hit_pos));
	memset(hit_normal, 0, sizeof(hit_normal));
	status = navmesh->query->findDistanceToWall(nearest_ref, nearest_pt, 4096.0f, &filter, &wall_distance, hit_pos, hit_normal);
	if (dtStatusFailed(status))
		wall_distance = 0.0f;

	result->found = 1;
	result->is_over_poly = is_over_poly ? 1 : 0;
	result->poly_ref = static_cast<unsigned long long>(nearest_ref);
	nav_recast_to_quake(nearest_pt, result->nearest_point);
	nav_recast_to_quake(poly_center, result->poly_center);
	result->wall_distance = wall_distance;
	result->neighbor_count = nav_mesh_collect_neighbors(navmesh, nearest_ref, result->neighbor_refs, NAV_MESH_MAX_NEIGHBORS);
	return 1;
}

extern "C" int nav_mesh_collect_polys(
	const nav_mesh_runtime_t *navmesh,
	nav_mesh_poly_record_t **records,
	int *record_count,
	char *error,
	size_t error_size)
{
	int total;
	int tile_index;
	int write_index;
	nav_mesh_poly_record_t *out;

	if (records == nullptr || record_count == nullptr)
	{
		nav_set_error(error, error_size, "Poly enumeration requires output pointers");
		return 0;
	}
	*records = nullptr;
	*record_count = 0;
	if (navmesh == nullptr || navmesh->navmesh == nullptr)
	{
		nav_set_error(error, error_size, "Navmesh polygons requested before navmesh was initialized");
		return 0;
	}

	total = 0;
	const dtNavMesh *detour_navmesh = navmesh->navmesh;
	for (tile_index = 0; tile_index < navmesh->navmesh->getMaxTiles(); ++tile_index)
	{
		const dtMeshTile *tile;
		int poly_index;

		tile = detour_navmesh->getTile(tile_index);
		if (tile == nullptr || tile->header == nullptr || tile->polys == nullptr)
			continue;
		for (poly_index = 0; poly_index < tile->header->polyCount; ++poly_index)
		{
			if (tile->polys[poly_index].getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;
			total += 1;
		}
	}
	if (total <= 0)
	{
		nav_set_error(error, error_size, "Detour navmesh does not contain any polygons");
		return 0;
	}

	out = static_cast<nav_mesh_poly_record_t *>(calloc(static_cast<size_t>(total), sizeof(*out)));
	if (out == nullptr)
	{
		nav_set_error(error, error_size, "Out of memory while enumerating navmesh polygons");
		return 0;
	}

	write_index = 0;
	for (tile_index = 0; tile_index < navmesh->navmesh->getMaxTiles(); ++tile_index)
	{
		const dtMeshTile *tile;
		dtPolyRef base_ref;
		int poly_index;

		tile = detour_navmesh->getTile(tile_index);
		if (tile == nullptr || tile->header == nullptr || tile->polys == nullptr)
			continue;
		base_ref = detour_navmesh->getPolyRefBase(tile);
		for (poly_index = 0; poly_index < tile->header->polyCount; ++poly_index)
		{
			const dtPoly *poly;
			float center[3];
			float mins[3];
			float maxs[3];
			dtPolyRef ref;

			poly = &tile->polys[poly_index];
			if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			ref = base_ref | static_cast<dtPolyRef>(poly_index);
			nav_mesh_poly_center(tile, poly, center);
			nav_mesh_poly_bounds(tile, poly, mins, maxs);
			out[write_index].poly_ref = static_cast<unsigned long long>(ref);
			nav_recast_to_quake(center, out[write_index].center);
			nav_recast_to_quake(mins, out[write_index].bounds_min);
			nav_recast_to_quake(maxs, out[write_index].bounds_max);
			out[write_index].neighbor_count = nav_mesh_collect_neighbors(navmesh, ref, out[write_index].neighbor_refs, NAV_MESH_MAX_NEIGHBORS);
			write_index += 1;
		}
	}

	*records = out;
	*record_count = write_index;
	return 1;
}

extern "C" int nav_mesh_find_path(
	const nav_mesh_runtime_t *navmesh,
	const float *start,
	const float *end,
	nav_mesh_path_result_t *result,
	char *error,
	size_t error_size)
{
	dtPolyRef start_ref;
	dtPolyRef end_ref;
	float start_nearest[3];
	float end_nearest[3];
	bool start_over_poly;
	bool end_over_poly;
	dtQueryFilter filter; nav_setup_filter(&filter);
	dtPolyRef path_refs[NAV_MESH_MAX_PATH_REFS];
	int path_count;
	float straight_path[NAV_MESH_MAX_STRAIGHT_POINTS * 3];
	unsigned char straight_flags[NAV_MESH_MAX_STRAIGHT_POINTS];
	dtPolyRef straight_refs[NAV_MESH_MAX_STRAIGHT_POINTS];
	int straight_count;
	dtStatus status;
	int i;

	if (result == nullptr || start == nullptr || end == nullptr)
	{
		nav_set_error(error, error_size, "Path navmesh query requires result, start, and end points");
		return 0;
	}
	memset(result, 0, sizeof(*result));

	start_ref = 0;
	end_ref = 0;
	memset(start_nearest, 0, sizeof(start_nearest));
	memset(end_nearest, 0, sizeof(end_nearest));
	start_over_poly = false;
	end_over_poly = false;
	/* Tight extents for start (agent is on the mesh), wide for end (items may float). */
	if (!nav_mesh_find_nearest_internal(navmesh, start, &start_ref, start_nearest, &start_over_poly, navmesh->query_half_extents_tight, error, error_size)
		|| !nav_mesh_find_nearest_internal(navmesh, end, &end_ref, end_nearest, &end_over_poly, NULL, error, error_size))
		return 0;

	/* Reject if goal snapped too far vertically (wrong floor).
	   end_nearest is Recast coords (Y=up), end is Quake (Z=up).
	   Recast Y = Quake Z, so compare end_nearest[1] vs end[2]. */
	{
		float goal_snap_dz = end_nearest[1] - end[2];
		if (goal_snap_dz < 0) goal_snap_dz = -goal_snap_dz;
		if (goal_snap_dz > NAV_MESH_GOAL_SNAP_MAX_Z)
		{
			nav_set_error(error, error_size, "Goal snapped too far vertically (%.0f)", goal_snap_dz);
			return 0;
		}
	}

	path_count = 0;
	status = navmesh->query->findPath(
		start_ref,
		end_ref,
		start_nearest,
		end_nearest,
		&filter,
		path_refs,
		&path_count,
		NAV_MESH_MAX_PATH_REFS);
	if (dtStatusFailed(status) || path_count <= 0)
	{
		nav_set_error(error, error_size, "Detour findPath failed");
		return 0;
	}
	/* Reject truly unreachable goals.  DT_PARTIAL_RESULT fires both
	   when the goal is on a disconnected island AND when the path
	   corridor exceeds the buffer (128 polys).  Only reject if the
	   corridor didn't actually reach the goal polygon. */
	if (dtStatusDetail(status, DT_PARTIAL_RESULT))
	{
		if (path_count <= 0 || path_refs[path_count - 1] != end_ref)
		{
			nav_set_error(error, error_size, "Detour findPath: goal unreachable (partial)");
			return 0;
		}
		/* Buffer-truncated but reached goal — accept. */
	}

	straight_count = 0;
	status = navmesh->query->findStraightPath(
		start_nearest,
		end_nearest,
		path_refs,
		path_count,
		straight_path,
		straight_flags,
		straight_refs,
		&straight_count,
		NAV_MESH_MAX_STRAIGHT_POINTS,
		0);
	if (dtStatusFailed(status) || straight_count <= 0)
	{
		nav_set_error(error, error_size, "Detour findStraightPath failed");
		return 0;
	}

	result->found = 1;
	result->start_over_poly = start_over_poly ? 1 : 0;
	result->start_ref = static_cast<unsigned long long>(start_ref);
	result->end_ref = static_cast<unsigned long long>(end_ref);
	nav_recast_to_quake(start_nearest, result->start_point);
	nav_recast_to_quake(end_nearest, result->end_point);
	result->path_ref_count = path_count;
	for (i = 0; i < path_count; ++i)
		result->path_refs[i] = static_cast<unsigned long long>(path_refs[i]);
	result->straight_point_count = straight_count;
	for (i = 0; i < straight_count; ++i)
	{
		nav_recast_to_quake(&straight_path[i * 3], &result->straight_points[i * 3]);
		result->straight_flags[i] = straight_flags[i];
		result->straight_refs[i] = static_cast<unsigned long long>(straight_refs[i]);
	}
	result->travel_distance = nav_mesh_path_distance(result->straight_points, straight_count);
	return 1;
}

extern "C" int nav_mesh_move_along_surface(
	const nav_mesh_runtime_t *navmesh,
	const float *start, const float *end,
	float *result_pos,
	char *error, size_t error_size)
{
	dtPolyRef start_ref;
	float start_nearest[3], end_recast[3], result_recast[3];
	bool start_over;
	dtQueryFilter filter; nav_setup_filter(&filter);
	dtPolyRef visited[16];
	int visited_count;
	dtStatus status;

	if (navmesh == nullptr || start == nullptr || end == nullptr || result_pos == nullptr)
	{
		nav_set_error(error, error_size, "moveAlongSurface: null argument");
		return 0;
	}

	if (!nav_mesh_find_nearest_internal(navmesh, start, &start_ref, start_nearest,
		&start_over, navmesh->query_half_extents_tight, error, error_size))
		return 0;

	nav_quake_to_recast(end, end_recast);

	status = navmesh->query->moveAlongSurface(
		start_ref, start_nearest, end_recast, &filter,
		result_recast, visited, &visited_count, 16);

	if (dtStatusFailed(status))
	{
		nav_set_error(error, error_size, "moveAlongSurface failed");
		return 0;
	}

	nav_recast_to_quake(result_recast, result_pos);
	return 1;
}

extern "C" int nav_mesh_collect_boundary_edges(
	const nav_mesh_runtime_t *navmesh,
	nav_mesh_boundary_edge_t **out_edges,
	int *out_count)
{
	std::vector<nav_mesh_boundary_edge_t> edges;
	int ti, pi, ei;

	if (navmesh == nullptr || navmesh->navmesh == nullptr
		|| out_edges == nullptr || out_count == nullptr)
		return 0;

	*out_edges = nullptr;
	*out_count = 0;

	const dtNavMesh *nm = navmesh->navmesh;
	for (ti = 0; ti < nm->getMaxTiles(); ++ti)
	{
		const dtMeshTile *tile = nm->getTile(ti);
		if (tile == nullptr || tile->header == nullptr)
			continue;

		for (pi = 0; pi < tile->header->polyCount; ++pi)
		{
			const dtPoly *poly = &tile->polys[pi];
			if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
				continue;

			for (ei = 0; ei < poly->vertCount; ++ei)
			{
				/* Only boundary edges (no neighbor polygon) */
				if (poly->neis[ei] != 0)
					continue;

				/* Get the two vertices of this edge (Recast coords) */
				const float *v0 = &tile->verts[poly->verts[ei] * 3];
				const float *v1 = &tile->verts[poly->verts[(ei + 1) % poly->vertCount] * 3];

				/* Edge midpoint (Recast coords) */
				float mid_rc[3];
				mid_rc[0] = (v0[0] + v1[0]) * 0.5f;
				mid_rc[1] = (v0[1] + v1[1]) * 0.5f;
				mid_rc[2] = (v0[2] + v1[2]) * 0.5f;

				/* Edge direction (Recast XZ plane) */
				float dx = v1[0] - v0[0];
				float dz = v1[2] - v0[2];
				float len = sqrtf(dx * dx + dz * dz);
				if (len < 0.001f)
					continue;

				/* Outward normal: perpendicular to edge in XZ plane.
				   Recast: X,Z = horizontal.  Normal = rotate edge 90° CW. */
				float nx = dz / len;
				float nz = -dx / len;

				/* Verify normal points outward: check against polygon center.
				   If dot(normal, mid→center) > 0, normal points inward — flip. */
				float cx = 0, cz = 0;
				for (int vi = 0; vi < poly->vertCount; ++vi)
				{
					cx += tile->verts[poly->verts[vi] * 3 + 0];
					cz += tile->verts[poly->verts[vi] * 3 + 2];
				}
				cx /= poly->vertCount;
				cz /= poly->vertCount;
				if (nx * (cx - mid_rc[0]) + nz * (cz - mid_rc[2]) > 0)
				{
					nx = -nx;
					nz = -nz;
				}

				/* Convert to Quake coords */
				nav_mesh_boundary_edge_t edge;
				nav_recast_to_quake(mid_rc, edge.midpoint);
				/* Normal: Recast (nx, 0, nz) → Quake (nx, nz, 0) */
				edge.normal[0] = nx;
				edge.normal[1] = nz;
				edge.normal[2] = 0.0f;

				edges.push_back(edge);
			}
		}
	}

	if (edges.empty())
		return 1;

	*out_count = static_cast<int>(edges.size());
	*out_edges = static_cast<nav_mesh_boundary_edge_t *>(
		malloc(edges.size() * sizeof(nav_mesh_boundary_edge_t)));
	memcpy(*out_edges, edges.data(), edges.size() * sizeof(nav_mesh_boundary_edge_t));
	return 1;
}

extern "C" void nav_mesh_free_poly_records(nav_mesh_poly_record_t *records)
{
	free(records);
}

extern "C" const nav_off_mesh_link_t *nav_mesh_get_link(
	const nav_mesh_runtime_t *navmesh, int link_index)
{
	if (navmesh == nullptr || link_index < 0 || link_index >= navmesh->link_count)
		return nullptr;
	return &navmesh->links[link_index];
}

extern "C" int nav_mesh_get_link_type(
	const nav_mesh_runtime_t *navmesh, unsigned long long poly_ref)
{
	const dtOffMeshConnection *con;
	int idx;

	if (navmesh == nullptr || navmesh->navmesh == nullptr || poly_ref == 0)
		return 0;
	con = navmesh->navmesh->getOffMeshConnectionByRef(static_cast<dtPolyRef>(poly_ref));
	if (con == nullptr)
		return 0;
	idx = static_cast<int>(con->userId);
	if (idx < 0 || idx >= navmesh->link_count)
		return 0;
	return navmesh->links[idx].link_type;
}

extern "C" void nav_mesh_destroy(nav_mesh_runtime_t *navmesh)
{
	if (navmesh == nullptr)
		return;
	free(navmesh->links);
	if (navmesh->query != nullptr)
		dtFreeNavMeshQuery(navmesh->query);
	if (navmesh->navmesh != nullptr)
		dtFreeNavMesh(navmesh->navmesh);
	delete navmesh;
}
