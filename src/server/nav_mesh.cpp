#include "nav_mesh.h"

#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
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

struct nav_heightfield_s
{
	rcCompactHeightfield *compact;
	rcConfig config; /* cell_size, cell_height, bmin, walkableHeight etc. */
};

/* nav_mesh_runtime_s is defined in nav_mesh.h (C++ section) */

/* Set up area costs on a query filter. */
void nav_mesh_setup_filter(dtQueryFilter *filter)
{
	filter->setAreaCost(NAV_AREA_WALK, 1.0f);
	filter->setAreaCost(NAV_AREA_JUMP, 3.0f);
	filter->setAreaCost(NAV_AREA_DROP, 2.0f);
	filter->setAreaCost(NAV_AREA_PLAT, 5.0f);
	filter->setAreaCost(NAV_AREA_DOOR, 2.0f);
	filter->setAreaCost(NAV_AREA_RJ, 10.0f);
	filter->setAreaCost(NAV_AREA_NEAR_WALL, 3.0f);
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

static float nav_mesh_horizontal_dist_sq(const float *a, const float *b)
{
	float dx;
	float dz;

	dx = a[0] - b[0];
	dz = a[2] - b[2];
	return dx * dx + dz * dz;
}

static float nav_mesh_snap_horizontal_limit(const nav_mesh_runtime_t *navmesh, const float *extents)
{
	float limit;

	limit = fmaxf(extents[0], extents[2]) * 0.75f;
	if (navmesh != nullptr && extents == navmesh->query_half_extents_actor_origin)
		limit = fminf(limit, 24.0f);
	else
		limit = fminf(limit, 48.0f);
	return fmaxf(limit, 16.0f);
}

static int nav_mesh_build_regions(
	NavRcContext *ctx,
	rcCompactHeightfield *compact,
	const rcConfig *config)
{
	/* Layer regions: designed for multi-layer (multi-floor) maps.
	   Handles overlapping spans and narrow ledges better than
	   watershed or monotone partitioning. */
	return rcBuildLayerRegions(
		ctx,
		*compact,
		0,
		config->minRegionArea);
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
	dtQueryFilter filter; nav_mesh_setup_filter(&filter);
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
	if (*nearest_ref != 0)
	{
		float dist_sq;
		float limit;

		dist_sq = nav_mesh_horizontal_dist_sq(recast_point, nearest_pt);
		limit = nav_mesh_snap_horizontal_limit(navmesh, extents);
		if (dist_sq > limit * limit)
		{
			nav_set_error(error, error_size, "Nearest polygon snapped %.0f units away", sqrtf(dist_sq));
			*nearest_ref = 0;
			memset(nearest_pt, 0, sizeof(float) * 3);
			if (is_over_poly != nullptr)
				*is_over_poly = false;
			return 0;
		}
	}
	return *nearest_ref != 0 ? 1 : 0;
}

struct nav_heightfield_gap_fill_t
{
	bool valid;
	unsigned short smin;
	unsigned short smax;
	int score;

	nav_heightfield_gap_fill_t()
		: valid(false), smin(0), smax(0), score(0x7fffffff)
	{
	}
};

struct nav_heightfield_support_pair_t
{
	bool valid;
	unsigned short first_smax;
	unsigned short second_smax;
	int height_delta;

	nav_heightfield_support_pair_t()
		: valid(false), first_smax(0), second_smax(0), height_delta(0x7fffffff)
	{
	}
};

struct nav_heightfield_axis_support_t
{
	int x;
	int z;
	int distance;

	nav_heightfield_axis_support_t()
		: x(0), z(0), distance(0)
	{
	}
};

static bool nav_heightfield_column_has_walkable_span(const rcHeightfield *hf, int x, int z)
{
	const rcSpan *span;

	if (hf == nullptr || x < 0 || z < 0 || x >= hf->width || z >= hf->height)
		return false;

	for (span = hf->spans[x + z * hf->width]; span != nullptr; span = span->next)
	{
		if (span->area != RC_NULL_AREA)
			return true;
	}
	return false;
}

static bool nav_heightfield_column_has_span_near_height(
	const rcHeightfield *hf,
	int x,
	int z,
	unsigned short target_smax,
	int tolerance,
	bool walkable_only)
{
	const rcSpan *span;

	if (hf == nullptr || x < 0 || z < 0 || x >= hf->width || z >= hf->height)
		return false;

	for (span = hf->spans[x + z * hf->width]; span != nullptr; span = span->next)
	{
		if (walkable_only && span->area == RC_NULL_AREA)
			continue;
		if (rcAbs(static_cast<int>(span->smax) - static_cast<int>(target_smax)) <= tolerance)
			return true;
	}
	return false;
}

static void nav_heightfield_collect_axis_supports(
	const rcHeightfield *hf,
	int x,
	int z,
	int dx,
	int dz,
	int max_distance,
	std::vector<nav_heightfield_axis_support_t> &supports)
{
	int distance;

	supports.clear();
	if (hf == nullptr)
		return;

	for (distance = 1; distance <= max_distance; ++distance)
	{
		const int sx = x + dx * distance;
		const int sz = z + dz * distance;
		nav_heightfield_axis_support_t support;

		if (sx < 0 || sz < 0 || sx >= hf->width || sz >= hf->height)
			return;
		if (hf->spans[sx + sz * hf->width] == nullptr)
			continue;
		if (!nav_heightfield_column_has_walkable_span(hf, sx, sz))
			return;

		support.x = sx;
		support.z = sz;
		support.distance = distance;
		supports.push_back(support);
	}
}

static nav_heightfield_support_pair_t nav_heightfield_find_best_support_pair(
	const rcHeightfield *hf,
	int ax,
	int az,
	int bx,
	int bz,
	int walkable_climb)
{
	const rcSpan *a;
	const rcSpan *b;
	nav_heightfield_support_pair_t best;

	if (hf == nullptr)
		return best;

	for (a = hf->spans[ax + az * hf->width]; a != nullptr; a = a->next)
	{
		if (a->area == RC_NULL_AREA)
			continue;
		for (b = hf->spans[bx + bz * hf->width]; b != nullptr; b = b->next)
		{
			const int diff = rcAbs(static_cast<int>(a->smax) - static_cast<int>(b->smax));

			if (b->area == RC_NULL_AREA || diff > walkable_climb)
				continue;
			if (best.valid && diff >= best.height_delta)
				continue;

			best.valid = true;
			best.first_smax = a->smax;
			best.second_smax = b->smax;
			best.height_delta = diff;
		}
	}
	return best;
}

static void nav_heightfield_propose_gap_fill(
	std::vector<nav_heightfield_gap_fill_t> &fills,
	int index,
	unsigned short smax,
	int score)
{
	nav_heightfield_gap_fill_t &fill = fills[static_cast<size_t>(index)];

	if (fill.valid && score >= fill.score)
		return;

	fill.valid = true;
	fill.smax = smax;
	fill.smin = smax > 0 ? static_cast<unsigned short>(smax - 1) : 0;
	fill.score = score;
}

static void nav_heightfield_propose_axis_gap_fills(
	std::vector<nav_heightfield_gap_fill_t> &fills,
	const rcHeightfield *hf,
	const rcConfig *config,
	int x,
	int z,
	int back_dx,
	int back_dz,
	int forward_dx,
	int forward_dz)
{
	int best_average_smax;
	int best_total_distance;
	int best_height_delta;
	int step;
	int best_back_x;
	int best_back_z;
	int best_forward_x;
	int best_forward_z;
	nav_heightfield_support_pair_t best_pair;
	std::vector<nav_heightfield_axis_support_t> back_supports;
	std::vector<nav_heightfield_axis_support_t> forward_supports;

	if (hf == nullptr || config == nullptr || config->walkableRadius < 1)
		return;
	nav_heightfield_collect_axis_supports(
		hf,
		x,
		z,
		back_dx,
		back_dz,
		config->walkableRadius,
		back_supports);
	nav_heightfield_collect_axis_supports(
		hf,
		x,
		z,
		forward_dx,
		forward_dz,
		config->walkableRadius,
		forward_supports);
	if (back_supports.empty() || forward_supports.empty())
		return;

	best_average_smax = -1;
	best_total_distance = 0x7fffffff;
	best_height_delta = 0x7fffffff;
	best_back_x = 0;
	best_back_z = 0;
	best_forward_x = 0;
	best_forward_z = 0;
	for (size_t back_index = 0; back_index < back_supports.size(); ++back_index)
	{
		for (size_t forward_index = 0; forward_index < forward_supports.size(); ++forward_index)
		{
			const nav_heightfield_axis_support_t &back = back_supports[back_index];
			const nav_heightfield_axis_support_t &forward = forward_supports[forward_index];
			const int total_distance = back.distance + forward.distance;
			const int step_dx = forward.x > back.x ? 1 : (forward.x < back.x ? -1 : 0);
			const int step_dz = forward.z > back.z ? 1 : (forward.z < back.z ? -1 : 0);
			nav_heightfield_support_pair_t pair = nav_heightfield_find_best_support_pair(
				hf,
				back.x,
				back.z,
				forward.x,
				forward.z,
				config->walkableClimb);

			if (!pair.valid)
				continue;

			for (step = 1; step < total_distance; ++step)
			{
				const int fill_x = back.x + step_dx * step;
				const int fill_z = back.z + step_dz * step;
				const unsigned short smax = static_cast<unsigned short>(
					(static_cast<int>(pair.first_smax) * (total_distance - step) +
					 static_cast<int>(pair.second_smax) * step +
					 total_distance / 2) /
					total_distance);

				if (nav_heightfield_column_has_span_near_height(
						hf,
						fill_x,
						fill_z,
						smax,
						config->walkableClimb,
						false))
					break;
			}
			if (step != total_distance)
				continue;

			{
				const int average_smax =
					(static_cast<int>(pair.first_smax) + static_cast<int>(pair.second_smax)) / 2;

				if (best_pair.valid &&
					(average_smax < best_average_smax ||
					 (average_smax == best_average_smax && total_distance > best_total_distance) ||
					 (average_smax == best_average_smax && total_distance == best_total_distance &&
					  pair.height_delta >= best_height_delta)))
					continue;

				best_pair = pair;
				best_average_smax = average_smax;
				best_total_distance = total_distance;
				best_height_delta = pair.height_delta;
				best_back_x = back.x;
				best_back_z = back.z;
				best_forward_x = forward.x;
				best_forward_z = forward.z;
			}
		}
	}
	if (!best_pair.valid)
		return;

	{
		const int total_distance = rcAbs(best_forward_x - best_back_x) + rcAbs(best_forward_z - best_back_z);
		const int step_dx = best_forward_x > best_back_x ? 1 : (best_forward_x < best_back_x ? -1 : 0);
		const int step_dz = best_forward_z > best_back_z ? 1 : (best_forward_z < best_back_z ? -1 : 0);
		const int score = best_total_distance * 1024 + best_pair.height_delta - best_average_smax;

		for (step = 1; step < total_distance; ++step)
		{
			const int fill_x = best_back_x + step_dx * step;
			const int fill_z = best_back_z + step_dz * step;
			const int index = fill_x + fill_z * hf->width;
			const unsigned short smax = static_cast<unsigned short>(
				(static_cast<int>(best_pair.first_smax) * (total_distance - step) +
				 static_cast<int>(best_pair.second_smax) * step +
				 total_distance / 2) /
				total_distance);

			nav_heightfield_propose_gap_fill(fills, index, smax, score);
		}
	}
}

/* Fill tiny empty runs bracketed by walkable support columns on the same
   effective floor. The maximum repaired width is bounded by walkableRadius,
   so the pass only repairs cracks the agent radius should already tolerate. */
static void nav_heightfield_bridge_small_gaps(
	rcContext *ctx,
	rcHeightfield *hf,
	const rcConfig *config)
{
	int x;
	int z;
	int fills_applied;
	std::vector<nav_heightfield_gap_fill_t> fills;

	if (ctx == nullptr || hf == nullptr || config == nullptr)
		return;
	if (config->walkableRadius < 1)
		return;

	fills.resize(static_cast<size_t>(hf->width * hf->height));

	for (z = 0; z < hf->height; ++z)
	{
		for (x = 0; x < hf->width; ++x)
		{
			nav_heightfield_propose_axis_gap_fills(fills, hf, config, x, z, -1, 0, 1, 0);
			nav_heightfield_propose_axis_gap_fills(fills, hf, config, x, z, 0, -1, 0, 1);
		}
	}

	fills_applied = 0;
	for (z = 0; z < hf->height; ++z)
	{
		for (x = 0; x < hf->width; ++x)
		{
			const int index = x + z * hf->width;
			const nav_heightfield_gap_fill_t &fill = fills[static_cast<size_t>(index)];

			if (!fill.valid)
				continue;
			if (!rcAddSpan(ctx, *hf, x, z, fill.smin, fill.smax, RC_WALKABLE_AREA, config->walkableClimb))
				continue;
			fills_applied++;
		}
	}

	if (fills_applied > 0)
		fprintf(stderr, "Nav: bridged %d tiny raster gaps before ledge filtering\n", fills_applied);
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
	nav_mesh_link_callback_t link_callback,
	void *callback_data,
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
	nav_heightfield_bridge_small_gaps(&ctx, guard.solid, &rc_config);
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
	/* Wall-only erosion: erode walkable area from walls but NOT from ledge
	   edges.  Standard erosion kills narrow ledges (DM4 platforms above
	   lava) because it treats drop-offs the same as walls.  We build a
	   custom distance field seeded only from wall borders, then erode
	   spans that are too close to walls.  Ledge-adjacent spans survive. */
	{
		const int w = guard.compact->width;
		const int h = guard.compact->height;
		const int span_count = guard.compact->spanCount;
		const unsigned short erode_dist = (unsigned short)(rc_config.walkableRadius * 2);

		std::vector<unsigned short> wd(static_cast<size_t>(span_count), 0xffff);

		/* Seed: distance 0 for walkable spans adjacent to a wall.
		   A non-connected neighbor is a WALL if the neighbor column has
		   solid geometry at our floor height.  It is a LEDGE if the
		   neighbor column has open air at our floor height (drop-off). */
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					if (guard.compact->areas[si] == RC_NULL_AREA)
						continue;
					const rcCompactSpan &s = guard.compact->spans[si];

					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) != RC_NOT_CONNECTED)
							continue;

						const int nx = x + rcGetDirOffsetX(dir);
						const int ny = y + rcGetDirOffsetY(dir);

						if (nx < 0 || nx >= w || ny < 0 || ny >= h)
						{
							wd[si] = 0; /* out of bounds = wall */
							goto next_span_seed;
						}

						const rcCompactCell &nc = guard.compact->cells[nx + ny * w];
						if (nc.count == 0)
						{
							wd[si] = 0; /* no spans = solid = wall */
							goto next_span_seed;
						}

						/* Check if any neighbor span has open space at our
						   floor height.  If yes → ledge.  If no → wall. */
						bool is_open = false;
						for (int ni = (int)nc.index, nn = (int)(nc.index + nc.count); ni < nn; ++ni)
						{
							const rcCompactSpan &ns = guard.compact->spans[ni];
							if (s.y >= ns.y && s.y < (int)ns.y + (int)ns.h)
							{
								is_open = true;
								break;
							}
						}
						if (!is_open)
						{
							wd[si] = 0; /* solid at our height = wall */
							goto next_span_seed;
						}
					}
					next_span_seed:;
				}
			}
		}

		/* 2-pass Chamfer distance transform (matches Recast's internal
		   algorithm: cardinal weight 2, diagonal weight 3). */

		/* Pass 1: top-left → bottom-right */
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					const rcCompactSpan &s = guard.compact->spans[si];

					if (rcGetCon(s, 0) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(0);
						const int ay = y + rcGetDirOffsetY(0);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, 0);
						if (wd[ai] + 2 < wd[si])
							wd[si] = wd[ai] + 2;

						const rcCompactSpan &as = guard.compact->spans[ai];
						if (rcGetCon(as, 3) != RC_NOT_CONNECTED)
						{
							const int bx = ax + rcGetDirOffsetX(3);
							const int by = ay + rcGetDirOffsetY(3);
							const int bi = (int)guard.compact->cells[bx + by * w].index + rcGetCon(as, 3);
							if (wd[bi] + 3 < wd[si])
								wd[si] = wd[bi] + 3;
						}
					}
					if (rcGetCon(s, 3) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(3);
						const int ay = y + rcGetDirOffsetY(3);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, 3);
						if (wd[ai] + 2 < wd[si])
							wd[si] = wd[ai] + 2;

						const rcCompactSpan &as = guard.compact->spans[ai];
						if (rcGetCon(as, 2) != RC_NOT_CONNECTED)
						{
							const int bx = ax + rcGetDirOffsetX(2);
							const int by = ay + rcGetDirOffsetY(2);
							const int bi = (int)guard.compact->cells[bx + by * w].index + rcGetCon(as, 2);
							if (wd[bi] + 3 < wd[si])
								wd[si] = wd[bi] + 3;
						}
					}
				}
			}
		}

		/* Pass 2: bottom-right → top-left */
		for (int y = h - 1; y >= 0; --y)
		{
			for (int x = w - 1; x >= 0; --x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					const rcCompactSpan &s = guard.compact->spans[si];

					if (rcGetCon(s, 2) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(2);
						const int ay = y + rcGetDirOffsetY(2);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, 2);
						if (wd[ai] + 2 < wd[si])
							wd[si] = wd[ai] + 2;

						const rcCompactSpan &as = guard.compact->spans[ai];
						if (rcGetCon(as, 1) != RC_NOT_CONNECTED)
						{
							const int bx = ax + rcGetDirOffsetX(1);
							const int by = ay + rcGetDirOffsetY(1);
							const int bi = (int)guard.compact->cells[bx + by * w].index + rcGetCon(as, 1);
							if (wd[bi] + 3 < wd[si])
								wd[si] = wd[bi] + 3;
						}
					}
					if (rcGetCon(s, 1) != RC_NOT_CONNECTED)
					{
						const int ax = x + rcGetDirOffsetX(1);
						const int ay = y + rcGetDirOffsetY(1);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, 1);
						if (wd[ai] + 2 < wd[si])
							wd[si] = wd[ai] + 2;

						const rcCompactSpan &as = guard.compact->spans[ai];
						if (rcGetCon(as, 0) != RC_NOT_CONNECTED)
						{
							const int bx = ax + rcGetDirOffsetX(0);
							const int by = ay + rcGetDirOffsetY(0);
							const int bi = (int)guard.compact->cells[bx + by * w].index + rcGetCon(as, 0);
							if (wd[bi] + 3 < wd[si])
								wd[si] = wd[bi] + 3;
						}
					}
				}
			}
		}

		/* Erode spans too close to walls */
		int eroded = 0;
		for (int si = 0; si < span_count; ++si)
		{
			if (guard.compact->areas[si] != RC_NULL_AREA && wd[si] < erode_dist)
			{
				guard.compact->areas[si] = RC_NULL_AREA;
				eroded++;
			}
		}
		fprintf(stderr, "Nav: wall-only erosion: %d spans eroded (radius=%d)\n",
			eroded, rc_config.walkableRadius);
	}
	/* The custom erosion above changes walkable spans, so rebuild the
	   distance field afterward. Regions and near-wall costs both consume
	   compact->dist and must see the post-erosion topology. */
	if (!rcBuildDistanceField(&ctx, *guard.compact))
	{
		nav_set_error(error, error_size, "Failed to build post-erosion distance field");
		return nullptr;
	}
	if (!nav_mesh_build_regions(&ctx, guard.compact, &rc_config))
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

	/* Extract boundary edges from contours and invoke callback for
	   additional off-mesh links (jump/drop detection) in a single pass. */
	nav_off_mesh_link_t *callback_links = nullptr;
	int callback_link_count = 0;
	if (link_callback != nullptr)
	{
		/* Extract boundary edges from contour set */
		std::vector<nav_mesh_boundary_edge_t> contour_edges;
		const rcContourSet *cset = guard.contours;
		for (int ci = 0; ci < cset->nconts; ++ci)
		{
			const rcContour &cont = cset->conts[ci];
			if (cont.nverts < 3) continue;
			for (int vi = 0; vi < cont.nverts; ++vi)
			{
				const int *va = &cont.verts[vi * 4];
				const int *vb = &cont.verts[((vi + 1) % cont.nverts) * 4];
				/* boundary edge: neighbor region == 0 */
				if ((va[3] & RC_CONTOUR_REG_MASK) != 0) continue;

				/* Convert cell coords to world (Recast coords) */
				float ax = cset->bmin[0] + va[0] * cset->cs;
				float ay = cset->bmin[1] + va[1] * cset->ch;
				float az = cset->bmin[2] + va[2] * cset->cs;
				float bx = cset->bmin[0] + vb[0] * cset->cs;
				float by = cset->bmin[1] + vb[1] * cset->ch;
				float bz = cset->bmin[2] + vb[2] * cset->cs;

				nav_mesh_boundary_edge_t edge;
				/* midpoint in Quake coords (Recast X,Z,Y → Quake X,Y,Z) */
				edge.midpoint[0] = (ax + bx) * 0.5f;
				edge.midpoint[1] = (az + bz) * 0.5f;
				edge.midpoint[2] = (ay + by) * 0.5f;

				/* outward 2D normal (perpendicular to edge direction) */
				float dx = bx - ax, dz = bz - az;
				float len = sqrtf(dx * dx + dz * dz);
				if (len > 0.001f)
				{
					edge.normal[0] = dz / len;   /* Quake X = Recast perp Z */
					edge.normal[1] = -dx / len;  /* Quake Y = Recast perp -X */
					edge.normal[2] = 0;
				}
				else
				{
					edge.normal[0] = edge.normal[1] = edge.normal[2] = 0;
				}
				contour_edges.push_back(edge);
			}
		}

		/* Provide heightfield for wall checking (non-owning wrapper) */
		nav_heightfield_t hf_wrapper;
		nav_heightfield_t *hf_for_callback = nullptr;
		if (guard.compact != nullptr)
		{
			hf_wrapper.compact = guard.compact;
			memcpy(&hf_wrapper.config, &rc_config, sizeof(rc_config));
			hf_for_callback = &hf_wrapper;
		}

		callback_link_count = link_callback(
			contour_edges.data(), (int)contour_edges.size(),
			hf_for_callback, &callback_links, callback_data);
	}

	guard.poly_mesh = rcAllocPolyMesh();
	if (guard.poly_mesh == nullptr)
	{
		nav_set_error(error, error_size, "Failed to allocate polygon mesh");
		free(callback_links);
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

	/* Mark poly areas using distance-field wall proximity.
	   Two-pass approach:
	   1. Sample each poly's wall distance from the compact heightfield.
	   2. Mark a poly as NAV_AREA_NEAR_WALL only if it is near a wall AND
	      has a neighbor that is NOT near a wall (i.e., a center-of-corridor
	      alternative exists).  Narrow corridors where ALL polys are near
	      walls stay as normal WALK cost — no penalty when there's no choice. */
	{
		const unsigned short *pverts = guard.poly_mesh->verts;
		const unsigned short *ppolys = guard.poly_mesh->polys;
		const int nvp = guard.poly_mesh->nvp;
		const int npoly = guard.poly_mesh->npolys;
		const unsigned short dist_threshold =
			(unsigned short)(config->walkable_radius / config->cell_size * 2.0f);

		/* Pass 1: compute per-poly "near wall" flag via distance field */
		std::vector<bool> poly_near_wall(static_cast<size_t>(npoly), false);

		for (i = 0; i < npoly; ++i)
		{
			if (guard.poly_mesh->areas[i] != RC_WALKABLE_AREA)
				continue;

			float cx = 0, cz = 0;
			int vc = 0;
			const unsigned short *p = &ppolys[i * nvp * 2];
			for (int vi = 0; vi < nvp && p[vi] != RC_MESH_NULL_IDX; ++vi)
			{
				cx += pverts[p[vi] * 3 + 0];
				cz += pverts[p[vi] * 3 + 2];
				vc++;
			}
			if (vc > 0) { cx /= vc; cz /= vc; }

			int gx = (int)cx;
			int gz = (int)cz;
			if (gx < 0) gx = 0;
			if (gz < 0) gz = 0;
			if (gx >= guard.compact->width) gx = guard.compact->width - 1;
			if (gz >= guard.compact->height) gz = guard.compact->height - 1;

			const rcCompactCell *cell = &guard.compact->cells[gx + gz * guard.compact->width];
			for (int si = (int)cell->index, sn = (int)(cell->index + cell->count); si < sn; ++si)
			{
				if (guard.compact->dist[si] < dist_threshold)
				{
					poly_near_wall[static_cast<size_t>(i)] = true;
					break;
				}
			}
		}

		/* Pass 2: only penalize near-wall polys that have a non-near-wall neighbor
		   (meaning a center path exists).  Neighbor indices are in the second
		   half of each poly's entry: ppolys[i * nvp * 2 + nvp + edge]. */
		int n_walk = 0, n_nearwall = 0;
		for (i = 0; i < npoly; ++i)
		{
			guard.poly_mesh->flags[i] = kPolyFlagWalk;

			if (guard.poly_mesh->areas[i] != RC_WALKABLE_AREA)
				continue;

			if (!poly_near_wall[static_cast<size_t>(i)])
			{
				guard.poly_mesh->areas[i] = kAreaWalkable;
				n_walk++;
				continue;
			}

			/* Check neighbors: does any adjacent poly have open space? */
			int has_open_neighbor = 0;
			const unsigned short *adj = &ppolys[i * nvp * 2 + nvp];
			for (int ei = 0; ei < nvp; ++ei)
			{
				unsigned short ni = adj[ei];
				if (ni == RC_MESH_NULL_IDX)
					continue;
				if (!poly_near_wall[static_cast<size_t>(ni)])
				{
					has_open_neighbor = 1;
					break;
				}
			}

			if (has_open_neighbor)
			{
				guard.poly_mesh->areas[i] = NAV_AREA_NEAR_WALL;
				n_nearwall++;
			}
			else
			{
				/* Narrow corridor — all neighbors also near wall, no center
				   path exists.  Keep as normal WALK cost. */
				guard.poly_mesh->areas[i] = kAreaWalkable;
				n_walk++;
			}
		}

		fprintf(stderr, "Nav: poly areas: %d walk, %d near-wall / %d total\n",
			n_walk, n_nearwall, npoly);
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
	/* Use walkable_height (not walkable_climb) for the Detour tile params.
	   This controls the Y search extent when linking off-mesh connections
	   to nearby ground polys. Off-mesh endpoints are authored from entity
	   origins, not guaranteed foot-contact points, so the search must cover
	   origin-to-surface separation even when the navmesh itself is on the floor. */
	params.walkableClimb = config->walkable_height;
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

	/* Build combined link list: caller links + callback links */
	{
		/* Helper to append a link array to the Detour parallel arrays */
		auto append_links = [&](const nav_off_mesh_link_t *links, int count, int id_base)
		{
			float rc_start[3], rc_end[3];
			for (int li = 0; li < count; li++)
			{
				nav_quake_to_recast(links[li].start, rc_start);
				nav_quake_to_recast(links[li].end, rc_end);
				omc_verts.push_back(rc_start[0]); omc_verts.push_back(rc_start[1]); omc_verts.push_back(rc_start[2]);
				omc_verts.push_back(rc_end[0]); omc_verts.push_back(rc_end[1]); omc_verts.push_back(rc_end[2]);
				omc_rad.push_back(links[li].radius);
				omc_flags.push_back(kPolyFlagWalk);
				omc_areas.push_back(nav_area_for_link(links[li].link_type));
				omc_dir.push_back(links[li].bidirectional ? DT_OFFMESH_CON_BIDIR : 0);
				omc_id.push_back((unsigned int)(id_base + li));
			}
		};

		if (off_mesh_links != nullptr && off_mesh_link_count > 0)
			append_links(off_mesh_links, off_mesh_link_count, 0);
		if (callback_links != nullptr && callback_link_count > 0)
			append_links(callback_links, callback_link_count, off_mesh_link_count);

		int total_links = off_mesh_link_count + callback_link_count;
		if (total_links > 0)
		{
			params.offMeshConVerts = omc_verts.data();
			params.offMeshConRad = omc_rad.data();
			params.offMeshConFlags = omc_flags.data();
			params.offMeshConAreas = omc_areas.data();
			params.offMeshConDir = omc_dir.data();
			params.offMeshConUserID = omc_id.data();
			params.offMeshConCount = total_links;
		}
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

	/* Log how many off-mesh connections Detour stored and linked. */
	{
		const dtNavMesh *nm = guard.runtime->navmesh;
		const dtMeshTile *tile = nm->getTile(0);
		if (tile && tile->header)
		{
			int linked = 0, unlinked = 0;
			for (int oi = 0; oi < tile->header->offMeshConCount; ++oi)
			{
				const dtPoly *p = &tile->polys[tile->header->offMeshBase + oi];
				if (p->firstLink == DT_NULL_LINK)
					unlinked++;
				else
					linked++;
			}
			fprintf(stderr, "Nav: Detour stored %d/%d off-mesh, linked=%d unlinked=%d\n",
				tile->header->offMeshConCount,
				off_mesh_link_count + callback_link_count,
				linked, unlinked);
		}
	}

	/* Wide extents for goal/item snapping.
	   Keep XZ tighter than the original 64u box to avoid snapping
	   through thin walls into adjacent rooms. */
	guard.runtime->query_half_extents[0] = fmaxf(config->walkable_radius * 3.0f, 48.0f);
	guard.runtime->query_half_extents[1] = fmaxf(config->walkable_height * 2.0f, 128.0f);
	guard.runtime->query_half_extents[2] = fmaxf(config->walkable_radius * 3.0f, 48.0f);

	/* Tight extents for actor-origin snapping.
	   Many callers pass an entity origin above the walkable surface rather
	   than a foot point on the floor, so Y must cover that origin-to-surface
	   offset while still staying below the next floor above. */
	guard.runtime->query_half_extents_actor_origin[0] = fmaxf(config->walkable_radius * 1.5f, 24.0f);
	guard.runtime->query_half_extents_actor_origin[1] = config->walkable_height;
	guard.runtime->query_half_extents_actor_origin[2] = fmaxf(config->walkable_radius * 1.5f, 24.0f);

	/* Store link metadata for userId lookup during path following. */
	{
		int total_links = off_mesh_link_count + callback_link_count;
		if (total_links > 0)
		{
			size_t sz = (size_t)total_links * sizeof(nav_off_mesh_link_t);
			guard.runtime->links = static_cast<nav_off_mesh_link_t *>(malloc(sz));
			if (off_mesh_link_count > 0 && off_mesh_links != nullptr)
				memcpy(guard.runtime->links, off_mesh_links,
					(size_t)off_mesh_link_count * sizeof(nav_off_mesh_link_t));
			if (callback_link_count > 0 && callback_links != nullptr)
				memcpy(guard.runtime->links + off_mesh_link_count, callback_links,
					(size_t)callback_link_count * sizeof(nav_off_mesh_link_t));
			guard.runtime->link_count = total_links;
		}
	}
	free(callback_links);

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
	dtQueryFilter filter; nav_mesh_setup_filter(&filter);
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
	dtQueryFilter filter; nav_mesh_setup_filter(&filter);
	dtPolyRef path_refs[NAV_MESH_MAX_PATH_REFS];
	int path_count;
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
	if (!nav_mesh_find_nearest_internal(navmesh, start, &start_ref, start_nearest, &start_over_poly, navmesh->query_half_extents_actor_origin, error, error_size)
		|| !nav_mesh_find_nearest_internal(navmesh, end, &end_ref, end_nearest, &end_over_poly, NULL, error, error_size))
		return 0;

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
	if (dtStatusDetail(status, DT_PARTIAL_RESULT))
	{
		if (path_count <= 0 || path_refs[path_count - 1] != end_ref)
		{
			nav_set_error(error, error_size, "Detour findPath: goal unreachable (partial)");
			return 0;
		}
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

/* ---- Path corridor wrapper ---- */

struct nav_corridor_s
{
	dtPathCorridor corridor;
	dtQueryFilter filter;
};

static int nav_corridor_reseed_to_poly(nav_corridor_t *c, dtPolyRef poly_ref, const float *poly_pos)
{
	const dtPolyRef *path;
	dtPolyRef trimmed[NAV_MESH_MAX_PATH_REFS];
	float target[3];
	int path_count;
	int i;

	if (c == nullptr || poly_ref == 0 || poly_pos == nullptr)
		return 0;

	path = c->corridor.getPath();
	path_count = c->corridor.getPathCount();
	if (path == nullptr || path_count < 1)
		return 0;

	for (i = 0; i < path_count; ++i)
	{
		if (path[i] != poly_ref)
			continue;
		if (i == 0)
			return 0;

		memcpy(target, c->corridor.getTarget(), sizeof(target));
		memcpy(trimmed, path + i, (size_t)(path_count - i) * sizeof(dtPolyRef));
		c->corridor.reset(poly_ref, poly_pos);
		c->corridor.setCorridor(target, trimmed, path_count - i);
		return 1;
	}

	return 0;
}

extern "C" nav_corridor_t *nav_corridor_create(int max_path)
{
	nav_corridor_t *c = new (std::nothrow) nav_corridor_t();
	if (c == nullptr) return nullptr;
	if (!c->corridor.init(max_path))
	{
		delete c;
		return nullptr;
	}
	nav_mesh_setup_filter(&c->filter);
	return c;
}

extern "C" void nav_corridor_destroy(nav_corridor_t *c)
{
	if (c != nullptr)
		delete c;
}

extern "C" int nav_corridor_set(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	const float *start, const float *target,
	const unsigned long long *path_refs, int path_count)
{
	float rc_start[3], rc_target[3];
	dtPolyRef polys[NAV_MESH_MAX_PATH_REFS];
	int i;

	if (c == nullptr || navmesh == nullptr || path_count < 1)
		return 0;
	if (path_count > NAV_MESH_MAX_PATH_REFS)
		path_count = NAV_MESH_MAX_PATH_REFS;

	nav_quake_to_recast(start, rc_start);
	nav_quake_to_recast(target, rc_target);
	for (i = 0; i < path_count; i++)
		polys[i] = static_cast<dtPolyRef>(path_refs[i]);

	c->corridor.reset(polys[0], rc_start);
	c->corridor.setCorridor(rc_target, polys, path_count);
	return 1;
}

extern "C" int navigate(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	const float *agent_pos,
	float *corner_pos,
	unsigned char *corner_flags,
	unsigned long long *corner_ref)
{
	float rc_pos[3];
	float snapped_pos[3];
	float corners[3 * 3]; /* max 3 corners */
	unsigned char flags[3];
	dtPolyRef refs[3];
	dtPolyRef snapped_ref;
	int have_snapped_pos;
	int ncorners;
	static int optimize_counter = 0;

	if (c == nullptr || navmesh == nullptr || navmesh->query == nullptr)
		return 0;

	/* Sync corridor position to the snapped navmesh position. If the actor
	   already traversed an off-mesh link in the game world, trim the path
	   forward to the matching polygon instead of advancing Detour early. */
	nav_quake_to_recast(agent_pos, rc_pos);
	snapped_ref = 0;
	memset(snapped_pos, 0, sizeof(snapped_pos));
	have_snapped_pos = nav_mesh_find_nearest_internal(
		navmesh,
		agent_pos,
		&snapped_ref,
		snapped_pos,
		nullptr,
		navmesh->query_half_extents_actor_origin,
		nullptr,
		0);

	if (have_snapped_pos)
	{
		if (!nav_corridor_reseed_to_poly(c, snapped_ref, snapped_pos)
			&& !c->corridor.isValid(8, navmesh->query, &c->filter))
			c->corridor.trimInvalidPath(snapped_ref, snapped_pos, navmesh->query, &c->filter);
	}

	if (c->corridor.getPathCount() < 1)
		return 0;

	if (!c->corridor.movePosition(have_snapped_pos ? snapped_pos : rc_pos, navmesh->query, &c->filter))
	{
		if (!have_snapped_pos || !nav_corridor_reseed_to_poly(c, snapped_ref, snapped_pos))
			return 0;
	}

	if (!c->corridor.isValid(8, navmesh->query, &c->filter))
	{
		if (!have_snapped_pos)
			return 0;
		c->corridor.trimInvalidPath(snapped_ref, snapped_pos, navmesh->query, &c->filter);
		if (!c->corridor.isValid(1, navmesh->query, &c->filter))
			return 0;
	}

	/* Check arrival: corridor position close to target = path done */
	if (dtVdist2DSqr(c->corridor.getPos(), c->corridor.getTarget()) < 32.0f * 32.0f)
		return 0;

	/* Get next corners */
	ncorners = c->corridor.findCorners(corners, flags, refs,
		3, navmesh->query, &c->filter);
	if (ncorners < 1)
		return 0;

	/* Off-mesh link traversal: when the bot is within trigger distance
	   of a link start, advance the corridor past the link.  This follows
	   Detour best practice (dtCrowd uses radius * 2.25 as trigger).
	   The corner is still returned to QC so it can animate the traversal;
	   on the NEXT call, the corridor is already past the link. */
	if (flags[0] & 0x04) /* DT_STRAIGHTPATH_OFFMESH_CONNECTION */
	{
		float trigger = 16.0f * 2.25f; /* agent radius * 2.25 */
		float dist2d = dtVdist2D(c->corridor.getPos(), corners);
		if (dist2d < trigger)
		{
			dtPolyRef advance_refs[2];
			float rc_start[3], rc_end[3];
			c->corridor.moveOverOffmeshConnection(
				refs[0], advance_refs, rc_start, rc_end, navmesh->query);
			/* Corridor is now past the link.  Next navigate() call will
			   see post-link waypoints.  Return the link start corner to
			   QC this frame so it can execute the traversal action. */
		}
	}

	/* Fix corner height using getPolyHeight */
	{
		float h = 0;
		if (dtStatusSucceed(navmesh->query->getPolyHeight(refs[0], corners, &h)))
			corners[1] = h; /* Recast Y = height */
	}

	/* Periodically optimize corridor */
	optimize_counter++;
	if ((optimize_counter & 15) == 0) /* every 16 frames */
	{
		if (ncorners > 1)
			c->corridor.optimizePathVisibility(
				&corners[3], 48.0f * 6.0f, /* ~6 polys look-ahead */
				navmesh->query, &c->filter);
		c->corridor.optimizePathTopology(navmesh->query, &c->filter);
	}

	/* Return first corner in Quake coords */
	nav_recast_to_quake(corners, corner_pos);
	*corner_flags = flags[0];
	*corner_ref = static_cast<unsigned long long>(refs[0]);
	return 1;
}

extern "C" int nav_corridor_offmesh(nav_corridor_t *c,
	const nav_mesh_runtime_t *navmesh,
	unsigned long long offmesh_ref,
	float *start_pos, float *end_pos)
{
	dtPolyRef refs[2];
	float rc_start[3], rc_end[3];

	if (c == nullptr || navmesh == nullptr || navmesh->query == nullptr)
		return 0;

	if (!c->corridor.moveOverOffmeshConnection(
		static_cast<dtPolyRef>(offmesh_ref), refs,
		rc_start, rc_end, navmesh->query))
		return 0;

	nav_recast_to_quake(rc_start, start_pos);
	nav_recast_to_quake(rc_end, end_pos);
	return 1;
}

extern "C" int nav_corridor_length(const nav_corridor_t *c)
{
	if (c == nullptr) return 0;
	return c->corridor.getPathCount();
}

/* ---- Heightfield probing ---- */

extern "C" int nav_heightfield_is_blocked(const nav_heightfield_t *hf,
	const float *point, float floor_z)
{
	float rc_point[3];
	int gx, gz;

	if (hf == nullptr || hf->compact == nullptr)
		return 0;

	/* Convert Quake coords to Recast heightfield grid coords */
	nav_quake_to_recast(point, rc_point);
	gx = (int)((rc_point[0] - hf->config.bmin[0]) / hf->config.cs);
	gz = (int)((rc_point[2] - hf->config.bmin[2]) / hf->config.cs);

	if (gx < 0 || gx >= hf->compact->width || gz < 0 || gz >= hf->compact->height)
		return 0; /* outside grid = open space */

	/* Check spans in this column: is there a NON-walkable span at
	   the probed height?  A walkable span means floor, not wall. */
	float rc_floor_z = floor_z; /* Quake Z = Recast Y */
	const rcCompactCell &cell = hf->compact->cells[gx + gz * hf->compact->width];
	for (int si = (int)cell.index, sn = (int)(cell.index + cell.count); si < sn; ++si)
	{
		const rcCompactSpan &span = hf->compact->spans[si];
		float span_y = hf->config.bmin[1] + (float)span.y * hf->config.ch;

		/* Span is near our probe height and NOT walkable = wall */
		if (span_y > rc_floor_z - hf->config.walkableHeight * hf->config.ch &&
			span_y < rc_floor_z + hf->config.walkableHeight * hf->config.ch)
		{
			if (hf->compact->areas[si] == RC_NULL_AREA)
				return 1; /* solid/wall */
		}
	}

	/* No solid span at this height = open */
	return 0;
}

extern "C" int nav_heightfield_floor_z(const nav_heightfield_t *hf,
	const float *point, float search_z, float *out_z)
{
	float rc_point[3];
	int gx, gz;

	if (hf == nullptr || hf->compact == nullptr)
		return 0;

	nav_quake_to_recast(point, rc_point);
	gx = (int)((rc_point[0] - hf->config.bmin[0]) / hf->config.cs);
	gz = (int)((rc_point[2] - hf->config.bmin[2]) / hf->config.cs);

	if (gx < 0 || gx >= hf->compact->width || gz < 0 || gz >= hf->compact->height)
		return 0;

	float rc_search = search_z; /* Quake Z = Recast Y */
	float best_y = -999999.0f;
	int found = 0;
	const rcCompactCell &cell = hf->compact->cells[gx + gz * hf->compact->width];
	for (int si = (int)cell.index, sn = (int)(cell.index + cell.count); si < sn; ++si)
	{
		if (hf->compact->areas[si] == RC_NULL_AREA)
			continue; /* not walkable */
		float span_y = hf->config.bmin[1] + (float)hf->compact->spans[si].y * hf->config.ch;
		float dist = fabsf(span_y - rc_search);
		float best_dist = fabsf(best_y - rc_search);
		if (!found || dist < best_dist)
		{
			best_y = span_y;
			found = 1;
		}
	}

	if (found && out_z != nullptr)
		*out_z = best_y; /* Recast Y = Quake Z */
	return found;
}

extern "C" int nav_heightfield_floors_below(const nav_heightfield_t *hf,
	const float *point, float edge_z, float min_z,
	float *out_floors, int max_floors)
{
	float rc_point[3];
	int gx, gz, count = 0;

	if (hf == nullptr || hf->compact == nullptr || out_floors == nullptr || max_floors < 1)
		return 0;

	nav_quake_to_recast(point, rc_point);
	gx = (int)((rc_point[0] - hf->config.bmin[0]) / hf->config.cs);
	gz = (int)((rc_point[2] - hf->config.bmin[2]) / hf->config.cs);

	if (gx < 0 || gx >= hf->compact->width || gz < 0 || gz >= hf->compact->height)
		return 0;

	/* Recast Y = Quake Z.  Collect walkable spans below edge_z and above min_z. */
	float rc_edge = edge_z;
	float rc_min = min_z;
	float climb = (float)hf->config.walkableClimb * hf->config.ch;

	const rcCompactCell &cell = hf->compact->cells[gx + gz * hf->compact->width];
	for (int si = (int)cell.index, sn = (int)(cell.index + cell.count); si < sn; ++si)
	{
		if (hf->compact->areas[si] == RC_NULL_AREA)
			continue;
		float span_y = hf->config.bmin[1] + (float)hf->compact->spans[si].y * hf->config.ch;
		if (span_y > rc_edge - climb || span_y < rc_min)
			continue;
		if (count < max_floors)
			out_floors[count++] = span_y; /* Recast Y = Quake Z */
	}

	/* Sort descending (highest/nearest first) — simple insertion sort, count is small */
	for (int i = 1; i < count; i++)
	{
		float val = out_floors[i];
		int j = i - 1;
		while (j >= 0 && out_floors[j] < val)
		{
			out_floors[j + 1] = out_floors[j];
			j--;
		}
		out_floors[j + 1] = val;
	}

	return count;
}

extern "C" int nav_heightfield_floor_above(const nav_heightfield_t *hf,
	const float *point, float min_z, float max_z, float *out_z)
{
	float rc_point[3];
	int gx, gz;

	if (hf == nullptr || hf->compact == nullptr)
		return 0;

	nav_quake_to_recast(point, rc_point);
	gx = (int)((rc_point[0] - hf->config.bmin[0]) / hf->config.cs);
	gz = (int)((rc_point[2] - hf->config.bmin[2]) / hf->config.cs);

	if (gx < 0 || gx >= hf->compact->width || gz < 0 || gz >= hf->compact->height)
		return 0;

	/* Find the lowest walkable span between min_z and max_z (Recast Y). */
	float best = 999999.0f;
	int found = 0;
	const rcCompactCell &cell = hf->compact->cells[gx + gz * hf->compact->width];
	for (int si = (int)cell.index, sn = (int)(cell.index + cell.count); si < sn; ++si)
	{
		if (hf->compact->areas[si] == RC_NULL_AREA)
			continue;
		float span_y = hf->config.bmin[1] + (float)hf->compact->spans[si].y * hf->config.ch;
		if (span_y >= min_z && span_y <= max_z && span_y < best)
		{
			best = span_y;
			found = 1;
		}
	}

	if (found && out_z != nullptr)
		*out_z = best;
	return found;
}

extern "C" void nav_heightfield_free(nav_heightfield_t *hf)
{
	if (hf == nullptr) return;
	if (hf->compact != nullptr)
		rcFreeCompactHeightfield(hf->compact);
	delete hf;
}
