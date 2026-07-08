#include "nav_mesh.h"

#include <cfloat>
#include <cmath>
#include <cstdarg>
#include <cstdlib>
#include <cstdio>
#include <cstring>
#include <new>
#include <string>
#include <algorithm>
#include <vector>

#include "DetourCommon.h"
#include "DetourNavMesh.h"
#include "DetourNavMeshBuilder.h"
#include "DetourNavMeshQuery.h"
#include "DetourPathCorridor.h"
#include "Recast.h"

#define NAV_MESH_GOAL_SNAP_MAX_Z 128.0f

/* Vertical tolerance for snapping entity origins to floor polys.
   Entity origins sit up to ~24u above the floor (player origin is
   feet+24); keep this independent of config->walkable_height, which
   with hull-1 geometry is the residual hull gap (8u), not a player
   dimension. */
#define NAV_MESH_QUERY_CLIMB 56.0f

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
		if (getenv("NAV_RECAST_LOG") != NULL)
			fprintf(stderr, "Nav: recast[%d]: %.*s\n", (int)category, len, msg);
		if (category != RC_LOG_ERROR)
			return;
		last_error.assign(msg, static_cast<size_t>(len));
	}
};

extern "C" void nav_quake_to_recast(const float *quake, float *recast)
{
	recast[0] = quake[0];
	recast[1] = quake[2];
	recast[2] = quake[1];
}

extern "C" void nav_recast_to_quake(const float *recast, float *quake)
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
	filter->setAreaCost(NAV_AREA_HAZARD, 40.0f);
}

/* Level-exit points (trigger_changelevel volumes): a component holding
   one is escapable by definition -- players leave the level there -- so
   the deep-drop no-trap gate must not treat it as a trap (e2m4's
   moat/exit courtyard has no walking route back into the map). */
#define NAV_MAX_EXIT_POINTS 32
static float nav_exit_points[NAV_MAX_EXIT_POINTS][3];
static int nav_exit_point_count;

void nav_mesh_set_exit_points(const float *pts, int count)
{
	if (count > NAV_MAX_EXIT_POINTS)
		count = NAV_MAX_EXIT_POINTS;
	nav_exit_point_count = count;
	memcpy(nav_exit_points, pts, (size_t)count * 3 * sizeof(float));
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

int nav_mesh_poly_center_by_ref(const nav_mesh_runtime_t *navmesh,
	unsigned long long ref, float *quake_center)
{
	const dtMeshTile *tile;
	const dtPoly *poly;
	float rc[3];

	if (navmesh == nullptr || navmesh->navmesh == nullptr)
		return 0;
	if (dtStatusFailed(navmesh->navmesh->getTileAndPolyByRef((dtPolyRef)ref, &tile, &poly)))
		return 0;
	nav_mesh_poly_center(tile, poly, rc);
	nav_recast_to_quake(rc, quake_center);
	return poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION ? 2 : 1;
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

	if (navmesh != nullptr && extents == navmesh->query_half_extents_actor_origin)
		limit = fminf(fmaxf(extents[0], extents[2]) * 0.75f, 24.0f);
	else
		/* Goal snapping serves item pickup, and pickup is pure AABB
		   touch: player half-width (16) + item half-width (16) +
		   FL_ITEM expansion (15) = 47u of true horizontal reach --
		   even through thin walls, since touch never traces. A poly
		   whose closest point is within that reach really can grab
		   the item (hip1m5's SNG window sill sits 40u out). */
		limit = fminf(fmaxf(extents[0], extents[2]), 47.0f);
	return fmaxf(limit, 16.0f);
}

static int nav_mesh_build_regions(
	NavRcContext *ctx,
	rcCompactHeightfield *compact,
	const rcConfig *config)
{
	/* Watershed (stock Recast): best poly quality on simple,
	   single-layer maps like Quake DM geometry. Caller is
	   responsible for rcBuildDistanceField. */
	return rcBuildRegions(
		ctx,
		*compact,
		0,
		config->minRegionArea,
		config->mergeRegionArea);
}

/* Watershed can emit "overlapping regions": one region id whose spans
   form disjoint patches or fold back over themselves (layered hull-1
   geometry — ramps climbing over floors, floors under baked brush
   entities).  rcBuildContours is only defined for simple regions: the
   boundary walk on a folded region escapes through its "Should not
   happen" exit, leaving an OPEN contour that cuts across the room, and
   every span outside it silently gets NO polys — a mesh hole the
   item-connectivity oracle reports as a snap failure.  Global
   repartitioning (monotone/layer) trades one set of holes for another,
   so repair surgically: find the broken region ids and re-shard ONLY
   their spans into single-row runs.  A 1-cell-tall region can neither
   fold nor pinch, so its contour is always a simple ring; poly
   adjacency is edge-based downstream, so walkability across run seams
   is preserved.  Returns the number of regions repaired. */
static int nav_mesh_repair_broken_regions(rcCompactHeightfield *compact)
{
	const int w = compact->width;
	const int h = compact->height;
	const int nspans = compact->spanCount;
	std::vector<unsigned char> bad(compact->maxRegions, 0);
	int nbad = 0;

	/* Broken test 1: two spans of the same region in one cell. */
	for (int ci = 0; ci < w * h; ++ci)
	{
		const rcCompactCell &c = compact->cells[ci];
		for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
		{
			const unsigned short reg = compact->spans[i].reg;
			if (reg == 0 || (reg & RC_BORDER_REG) || bad[reg])
				continue;
			for (int j = (int)c.index; j < i; ++j)
				if (compact->spans[j].reg == reg)
				{
					bad[reg] = 1;
					nbad++;
					break;
				}
		}
	}

	/* Broken test 2: region spans form more than one 4-connected
	   component (only the first patch would get a contour). */
	{
		std::vector<int> span_x(nspans), span_z(nspans);
		std::vector<unsigned char> visited(nspans, 0);
		std::vector<unsigned char> reg_seen(compact->maxRegions, 0);
		std::vector<int> stack;

		for (int z = 0; z < h; ++z)
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = compact->cells[x + z * w];
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					span_x[i] = x;
					span_z[i] = z;
				}
			}

		for (int i = 0; i < nspans; ++i)
		{
			const unsigned short reg = compact->spans[i].reg;
			if (visited[i] || reg == 0 || (reg & RC_BORDER_REG))
				continue;
			if (reg_seen[reg])
			{
				if (!bad[reg])
				{
					bad[reg] = 1;
					nbad++;
				}
				continue;
			}
			reg_seen[reg] = 1;
			stack.clear();
			stack.push_back(i);
			visited[i] = 1;
			while (!stack.empty())
			{
				const int cur = stack.back();
				stack.pop_back();
				const rcCompactSpan &s = compact->spans[cur];
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) == RC_NOT_CONNECTED)
						continue;
					const int ax = span_x[cur] + rcGetDirOffsetX(dir);
					const int az = span_z[cur] + rcGetDirOffsetY(dir);
					const int ai = (int)compact->cells[ax + az * w].index + rcGetCon(s, dir);
					if (!visited[ai] && compact->spans[ai].reg == reg)
					{
						visited[ai] = 1;
						stack.push_back(ai);
					}
				}
			}
		}
	}

	if (nbad == 0)
		return 0;

	/* Re-shard the broken regions' spans into single-row runs.  A run
	   extends west→east while the west-connected neighbor belongs to
	   the same original region and its run hasn't already claimed a
	   span in this cell (two layers may share one west neighbor). */
	{
		std::vector<unsigned short> orig(nspans);
		std::vector<unsigned short> newreg(nspans, 0);
		std::vector<unsigned short> cell_runs;

		for (int i = 0; i < nspans; ++i)
			orig[i] = compact->spans[i].reg;

		for (int z = 0; z < h; ++z)
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = compact->cells[x + z * w];
				cell_runs.clear();
				for (int i = (int)c.index, ni = (int)(c.index + c.count); i < ni; ++i)
				{
					const unsigned short reg = orig[i];
					if (reg == 0 || (reg & RC_BORDER_REG) || !bad[reg])
						continue;
					unsigned short run = 0;
					const rcCompactSpan &s = compact->spans[i];
					if (x > 0 && rcGetCon(s, 0) != RC_NOT_CONNECTED)
					{
						const int ai = (int)compact->cells[(x - 1) + z * w].index + rcGetCon(s, 0);
						if (orig[ai] == reg && newreg[ai] != 0)
						{
							run = newreg[ai];
							for (size_t k = 0; k < cell_runs.size(); ++k)
								if (cell_runs[k] == run)
								{
									run = 0;
									break;
								}
						}
					}
					if (run == 0)
					{
						if (compact->maxRegions >= RC_BORDER_REG)
							return nbad; /* id space exhausted; keep mesh valid */
						run = compact->maxRegions++;
					}
					newreg[i] = run;
					compact->spans[i].reg = run;
					cell_runs.push_back(run);
				}
			}
	}
	return nbad;
}

/* Actor-origin poly search box, biased downward.  Actors stand ON
   floors: the nav surface sits ~22u below the origin and can never be
   more than a step above it.  A symmetric box snaps wall-hugging actors
   to ledge/pedestal polys overhead (the hull-1 shadow band next to tall
   brushes has no floor polys), and paths from there skip required jump
   links (dm6 armor1 pedestal, dm4 under-stair). */
extern "C" void nav_mesh_actor_snap_box(const nav_mesh_runtime_t *navmesh,
	const float *rc_point, float *center, float *half_extents)
{
	const float up = 8.0f;
	const float down = navmesh->query_half_extents_actor_origin[1] + 32.0f;

	center[0] = rc_point[0];
	center[1] = rc_point[1] - (down - up) * 0.5f;
	center[2] = rc_point[2];
	half_extents[0] = navmesh->query_half_extents_actor_origin[0];
	half_extents[1] = (up + down) * 0.5f;
	half_extents[2] = navmesh->query_half_extents_actor_origin[2];
}

/* Floor-capped actor snap.  findNearestPoly selects candidates by poly
   BOUNDS overlap, so a down-biased box can still return a surface point
   well above the actor (shelf poly whose bbox dips into the box — dm6
   armor pedestal snapped bots +24u onto the shelf, skipping the jump
   link).  Query candidates with the biased box, then reject any whose
   closest surface point is above origin+8: a real floor is ~22u below
   the origin, never above it.

   Ranking is horizontal-first, not 3D-nearest: hull widening extends
   ledge polys 16-24u sideways, so a bot standing on a low floor next
   to a 26u rim is often 3D-closer to the rim's edge (lateral ~16u)
   than to its own floor (24u straight down).  Snapping to the rim
   builds corridors the bot can't physically walk (e2m1 moat rim pin).
   Weight lateral distance 4x and measure vertical distance from the
   expected feet level (origin - 24) so the poly underfoot wins. */
int nav_mesh_actor_floor_snap(const nav_mesh_runtime_t *navmesh,
	const dtQueryFilter *filter, const float *rc_point,
	dtPolyRef *out_ref, float *out_pt, bool *out_over)
{
	float center[3], half[3];
	dtPolyRef polys[128];
	int n = 0;

	nav_mesh_actor_snap_box(navmesh, rc_point, center, half);
	if (dtStatusFailed(navmesh->query->queryPolygons(
			center, half, filter, polys, &n, 128)))
		return 0;

	const float max_y = rc_point[1] + 8.0f;
	float best_d = FLT_MAX;
	*out_ref = 0;
	if (out_over)
		*out_over = false;

	for (int i = 0; i < n; i++)
	{
		float pt[3];
		bool over = false;
		if (dtStatusFailed(navmesh->query->closestPointOnPoly(
				polys[i], rc_point, pt, &over)))
			continue;
		if (pt[1] > max_y)
			continue;
		float dx = pt[0] - rc_point[0];
		float dz = pt[2] - rc_point[2];
		float dy = pt[1] - (rc_point[1] - 24.0f);
		float d = sqrtf(dx * dx + dz * dz) * 4.0f + fabsf(dy);
		if (d < best_d)
		{
			best_d = d;
			*out_ref = polys[i];
			dtVcopy(out_pt, pt);
			if (out_over)
				*out_over = over;
		}
	}
	return *out_ref != 0;
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

	if (extents == navmesh->query_half_extents_actor_origin)
	{
		bool over = false;
		if (!nav_mesh_actor_floor_snap(navmesh, &filter, recast_point,
				nearest_ref, nearest_pt, &over))
		{
			nav_set_error(error, error_size, "No floor polygon at actor position");
			*nearest_ref = 0;
			memset(nearest_pt, 0, sizeof(float) * 3);
			if (is_over_poly != nullptr)
				*is_over_poly = false;
			return 0;
		}
		if (is_over_poly != nullptr)
			*is_over_poly = over;
	}
	else
	{
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
	if (*nearest_ref == 0)
	{
		nav_set_error(error, error_size, "No polygon within query extents");
		return 0;
	}
	return 1;
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

/* Build an off-mesh link; height_delta is implied by the endpoints and the
   remaining fields default to zero. */
static nav_off_mesh_link_t nav_make_link(const float *start, const float *end,
	int type, int bidirectional, float radius)
{
	nav_off_mesh_link_t lk;
	memset(&lk, 0, sizeof(lk));
	lk.start[0] = start[0]; lk.start[1] = start[1]; lk.start[2] = start[2];
	lk.end[0] = end[0]; lk.end[1] = end[1]; lk.end[2] = end[2];
	lk.radius = radius;
	lk.bidirectional = bidirectional;
	lk.link_type = type;
	lk.height_delta = end[2] - start[2];
	lk.required_speed = 0;
	return lk;
}

/* Quake-coord centroids for the ground polys [0, ground). */
static void nav_collect_ground_centroids(const dtMeshTile *tile, int ground,
	std::vector<float> &q)
{
	q.resize((size_t)ground * 3);
	for (int i = 0; i < ground; i++)
	{
		float c[3];
		nav_mesh_poly_center(tile, &tile->polys[i], c);
		nav_recast_to_quake(c, &q[i * 3]);
	}
}

/* Union-find over ground<->ground poly adjacency only (off-mesh links never
   bridge here -- ground ends at tile->header->offMeshBase) -- the
   contiguous walkable-floor components shared by the link-generation
   passes below.  When skip_disabled is set, an edge touching a disabled
   poly (flags==0) doesn't bridge components either, so a flagged-off poly
   can't smuggle two patches together (used by the drop/swim passes). */
static std::vector<int> nav_mesh_ground_uf(const dtMeshTile *tile,
	const dtNavMesh *mesh, int ground, bool skip_disabled)
{
	std::vector<int> ga(ground);
	for (int i = 0; i < ground; i++) ga[i] = i;
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	for (int i = 0; i < ground; i++)
	{
		const dtPoly *p = &tile->polys[i];
		if (skip_disabled && p->flags == 0) continue;
		for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
			if ((int)np < ground && (!skip_disabled || tile->polys[np].flags != 0))
				ga[gf(i)] = gf((int)np);
		}
	}
	return ga;
}

extern "C" int nav_mesh_compute_orphan_jumps(
	nav_mesh_runtime_t *nav,
	nav_jump_validate_fn validate, void *user,
	nav_off_mesh_link_t **out_jumps)
{
	*out_jumps = nullptr;
	if (nav == nullptr || nav->navmesh == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = nav->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	if (ground <= 0)
		return 0;

	/* union-find over ground polys using the full current graph (ground
	   adjacencies + every off-mesh connection) -> who is already reachable. */
	std::vector<int> uf(ground);
	for (int i = 0; i < ground; i++) uf[i] = i;
	auto ff = [&](int x) { while (uf[x] != x) { uf[x] = uf[uf[x]]; x = uf[x]; } return x; };
	auto uni = [&](int a, int b) { if (a >= 0 && b >= 0) uf[ff(a)] = ff(b); };
	for (int i = 0; i < ground; i++)
	{
		const dtPoly *p = &tile->polys[i];
		for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
			if ((int)np < ground) uni(i, (int)np);
		}
	}
	for (int pi = ground; pi < tile->header->polyCount; pi++)
	{
		const dtPoly *P = &tile->polys[pi];
		int ep[2] = {-1, -1}, ne = 0;
		for (unsigned int k = P->firstLink; k != DT_NULL_LINK && ne < 2; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
			if ((int)np < ground) ep[ne++] = (int)np;
		}
		uni(ep[0], ep[1]);
	}

	std::vector<int> comp(ground, -1), csize;
	for (int i = 0; i < ground; i++)
	{
		int r = ff(i);
		if (comp[r] < 0) { comp[r] = (int)csize.size(); csize.push_back(0); }
		comp[i] = comp[r]; csize[comp[i]]++;
	}
	int largest = 0;
	for (size_t c = 1; c < csize.size(); c++)
		if (csize[c] > csize[largest]) largest = (int)c;

	/* Quake-coord poly centroids. */
	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	/* Connect components to the main mesh in WAVES (Prim-style growth): a
	   cluster of orphans walkably/jumpably chained off the main area links one
	   hop at a time -- pass 1 connects whatever reaches main directly, pass 2
	   whatever reaches a pass-1 poly, and so on.  Linking only orphan->main
	   (one pass) can't chain such clusters in.  validate() picks walk vs jump
	   per pair; both are bidirectional. */
	std::vector<char> conn(csize.size(), 0);
	conn[largest] = 1;
	std::vector<nav_off_mesh_link_t> jumps;
	bool progress = true;
	while (progress)
	{
		progress = false;
		for (int c = 0; c < (int)csize.size(); c++)
		{
			if (conn[c]) continue;
			float bestcost = 1e9f, bestStart[3] = {0,0,0}, bestEnd[3] = {0,0,0};
			int bestType = 0;
			for (int o = 0; o < ground; o++)
			{
				if (comp[o] != c) continue;
				for (int m = 0; m < ground; m++)
				{
					if (!conn[comp[m]]) continue; /* link to an already-connected poly */
					const float *qo = &q[o * 3], *qm = &q[m * 3];
					float dz = qo[2] - qm[2];
					float adz = dz < 0 ? -dz : dz;
					float dx = qo[0] - qm[0], dy = qo[1] - qm[1];
					float hd = sqrtf(dx * dx + dy * dy);
					/* 48u is a run-jump's reach; allow up to a rocket-jump's
					   reach so validate can offer an AI_SUPER_JUMP for orphan
					   ledges above that.  validate still gates the physics, and
					   cost prefers the cheaper walk/jump, so this only adds
					   links for components nothing else could connect. */
					if (adz > 256.0f) continue;
					if (hd > 280.0f || hd < 8.0f) continue;
					float cost = hd + adz;
					if (cost >= bestcost) continue;
					int t = validate(qm, qo, user);
					/* Don't connect an orphan by rocket jump: RJ is unreliable
					   even for bots that own the launcher (they grind the link),
					   and impossible for those that don't.  Leave an RJ-only
					   area disconnected — bots ignore it instead of pinning
					   under it (dm2 sunken ledge: bots stuck 90%+). */
					if (t == AI_SUPER_JUMP)
						continue;
					if (t)
					{
						bestcost = cost; bestType = t;
						bestStart[0] = qm[0]; bestStart[1] = qm[1]; bestStart[2] = qm[2];
						bestEnd[0] = qo[0]; bestEnd[1] = qo[1]; bestEnd[2] = qo[2];
					}
				}
			}
			if (bestType)
			{
				/* walk + jump are both 2-way */
				jumps.push_back(nav_make_link(bestStart, bestEnd, bestType, 1, 32.0f));
				conn[c] = 1;
				progress = true;
			}
		}
	}

	if (jumps.empty())
		return 0;
	nav_off_mesh_link_t *out = (nav_off_mesh_link_t *)malloc(jumps.size() * sizeof(nav_off_mesh_link_t));
	if (out == nullptr)
		return 0;
	memcpy(out, jumps.data(), jumps.size() * sizeof(nav_off_mesh_link_t));
	*out_jumps = out;
	fprintf(stderr, "Nav: computed %d orphan-connecting jump links\n", (int)jumps.size());
	return (int)jumps.size();
}

/* Post-build pass: complete DIRECTED connectivity.  The orphan pass joins
   areas with no connection at all; this one fixes areas connected only ONE
   way -- you can leave but not enter (a drop room whose only exit is a
   teleport), or enter but not leave.  Such an area's waypoint edges path
   PARTIAL even though both ends mesh.

   Forward set F = ground reachable FROM the main area following directed
   links; backward set B = ground that can REACH main.  A ground-adjacency
   component (a contiguous walkable patch -- F and B are unions of these,
   since floor adjacency is two-way) that is in B but not F needs an IN link;
   in F but not B needs an OUT link.  We add ONLY the missing direction -- the
   complementary direction already exists by definition, so this can never
   strand a bot (the trap the earlier bidirectional SCC attempt caused).  */
int nav_mesh_compute_directed_links(
	nav_mesh_runtime_t *navmesh,
	nav_jump_validate_fn validate, void *user,
	nav_off_mesh_link_t **out_links)
{
	*out_links = nullptr;
	if (navmesh == nullptr || navmesh->navmesh == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	const int npolys = tile->header->polyCount;
	if (ground <= 0)
		return 0;

	/* Ground-adjacency components: union only ground<->ground edges (an
	   off-mesh link goes ground -> offmesh poly -> ground, so it never
	   unions here -- exactly the contiguous-floor patches we want). */
	std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, false);
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	std::vector<int> gacomp(ground, -1), gasize;
	for (int i = 0; i < ground; i++)
	{
		int r = gf(i);
		if (gacomp[r] < 0) { gacomp[r] = (int)gasize.size(); gasize.push_back(0); }
		gacomp[i] = gacomp[r]; gasize[gacomp[i]]++;
	}
	int maingc = 0;
	for (size_t c = 1; c < gasize.size(); c++)
		if (gasize[c] > gasize[maingc]) maingc = (int)c;

	/* Reverse adjacency from the tile (built once; per-round extras kept
	   separately below). */
	std::vector<std::vector<int>> rev(npolys);
	for (int i = 0; i < npolys; i++)
	{
		const dtPoly *p = &tile->polys[i];
		for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
			if ((int)np < npolys) rev[(int)np].push_back(i);
		}
	}

	/* Quake-coord centroids. */
	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	int dbg_comp = -1;
	if (const char *db = getenv("NAV_DIR_DEBUG"))
	{
		float tx, ty, tz;
		if (sscanf(db, "%f %f %f", &tx, &ty, &tz) == 3)
		{
			float best = 1e9f;
			for (int i = 0; i < ground; i++)
			{
				float dx = q[i * 3] - tx, dy = q[i * 3 + 1] - ty, dz = q[i * 3 + 2] - tz;
				float d = dx * dx + dy * dy + dz * dz;
				if (d < best) { best = d; dbg_comp = gacomp[i]; }
			}
			fprintf(stderr, "Nav: DIRDBG target comp=%d size=%d main=%d (nearest dist %.0f)\n",
				dbg_comp, dbg_comp >= 0 ? gasize[dbg_comp] : 0, maingc, sqrtf(best));
		}
	}

	std::vector<nav_off_mesh_link_t> links;
	/* Links accepted this call, as directed poly-index edges, so later
	   rounds see the reachability the earlier repairs created.  A single
	   snapshot pass cannot cascade: on hip2m6 the item ledge's only valid
	   IN source poly is itself only main-reachable via another comp's
	   repair link from the same pass, so the ledge stayed orphaned.
	   Iterate to a fixpoint instead.  This also lets an isolated pocket
	   earn an IN link in a later round (its round-1 OUT makes it
	   B-not-F): unlike the reverted bidirectional hack, that IN is a
	   fresh candidate search, independently validated as a real one-way
	   move (jump up or survivable drop), so it cannot send bots at
	   physically-infeasible reverse climbs. */
	std::vector<std::vector<int>> extra_adj(npolys), extra_rev(npolys);
	std::vector<int> stack;
	int progress = 1, rounds = 0;
	while (progress && rounds++ < 16)
	{
		progress = 0;

		/* Forward BFS from main over directed links (through off-mesh
		   polys), plus the edges accepted in earlier rounds. */
		std::vector<char> fwd(npolys, 0);
		for (int i = 0; i < ground; i++)
			if (gacomp[i] == maingc) { fwd[i] = 1; stack.push_back(i); }
		while (!stack.empty())
		{
			int i = stack.back(); stack.pop_back();
			const dtPoly *p = &tile->polys[i];
			for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
			{
				if (tile->links[k].ref == 0) continue;
				unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
				if ((int)np < npolys && !fwd[np]) { fwd[np] = 1; stack.push_back((int)np); }
			}
			for (size_t e = 0; e < extra_adj[i].size(); e++)
			{
				int j = extra_adj[i][e];
				if (!fwd[j]) { fwd[j] = 1; stack.push_back(j); }
			}
		}

		/* Backward BFS (ground that can reach main). */
		std::vector<char> bwd(npolys, 0);
		for (int i = 0; i < ground; i++)
			if (gacomp[i] == maingc) { bwd[i] = 1; stack.push_back(i); }
		while (!stack.empty())
		{
			int i = stack.back(); stack.pop_back();
			for (size_t e = 0; e < rev[i].size(); e++)
			{
				int j = rev[i][e];
				if (!bwd[j]) { bwd[j] = 1; stack.push_back(j); }
			}
			for (size_t e = 0; e < extra_rev[i].size(); e++)
			{
				int j = extra_rev[i][e];
				if (!bwd[j]) { bwd[j] = 1; stack.push_back(j); }
			}
		}

		/* Per-GA-component forward/backward reachability. */
		std::vector<char> gc_fwd(gasize.size(), 0), gc_bwd(gasize.size(), 0);
		for (int i = 0; i < ground; i++)
		{
			if (fwd[i]) gc_fwd[gacomp[i]] = 1;
			if (bwd[i]) gc_bwd[gacomp[i]] = 1;
		}

		/* For each off-main GA-comp connected only one way, add the missing
		   direction.  ENTER (need IN, comp in B not F): a forward poly -> the
		   comp.  EXIT (need OUT, comp in F not B): the comp -> a backward poly. */
		for (int c = 0; c < (int)gasize.size(); c++)
		{
			if (c == maingc) continue;
			int need_in = (gc_bwd[c] && !gc_fwd[c]);
			int need_out = (gc_fwd[c] && !gc_bwd[c]);
			/* A component with NO existing link either way (never reached by
			   the JUMP/SUPER_JUMP passes, e.g. an isolated ledge or an item
			   alcove) falls through both checks above and used to be skipped
			   entirely -- permanently trapping any bot that spawns or lands
			   there, and hiding any item sitting in it from every spawn.
			   Search for an OUT candidate (that's the minimum a dead-end spot
			   needs); a later round can then give it a validated IN. */
			int isolated = (!gc_fwd[c] && !gc_bwd[c]);
			if (!need_in && !need_out)
			{
				if (isolated)
					need_out = 1;
				else
				{
					if (c == dbg_comp)
						fprintf(stderr, "Nav: DIRDBG round=%d comp=%d healthy (fwd=%d bwd=%d), skipped\n",
							rounds, c, gc_fwd[c], gc_bwd[c]);
					continue;
				}
			}

			float bestcost = 1e9f, bestS[3] = {0,0,0}, bestE[3] = {0,0,0};
			int bestType = 0, bestFrom = -1, bestTo = -1;
			int dbg = (c == dbg_comp);
			int rej_near = 0, rej_far = 0, rej_dz = 0, rej_val = 0, rej_dir = 0;
			if (dbg)
				fprintf(stderr, "Nav: DIRDBG round=%d comp=%d fwd=%d bwd=%d need_in=%d need_out=%d iso=%d\n",
					rounds, c, gc_fwd[c], gc_bwd[c], need_in, need_out, isolated);
			for (int o = 0; o < ground; o++)
			{
				if (gacomp[o] != c) continue;
				for (int m = 0; m < ground; m++)
				{
					/* IN link comes from a forward poly; OUT link goes to a
					   backward poly. */
					if (need_in && !fwd[m]) { rej_dir++; continue; }
					if (need_out && !bwd[m]) { rej_dir++; continue; }
					if (gacomp[m] == c) continue;
					const float *qo = &q[o * 3], *qm = &q[m * 3];
					float dx = qo[0] - qm[0], dy = qo[1] - qm[1];
					float hd = sqrtf(dx * dx + dy * dy);
					float dz = qo[2] - qm[2], adz = dz < 0 ? -dz : dz;
					if (hd > 320.0f) { rej_far++; continue; }
					if (hd < 8.0f) { rej_near++; continue; }
					if (adz > 320.0f) { rej_dz++; continue; }
					float cost = hd + adz;
					if (cost >= bestcost) continue;

					/* The link runs FROM the main side TO the comp for IN, and
					   FROM the comp TO the main side for OUT.  Name the ends so
					   validate sees the actual direction of travel. */
					const float *from = need_in ? qm : qo;
					const float *to   = need_in ? qo : qm;
					int type = validate(from, to, user);
					/* Never restore one-way connectivity with a rocket jump: bots
					   grind RJ links (they can't reliably execute them, or own no
					   launcher), so an area reachable only by RJ is a trap, not a
					   fix.  Leave it one-way and let it stay that way. */
					if (type == AI_SUPER_JUMP)
						continue;
					if (!type)
					{
						/* validate only models level/up moves.  A downward move
						   is a drop -- survivable fall, clear-ish column, not into
						   lava (the cycle's other half already exists, so the bot
						   won't be stranded down there). */
						float ddz = to[2] - from[2];
						if (ddz < -18.0f && ddz > -320.0f)  /* below step height = a drop */
							type = AI_DROP;
					}
					if (!type)
					{
						rej_val++;
						if (dbg && rej_val <= 10)
							fprintf(stderr, "Nav: DIRDBG valfail (%.0f %.0f %.0f)->(%.0f %.0f %.0f) hd=%.0f dz=%.0f\n",
								from[0], from[1], from[2], to[0], to[1], to[2], hd, dz);
						continue;
					}
					bestcost = cost; bestType = type;
					bestS[0] = from[0]; bestS[1] = from[1]; bestS[2] = from[2];
					bestE[0] = to[0]; bestE[1] = to[1]; bestE[2] = to[2];
					bestFrom = need_in ? m : o;
					bestTo   = need_in ? o : m;
				}
			}
			if (dbg)
				fprintf(stderr, "Nav: DIRDBG comp=%d result type=%d cost=%.0f from=(%.0f %.0f %.0f) to=(%.0f %.0f %.0f) rej near=%d far=%d dz=%d val=%d dir=%d\n",
					c, bestType, bestcost, bestS[0], bestS[1], bestS[2], bestE[0], bestE[1], bestE[2],
					rej_near, rej_far, rej_dz, rej_val, rej_dir);
			if (bestType)
			{
				/* One-way only: DROP/JUMP links found by this merge pass
				   aren't necessarily traversable in reverse (the reverted
				   bidirectional attempt sent bots at "climb the drop" moves,
				   regressing e1m1/e1m7/e2m2/e2m4/e2m7).  If the pocket needs
				   the other direction too, a later round finds and validates
				   it as its own link. */
				links.push_back(nav_make_link(bestS, bestE, bestType, 0, 32.0f));
				extra_adj[bestFrom].push_back(bestTo);
				extra_rev[bestTo].push_back(bestFrom);
				progress = 1;
			}
		}
	}

	if (links.empty())
		return 0;
	nav_off_mesh_link_t *outp = (nav_off_mesh_link_t *)malloc(links.size() * sizeof(nav_off_mesh_link_t));
	if (outp == nullptr)
		return 0;
	memcpy(outp, links.data(), links.size() * sizeof(nav_off_mesh_link_t));
	*out_links = outp;
	fprintf(stderr, "Nav: computed %d directed-connectivity links\n", (int)links.size());
	return (int)links.size();
}

/* Post-build pass: bridge LOCAL connectivity gaps with run-jumps.  The
   orphan pass joins areas with no connection at all; the directed pass
   completes one-way areas.  This one handles two walkable patches each
   connected to the main mesh independently but NOT to each other -- two
   ledges reachable from below by their own drops/jumps, with no edge
   between them (dm3 wp62->wp63).  Their waypoint edge paths PARTIAL even
   though both ends mesh and both reach main.

   Candidates: cross-ground-component poly pairs within a run-jump's reach.
   GATE: add the jump ONLY when no direct mesh path between them exists
   today (findPath returns no full path).  Every link is then NEW
   reachability, never a redundant shortcut over an existing route -- the
   shortcut jumps that historically sprayed false links and regressed
   behavior.  Run-jumps only (no RJ: bots grind it) and bidirectional (a
   run-jump clears both ways, so it can't strand a bot). */
int nav_mesh_compute_gap_jumps(
	nav_mesh_runtime_t *navmesh,
	nav_jump_validate_fn validate, void *user,
	nav_off_mesh_link_t **out_links)
{
	*out_links = nullptr;
	if (navmesh == nullptr || navmesh->navmesh == nullptr
		|| navmesh->query == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	if (ground <= 0)
		return 0;

	/* Ground-adjacency components (ground<->ground links only): contiguous
	   walkable patches.  Same construction as the directed pass. */
	std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, false);
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	std::vector<int> gacomp(ground);
	for (int i = 0; i < ground; i++) gacomp[i] = gf(i);

	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	const dtPolyRef base = mesh->getPolyRefBase(tile);
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	/* One bridge per pair of patches, not per poly: linear-scanned seen-list
	   (component count is small, links fewer still). */
	std::vector<int> seenLo, seenHi;
	auto pair_seen = [&](int x, int y) {
		int lo = x < y ? x : y, hi = x < y ? y : x;
		for (size_t s = 0; s < seenLo.size(); s++)
			if (seenLo[s] == lo && seenHi[s] == hi) return true;
		return false;
	};

	std::vector<nav_off_mesh_link_t> links;
	for (int a = 0; a < ground; a++)
	{
		if (tile->polys[a].flags == 0) continue; /* disabled sliver (island cull) */

		/* Cheapest cross-patch poly within run-jump reach.  Horizontal cap
		   216u matches the validator's window; vertical cap 48u brackets a
		   run-jump's apex (~45u) -- validate() does the exact physics. */
		float bestcost = 1e9f; int bestB = -1;
		for (int b = 0; b < ground; b++)
		{
			if (tile->polys[b].flags == 0) continue;
			if (gacomp[a] == gacomp[b]) continue;
			const float *qa = &q[a * 3], *qb = &q[b * 3];
			float dx = qa[0]-qb[0], dy = qa[1]-qb[1], dz = qa[2]-qb[2];
			float hd = sqrtf(dx*dx + dy*dy);
			float adz = dz < 0 ? -dz : dz;
			if (hd < 8.0f || hd > 216.0f) continue;
			if (adz > 48.0f) continue;
			float cost = hd + adz;
			if (cost < bestcost) { bestcost = cost; bestB = b; }
		}
		if (bestB < 0) continue;
		if (pair_seen(gacomp[a], gacomp[bestB])) continue;

		const float *qa = &q[a * 3], *qb = &q[bestB * 3];

		/* A genuine run-jump both ways -- validate gates the apex arc and
		   the harder (upward) direction.  Reject WALK/SUPER_JUMP/DROP/none. */
		if (validate(qa, qb, user) != AI_JUMP)
			continue;

		/* Connectivity gate: skip if a direct mesh path already exists --
		   adding it would be a redundant shortcut.  A PARTIAL/failed path
		   means the two ledges have no route between them today. */
		float ra[3], rb[3];
		dtPolyRef path[256]; int pc = 0;
		nav_quake_to_recast(qa, ra);
		nav_quake_to_recast(qb, rb);
		dtStatus st = navmesh->query->findPath(base | (dtPolyRef)a, base | (dtPolyRef)bestB,
			ra, rb, &filter, path, &pc, 256);
		if (!dtStatusFailed(st) && !dtStatusDetail(st, DT_PARTIAL_RESULT) && pc > 0)
			continue;

		links.push_back(nav_make_link(qa, qb, AI_JUMP, 1, 32.0f));
		seenLo.push_back(gacomp[a] < gacomp[bestB] ? gacomp[a] : gacomp[bestB]);
		seenHi.push_back(gacomp[a] < gacomp[bestB] ? gacomp[bestB] : gacomp[a]);
	}

	if (links.empty())
		return 0;
	nav_off_mesh_link_t *outp = (nav_off_mesh_link_t *)malloc(links.size() * sizeof(nav_off_mesh_link_t));
	if (outp == nullptr)
		return 0;
	memcpy(outp, links.data(), links.size() * sizeof(nav_off_mesh_link_t));
	*out_links = outp;
	fprintf(stderr, "Nav: computed %d gap-bridging jump links\n", (int)links.size());
	return (int)links.size();
}

/* Post-build pass: rocket-jump links.  The manual graphs tag a few high
   ledges AI_SUPER_JUMP -- reachable only by firing a rocket at your feet
   mid-jump for a boost a run-jump can't make.  The orphan and directed
   passes deliberately REFUSE rocket jumps: an area reachable ONLY by RJ
   traps a bot that owns no launcher (it grinds the link forever).  This
   pass sidesteps that trap with a strict safety gate -- it adds a one-way
   RJ-UP only when the high end can ALREADY get back down by some existing
   route (findPath high->low succeeds), so the RJ is a bonus up-route for
   launcher-owning bots, never the sole access.  A bot without the rocket
   launcher abandons the goal (bot_move) rather than pinning.

   validate() does the RJ envelope physics (up past run-jump reach but
   within a single rocket's lift, horizontal tight).  Directional (low->high
   only): a rocket jump can't go down, and the return path already exists. */
int nav_mesh_compute_rocket_jumps(
	nav_mesh_runtime_t *navmesh,
	nav_jump_validate_fn validate, void *user,
	nav_jump_value_fn has_value, void *value_user,
	nav_off_mesh_link_t **out_links)
{
	*out_links = nullptr;
	if (navmesh == nullptr || navmesh->navmesh == nullptr
		|| navmesh->query == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	if (ground <= 0)
		return 0;

	std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, false);
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	std::vector<int> gacomp(ground);
	for (int i = 0; i < ground; i++) gacomp[i] = gf(i);

	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	const dtPolyRef base = mesh->getPolyRefBase(tile);
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	std::vector<int> seenLo, seenHi;
	auto pair_seen = [&](int x, int y) {
		int lo = x < y ? x : y, hi = x < y ? y : x;
		for (size_t s = 0; s < seenLo.size(); s++)
			if (seenLo[s] == lo && seenHi[s] == hi) return true;
		return false;
	};

	std::vector<nav_off_mesh_link_t> links;
	for (int lo = 0; lo < ground; lo++)
	{
		if (tile->polys[lo].flags == 0) continue;

		/* Cheapest higher cross-patch poly inside the RJ-up envelope:
		   up past a run-jump's apex (48u) but within one rocket's lift
		   (256u), horizontal tight (128u) -- validate() does the exact
		   physics and overhead-clearance. */
		float bestcost = 1e9f; int bestHi = -1;
		for (int hi = 0; hi < ground; hi++)
		{
			if (tile->polys[hi].flags == 0) continue;
			if (gacomp[lo] == gacomp[hi]) continue;
			const float *ql = &q[lo * 3], *qh = &q[hi * 3];
			float dz = qh[2] - ql[2];
			if (dz <= 48.0f || dz > 256.0f) continue;
			float dx = qh[0]-ql[0], dy = qh[1]-ql[1];
			float hd = sqrtf(dx*dx + dy*dy);
			if (hd < 8.0f || hd > 128.0f) continue;
			float cost = hd + dz;
			if (cost < bestcost) { bestcost = cost; bestHi = hi; }
		}
		if (bestHi < 0) continue;
		if (pair_seen(gacomp[lo], gacomp[bestHi])) continue;

		const float *ql = &q[lo * 3], *qh = &q[bestHi * 3];

		if (validate(ql, qh, user) != AI_SUPER_JUMP)
			continue;

		float rl[3], rh[3];
		dtPolyRef path[256]; int pc = 0;
		nav_quake_to_recast(ql, rl);
		nav_quake_to_recast(qh, rh);

		/* Up-access gate: skip if the bot can already reach the high ledge
		   (an RJ shortcut over an existing climb just invites grinding). */
		dtStatus up = navmesh->query->findPath(base | (dtPolyRef)lo, base | (dtPolyRef)bestHi,
			rl, rh, &filter, path, &pc, 256);
		if (!dtStatusFailed(up) && !dtStatusDetail(up, DT_PARTIAL_RESULT) && pc > 0)
			continue;

		/* No-trap gate: the high end must ALREADY get back down some other
		   way, so the one-way RJ-up can never strand a bot up top. */
		dtStatus down = navmesh->query->findPath(base | (dtPolyRef)bestHi, base | (dtPolyRef)lo,
			rh, rl, &filter, path, &pc, 256);
		if (dtStatusFailed(down) || dtStatusDetail(down, DT_PARTIAL_RESULT) || pc < 1)
			continue;

		/* Worth-it gate: skip ledges with nothing to reach.  Both prior
		   gates already established the ledge is otherwise unreachable --
		   but that alone isn't sufficient reason to spawn a link, since
		   plenty of scenery nubs (monster perches, window sills) fail the
		   same reachability probes and were never meant to be player-
		   reachable.  Without this, those get a "rocket jump to nowhere"
		   that a bot will actually use once it meets the health/quad gate.

		   Hands over every ground centroid reachable from bestHi by walking
		   the CURRENT graph forward (a BFS over tile->links, not the
		   ground-only union-find used to seed candidates above), crossing
		   already-baked door/orphan-jump/directed/gap links -- so a landing
		   that leads into a room via one more ordinary jump still counts,
		   not just a landing that happens to sit in the same bare-floor
		   patch as the item.  Deliberately does NOT cross teleporters
		   (off-mesh polys carrying NAV_AREA_WALK) or plat/train rides
		   (NAV_AREA_PLAT): either can bridge to a totally unrelated part of
		   the map, which would make nearly any landing "reach" nearly any
		   item and defeat the gate (seen on e1m2: every candidate near a
		   door found the same item three rooms away through a teleporter). */
		if (has_value != nullptr)
		{
			const int npolys = tile->header->polyCount;
			std::vector<char> seenPoly(npolys, 0);
			std::vector<char> hopUsed(npolys, 0);
			std::vector<int> frontier;
			seenPoly[bestHi] = 1;
			frontier.push_back(bestHi);
			for (size_t fi = 0; fi < frontier.size(); fi++)
			{
				int u = frontier[fi];
				const dtPoly *pu = &tile->polys[u];
				for (unsigned int k = pu->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
				{
					if (tile->links[k].ref == 0) continue;
					unsigned int s, t, np; mesh->decodePolyId(tile->links[k].ref, s, t, np);
					if ((int)np >= npolys || seenPoly[np]) continue;

					/* Off-mesh connection polys (np >= ground) need an area
					   check before crossing: teleporters are area NAV_AREA_WALK
					   (indistinguishable from plain ground EXCEPT that only an
					   off-mesh conn poly can carry that area at this index range)
					   and plat/train rides are NAV_AREA_PLAT -- both can bridge to
					   a totally unrelated part of the map, which would make almost
					   any RJ landing "reach" almost any item and defeat this gate
					   entirely (seen on e1m2: every candidate near the door found
					   the same item through a teleporter three rooms away). Only
					   ordinary jump/drop/door links stay local enough to count as
					   "this same neighborhood, one more hop."

					   That hop budget has to actually be enforced, not just
					   type-filtered: walking within a component once crossed is
					   unrestricted below (ordinary ground-to-ground links aren't
					   hop-limited), so without a cap the walk can cross a SECOND,
					   THIRD, etc. door/jump/drop out of that component and cascade
					   through the whole level one legitimate-looking hop at a
					   time -- the same "any item counts as reachable" failure the
					   type filter above was meant to prevent, just via a chain of
					   ordinary links instead of one teleporter (seen on e1m2: a
					   landing on top of a hallway door chained through a second
					   door deep into the map and credited an item rooms away that
					   was never actually near the landing). Cap crossings to one:
					   once a poly has been reached via an off-mesh hop, it can
					   still be walked freely, but can't cross another. */
					char nextHop = hopUsed[u];
					if ((int)np >= ground)
					{
						unsigned char area = tile->polys[np].getArea();
						if (area != NAV_AREA_JUMP && area != NAV_AREA_DROP && area != NAV_AREA_DOOR)
							continue;
						if (hopUsed[u] >= 1) continue;
						nextHop = hopUsed[u] + 1;
					}
					seenPoly[np] = 1;
					hopUsed[np] = nextHop;
					frontier.push_back((int)np);
				}
			}
			std::vector<float> reachPts;
			for (int ii = 0; ii < ground; ii++)
			{
				if (!seenPoly[ii]) continue;
				reachPts.push_back(q[ii * 3 + 0]);
				reachPts.push_back(q[ii * 3 + 1]);
				reachPts.push_back(q[ii * 3 + 2]);
			}
			if (!has_value(reachPts.data(), (int)(reachPts.size() / 3), value_user))
				continue;
		}

		links.push_back(nav_make_link(ql, qh, AI_SUPER_JUMP, 0, 32.0f));
		seenLo.push_back(gacomp[lo] < gacomp[bestHi] ? gacomp[lo] : gacomp[bestHi]);
		seenHi.push_back(gacomp[lo] < gacomp[bestHi] ? gacomp[bestHi] : gacomp[lo]);
	}

	if (links.empty())
		return 0;
	nav_off_mesh_link_t *outp = (nav_off_mesh_link_t *)malloc(links.size() * sizeof(nav_off_mesh_link_t));
	if (outp == nullptr)
		return 0;
	memcpy(outp, links.data(), links.size() * sizeof(nav_off_mesh_link_t));
	*out_links = outp;
	fprintf(stderr, "Nav: computed %d rocket-jump links\n", (int)links.size());
	return (int)links.size();
}

int nav_mesh_compute_deep_drops(
	nav_mesh_runtime_t *navmesh,
	nav_jump_validate_fn validate, void *user,
	nav_off_mesh_link_t **out_links)
{
	/* Envelope: below 48 the walk/jump passes own the space; capped short
	   of a lethal fall (~800u kills).  Horizontal reach is physics, not a
	   fixed radius: during a dz fall a running bot covers up to
	   run_speed * sqrt(2*dz/800), so a candidate is feasible whenever the
	   required launch speed stays under a full run (with margin). */
	const float kDeepDropMin = 48.0f;
	/* Envelope only: the validator enforces the true lethal-fall cap (~700u)
	   on the DRY part of the fall, so a landing poly deep under water may sit
	   far below that.  This just bounds the candidate search. */
	const float kDeepDropMax = 1400.0f;
	const float kDeepDropMaxSpeed = 300.0f;
	/* Envelope slack for water landings: after splashdown the remaining
	   horizontal distance is swum, not flown -- the validator's entry-column
	   search enforces the real per-offset drift caps and the wet leg. */
	const float kDeepDropSwimSlack = 600.0f;

	*out_links = nullptr;
	if (navmesh == nullptr || navmesh->navmesh == nullptr
		|| navmesh->query == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	if (ground <= 0)
		return 0;

	/* Walk-adjacency components (off-mesh hops excluded on purpose --
	   the findPath gates below are what see those). */
	std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, true);
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	std::vector<int> gacomp(ground);
	std::vector<int> compsize(ground, 0);
	for (int i = 0; i < ground; i++) { gacomp[i] = gf(i); compsize[gacomp[i]]++; }
	int maincomp = 0;
	for (int i = 0; i < ground; i++)
		if (compsize[i] > compsize[maincomp]) maincomp = i;
	int mainrep = -1;
	for (int i = 0; i < ground; i++)
		if (gacomp[i] == maincomp && tile->polys[i].flags != 0) { mainrep = i; break; }

	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	const dtPolyRef base = mesh->getPolyRefBase(tile);
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	/* Components that contain a level exit are escapable by definition. */
	std::vector<char> compexit(ground, 0);
	for (int e = 0; e < nav_exit_point_count; e++)
	{
		float re[3], nearest[3];
		const float ext[3] = { 160.0f, 160.0f, 160.0f };
		dtPolyRef ref = 0;
		nav_quake_to_recast(nav_exit_points[e], re);
		navmesh->query->findNearestPoly(re, ext, &filter, &ref, nearest);
		if (ref != 0)
		{
			unsigned int s, t, np;
			mesh->decodePolyId(ref, s, t, np);
			if ((int)np < ground)
				compexit[gacomp[np]] = 1;
		}
	}

	std::vector<int> seenLo, seenHi;
	auto pair_seen = [&](int x, int y) {
		int lo = x < y ? x : y, hi = x < y ? y : x;
		for (size_t s = 0; s < seenLo.size(); s++)
			if (seenLo[s] == lo && seenHi[s] == hi) return true;
		return false;
	};

	/* Per-poly horizontal radius and z-range: the prefilter below works on
	   centroids, and a rim-adjacent pair of large polys can have centroids
	   hundreds of units apart -- slack by the radii so those still qualify,
	   then verify the exact geometry on the closest vertex pair. */
	std::vector<float> pr(ground, 0.0f), pzmin(ground, 0.0f), pzmax(ground, 0.0f);
	for (int i = 0; i < ground; i++)
	{
		const dtPoly *p = &tile->polys[i];
		const float *c = &q[i * 3];
		float r = 0.0f, zmin = 1e9f, zmax = -1e9f;
		for (int v = 0; v < p->vertCount; v++)
		{
			float vq[3];
			nav_recast_to_quake(&tile->verts[p->verts[v] * 3], vq);
			float dx = vq[0]-c[0], dy = vq[1]-c[1];
			float d = sqrtf(dx*dx + dy*dy);
			if (d > r) r = d;
			if (vq[2] < zmin) zmin = vq[2];
			if (vq[2] > zmax) zmax = vq[2];
		}
		pr[i] = r; pzmin[i] = zmin; pzmax[i] = zmax;
	}

	auto closest_verts = [&](int a, int b, float *qa, float *qb) {
		const dtPoly *pa = &tile->polys[a], *pb = &tile->polys[b];
		float best = 1e18f;
		for (int va = 0; va < pa->vertCount; va++)
		{
			float aq[3];
			nav_recast_to_quake(&tile->verts[pa->verts[va] * 3], aq);
			for (int vb = 0; vb < pb->vertCount; vb++)
			{
				float bq[3];
				nav_recast_to_quake(&tile->verts[pb->verts[vb] * 3], bq);
				float dx = aq[0]-bq[0], dy = aq[1]-bq[1], dz = aq[2]-bq[2];
				float d = dx*dx + dy*dy + dz*dz;
				if (d < best)
				{
					best = d;
					memcpy(qa, aq, sizeof(float) * 3);
					memcpy(qb, bq, sizeof(float) * 3);
				}
			}
		}
	};

	/* Per-component central poly: the poly nearest the component's mean
	   centroid.  Nearest-rim candidates can all sit tucked under overhangs
	   (e3m5's hole floor rims under its donut platform); the central poly
	   gives every nearby component one open-ground attempt too. */
	std::vector<double> csum(3 * (size_t)ground, 0.0);
	std::vector<int> ccnt(ground, 0);
	for (int i = 0; i < ground; i++)
	{
		if (tile->polys[i].flags == 0) continue;
		int c = gacomp[i];
		csum[c*3+0] += q[i*3+0]; csum[c*3+1] += q[i*3+1]; csum[c*3+2] += q[i*3+2];
		ccnt[c]++;
	}
	std::vector<int> comprep(ground, -1);
	std::vector<float> repd(ground, 1e18f);
	for (int i = 0; i < ground; i++)
	{
		if (tile->polys[i].flags == 0) continue;
		int c = gacomp[i];
		float mx = (float)(csum[c*3+0] / ccnt[c]), my = (float)(csum[c*3+1] / ccnt[c]);
		float dx = q[i*3+0] - mx, dy = q[i*3+1] - my;
		float d2 = dx*dx + dy*dy;
		if (d2 < repd[c]) { repd[c] = d2; comprep[c] = i; }
	}

	/* Temporary diagnostic: trace candidate rejection for hi polys inside an
	   env-supplied bbox ("xmin ymin xmax ymax"). */
	float dbg[4] = {0,0,0,0}; int dbg_on = 0;
	if (const char *db = getenv("NAV_DD_DEBUG"))
		dbg_on = (sscanf(db, "%f %f %f %f", &dbg[0], &dbg[1], &dbg[2], &dbg[3]) == 4);

	std::vector<nav_off_mesh_link_t> links;
	for (int hi = 0; hi < ground; hi++)
	{
		if (tile->polys[hi].flags == 0) continue;
		int dbg_hi = dbg_on && q[hi*3] >= dbg[0] && q[hi*3+1] >= dbg[1]
			&& q[hi*3] <= dbg[2] && q[hi*3+1] <= dbg[3];

		/* Candidate lower cross-patch polys, cheapest first; try several,
		   since the cheapest can fail validation while a slightly worse
		   one is physically clean. */
		struct cand { float cost; int lo; };
		auto mkcand = [&](int lo, cand *out) -> bool {
			if (tile->polys[lo].flags == 0) return false;
			if (gacomp[hi] == gacomp[lo]) return false;
			const float *qh = &q[hi * 3], *ql = &q[lo * 3];
			float dmax = pzmax[hi] - pzmin[lo];
			float dmin = pzmin[hi] - pzmax[lo];
			if (dmax <= kDeepDropMin || dmin > kDeepDropMax) return false;
			float dx = ql[0]-qh[0], dy = ql[1]-qh[1];
			float hd = sqrtf(dx*dx + dy*dy);
			float hd_min = hd - pr[hi] - pr[lo];
			if (hd_min < 8.0f) hd_min = 8.0f;
			float dz_cap = dmax < kDeepDropMax ? dmax : kDeepDropMax;
			if (hd_min > kDeepDropMaxSpeed * sqrtf(2.0f * dz_cap / 800.0f) + kDeepDropSwimSlack) return false;
			out->cost = hd + (qh[2] - ql[2]) * 0.25f;
			out->lo = lo;
			return true;
		};
		std::vector<cand> cands;
		for (int lo = 0; lo < ground; lo++)
		{
			cand c;
			if (mkcand(lo, &c))
				cands.push_back(c);
		}
		if (cands.empty()) continue;
		std::sort(cands.begin(), cands.end(),
			[](const cand &a, const cand &b) { return a.cost < b.cost; });

		/* One try per component: keep the cheapest poly per lo component,
		   and add that component's central poly as a second, open-ground
		   attempt (rim-nearest polys can all be tucked under overhangs). */
		{
			std::vector<cand> picks;
			std::vector<int> comps;
			for (size_t ci = 0; ci < cands.size(); ci++)
			{
				int cc = gacomp[cands[ci].lo];
				bool seen = false;
				for (size_t k = 0; k < comps.size(); k++)
					if (comps[k] == cc) { seen = true; break; }
				if (seen) continue;
				comps.push_back(cc);
				picks.push_back(cands[ci]);
				int rp = comprep[cc];
				cand rc;
				if (rp >= 0 && rp != cands[ci].lo && mkcand(rp, &rc))
					picks.push_back(rc);
			}
			cands.swap(picks);
			std::sort(cands.begin(), cands.end(),
				[](const cand &a, const cand &b) { return a.cost < b.cost; });
		}

		const int kTryMax = 12;
		for (size_t ci = 0; ci < cands.size() && (int)ci < kTryMax; ci++)
		{
			int lo = cands[ci].lo;
			const char *why = NULL;
			if (pair_seen(gacomp[hi], gacomp[lo]))
				why = "seen";

			/* Exact geometry on the closest rim pair, not the centroids. */
			float qh[3], ql[3];
			closest_verts(hi, lo, qh, ql);
			float ddz = qh[2] - ql[2];
			float dx = ql[0]-qh[0], dy = ql[1]-qh[1];
			float hd = sqrtf(dx*dx + dy*dy);
			if (hd < 8.0f) { hd = 8.0f; }
			if (!why && (ddz <= kDeepDropMin || ddz > kDeepDropMax))
				why = "dz";
			if (!why && hd > kDeepDropMaxSpeed * sqrtf(2.0f * ddz / 800.0f) + kDeepDropSwimSlack)
				why = "speed";
			if (!why && validate(qh, ql, user) != AI_DROP)
				why = "validate";
			if (why)
			{
				if (dbg_hi)
					fprintf(stderr, "DDDBG hi=%d (%.0f %.0f %.0f) lo=%d (%.0f %.0f %.0f) dz=%.0f hd=%.0f: %s\n",
						hi, qh[0], qh[1], qh[2], lo, ql[0], ql[1], ql[2], ddz, hd, why);
				continue;
			}

			float rh[3], rl[3];
			dtPolyRef path[256]; int pc = 0;
			nav_quake_to_recast(qh, rh);
			nav_quake_to_recast(ql, rl);

			/* New down-access only: if the low patch is already reachable from
			   here, another route exists and this link is just a shortcut. */
			dtStatus down = navmesh->query->findPath(base | (dtPolyRef)hi, base | (dtPolyRef)lo,
				rh, rl, &filter, path, &pc, 256);
			if (!dtStatusFailed(down) && !dtStatusDetail(down, DT_PARTIAL_RESULT) && pc > 0)
			{
				if (dbg_hi)
					fprintf(stderr, "DDDBG hi=%d lo=%d: already-connected\n", hi, lo);
				break;
			}

			/* No-trap gate: the landing must already path back OUT -- up to the
			   start, or to the main mesh -- before we offer a way in.  A pit
			   with no exit never gets a link (the cap-320 dm3 regression). */
			int escapes = (gacomp[lo] == maincomp) || compexit[gacomp[lo]];
			if (!escapes)
			{
				dtStatus up = navmesh->query->findPath(base | (dtPolyRef)lo, base | (dtPolyRef)hi,
					rl, rh, &filter, path, &pc, 256);
				escapes = (!dtStatusFailed(up) && !dtStatusDetail(up, DT_PARTIAL_RESULT) && pc > 0);
			}
			float stopq[3] = { 0, 0, 0 };
			int stop_ok = 0;
			if (!escapes && mainrep >= 0)
			{
				float rm[3];
				nav_quake_to_recast(&q[mainrep * 3], rm);
				dtStatus esc = navmesh->query->findPath(base | (dtPolyRef)lo, base | (dtPolyRef)mainrep,
					rl, rm, &filter, path, &pc, 256);
				escapes = (!dtStatusFailed(esc) && !dtStatusDetail(esc, DT_PARTIAL_RESULT) && pc > 0);
				if (!escapes && pc > 0)
				{
					unsigned int s, t, np;
					navmesh->navmesh->decodePolyId(path[pc - 1], s, t, np);
					if ((int)np < ground)
					{
						memcpy(stopq, &q[np * 3], sizeof(stopq));
						stop_ok = 1;
					}
				}
			}
			if (!escapes)
			{
				if (dbg_hi)
					fprintf(stderr, "DDDBG hi=%d lo=%d (%.0f %.0f %.0f): no-escape stop=(%.0f %.0f %.0f)%s\n",
						hi, lo, ql[0], ql[1], ql[2], stopq[0], stopq[1], stopq[2],
						stop_ok ? "" : " (n/a)");
				continue;
			}

			nav_off_mesh_link_t lk = nav_make_link(qh, ql, AI_DROP, 0, 32.0f);
			{
				float fall_time = sqrtf(2.0f * ddz / 800.0f);
				lk.required_speed = hd / fall_time;
				if (lk.required_speed < 10.0f) lk.required_speed = 10.0f;
			}
			links.push_back(lk);
			fprintf(stderr, "Nav: LINK DEEPDROP start=(%.0f %.0f %.0f) end=(%.0f %.0f %.0f) dz=%.0f spd=%.0f\n",
				qh[0], qh[1], qh[2], ql[0], ql[1], ql[2], ddz, lk.required_speed);
			seenLo.push_back(gacomp[hi] < gacomp[lo] ? gacomp[hi] : gacomp[lo]);
			seenHi.push_back(gacomp[hi] < gacomp[lo] ? gacomp[lo] : gacomp[hi]);
			break;
		}
	}

	if (links.empty())
		return 0;
	nav_off_mesh_link_t *outp = (nav_off_mesh_link_t *)malloc(links.size() * sizeof(nav_off_mesh_link_t));
	if (outp == nullptr)
		return 0;
	memcpy(outp, links.data(), links.size() * sizeof(nav_off_mesh_link_t));
	*out_links = outp;
	fprintf(stderr, "Nav: computed %d deep-drop links\n", (int)links.size());
	return (int)links.size();
}

int nav_mesh_compute_swim_links(
	nav_mesh_runtime_t *navmesh,
	nav_jump_validate_fn validate, void *user,
	nav_off_mesh_link_t **out_links)
{
	/* 3D reach cap: swimming is slow, and any submerged route longer than
	   this almost certainly has intermediate polys the straight segment
	   would rather hop through anyway.  Sized for bent routes too: a well
	   floor exits by rising ~500u through its own column before crossing,
	   and the validator walks that whole L. */
	const float kSwimMax = 800.0f;

	*out_links = nullptr;
	if (navmesh == nullptr || navmesh->navmesh == nullptr
		|| navmesh->query == nullptr || validate == nullptr)
		return 0;
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return 0;
	const int ground = tile->header->offMeshBase;
	if (ground <= 0)
		return 0;

	/* Walk-adjacency components, same as the deep-drop pass. */
	std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, true);
	auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
	std::vector<int> gacomp(ground);
	for (int i = 0; i < ground; i++) gacomp[i] = gf(i);

	std::vector<float> q;
	nav_collect_ground_centroids(tile, ground, q);

	const dtPolyRef base = mesh->getPolyRefBase(tile);
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	std::vector<int> seenLo, seenHi;
	auto pair_seen = [&](int x, int y) {
		int lo = x < y ? x : y, hi = x < y ? y : x;
		for (size_t s = 0; s < seenLo.size(); s++)
			if (seenLo[s] == lo && seenHi[s] == hi) return true;
		return false;
	};

	/* Centroid prefilter slack, as in the deep-drop pass. */
	std::vector<float> pr(ground, 0.0f), pzmin(ground, 0.0f), pzmax(ground, 0.0f);
	for (int i = 0; i < ground; i++)
	{
		const dtPoly *p = &tile->polys[i];
		const float *c = &q[i * 3];
		float r = 0.0f, zmin = 1e9f, zmax = -1e9f;
		for (int v = 0; v < p->vertCount; v++)
		{
			float vq[3];
			nav_recast_to_quake(&tile->verts[p->verts[v] * 3], vq);
			float dx = vq[0]-c[0], dy = vq[1]-c[1];
			float d = sqrtf(dx*dx + dy*dy);
			if (d > r) r = d;
			if (vq[2] < zmin) zmin = vq[2];
			if (vq[2] > zmax) zmax = vq[2];
		}
		pr[i] = r; pzmin[i] = zmin; pzmax[i] = zmax;
	}

	auto closest_verts = [&](int a, int b, float *qa, float *qb) {
		const dtPoly *pa = &tile->polys[a], *pb = &tile->polys[b];
		float best = 1e18f;
		for (int va = 0; va < pa->vertCount; va++)
		{
			float aq[3];
			nav_recast_to_quake(&tile->verts[pa->verts[va] * 3], aq);
			for (int vb = 0; vb < pb->vertCount; vb++)
			{
				float bq[3];
				nav_recast_to_quake(&tile->verts[pb->verts[vb] * 3], bq);
				float dx = aq[0]-bq[0], dy = aq[1]-bq[1], dz = aq[2]-bq[2];
				float d = dx*dx + dy*dy + dz*dz;
				if (d < best)
				{
					best = d;
					memcpy(qa, aq, sizeof(float) * 3);
					memcpy(qb, bq, sizeof(float) * 3);
				}
			}
		}
	};

	std::vector<nav_off_mesh_link_t> links;
	for (int i = 0; i < ground; i++)
	{
		if (tile->polys[i].flags == 0) continue;

		struct cand { float cost; int j; };
		std::vector<cand> cands;
		for (int j = 0; j < ground; j++)
		{
			if (j == i || tile->polys[j].flags == 0) continue;
			if (gacomp[i] == gacomp[j]) continue;
			const float *qi = &q[i * 3], *qj = &q[j * 3];
			float dx = qj[0]-qi[0], dy = qj[1]-qi[1];
			float hd_min = sqrtf(dx*dx + dy*dy) - pr[i] - pr[j];
			if (hd_min < 0.0f) hd_min = 0.0f;
			float dz_min = pzmin[i] - pzmax[j];
			if (pzmin[j] - pzmax[i] > dz_min) dz_min = pzmin[j] - pzmax[i];
			if (dz_min < 0.0f) dz_min = 0.0f;
			float d3 = sqrtf(hd_min*hd_min + dz_min*dz_min);
			if (d3 > kSwimMax) continue;
			cand c = { d3, j };
			cands.push_back(c);
		}
		if (cands.empty()) continue;
		std::sort(cands.begin(), cands.end(),
			[](const cand &a, const cand &b) { return a.cost < b.cost; });

		/* One try per target component: a fragmented neighbor (a chamber
		   floor split into slivers) must not eat every try slot while a
		   farther, genuinely distinct component starves (e3m5's hole floor
		   never got to try its water surface). */
		{
			std::vector<cand> picks;
			std::vector<int> comps;
			for (size_t ci = 0; ci < cands.size(); ci++)
			{
				int cc = gacomp[cands[ci].j];
				bool seen = false;
				for (size_t k = 0; k < comps.size(); k++)
					if (comps[k] == cc) { seen = true; break; }
				if (seen) continue;
				comps.push_back(cc);
				picks.push_back(cands[ci]);
			}
			cands.swap(picks);
		}

		/* Candidate endpoints on a poly: each vertex nudged 16u toward the
		   centroid (rim points hug walls and the validator sweeps the full
		   player hull), plus the centroid itself.  The one straight
		   hull-clear corridor can sit anywhere on the two polys (end's
		   shaft is barely hull-wide), so search pairs instead of trusting
		   the single closest-vertex pair. */
		auto gather_pts = [&](int poly, float pts[7][3]) {
			const dtPoly *p = &tile->polys[poly];
			const float *c = &q[poly * 3];
			int n = 0;
			for (int v = 0; v < p->vertCount && n < 6; v++, n++)
			{
				float vq[3];
				nav_recast_to_quake(&tile->verts[p->verts[v] * 3], vq);
				float d[3] = { c[0]-vq[0], c[1]-vq[1], c[2]-vq[2] };
				float len = sqrtf(d[0]*d[0] + d[1]*d[1] + d[2]*d[2]);
				float t = (len < 16.0f || len < 1.0f) ? 1.0f : 16.0f / len;
				pts[n][0] = vq[0] + d[0]*t;
				pts[n][1] = vq[1] + d[1]*t;
				pts[n][2] = vq[2] + d[2]*t;
			}
			pts[n][0] = c[0]; pts[n][1] = c[1]; pts[n][2] = c[2];
			return n + 1;
		};

		static float swdbg[4]; static int swdbg_on = -1;
		if (swdbg_on < 0)
		{
			const char *db = getenv("NAV_DD_DEBUG");
			swdbg_on = db && sscanf(db, "%f %f %f %f", &swdbg[0], &swdbg[1], &swdbg[2], &swdbg[3]) == 4;
		}
		int dbg_i = swdbg_on && q[i*3] >= swdbg[0] && q[i*3+1] >= swdbg[1]
			&& q[i*3] <= swdbg[2] && q[i*3+1] <= swdbg[3];

		const int kTryMax = 8;
		for (size_t ci = 0; ci < cands.size() && (int)ci < kTryMax; ci++)
		{
			int j = cands[ci].j;
			if (pair_seen(gacomp[i], gacomp[j])) continue;
			if (dbg_i)
				fprintf(stderr, "SWDBG i=%d (%.0f %.0f %.0f) j=%d (%.0f %.0f %.0f) d3=%.0f\n",
					i, q[i*3], q[i*3+1], q[i*3+2], j, q[j*3], q[j*3+1], q[j*3+2], cands[ci].cost);

			float pa[7][3], pb[7][3];
			int na = gather_pts(i, pa), nb = gather_pts(j, pb);
			struct ppair { float d; int a, b; };
			std::vector<ppair> pairs;
			for (int a = 0; a < na; a++)
				for (int b = 0; b < nb; b++)
				{
					float dx = pb[b][0]-pa[a][0], dy = pb[b][1]-pa[a][1], dz = pb[b][2]-pa[a][2];
					float d = sqrtf(dx*dx + dy*dy + dz*dz);
					if (d > kSwimMax) continue;
					ppair pp = { d, a, b };
					pairs.push_back(pp);
				}
			std::sort(pairs.begin(), pairs.end(),
				[](const ppair &x, const ppair &y) { return x.d < y.d; });

			/* Validator proves the whole segment is underwater and the
			   player hull can swim it. */
			float qa[3], qb[3];
			int found = 0;
			const int kPairTryMax = 16;
			for (size_t pi = 0; pi < pairs.size() && (int)pi < kPairTryMax; pi++)
			{
				if (validate(pa[pairs[pi].a], pb[pairs[pi].b], user) == AI_DROP)
				{
					memcpy(qa, pa[pairs[pi].a], sizeof(qa));
					memcpy(qb, pb[pairs[pi].b], sizeof(qb));
					found = 1;
					break;
				}
			}
			if (!found)
				continue;

			/* New access only: if a path already exists either way, this
			   would just be a shortcut inside connected water. */
			float ra[3], rb[3];
			dtPolyRef path[256]; int pc = 0;
			nav_quake_to_recast(qa, ra);
			nav_quake_to_recast(qb, rb);
			dtStatus fwd = navmesh->query->findPath(base | (dtPolyRef)i, base | (dtPolyRef)j,
				ra, rb, &filter, path, &pc, 256);
			if (!dtStatusFailed(fwd) && !dtStatusDetail(fwd, DT_PARTIAL_RESULT) && pc > 0)
			{
				if (dbg_i)
					fprintf(stderr, "SWDBG i=%d j=%d: validated but already connected\n", i, j);
				/* Keep trying other components: an already-linked neighbor
				   must not starve a farther, genuinely stranded one. */
				continue;
			}

			links.push_back(nav_make_link(qa, qb, AI_DROP, 1, 32.0f));
			fprintf(stderr, "Nav: LINK SWIM start=(%.0f %.0f %.0f) end=(%.0f %.0f %.0f)\n",
				qa[0], qa[1], qa[2], qb[0], qb[1], qb[2]);
			seenLo.push_back(gacomp[i] < gacomp[j] ? gacomp[i] : gacomp[j]);
			seenHi.push_back(gacomp[i] < gacomp[j] ? gacomp[j] : gacomp[i]);
			break;
		}
	}

	if (links.empty())
		return 0;
	nav_off_mesh_link_t *outp = (nav_off_mesh_link_t *)malloc(links.size() * sizeof(nav_off_mesh_link_t));
	if (outp == nullptr)
		return 0;
	memcpy(outp, links.data(), links.size() * sizeof(nav_off_mesh_link_t));
	*out_links = outp;
	fprintf(stderr, "Nav: computed %d swim links\n", (int)links.size());
	return (int)links.size();
}

int nav_mesh_gap_probe(
	const nav_mesh_runtime_t *navmesh,
	const float *starts, int start_count,
	const float *goal,
	float *out_from, float *out_to,
	char *error, size_t error_size)
{
	if (navmesh == nullptr || navmesh->navmesh == nullptr || navmesh->query == nullptr)
	{
		nav_set_error(error, error_size, "navmesh not built");
		return 0;
	}
	dtNavMesh *mesh = navmesh->navmesh;
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
	{
		nav_set_error(error, error_size, "no tile");
		return 0;
	}
	const int npolys = tile->header->polyCount;
	const int ground = tile->header->offMeshBase > 0 ? tile->header->offMeshBase : npolys;

	/* Temporary diagnostic: dump every ground-poly centroid once. */
	static int dumped = 0;
	if (!dumped && getenv("NAV_DUMP_POLYS"))
	{
		std::vector<float> qc;
		nav_collect_ground_centroids(tile, ground, qc);
		std::vector<int> ga = nav_mesh_ground_uf(tile, mesh, ground, true);
		auto gf = [&](int x) { while (ga[x] != x) { ga[x] = ga[ga[x]]; x = ga[x]; } return x; };
		for (int i = 0; i < ground; i++)
			fprintf(stderr, "POLYDUMP %d (%.0f %.0f %.0f) f=%d c=%d\n",
				i, qc[i*3], qc[i*3+1], qc[i*3+2], tile->polys[i].flags, gf(i));
		if (getenv("NAV_DUMP_POLY_VERTS"))
		{
			float bx0, by0, bx1, by1;
			if (sscanf(getenv("NAV_DUMP_POLY_VERTS"), "%f %f %f %f", &bx0, &by0, &bx1, &by1) == 4)
			{
				for (int i = 0; i < ground; i++)
				{
					if (qc[i*3] < bx0 || qc[i*3] > bx1 || qc[i*3+1] < by0 || qc[i*3+1] > by1)
						continue;
					const dtPoly *p = &tile->polys[i];
					fprintf(stderr, "POLYVERTS %d:", i);
					for (int vi = 0; vi < p->vertCount; vi++)
					{
						float q[3];
						nav_recast_to_quake(&tile->verts[p->verts[vi] * 3], q);
						fprintf(stderr, " (%.0f %.0f %.0f)", q[0], q[1], q[2]);
					}
					fprintf(stderr, "\n");
				}
			}
		}
		dumped = 1;
	}

	/* Directed BFS from every start poly over the FULL link graph
	   (walk adjacency + off-mesh connections) = everything a bot can
	   actually reach.  Undirected BFS from the goal poly = the goal's
	   island.  The closest ground-poly pair across the two sets is the
	   physical gap a new link would have to bridge. */
	std::vector<char> reach(npolys, 0), island(npolys, 0);
	std::vector<int> stack;

	auto resolve = [&](const float *qpos) -> int {
		nav_mesh_nearest_result_t nr;
		char nerr[64];
		if (!nav_mesh_find_nearest(navmesh, qpos, &nr, nerr, sizeof(nerr)) || !nr.found)
			return -1;
		unsigned int s, t, ip;
		mesh->decodePolyId((dtPolyRef)nr.poly_ref, s, t, ip);
		return (int)ip < npolys ? (int)ip : -1;
	};

	for (int s = 0; s < start_count; s++)
	{
		int sp = resolve(&starts[s * 3]);
		if (sp >= 0 && !reach[sp]) { reach[sp] = 1; stack.push_back(sp); }
	}
	while (!stack.empty())
	{
		int i = stack.back(); stack.pop_back();
		const dtPoly *p = &tile->polys[i];
		for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int sa, t, ip; mesh->decodePolyId(tile->links[k].ref, sa, t, ip);
			if ((int)ip < npolys && !reach[ip]) { reach[ip] = 1; stack.push_back((int)ip); }
		}
	}

	int gp = resolve(goal);
	if (gp < 0)
	{
		nav_set_error(error, error_size, "goal resolves to no poly");
		return 0;
	}
	if (reach[gp])
	{
		nav_set_error(error, error_size, "goal poly is reachable (stale query?)");
		return 0;
	}

	/* Reverse-directed BFS from the goal: every poly that can REACH the
	   goal.  Disjoint from the reachable set by construction (overlap
	   would mean the goal is reachable), so the closest pair across the
	   two sets is the true directed gap -- an undirected island bleeds
	   back through one-way links and degenerates to a zero-length "gap". */
	std::vector<std::vector<int>> rev(npolys);
	for (int i = 0; i < npolys; i++)
	{
		const dtPoly *p = &tile->polys[i];
		for (unsigned int k = p->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			if (tile->links[k].ref == 0) continue;
			unsigned int sa, t, ip; mesh->decodePolyId(tile->links[k].ref, sa, t, ip);
			if ((int)ip < npolys) rev[ip].push_back(i);
		}
	}
	island[gp] = 1; stack.push_back(gp);
	while (!stack.empty())
	{
		int i = stack.back(); stack.pop_back();
		for (size_t k = 0; k < rev[i].size(); k++)
		{
			int j = rev[i][k];
			if (!island[j]) { island[j] = 1; stack.push_back(j); }
		}
	}

	/* Island footprint helps tell a one-poly pocket from a whole wing. */
	{
		int nis = 0, nre = 0;
		float bb[6] = {1e9f,1e9f,1e9f,-1e9f,-1e9f,-1e9f};
		for (int i = 0; i < ground; i++)
		{
			if (reach[i]) nre++;
			if (!island[i]) continue;
			nis++;
			float c[3], qc[3];
			nav_mesh_poly_center(tile, &tile->polys[i], c);
			nav_recast_to_quake(c, qc);
			for (int a = 0; a < 3; a++)
			{
				if (qc[a] < bb[a]) bb[a] = qc[a];
				if (qc[a] > bb[a+3]) bb[a+3] = qc[a];
			}
		}
		fprintf(stderr, "Nav: gap-probe island: %d polys (reach %d) bbox (%.0f %.0f %.0f)-(%.0f %.0f %.0f)\n",
			nis, nre, bb[0], bb[1], bb[2], bb[3], bb[4], bb[5]);
	}

	/* Rank candidate pairs by center distance, then refine the best few
	   by closest VERTEX pair -- centers of large polys overstate the gap
	   badly (a 300u "gap" can be a 40u hole between two big floors). */
	struct cand { float d; int i, j; };
	std::vector<cand> top;
	const size_t kTop = 48;
	for (int i = 0; i < ground; i++)
	{
		if (!reach[i] || tile->polys[i].flags == 0) continue;
		float ci[3];
		nav_mesh_poly_center(tile, &tile->polys[i], ci);
		for (int j = 0; j < ground; j++)
		{
			if (!island[j] || tile->polys[j].flags == 0) continue;
			float cj[3];
			nav_mesh_poly_center(tile, &tile->polys[j], cj);
			float dx = ci[0]-cj[0], dy = ci[1]-cj[1], dz = ci[2]-cj[2];
			float d = dx*dx + dy*dy + dz*dz;
			if (top.size() < kTop || d < top.back().d)
			{
				cand c = { d, i, j };
				size_t at = top.size();
				top.push_back(c);
				while (at > 0 && top[at - 1].d > d)
				{
					top[at] = top[at - 1];
					at--;
				}
				top[at] = c;
				if (top.size() > kTop) top.pop_back();
			}
		}
	}
	if (top.empty())
	{
		nav_set_error(error, error_size, "no reachable/island ground poly pair");
		return 0;
	}
	float best = 1e18f;
	float bf[3] = {0,0,0}, bt[3] = {0,0,0};
	for (size_t c = 0; c < top.size(); c++)
	{
		const dtPoly *pi = &tile->polys[top[c].i];
		const dtPoly *pj = &tile->polys[top[c].j];
		for (int vi = 0; vi < pi->vertCount; vi++)
		{
			const float *a = &tile->verts[pi->verts[vi] * 3];
			for (int vj = 0; vj < pj->vertCount; vj++)
			{
				const float *b = &tile->verts[pj->verts[vj] * 3];
				float dx = a[0]-b[0], dy = a[1]-b[1], dz = a[2]-b[2];
				float d = dx*dx + dy*dy + dz*dz;
				if (d < best)
				{
					best = d;
					dtVcopy(bf, a);
					dtVcopy(bt, b);
				}
			}
		}
	}
	nav_recast_to_quake(bf, out_from);
	nav_recast_to_quake(bt, out_to);
	return 1;
}

/* Disable polys not connected to the largest mesh component.
   Hull-1 extraction emits sliver floors (lintels, beams) that bots
   reseed onto and then roam forever inside a 1-poly island.  Traversal
   follows Detour link chains, which include off-mesh connections, so
   ledges reachable only by jump/drop/teleport links stay enabled. */
static void nav_mesh_disable_islands(dtNavMesh *mesh)
{
	const dtMeshTile *tile = static_cast<const dtNavMesh *>(mesh)->getTile(0);
	if (tile == nullptr || tile->header == nullptr)
		return;

	const int npolys = tile->header->polyCount;
	const dtPolyRef base = mesh->getPolyRefBase(tile);

	/* Union-find over links, ignoring direction: a pocket whose only
	   connection is a one-way escape drop still reaches the main mesh,
	   so it is navigable — not junk. */
	std::vector<int> uf(npolys);
	for (int i = 0; i < npolys; i++)
		uf[i] = i;
	auto uf_find = [&](int x) {
		while (uf[x] != x)
		{
			uf[x] = uf[uf[x]];
			x = uf[x];
		}
		return x;
	};
	for (int i = 0; i < npolys; i++)
	{
		const dtPoly *poly = &tile->polys[i];
		for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
		{
			const dtPolyRef nref = tile->links[k].ref;
			if (nref == 0)
				continue;
			unsigned int salt, it, ip;
			mesh->decodePolyId(nref, salt, it, ip);
			if ((int)ip >= npolys)
				continue;
			uf[uf_find(i)] = uf_find((int)ip);
		}
	}
	std::vector<int> comp(npolys, -1);
	std::vector<int> comp_size;
	for (int i = 0; i < npolys; i++)
	{
		const int r = uf_find(i);
		if (comp[r] < 0)
		{
			comp[r] = (int)comp_size.size();
			comp_size.push_back(0);
		}
		comp[i] = comp[r];
		comp_size[comp[i]]++;
	}

	/* A junk sliver is a small unlinked component hovering just above
	   main-mesh floor (door lintels, beams — hull-1 artifacts).  Real
	   isolated areas (super-jump ledges, rooms awaiting plat/door links)
	   ARE the floor: nothing walkable sits under them, so keep those for
	   goal snapping and future links. */
	const int kSliverMaxPolys = 4;
	const float kSliverMaxHover = 48.0f;
	/* Hull-1 expansion makes slivers overhang the floor below by up to
	   the player half-width, so test containment with XZ slack.  Keep it
	   under ~half-width: at 26u+ of slack, real window ledges (dm3 wp77)
	   start matching neighboring floor and get wrongly culled. */
	const float kSliverXZSlack = 20.0f;

	int largest = 0;
	for (int c = 1; c < (int)comp_size.size(); c++)
		if (comp_size[c] > comp_size[largest])
			largest = c;

	/* Poly centroids + XZ bounds (recast coords: y is up). */
	std::vector<float> ctr(npolys * 3);
	std::vector<float> bb(npolys * 4); /* xmin xmax zmin zmax */
	for (int i = 0; i < npolys; i++)
	{
		const dtPoly *poly = &tile->polys[i];
		float cx = 0, cy = 0, cz = 0;
		float xmin = FLT_MAX, xmax = -FLT_MAX, zmin = FLT_MAX, zmax = -FLT_MAX;
		for (int vi = 0; vi < poly->vertCount; vi++)
		{
			const float *v = &tile->verts[poly->verts[vi] * 3];
			cx += v[0]; cy += v[1]; cz += v[2];
			if (v[0] < xmin) xmin = v[0];
			if (v[0] > xmax) xmax = v[0];
			if (v[2] < zmin) zmin = v[2];
			if (v[2] > zmax) zmax = v[2];
		}
		ctr[i * 3 + 0] = cx / poly->vertCount;
		ctr[i * 3 + 1] = cy / poly->vertCount;
		ctr[i * 3 + 2] = cz / poly->vertCount;
		bb[i * 4 + 0] = xmin; bb[i * 4 + 1] = xmax;
		bb[i * 4 + 2] = zmin; bb[i * 4 + 3] = zmax;
	}

	int disabled = 0;
	for (int c = 0; c < (int)comp_size.size(); c++)
	{
		if (c == largest || comp_size[c] > kSliverMaxPolys)
			continue;

		/* Every poly in the component must shadow main-mesh floor. */
		bool all_hover = true;
		for (int i = 0; i < npolys && all_hover; i++)
		{
			if (comp[i] != c)
				continue;
			bool floor_below = false;
			for (int j = 0; j < npolys; j++)
			{
				if (comp[j] != largest)
					continue;
				if (ctr[i * 3 + 0] < bb[j * 4 + 0] - kSliverXZSlack ||
				    ctr[i * 3 + 0] > bb[j * 4 + 1] + kSliverXZSlack ||
				    ctr[i * 3 + 2] < bb[j * 4 + 2] - kSliverXZSlack ||
				    ctr[i * 3 + 2] > bb[j * 4 + 3] + kSliverXZSlack)
					continue;
				const float drop = ctr[i * 3 + 1] - ctr[j * 3 + 1];
				if (drop > 0.0f && drop <= kSliverMaxHover)
				{
					floor_below = true;
					break;
				}
			}
			if (!floor_below)
				all_hover = false;
		}
		if (!all_hover)
			continue;

		for (int i = 0; i < npolys; i++)
		{
			if (comp[i] != c)
				continue;
			mesh->setPolyFlags(base | (dtPolyRef)i, 0);
			disabled++;
			if (getenv("NAV_ISLAND_DEBUG") != NULL)
			{
				float qc[3], rc[3] = { ctr[i * 3], ctr[i * 3 + 1], ctr[i * 3 + 2] };
				nav_recast_to_quake(rc, qc);
				fprintf(stderr, "Nav: sliver poly %d comp=%d size=%d at (%.0f %.0f %.0f)\n",
					i, c, comp_size[c], qc[0], qc[1], qc[2]);
			}
		}
	}

	if (disabled > 0)
		fprintf(stderr, "Nav: disabled %d sliver polys hovering over main mesh (%d components, largest=%d)\n",
			disabled, (int)comp_size.size(), comp_size[largest]);

	/* DIAGNOSTIC (no behaviour change): catalogue orphan components that
	   survived the conservative cull -- these are reachable-in-game areas the
	   navmesh failed to LINK.  Print size + centroid so we can find the missing
	   drop/jump/etc. link for each.  Skip 1-2 poly slivers (junk noise). */
	for (int c = 0; c < (int)comp_size.size(); c++)
	{
		if (c == largest || comp_size[c] < 3)
			continue;
		double cx = 0, cy = 0, cz = 0;
		int n = 0;
		for (int i = 0; i < npolys; i++)
		{
			if (comp[i] != c)
				continue;
			cx += ctr[i * 3 + 0];
			cy += ctr[i * 3 + 1];
			cz += ctr[i * 3 + 2];
			n++;
		}
		if (n > 0)
			fprintf(stderr, "Nav: ORPHAN comp size=%d at quake (%.0f %.0f %.0f)\n",
				comp_size[c], cx / n, cz / n, cy / n);
	}
}

extern "C" nav_mesh_runtime_t *nav_mesh_build(
	const float *verts,
	int vertex_count,
	const int *tris,
	int triangle_count,
	const unsigned char *tri_hazard,
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

	/* Slime ground (never lava -- lava is excluded outright at mesh-build
	   time in nav_hull.cpp, since it's an effectively-instant kill and must
	   never be a routable option) reads walkable off the clip hull (Quake's
	   slime is walk-through-with-damage, not solid) but should still be the
	   pathfinder's last resort: tag it a distinct area instead of
	   RC_WALKABLE_AREA so nav_mesh_setup_filter's steep NAV_AREA_HAZARD
	   cost steers a bot around it whenever a dry route exists, while
	   leaving it in the mesh (and reachable) when it's the only route
	   in.  Only downgrades triangles rcMarkWalkableTriangles already
	   called walkable -- never turns a too-steep face into hazard. */
	if (tri_hazard != nullptr)
	{
		for (i = 0; i < triangle_count; ++i)
			if (tri_hazard[i] && areas[i] == RC_WALKABLE_AREA)
				areas[i] = NAV_AREA_HAZARD;
	}

	if (!rcRasterizeTriangles(&ctx, recast_verts.data(), vertex_count, tris, areas.data(), triangle_count, *guard.solid, rc_config.walkableClimb))
	{
		nav_set_error(error, error_size, "Failed to rasterize triangles into heightfield");
		return nullptr;
	}

	/* Merge spans separated by a sub-walkable air gap: nothing can stand
	   in a gap shorter than walkableHeight, so the two solids are one
	   surface whose top is the UPPER span (hull-1 bevel expansion emits
	   thin shelves a few units above real floors; the low-height filter
	   then kills the real floor and the shelf becomes an unlinked island
	   — dm4 GL alcove).  Walkability follows the surviving top surface.
	   Unlinked spans go back on no freelist: the heightfield pool frees
	   them wholesale. */
	{
		const int w = guard.solid->width, h = guard.solid->height;

		for (int ci = 0; ci < w * h; ci++)
		{
			for (rcSpan *s = guard.solid->spans[ci]; s != nullptr && s->next != nullptr; )
			{
				if ((int)s->next->smin - (int)s->smax < rc_config.walkableHeight)
				{
					rcSpan *up = s->next;
					/* Walkability follows the surviving top surface, but a
					   thin unwalkable shelf (sloped bevel face) within climb
					   of a walkable floor is standable — same promotion the
					   low-hanging filter would have applied pre-merge. */
					if ((int)up->smax - (int)s->smax > rc_config.walkableClimb || up->area > s->area)
						s->area = up->area;
					s->smax = up->smax;
					s->next = up->next;

				}
				else
					s = s->next;
			}
		}

	}

	rcFilterLowHangingWalkableObstacles(&ctx, rc_config.walkableClimb, *guard.solid);
	/* Stock Recast pipeline (experiment/stock-recast):
	   no gap-bridging, no custom ledge filter, no wall-only erosion. */
	rcFilterLedgeSpans(&ctx, rc_config.walkableHeight, rc_config.walkableClimb, *guard.solid);
	rcFilterWalkableLowHeightSpans(&ctx, rc_config.walkableHeight, *guard.solid);

#if 0
	/* Custom ledge filter: only remove a span if it has NO walkable
	   neighbor at a similar height. The standard rcFilterLedgeSpans
	   removes a span if ANY neighbor is a ledge, which kills narrow
	   walkways along walls. Our version keeps walkway spans that have
	   at least one same-height neighbor. */
	{
		const int xSize = guard.solid->width;
		const int zSize = guard.solid->height;
		int filtered = 0, preserved = 0;
		for (int z = 0; z < zSize; ++z)
		{
			for (int x = 0; x < xSize; ++x)
			{
				for (rcSpan *span = guard.solid->spans[x + z * xSize]; span; span = span->next)
				{
					if (span->area == RC_NULL_AREA)
						continue;

					const int bot = (int)span->smax;
					int has_ledge = 0;
					int support_count = 0;
					int access_min = bot;
					int access_max = bot;

					for (int dir = 0; dir < 4; ++dir)
					{
						int nx = x + rcGetDirOffsetX(dir);
						int nz = z + rcGetDirOffsetY(dir);
						if (nx < 0 || nz < 0 || nx >= xSize || nz >= zSize)
						{
							has_ledge = 1;
							continue;
						}

						/* Find best neighbor span at similar height */
						int neighbor_ok = 0;
						for (const rcSpan *ns = guard.solid->spans[nx + nz * xSize]; ns; ns = ns->next)
						{
							int nbot = (int)ns->smax;
							int top = span->next ? (int)span->next->smin : 0xffff;
							int ntop = ns->next ? (int)ns->next->smin : 0xffff;
							if (rcMin(top, ntop) - rcMax(bot, nbot) > rc_config.walkableHeight)
							{
								if (rcAbs(nbot - bot) <= rc_config.walkableClimb)
								{
									neighbor_ok = 1;
									if (nbot < access_min) access_min = nbot;
									if (nbot > access_max) access_max = nbot;
								}
							}
						}

						if (neighbor_ok)
							support_count++;
						else
							has_ledge = 1;
					}

					/* Steep slope: accessible neighbors span too large a height range */
					int steep = (access_max - access_min) > rc_config.walkableClimb;

					/* Remove if:
					   - No walkable support (isolated ledge), OR
					   - On a steep slope (standard Recast check)
					   Keep if: has ledge on some sides but also has 2+
					   walkable supports and isn't on a steep slope. */
					if (steep || (has_ledge && support_count < 2))
					{
						span->area = RC_NULL_AREA;
						filtered++;
					}
					else if (has_ledge)
						preserved++;
				}
			}
		}
		fprintf(stderr, "Nav: ledge filter: %d removed, %d preserved (walkway spans)\n",
			filtered, preserved);
	}
	rcFilterWalkableLowHeightSpans(&ctx, rc_config.walkableHeight, *guard.solid);
#endif /* custom ledge filter disabled — using standard + BFS restore */

	/* NAV_DUMP_SPANS="x y" (Quake coords): print the filtered heightfield
	   column there plus its 8 neighbors, to tell which filter ate a floor. */
	if (const char *spanenv = getenv("NAV_DUMP_SPANS"))
	{
		float qx, qy;
		if (sscanf(spanenv, "%f %f", &qx, &qy) == 2)
		{
			int cx = (int)((qx - rc_config.bmin[0]) / rc_config.cs);
			int cz = (int)((qy - rc_config.bmin[2]) / rc_config.cs);
			for (int dz = -1; dz <= 1; dz++)
				for (int dx = -1; dx <= 1; dx++)
				{
					int x = cx + dx, z = cz + dz;
					if (x < 0 || z < 0 || x >= guard.solid->width || z >= guard.solid->height) continue;
					for (rcSpan *s = guard.solid->spans[x + z * guard.solid->width]; s; s = s->next)
						fprintf(stderr, "Nav: SPANDUMP cell(%d %d) q(%.0f %.0f) z=%.0f..%.0f area=%d\n",
							x, z, rc_config.bmin[0] + x * rc_config.cs, rc_config.bmin[2] + z * rc_config.cs,
							rc_config.bmin[1] + s->smin * rc_config.ch,
							rc_config.bmin[1] + s->smax * rc_config.ch, (int)s->area);
				}
		}
	}

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
	/* Stock erosion. With hull-1 geometry the agent is a point
	   (walkable_radius 0) and erosion is skipped entirely — the clip
	   hull already encodes player clearance. */
	if (rc_config.walkableRadius > 0 &&
		!rcErodeWalkableArea(&ctx, rc_config.walkableRadius, *guard.compact))
	{
		nav_set_error(error, error_size, "Failed to erode walkable area");
		return nullptr;
	}
#if 0
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

		/* Build span→position lookup table (avoids O(w*h) scan per span) */
		std::vector<unsigned short> span_x(static_cast<size_t>(span_count));
		std::vector<unsigned short> span_y_pos(static_cast<size_t>(span_count));
		for (int y = 0; y < h; ++y)
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					span_x[si] = (unsigned short)x;
					span_y_pos[si] = (unsigned short)y;
				}
			}

		/* Erode-then-restore: full erosion for wall clearance, then
		   selectively un-erode the minimum cells needed to maintain
		   connectivity between disconnected walkable regions.

		   Step 1: Erode all near-wall cells unconditionally. */
		int eroded = 0;
		std::vector<unsigned char> was_eroded(static_cast<size_t>(span_count), 0);
		for (int si = 0; si < span_count; ++si)
		{
			if (guard.compact->areas[si] != RC_NULL_AREA && wd[si] < erode_dist)
			{
				was_eroded[si] = guard.compact->areas[si]; /* save original area */
				guard.compact->areas[si] = RC_NULL_AREA;
				eroded++;
			}
		}

		/* Step 2: Flood-fill to find connected walkable regions. */
		std::vector<int> region_id(static_cast<size_t>(span_count), -1);
		int num_regions = 0;
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					if (guard.compact->areas[si] == RC_NULL_AREA || region_id[si] >= 0)
						continue;
					/* BFS flood fill */
					int rid = num_regions++;
					std::vector<int> queue;
					queue.push_back(si);
					region_id[si] = rid;
					for (size_t qi = 0; qi < queue.size(); ++qi)
					{
						int cur = queue[qi];
						int cx = span_x[cur], cy = span_y_pos[cur];
						{
							const rcCompactSpan &cs = guard.compact->spans[cur];
							for (int dir = 0; dir < 4; ++dir)
							{
								if (rcGetCon(cs, dir) == RC_NOT_CONNECTED)
									continue;
								const int ax = cx + rcGetDirOffsetX(dir);
								const int ay = cy + rcGetDirOffsetY(dir);
								const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(cs, dir);
								if (guard.compact->areas[ai] != RC_NULL_AREA && region_id[ai] < 0)
								{
									region_id[ai] = rid;
									queue.push_back(ai);
								}
							}
						}
					}
				}
			}
		}

		/* Step 3: BFS from all regions simultaneously into eroded cells.
		   Each eroded cell gets labeled with the nearest region.
		   When two expanding fronts from different regions meet at an
		   eroded cell, that cell is a bridge. Restore only bridge cells
		   and the minimum path between them. */
		std::vector<int> eroded_nearest_region(static_cast<size_t>(span_count), -1);
		std::vector<int> eroded_dist(static_cast<size_t>(span_count), 0x7fffffff);

		/* Seed BFS from all region border cells */
		std::vector<int> bfs_queue;
		for (int y = 0; y < h; ++y)
		{
			for (int x = 0; x < w; ++x)
			{
				const rcCompactCell &c = guard.compact->cells[x + y * w];
				for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
				{
					if (guard.compact->areas[si] == RC_NULL_AREA || region_id[si] < 0)
						continue;
					/* Check if this walkable cell borders an eroded cell */
					const rcCompactSpan &s = guard.compact->spans[si];
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) == RC_NOT_CONNECTED)
							continue;
						const int ax = x + rcGetDirOffsetX(dir);
						const int ay = y + rcGetDirOffsetY(dir);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, dir);
						if (was_eroded[ai] && guard.compact->areas[ai] == RC_NULL_AREA)
						{
							if (eroded_dist[ai] > 1)
							{
								eroded_nearest_region[ai] = region_id[si];
								eroded_dist[ai] = 1;
								bfs_queue.push_back(ai);
							}
						}
					}
				}
			}
		}

		/* BFS expansion through eroded cells */
		int restored = 0;
		std::vector<int> bridge_cells;
		for (size_t qi = 0; qi < bfs_queue.size(); ++qi)
		{
			int si = bfs_queue[qi];
			int my_dist = eroded_dist[si];
			/* No distance cap — bridge any gap, however wide */

			/* Find this cell's position */
			int cx = span_x[si], cy = span_y_pos[si];
			{
				const rcCompactSpan &s = guard.compact->spans[si];
				for (int dir = 0; dir < 4; ++dir)
				{
					if (rcGetCon(s, dir) == RC_NOT_CONNECTED)
						continue;
					const int ax = cx + rcGetDirOffsetX(dir);
					const int ay = cy + rcGetDirOffsetY(dir);
					const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, dir);

					/* Traverse through any non-walkable cell — both eroded
					   cells and cells that were never walkable (decorative
					   holes). We can restore eroded cells; for never-walkable
					   cells we just pass through to find bridges beyond. */
					if (guard.compact->areas[ai] != RC_NULL_AREA)
						continue; /* already walkable — in a region */

					if (eroded_nearest_region[ai] >= 0 &&
						eroded_nearest_region[ai] != eroded_nearest_region[si])
					{
						/* Two different region fronts meet — this is a bridge! */
						bridge_cells.push_back(si);
						bridge_cells.push_back(ai);
					}
					else if (eroded_dist[ai] > my_dist + 1)
					{
						eroded_nearest_region[ai] = eroded_nearest_region[si];
						eroded_dist[ai] = my_dist + 1;
						bfs_queue.push_back(ai);
					}
				}
			}
		}

		/* Restore bridge cells and trace back to regions */
		for (size_t bi = 0; bi < bridge_cells.size(); ++bi)
		{
			int si = bridge_cells[bi];
			if (guard.compact->areas[si] == RC_NULL_AREA)
			{
				guard.compact->areas[si] = was_eroded[si] ? was_eroded[si] : RC_WALKABLE_AREA;
				restored++;
			}
		}

		/* Also restore the BFS path from each bridge cell back to its region.
		   Walk backward through eroded_dist to find the shortest path. */
		for (size_t bi = 0; bi < bridge_cells.size(); ++bi)
		{
			int si = bridge_cells[bi];
			/* Trace back toward the region by following decreasing eroded_dist */
			int cur = si;
			for (int step = 0; step < 100; ++step) /* trace back up to 100 cells */
			{
				if (eroded_dist[cur] <= 0)
					break;
				if (guard.compact->areas[cur] == RC_NULL_AREA)
				{
					guard.compact->areas[cur] = was_eroded[cur] ? was_eroded[cur] : RC_WALKABLE_AREA;
					restored++;
				}
				/* Find neighbor with lower distance */
				{
					int cx2 = span_x[cur], cy2 = span_y_pos[cur];
					const rcCompactSpan &s = guard.compact->spans[cur];
					int best_dist = eroded_dist[cur];
					int best_ni = -1;
					for (int dir = 0; dir < 4; ++dir)
					{
						if (rcGetCon(s, dir) == RC_NOT_CONNECTED)
							continue;
						const int ax = cx2 + rcGetDirOffsetX(dir);
						const int ay = cy2 + rcGetDirOffsetY(dir);
						const int ai = (int)guard.compact->cells[ax + ay * w].index + rcGetCon(s, dir);
						if (eroded_dist[ai] < best_dist)
						{
							best_dist = eroded_dist[ai];
							best_ni = ai;
						}
					}
					if (best_ni < 0) break;
					cur = best_ni;
				}
			}
		}

		fprintf(stderr, "Nav: wall erosion: %d eroded, %d bridges found, %d restored → %d regions\n",
			eroded, (int)bridge_cells.size(), restored, num_regions);
	}
#endif /* custom wall-only erosion disabled — stock rcErodeWalkableArea above */
	/* Stock erosion above changed walkable spans, so build the distance
	   field. Regions (watershed) and near-wall costs both consume
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
	{
		int repaired = nav_mesh_repair_broken_regions(guard.compact);
		if (repaired > 0)
			fprintf(stderr, "Nav: repaired %d overlapping/split watershed regions\n", repaired);
	}

	/* NAV_DUMP_SPANS second stage: compact spans (area/region) post-region
	   build at the same column, to separate region loss from contour loss. */
	if (const char *spanenv2 = getenv("NAV_DUMP_SPANS"))
	{
		float qx, qy;
		int rad = 1;
		if (sscanf(spanenv2, "%f %f %d", &qx, &qy, &rad) >= 2)
		{
			int cx = (int)((qx - rc_config.bmin[0]) / rc_config.cs);
			int cz = (int)((qy - rc_config.bmin[2]) / rc_config.cs);
			for (int dz = -rad; dz <= rad; dz++)
				for (int dx = -rad; dx <= rad; dx++)
				{
					int x = cx + dx, z = cz + dz;
					if (x < 0 || z < 0 || x >= guard.compact->width || z >= guard.compact->height) continue;
					const rcCompactCell &c = guard.compact->cells[x + z * guard.compact->width];
					for (int si = (int)c.index, sn = (int)(c.index + c.count); si < sn; ++si)
						fprintf(stderr, "Nav: CSPANDUMP cell(%d %d) y=%d area=%d reg=%d dist=%d\n",
							x, z, (int)guard.compact->spans[si].y, (int)guard.compact->areas[si],
							(int)guard.compact->spans[si].reg, (int)guard.compact->dist[si]);
				}
		}
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

	/* NAV_DUMP_SPANS third stage: contours, to separate contour loss from
	   polymesh loss.  Prints every contour's region + vert count + bbox. */
	if (getenv("NAV_DUMP_SPANS") != NULL)
	{
		for (int ci = 0; ci < guard.contours->nconts; ci++)
		{
			const rcContour &cont = guard.contours->conts[ci];
			float xmin = FLT_MAX, xmax = -FLT_MAX, zmin = FLT_MAX, zmax = -FLT_MAX;
			for (int vi = 0; vi < cont.nverts; vi++)
			{
				float wx = rc_config.bmin[0] + cont.verts[vi * 4 + 0] * rc_config.cs;
				float wz = rc_config.bmin[2] + cont.verts[vi * 4 + 2] * rc_config.cs;
				if (wx < xmin) xmin = wx;
				if (wx > xmax) xmax = wx;
				if (wz < zmin) zmin = wz;
				if (wz > zmax) zmax = wz;
			}
			fprintf(stderr, "Nav: CONTDUMP %d reg=%d nverts=%d q(%.0f %.0f)-(%.0f %.0f)\n",
				ci, (int)cont.reg, cont.nverts, xmin, zmin, xmax, zmax);
			float qx, qy;
			if (sscanf(getenv("NAV_DUMP_SPANS"), "%f %f", &qx, &qy) == 2
				&& qx >= xmin && qx <= xmax && qy >= zmin && qy <= zmax)
			{
				for (int vi = 0; vi < cont.nverts; vi++)
					fprintf(stderr, "Nav: CONTVERT %d %d (%.0f %.0f %.0f) r=%d\n",
						ci, vi,
						rc_config.bmin[0] + cont.verts[vi * 4 + 0] * rc_config.cs,
						rc_config.bmin[2] + cont.verts[vi * 4 + 2] * rc_config.cs,
						rc_config.bmin[1] + cont.verts[vi * 4 + 1] * rc_config.ch,
						cont.verts[vi * 4 + 3] & RC_CONTOUR_REG_MASK);
				for (int vi = 0; vi < cont.nrverts; vi++)
					fprintf(stderr, "Nav: CONTRAW %d %d (%.0f %.0f %.0f)\n",
						ci, vi,
						rc_config.bmin[0] + cont.rverts[vi * 4 + 0] * rc_config.cs,
						rc_config.bmin[2] + cont.rverts[vi * 4 + 2] * rc_config.cs,
						rc_config.bmin[1] + cont.rverts[vi * 4 + 1] * rc_config.ch);
			}
		}
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

				/* Contour winding doesn't reliably give the outward side
				   (outer vs hole contours wind oppositely), so verify
				   against the heightfield: the region interior always has
				   walkable floor at edge height, the region-0 outside never
				   does.  If the normal side has such a floor, flip. */
				if (len > 0.001f && guard.compact != nullptr)
				{
					const float px = (ax + bx) * 0.5f + edge.normal[0] * 6.0f;
					const float pz = (az + bz) * 0.5f + edge.normal[1] * 6.0f;
					const float py = (ay + by) * 0.5f;
					const int gx = (int)((px - rc_config.bmin[0]) / rc_config.cs);
					const int gz = (int)((pz - rc_config.bmin[2]) / rc_config.cs);
					if (gx >= 0 && gx < guard.compact->width && gz >= 0 && gz < guard.compact->height)
					{
						const float climb = rc_config.walkableClimb * rc_config.ch;
						const rcCompactCell &cc = guard.compact->cells[gx + gz * guard.compact->width];
						for (unsigned int si = cc.index, sn = cc.index + cc.count; si < sn; si++)
						{
							if (guard.compact->areas[si] == RC_NULL_AREA)
								continue;
							const float sy = rc_config.bmin[1] + guard.compact->spans[si].y * rc_config.ch;
							if (fabsf(sy - py) <= climb)
							{
								edge.normal[0] = -edge.normal[0];
								edge.normal[1] = -edge.normal[1];
								break;
							}
						}
					}
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
	/* Query-time vertical tolerance, NOT the raster walkable height.
	   This controls the Y search extent when linking off-mesh connections
	   to nearby ground polys. Off-mesh endpoints are authored from entity
	   origins, not guaranteed foot-contact points, so the search must cover
	   origin-to-surface separation even when the navmesh itself is on the floor. */
	params.walkableClimb = NAV_MESH_QUERY_CLIMB;
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
				/* RJ links carry an extra flag so a bot without launcher/
				   rockets/health excludes them from its path (it routes the
				   normal way; the gate guarantees one exists). */
				omc_flags.push_back(links[li].link_type == AI_SUPER_JUMP
					? (unsigned short)(kPolyFlagWalk | NAV_POLYFLAG_RJ)
					: kPolyFlagWalk);
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

	/* 2048 nodes was too small once off-mesh links multiplied: A* can
	   exhaust the pool exploring cheap shortcut fans and report a
	   reachable goal as PARTIAL|OUT_OF_NODES (seen on e2m3). */
	status = guard.runtime->query->init(guard.runtime->navmesh, 16384);
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
				{
					unlinked++;
					const dtOffMeshConnection *con = &tile->offMeshCons[oi];
					float s[3], e[3];
					nav_recast_to_quake(&con->pos[0], s);
					nav_recast_to_quake(&con->pos[3], e);
					fprintf(stderr, "Nav: UNLINKED off-mesh (%.0f %.0f %.0f)->(%.0f %.0f %.0f) rad=%.0f\n",
						s[0], s[1], s[2], e[0], e[1], e[2], con->rad);
				}
				else
				{
					linked++;
					if (getenv("NAV_DUMP_LINKS") != NULL)
					{
						const dtOffMeshConnection *con = &tile->offMeshCons[oi];
						float s[3], e[3], vs[3], ve[3];
						const float *v0 = &tile->verts[p->verts[0] * 3];
						const float *v1 = &tile->verts[p->verts[1] * 3];
						nav_recast_to_quake(&con->pos[0], s);
						nav_recast_to_quake(&con->pos[3], e);
						nav_recast_to_quake(v0, vs);
						nav_recast_to_quake(v1, ve);
						fprintf(stderr, "Nav: OMLINK id=%u orig (%.0f %.0f %.0f)->(%.0f %.0f %.0f) snap (%.0f %.0f %.0f)->(%.0f %.0f %.0f) rad=%.0f",
							con->userId, s[0], s[1], s[2], e[0], e[1], e[2],
							vs[0], vs[1], vs[2], ve[0], ve[1], ve[2], con->rad);
						for (unsigned int lk = p->firstLink; lk != DT_NULL_LINK; lk = tile->links[lk].next)
						{
							unsigned int ls, lt, lp;
							if (tile->links[lk].ref == 0) continue;
							nm->decodePolyId(tile->links[lk].ref, ls, lt, lp);
							fprintf(stderr, " ->%u", lp);
						}
						fprintf(stderr, "\n");
					}
				}
			}
			fprintf(stderr, "Nav: Detour stored %d/%d off-mesh, linked=%d unlinked=%d\n",
				tile->header->offMeshConCount,
				off_mesh_link_count + callback_link_count,
				linked, unlinked);
			if (const char *pd = getenv("NAV_DUMP_POLYS_BOX"))
			{
				float bx0, by0, bx1, by1;
				if (sscanf(pd, "%f %f %f %f", &bx0, &by0, &bx1, &by1) == 4)
				{
					const int ground = tile->header->offMeshBase;
					std::vector<float> qc;
					nav_collect_ground_centroids(tile, ground, qc);
					for (int i = 0; i < ground; i++)
					{
						if (qc[i*3] < bx0 || qc[i*3] > bx1 || qc[i*3+1] < by0 || qc[i*3+1] > by1)
							continue;
						fprintf(stderr, "Nav: PBOX %d (%.0f %.0f %.0f) flags=%d nverts=%d",
							i, qc[i*3], qc[i*3+1], qc[i*3+2], tile->polys[i].flags,
							tile->polys[i].vertCount);
						for (int vi = 0; vi < tile->polys[i].vertCount; vi++)
						{
							float vq[3];
							nav_recast_to_quake(&tile->verts[tile->polys[i].verts[vi] * 3], vq);
							fprintf(stderr, " [%.0f %.0f %.0f]", vq[0], vq[1], vq[2]);
						}
						fprintf(stderr, "\n");
					}
				}
			}
		}
	}

	nav_mesh_disable_islands(guard.runtime->navmesh);

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
	guard.runtime->query_half_extents_actor_origin[1] = NAV_MESH_QUERY_CLIMB;
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

/* Every enabled poly overlapping the box, as closest-points to the query
   point, nearest first.  No snap-distance cap: callers doing physical
   reach tests (jump-grab pickups) supply their own geometry limits. */
extern "C" int nav_mesh_query_poly_points(
	const nav_mesh_runtime_t *navmesh,
	const float *point,
	const float *half_extents,
	float (*out_points)[3],
	int max_points)
{
	float rc_point[3], rc_extents[3];
	dtPolyRef polys[32];
	int npolys = 0;
	dtQueryFilter filter; nav_mesh_setup_filter(&filter);
	struct cand_t { float d; float pt[3]; };
	cand_t cands[32];
	int n, i, j;

	if (navmesh == nullptr || navmesh->query == nullptr || max_points <= 0)
		return 0;
	nav_quake_to_recast(point, rc_point);
	nav_quake_to_recast(half_extents, rc_extents);
	if (dtStatusFailed(navmesh->query->queryPolygons(rc_point, rc_extents, &filter, polys, &npolys, 32)))
		return 0;
	n = 0;
	for (i = 0; i < npolys; i++)
	{
		float pt[3];
		bool over = false;
		if (dtStatusFailed(navmesh->query->closestPointOnPoly(polys[i], rc_point, pt, &over)))
			continue;
		cands[n].d = dtVdistSqr(rc_point, pt);
		dtVcopy(cands[n].pt, pt);
		n++;
	}
	for (i = 1; i < n; i++)
	{
		cand_t key = cands[i];
		for (j = i - 1; j >= 0 && cands[j].d > key.d; j--)
			cands[j + 1] = cands[j];
		cands[j + 1] = key;
	}
	if (n > max_points)
		n = max_points;
	for (i = 0; i < n; i++)
		nav_recast_to_quake(cands[i].pt, out_points[i]);
	return n;
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
			/* Out-of-nodes is a search-budget failure, not proof of
			   unreachability -- flag it so connectivity checks don't
			   chase phantom mesh gaps. */
			const char *oon = dtStatusDetail(status, DT_OUT_OF_NODES)
				? ", out of nodes" : "";
			/* Report WHERE the search dead-ended, in quake coords -- the
			   gap between here and the goal is the thing to go look at. */
			const dtMeshTile *stop_tile = nullptr;
			const dtPoly *stop_poly = nullptr;
			if (path_count > 0
				&& !dtStatusFailed(navmesh->navmesh->getTileAndPolyByRef(path_refs[path_count - 1], &stop_tile, &stop_poly)))
			{
				float stop_center[3], stop_quake[3];
				nav_mesh_poly_center(stop_tile, stop_poly, stop_center);
				nav_recast_to_quake(stop_center, stop_quake);
				nav_set_error(error, error_size, "Detour findPath: goal unreachable (partial%s, stopped at %.0f %.0f %.0f)",
					oon, stop_quake[0], stop_quake[1], stop_quake[2]);
			}
			else
				nav_set_error(error, error_size, "Detour findPath: goal unreachable (partial%s)", oon);
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
	/* Off-mesh link in traversal: set when the corridor advances past a
	   link, cleared when the actor reaches the far side or gets a new
	   path.  While set, navigate() keeps steering at the link end with
	   the link's type tag — otherwise the corridor advance (which fires
	   36u early, per dtCrowd) loses the tag before the bot has jumped/
	   dropped, leaving it staring at an unreachable lt=0 corner. */
	dtPolyRef pending_link_ref = 0;
	dtPolyRef pending_land_ref = 0;
	float pending_start[3] = {0, 0, 0};
	float pending_end[3] = {0, 0, 0};
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
	c->pending_link_ref = 0;
	return 1;
}

/* Per-bot rocket-jump gate: when the bot can't rocket-jump, exclude RJ
   off-mesh polys from its corridor filter so navigate() steers it the
   normal way instead of onto a launch it can't make. */
extern "C" void nav_corridor_set_rj(nav_corridor_t *c, int allowed)
{
	if (c == nullptr)
		return;
	c->filter.setExcludeFlags(allowed ? 0 : NAV_POLYFLAG_RJ);
}

/* Average of a ground poly's vertices (Recast coords).  Returns 0 for
   off-mesh connection polys — their "center" is mid-air. */
static int nav_poly_center(const nav_mesh_runtime_t *navmesh,
	dtPolyRef ref, float *center)
{
	const dtMeshTile *tile = nullptr;
	const dtPoly *poly = nullptr;

	if (dtStatusFailed(navmesh->navmesh->getTileAndPolyByRef(ref, &tile, &poly))
		|| poly->vertCount == 0
		|| poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION)
		return 0;
	nav_mesh_poly_center(tile, poly, center);
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

	/* Off-mesh link mid-traversal: the corridor is already past the link
	   (advanced at the 36u trigger below), but the actor hasn't landed on
	   the far side yet.  Keep steering at the link end with the type tag
	   so QC keeps executing the jump/drop; the corridor stays frozen at
	   the link end until arrival.  A failed traversal is recovered by
	   QC's progress stall, which requests a new path (clears pending). */
	if (c->pending_link_ref != 0)
	{
		/* Arrival must be 3D: near-vertical links (40u jump up a ledge)
		   put the actor horizontally next to the end while still on the
		   wrong level — a 2D test "arrives" instantly and re-loses the
		   type tag this mechanism exists to keep. */
		if (have_snapped_pos
			&& (snapped_ref == c->pending_land_ref
				|| dtVdist(snapped_pos, c->pending_end) < 24.0f))
		{
			c->pending_link_ref = 0;
		}
		else
		{
			/* Approach the link start first so jumps get their run-up
			   geometry; only steer at the end once committed (close to
			   the start, or already off the start level mid-traversal). */
			const float *steer_to = c->pending_end;
			if (have_snapped_pos
				&& dtVdist2D(snapped_pos, c->pending_start) > 24.0f
				&& fabsf(snapped_pos[1] - c->pending_start[1]) <= NAV_MESH_QUERY_CLIMB)
				steer_to = c->pending_start;
			nav_recast_to_quake(steer_to, corner_pos);
			*corner_flags = DT_STRAIGHTPATH_OFFMESH_CONNECTION;
			*corner_ref = static_cast<unsigned long long>(c->pending_link_ref);
			return 1;
		}
	}

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

	/* Stale-corridor check: if the corridor surface is far ABOVE the
	   actor (bot fell or was knocked off a ledge), movePosition keeps
	   XY-tracking along the old level and feeds overhead corners the
	   bot can never reach (dm4 bots pinned under walkways).  Replan in
	   place from the actor's real poly toward the same target — failing
	   the call instead causes goal-abandon churn on every early fall.
	   One-sided: corridor BELOW the bot is normal mid-drop/jump state. */
	if (have_snapped_pos
		&& c->corridor.getPos()[1] - snapped_pos[1] > NAV_MESH_QUERY_CLIMB)
	{
		const dtPolyRef *opath = c->corridor.getPath();
		int ocount = c->corridor.getPathCount();
		dtPolyRef tgt_ref = (ocount > 0) ? opath[ocount - 1] : 0;
		float tgt_pos[3];
		dtPolyRef npath[NAV_MESH_MAX_PATH_REFS];
		int ncount = 0;

		memcpy(tgt_pos, c->corridor.getTarget(), sizeof(tgt_pos));
		dtStatus rst = navmesh->query->findPath(snapped_ref, tgt_ref,
			snapped_pos, tgt_pos, &c->filter, npath, &ncount, NAV_MESH_MAX_PATH_REFS);
		if (dtStatusFailed(rst) || ncount < 1)
			return 0;
		c->corridor.reset(snapped_ref, snapped_pos);
		c->corridor.setCorridor(tgt_pos, npath, ncount);
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
	if (flags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
	{
		float trigger = 16.0f * 2.25f; /* agent radius * 2.25 */
		float dist2d = dtVdist2D(c->corridor.getPos(), corners);
		if (dist2d < trigger)
		{
			dtPolyRef advance_refs[2];
			float rc_start[3], rc_end[3];
			if (c->corridor.moveOverOffmeshConnection(
				refs[0], advance_refs, rc_start, rc_end, navmesh->query))
			{
				/* Corridor is now past the link.  Track the traversal so
				   subsequent calls keep steering at the link END with the
				   type tag until the actor lands on the far side (see
				   pending check above).  Return the end this frame too so
				   the run-up/jump aims across the gap, not at the lip. */
				c->pending_link_ref = refs[0];
				c->pending_land_ref = (c->corridor.getPathCount() > 0)
					? c->corridor.getPath()[0] : 0;
				dtVcopy(c->pending_start, rc_start);
				dtVcopy(c->pending_end, rc_end);
			}
		}
	}

	/* Fix corner height using getPolyHeight */
	{
		float h = 0;
		if (dtStatusSucceed(navmesh->query->getPolyHeight(refs[0], corners, &h)))
			corners[1] = h; /* Recast Y = height */
	}

	/* Edge-hugging fallback: hull-1 widening (16u) plus raster dilation
	   (one cell) extends polys past the physically standable edge, and the
	   funnel pulls its string along that phantom border.  On stairways the
	   corner ends up far overhead while the straight line to it runs beside
	   the actual treads (dm6 armorInv staircase: 4 bots queued at 100% stk).
	   When the corner is a climb the bot cannot make and is not an off-mesh
	   link, steer through the next corridor poly's center instead — centers
	   sit in the physically walkable interior, and the corridor keeps
	   advancing poly by poly until the corner is reachable again. */
	if (have_snapped_pos
		&& !(flags[0] & DT_STRAIGHTPATH_OFFMESH_CONNECTION)
		&& corners[1] - snapped_pos[1] > NAV_MESH_QUERY_CLIMB
		&& c->corridor.getPathCount() > 1)
	{
		float center[3];
		dtPolyRef next_ref = c->corridor.getPath()[1];

		if (nav_poly_center(navmesh, next_ref, center))
		{
			float h = 0;
			if (dtStatusSucceed(navmesh->query->getPolyHeight(next_ref, center, &h)))
				center[1] = h;
			dtVcopy(corners, center);
			flags[0] = 0;
			refs[0] = next_ref;
		}
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
	const float *point, float max_z, float min_z,
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

	/* Recast Y = Quake Z.  Collect walkable spans between min_z and max_z. */
	const rcCompactCell &cell = hf->compact->cells[gx + gz * hf->compact->width];
	for (int si = (int)cell.index, sn = (int)(cell.index + cell.count); si < sn; ++si)
	{
		if (hf->compact->areas[si] == RC_NULL_AREA)
			continue;
		float span_y = hf->config.bmin[1] + (float)hf->compact->spans[si].y * hf->config.ch;
		if (span_y > max_z || span_y < min_z)
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
