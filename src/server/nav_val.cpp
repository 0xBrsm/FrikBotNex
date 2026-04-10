/*
 * nav_val.cpp — Waypoint-vs-navmesh validation
 *
 * Tests every hand-crafted waypoint edge against the Detour navmesh.
 * Hand-crafted edges are ground truth — any PARTIAL result is a
 * confirmed navmesh defect.  Called once after navmesh build.
 *
 * Data is auto-generated from FrikBot X++ waypoint files.
 */

#include "nav_mesh.h"
#include "nav_val_data.h"

extern "C" {
#include "quakedef.h"
}

void Nav_Validate(const nav_mesh_runtime_t *mesh, const char *mapname)
{
	int mi, ei;
	const nav_val_map_t *map = NULL;
	dtQueryFilter filter;
	int ok_count = 0, partial_count = 0, fail_count = 0, miss_count = 0;

	if (mesh == NULL || mapname == NULL)
		return;

	/* Find map data */
	for (mi = 0; mi < NAV_VAL_MAP_COUNT; mi++)
	{
		if (!strcasecmp(mapname, nav_val_maps[mi].name))
		{
			map = &nav_val_maps[mi];
			break;
		}
	}
	if (map == NULL)
		return; /* no validation data for this map */

	nav_mesh_setup_filter(&filter);

	fprintf(stderr, "Nav: %s WAYPOINT VALIDATION (%d edges):\n",
		map->name, map->edge_count);

	for (ei = 0; ei < map->edge_count; ei++)
	{
		int ai = map->edges[ei][0];
		int bi = map->edges[ei][1];
		if (ai < 0 || ai >= map->wp_count || bi < 0 || bi >= map->wp_count)
			continue;

		nav_mesh_nearest_result_t nra, nrb;
		char nerr[64];
		int fa = nav_mesh_find_nearest(mesh, map->wps[ai], &nra, nerr, sizeof(nerr));
		int fb = nav_mesh_find_nearest(mesh, map->wps[bi], &nrb, nerr, sizeof(nerr));

		if (!fa || !fb)
		{
			miss_count++;
			fprintf(stderr, "  MISS wp%d->wp%d%s%s\n", ai + 1, bi + 1,
				fa ? "" : " (start off mesh)", fb ? "" : " (end off mesh)");
			continue;
		}

		float rca[3], rcb[3];
		rca[0] = nra.nearest_point[0]; rca[1] = nra.nearest_point[2]; rca[2] = nra.nearest_point[1];
		rcb[0] = nrb.nearest_point[0]; rcb[1] = nrb.nearest_point[2]; rcb[2] = nrb.nearest_point[1];

		dtPolyRef path[512];
		int pc = 0;
		dtStatus st = mesh->query->findPath(
			(dtPolyRef)nra.poly_ref, (dtPolyRef)nrb.poly_ref,
			rca, rcb, &filter, path, &pc, 512);
		int partial = dtStatusDetail(st, DT_PARTIAL_RESULT) ? 1 : 0;
		int failed = dtStatusFailed(st) ? 1 : 0;

		if (failed)
		{
			fail_count++;
			fprintf(stderr, "  FAIL wp%d->wp%d\n", ai + 1, bi + 1);
		}
		else if (partial)
		{
			partial_count++;
			float dz = map->wps[bi][2] - map->wps[ai][2];
			fprintf(stderr, "  PARTIAL wp%d->wp%d (%.0f,%.0f,%.0f)->(%.0f,%.0f,%.0f) dz=%.0f polys=%d\n",
				ai + 1, bi + 1,
				map->wps[ai][0], map->wps[ai][1], map->wps[ai][2],
				map->wps[bi][0], map->wps[bi][1], map->wps[bi][2],
				dz, pc);
		}
		else
			ok_count++;
	}

	fprintf(stderr, "Nav: %s WAYPOINT VALIDATION: %d OK, %d PARTIAL, %d FAIL, %d MISS / %d total\n",
		map->name, ok_count, partial_count, fail_count, miss_count, map->edge_count);

	/* For same-level PARTIAL edges, probe along the path to find
	   the exact break point — where navmesh polys disappear. */
	for (ei = 0; ei < map->edge_count; ei++)
	{
		int ai = map->edges[ei][0];
		int bi = map->edges[ei][1];
		if (ai < 0 || ai >= map->wp_count || bi < 0 || bi >= map->wp_count)
			continue;
		float dz = map->wps[bi][2] - map->wps[ai][2];
		if (dz > 18 || dz < -18) continue; /* skip cross-level */

		nav_mesh_nearest_result_t nra, nrb;
		char nerr[64];
		if (!nav_mesh_find_nearest(mesh, map->wps[ai], &nra, nerr, sizeof(nerr)))
			continue;
		if (!nav_mesh_find_nearest(mesh, map->wps[bi], &nrb, nerr, sizeof(nerr)))
			continue;

		float rca[3], rcb[3];
		rca[0]=nra.nearest_point[0]; rca[1]=nra.nearest_point[2]; rca[2]=nra.nearest_point[1];
		rcb[0]=nrb.nearest_point[0]; rcb[1]=nrb.nearest_point[2]; rcb[2]=nrb.nearest_point[1];
		dtPolyRef path[512]; int pc = 0;
		dtStatus st = mesh->query->findPath(
			(dtPolyRef)nra.poly_ref, (dtPolyRef)nrb.poly_ref,
			rca, rcb, &filter, path, &pc, 512);
		if (!dtStatusDetail(st, DT_PARTIAL_RESULT)) continue;

		/* This is a same-level PARTIAL. Probe in 4u steps. */
		fprintf(stderr, "Nav: PROBE wp%d->wp%d (%.0f,%.0f,%.0f)->(%.0f,%.0f,%.0f):\n",
			ai+1, bi+1,
			map->wps[ai][0], map->wps[ai][1], map->wps[ai][2],
			map->wps[bi][0], map->wps[bi][1], map->wps[bi][2]);

		unsigned long long prev_ref = 0;
		int last_hit = 1;
		float dx = map->wps[bi][0] - map->wps[ai][0];
		float dy = map->wps[bi][1] - map->wps[ai][1];
		float dz2 = map->wps[bi][2] - map->wps[ai][2];
		float dist = sqrtf(dx*dx + dy*dy + dz2*dz2);
		int steps = (int)(dist / 4.0f);
		if (steps < 10) steps = 10;
		if (steps > 200) steps = 200;

		for (int s = 0; s <= steps; s++)
		{
			float t = (float)s / steps;
			float p[3];
			p[0] = map->wps[ai][0] + dx * t;
			p[1] = map->wps[ai][1] + dy * t;
			p[2] = map->wps[ai][2] + dz2 * t;

			nav_mesh_nearest_result_t nr;
			int found = nav_mesh_find_nearest(mesh, p, &nr, nerr, sizeof(nerr));
			float snap = found ? sqrtf(
				(p[0]-nr.nearest_point[0])*(p[0]-nr.nearest_point[0]) +
				(p[1]-nr.nearest_point[1])*(p[1]-nr.nearest_point[1]) +
				(p[2]-nr.nearest_point[2])*(p[2]-nr.nearest_point[2])) : 0;
			int is_hit = found && snap < 60;

			/* Log transitions: hit→miss or miss→hit or new poly ref */
			if (is_hit != last_hit || (is_hit && found && (unsigned long long)nr.poly_ref != prev_ref))
			{
				fprintf(stderr, "  t=%.2f (%.0f,%.0f) %s snap=%.0f ref=%llu\n",
					t, p[0], p[1],
					is_hit ? "HIT" : "MISS", snap,
					found ? nr.poly_ref : 0ULL);
				if (found) prev_ref = (unsigned long long)nr.poly_ref;
				last_hit = is_hit;
			}
		}
	}
}
