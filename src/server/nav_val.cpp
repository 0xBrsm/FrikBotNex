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

/* TEMP DEBUG — stall probes.  Live-bot stall positions (STALL bprint)
   paired with the goal the bot was navigating to.  Dumps the start
   snap, full findPath result, and every straight-path corner with
   off-mesh poly markers, to explain corners appearing overhead. */
typedef struct {
	const char *map;
	float s[3];
	float e[3];
} nav_stall_probe_t;
static const nav_stall_probe_t nav_stall_probes[] = {
	{"dm4",  {624, -202, -104},  {1136, -592, 0}},   /* Lore/Max -> armor2 */
	{"dm4",  {116, -368, -104},  {840, -584, 0}},    /* Erwin under stairs -> spikes */
	{"dm4",  {880, -400, -104},  {840, -584, 0}},    /* Max under slats */
	{"dm4",  {368, -1061, 24},   {440, -784, -128}}, /* Max ledge -> rockets */
	{"e2m1", {398, 1331, -108},  {384, 1384, -94}},  /* Erwin/Hudson pin */
	{"e1m1", {976, 1812, -408},  {896, 1840, -526}}, /* Lore at door t10 */
	{"dm6",  {1533, -484, 40},   {1736, -344, 144}}, /* 4-bot pile-up -> armorInv */
	{"dm6",  {152, -1920, 40},   {56, -2008, 64}},   /* pedestal pin -> armor1 */
	{"dm6",  {197.6f, -1909.2f, 40}, {192, -1908, 18}}, /* Erwin snap-rank pin */
};

static void Nav_StallProbes(const nav_mesh_runtime_t *mesh, const char *mapname)
{
	dtQueryFilter filter;
	nav_mesh_setup_filter(&filter);

	for (size_t pi = 0; pi < sizeof(nav_stall_probes) / sizeof(nav_stall_probes[0]); pi++)
	{
		const nav_stall_probe_t *pr = &nav_stall_probes[pi];
		if (strcasecmp(mapname, pr->map))
			continue;

		fprintf(stderr, "Nav: STALLPROBE (%.0f,%.0f,%.0f)->(%.0f,%.0f,%.0f)\n",
			pr->s[0], pr->s[1], pr->s[2], pr->e[0], pr->e[1], pr->e[2]);

		nav_mesh_nearest_result_t ns, ne;
		char nerr[96];
		if (!nav_mesh_find_nearest(mesh, pr->s, &ns, nerr, sizeof(nerr)))
		{
			fprintf(stderr, "  start MISS: %s\n", nerr);
			continue;
		}
		fprintf(stderr, "  start snap (%.0f,%.0f,%.0f) dz=%+.0f ref=%llu\n",
			ns.nearest_point[0], ns.nearest_point[1], ns.nearest_point[2],
			ns.nearest_point[2] - pr->s[2], (unsigned long long)ns.poly_ref);

		/* Actor-snap candidate dump: every poly in the biased box with
		   old (3D-nearest) and new (horizontal-first) scores. */
		{
			float rc[3] = {pr->s[0], pr->s[2], pr->s[1]};
			float center[3], half[3];
			dtPolyRef cand[64];
			int cn = 0;
			nav_mesh_actor_snap_box(mesh, rc, center, half);
			if (dtStatusSucceed(mesh->query->queryPolygons(
					center, half, &filter, cand, &cn, 64)))
			{
				for (int ci = 0; ci < cn; ci++)
				{
					float pt[3];
					bool over = false;
					if (dtStatusFailed(mesh->query->closestPointOnPoly(
							cand[ci], rc, pt, &over)))
						continue;
					float dx = pt[0] - rc[0];
					float dyv = pt[1] - rc[1];
					float dz = pt[2] - rc[2];
					float horiz = sqrtf(dx * dx + dz * dz);
					float dy = pt[1] - (rc[1] - 24.0f);
					fprintf(stderr,
						"  actorcand ref=%llu pt=(%.0f,%.0f,%.0f) horiz=%.1f dy=%+.1f old=%.1f new=%.1f%s%s\n",
						(unsigned long long)cand[ci], pt[0], pt[2], pt[1],
						horiz, dy,
						sqrtf(dx * dx + dyv * dyv + dz * dz),
						horiz * 4.0f + fabsf(dy),
						over ? " OVER" : "",
						(pt[1] > rc[1] + 8.0f) ? " CAPPED" : "");
				}
			}
		}
		if (!nav_mesh_find_nearest(mesh, pr->e, &ne, nerr, sizeof(nerr)))
		{
			fprintf(stderr, "  end MISS: %s\n", nerr);
			continue;
		}
		fprintf(stderr, "  end snap (%.0f,%.0f,%.0f) dz=%+.0f ref=%llu\n",
			ne.nearest_point[0], ne.nearest_point[1], ne.nearest_point[2],
			ne.nearest_point[2] - pr->e[2], (unsigned long long)ne.poly_ref);

		float rs[3] = {ns.nearest_point[0], ns.nearest_point[2], ns.nearest_point[1]};
		float re[3] = {ne.nearest_point[0], ne.nearest_point[2], ne.nearest_point[1]};
		dtPolyRef path[256];
		int pc = 0;
		dtStatus st = mesh->query->findPath(
			(dtPolyRef)ns.poly_ref, (dtPolyRef)ne.poly_ref,
			rs, re, &filter, path, &pc, 256);
		fprintf(stderr, "  findPath %s polys=%d\n",
			dtStatusFailed(st) ? "FAIL" :
			dtStatusDetail(st, DT_PARTIAL_RESULT) ? "PARTIAL" : "OK", pc);
		if (dtStatusFailed(st) || pc == 0)
			continue;

		for (int i = 0; i < pc; i++)
		{
			const dtMeshTile *tile = NULL;
			const dtPoly *poly = NULL;
			if (dtStatusFailed(mesh->navmesh->getTileAndPolyByRef(path[i], &tile, &poly)))
				continue;
			float cx = 0, cy = 0, cz = 0;
			for (int v = 0; v < poly->vertCount; v++)
			{
				const float *vp = &tile->verts[poly->verts[v] * 3];
				cx += vp[0]; cy += vp[1]; cz += vp[2];
			}
			if (poly->vertCount)
			{
				cx /= poly->vertCount; cy /= poly->vertCount; cz /= poly->vertCount;
			}
			fprintf(stderr, "   p%d ref=%llu type=%s center=(%.0f,%.0f,%.0f)\n",
				i, (unsigned long long)path[i],
				poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION ? "OFFMESH" : "ground",
				cx, cz, cy);
		}

		float spos[64 * 3];
		unsigned char sflags[64];
		dtPolyRef srefs[64];
		int sc = 0;
		mesh->query->findStraightPath(rs, re, path, pc,
			spos, sflags, srefs, &sc, 64, DT_STRAIGHTPATH_AREA_CROSSINGS);
		for (int i = 0; i < sc; i++)
		{
			const dtMeshTile *tile = NULL;
			const dtPoly *poly = NULL;
			int offmesh = 0;
			unsigned char area = 0;
			if (srefs[i] && dtStatusSucceed(
					mesh->navmesh->getTileAndPolyByRef(srefs[i], &tile, &poly)))
			{
				offmesh = (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION);
				area = poly->getArea();
			}
			fprintf(stderr, "   c%d (%.0f,%.0f,%.0f) sflags=%d area=%d%s\n",
				i, spos[i * 3], spos[i * 3 + 2], spos[i * 3 + 1],
				(int)sflags[i], (int)area, offmesh ? " OFFMESH" : "");
		}
	}
}

void Nav_Validate(const nav_mesh_runtime_t *mesh, const char *mapname)
{
	int mi, ei;
	const nav_val_map_t *map = NULL;
	dtQueryFilter filter;
	int ok_count = 0, partial_count = 0, fail_count = 0, miss_count = 0;

	if (mesh == NULL || mapname == NULL)
		return;

	Nav_StallProbes(mesh, mapname);

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

	/* Off-mesh waypoint probe: for every waypoint that the navmesh can't
	   snap to, hull-trace the BSP geometry to reveal whether floor exists,
	   how it differs from waypoint Z, and how wide the standable surface
	   is.  Narrow catwalks eaten by erosion show as floor present but
	   width < 32u (bot hull). */
	int probed_off = 0;
	for (int wi = 0; wi < map->wp_count; wi++)
	{
		nav_mesh_nearest_result_t nr;
		char nerr[96];
		nerr[0] = '\0';
		if (nav_mesh_find_nearest(mesh, map->wps[wi], &nr, nerr, sizeof(nerr)))
			continue;

		if (probed_off == 0)
			fprintf(stderr, "Nav: %s OFF-MESH WAYPOINTS:\n", map->name);
		probed_off++;
		fprintf(stderr, "  [why: %s]\n", nerr[0] ? nerr : "no result");

		vec3_t hmins = {-16,-16,-24}, hmaxs = {16,16,32};
		/* Try Z+8 first (low ceiling tolerant), fall back to Z+64. */
		vec3_t ts = {map->wps[wi][0], map->wps[wi][1], map->wps[wi][2] + 8};
		vec3_t te = {ts[0], ts[1], ts[2] - 320};
		trace_t tr = SV_Move(ts, hmins, hmaxs, te, MOVE_NOMONSTERS, NULL);
		int floor_ok = !tr.allsolid && tr.fraction < 1.0f;
		float floor_z = floor_ok ? tr.endpos[2] - 24 : -9999;

		/* Measure headroom: ray (point hull) upward from waypoint. */
		vec3_t pmins = {0,0,0}, pmaxs = {0,0,0};
		vec3_t us = {map->wps[wi][0], map->wps[wi][1], map->wps[wi][2]};
		vec3_t ue = {us[0], us[1], us[2] + 256};
		trace_t tu = SV_Move(us, pmins, pmaxs, ue, MOVE_NOMONSTERS, NULL);
		float ceil_dz = (!tu.allsolid && tu.fraction < 1.0f) ?
			(tu.endpos[2] - map->wps[wi][2]) : 999;

		if (!floor_ok)
		{
			/* Z+8 failed — try Z+64 (waypoint may be above tall solid). */
			vec3_t ts2 = {map->wps[wi][0], map->wps[wi][1], map->wps[wi][2] + 64};
			vec3_t te2 = {ts2[0], ts2[1], ts2[2] - 320};
			trace_t tr2 = SV_Move(ts2, hmins, hmaxs, te2, MOVE_NOMONSTERS, NULL);
			if (!tr2.allsolid && tr2.fraction < 1.0f)
			{
				floor_ok = 1;
				floor_z = tr2.endpos[2] - 24;
				fprintf(stderr, "  wp%d (%.0f,%.0f,%.0f) ceil_dz=%.0f floor_z=%.0f dz=%+.0f  (deep, hull blocked at Z+8)\n",
					wi + 1, map->wps[wi][0], map->wps[wi][1], map->wps[wi][2],
					ceil_dz, floor_z, floor_z - map->wps[wi][2]);
				continue;
			}
			fprintf(stderr, "  wp%d (%.0f,%.0f,%.0f) NO_FLOOR allsolid=%d frac=%.2f ceil_dz=%.0f\n",
				wi + 1, map->wps[wi][0], map->wps[wi][1], map->wps[wi][2],
				tr.allsolid, tr.fraction, ceil_dz);
			continue;
		}

		/* Sweep ±X, ±Y at floor_z+64 → floor_z-64, step 4u,
		   record max extent where hull still lands within 8u of floor_z. */
		static const float dirs[4][2] = {{1,0},{-1,0},{0,1},{0,-1}};
		int extent[4] = {0,0,0,0};
		for (int d = 0; d < 4; d++)
		{
			for (int step = 4; step <= 64; step += 4)
			{
				vec3_t ps = {map->wps[wi][0] + dirs[d][0] * step,
					map->wps[wi][1] + dirs[d][1] * step,
					floor_z + 64};
				vec3_t pe = {ps[0], ps[1], floor_z - 64};
				trace_t tp = SV_Move(ps, hmins, hmaxs, pe, MOVE_NOMONSTERS, NULL);
				if (tp.allsolid || tp.fraction >= 1.0f) break;
				float fz = tp.endpos[2] - 24;
				if (fabsf(fz - floor_z) > 8) break;
				extent[d] = step;
			}
		}

		fprintf(stderr, "  wp%d (%.0f,%.0f,%.0f) floor=%.0f dz=%+.0f ceil_dz=%.0f  Xw=%d Yw=%d\n",
			wi + 1, map->wps[wi][0], map->wps[wi][1], map->wps[wi][2],
			floor_z, floor_z - map->wps[wi][2], ceil_dz,
			extent[0] + extent[1] + 32, extent[2] + extent[3] + 32);
	}
	if (probed_off > 0)
		fprintf(stderr, "Nav: %s OFF-MESH: %d waypoints probed\n",
			map->name, probed_off);

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

			/* Also do a BSP hull trace to see what the engine says */
			vec3_t ts = {p[0], p[1], p[2] + 64};
			vec3_t te = {p[0], p[1], p[2] - 64};
			vec3_t hmins = {-16, -16, -24}, hmaxs = {16, 16, 32};
			trace_t tr = SV_Move(ts, hmins, hmaxs, te, MOVE_NOMONSTERS, NULL);
			float hull_z = tr.endpos[2] - 24; /* origin → floor */
			int hull_ok = !tr.allsolid && tr.fraction < 1.0f &&
				fabsf(hull_z - p[2]) < 20;

			/* Log transitions */
			int cur_state = is_hit ? (hull_ok ? 1 : 3) : (hull_ok ? 2 : 0);
			if (cur_state != last_hit || (is_hit && found && (unsigned long long)nr.poly_ref != prev_ref))
			{
				const char *label;
				if (is_hit && hull_ok) label = "BOTH_OK";
				else if (is_hit && !hull_ok) label = "MESH_ONLY";
				else if (!is_hit && hull_ok) label = "HULL_ONLY";
				else label = "NEITHER";
				fprintf(stderr, "  t=%.2f (%.0f,%.0f) %s snap=%.0f hull_z=%.0f ref=%llu\n",
					t, p[0], p[1], label, snap, hull_z,
					found ? nr.poly_ref : 0ULL);
				if (found) prev_ref = (unsigned long long)nr.poly_ref;
				last_hit = cur_state;
			}
		}
	}
}
