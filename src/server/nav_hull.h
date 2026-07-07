/*
 * nav_hull.h -- BSP clip-hull polygonization for navmesh geometry
 */
#ifndef NAV_HULL_H
#define NAV_HULL_H

#ifdef __cplusplus
extern "C" {
#endif

struct model_s;

void nav_hull_begin(void);
/* origin may be NULL (worldmodel). Returns triangles emitted. */
int nav_hull_add_model(struct model_s *mod, const float *origin);
/* Hands back malloc'd vertex/triangle soup, plus a malloc'd per-triangle
   hazard flag (1 if the triangle's centroid reads CONTENTS_SLIME in hull 0,
   0 otherwise). Lava triangles are never emitted at all -- see nav_hull.cpp
   for why lava and slime are treated differently. Resets internal state. */
int nav_hull_end(float **out_verts, int *out_vert_count,
	int **out_tris, int *out_tri_count,
	unsigned char **out_hazard);

#ifdef __cplusplus
}
#endif

#endif /* NAV_HULL_H */
