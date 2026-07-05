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
/* Hands back malloc'd vertex/triangle soup; resets internal state. */
int nav_hull_end(float **out_verts, int *out_vert_count,
	int **out_tris, int *out_tri_count);

#ifdef __cplusplus
}
#endif

#endif /* NAV_HULL_H */
