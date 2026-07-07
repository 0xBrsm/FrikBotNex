/*
 * nav_hull.cpp -- BSP clip-hull polygonization for navmesh geometry
 *
 * Builds navmesh input from hull 1 (the player clipping hull) instead of
 * render faces.  qbsp pre-expands hull 1 by the player bounding box, so
 * every point on a hull-1 floor is a valid player *origin* by
 * construction: narrow catwalks widen by the player's edge overhang,
 * walls push in by 16, and Recast can run with walkableRadius=0 — no
 * erosion, so none of the connectivity loss erosion causes.
 *
 * Method: carve the model's bounding box through the clipnode tree with
 * qbsp-style winding clipping.  Each non-solid leaf is a convex
 * polytope; the parts of its faces that border solid are the hull
 * boundary.  A face can border solid on one part and open space on
 * another (the neighbor subtree subdivides further), so each candidate
 * face is re-filtered through the tree and only the solid-backed pieces
 * are emitted.
 *
 * Output is translated down by 24 (player origin -> feet) so hull-1
 * floors land at the same Z as render-face floors, keeping all existing
 * query conventions.  Winding order follows Quake render faces:
 * geometric normal points out of the empty region's polytope (into the
 * solid), which the X/Y/Z -> X/Z/Y handedness flip in nav_mesh.cpp turns
 * into Recast-up for floors.
 */

extern "C" {
#include "quakedef.h"
}

#include "nav_hull.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <vector>

namespace {

const double NAV_HULL_EPS        = 0.02;
const double NAV_HULL_EXTENT     = 262144.0; /* base winding half-size */
const double NAV_HULL_BOX_PAD    = 64.0;     /* carve box beyond model bounds */
const double NAV_HULL_FACE_LIFT  = 0.5;      /* lift off plane for neighbor test */
const double NAV_HULL_MIN_AREA   = 0.5;
const float  NAV_HULL_FLOOR_DROP = 24.0f;    /* hull-1 floor -> feet level */
const double NAV_HULL_HAZARD_SAMPLE_AREA = 1024.0; /* subdivide until a tri is ~32x32 or smaller */
const int    NAV_HULL_HAZARD_MAX_DEPTH   = 8;      /* recursion cap for degenerate slivers */

struct V3
{
	double x, y, z;
};

static inline V3 v3(double x, double y, double z) { V3 r = {x, y, z}; return r; }
static inline V3 vadd(V3 a, V3 b) { return v3(a.x + b.x, a.y + b.y, a.z + b.z); }
static inline V3 vsub(V3 a, V3 b) { return v3(a.x - b.x, a.y - b.y, a.z - b.z); }
static inline V3 vscale(V3 a, double s) { return v3(a.x * s, a.y * s, a.z * s); }
static inline double vdot(V3 a, V3 b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
static inline V3 vcross(V3 a, V3 b)
{
	return v3(a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x);
}

typedef std::vector<V3> Winding;

/* Outward plane of a convex polytope face: interior satisfies n·x <= d.
   Winding is ordered so its geometric (right-hand) normal equals n. */
struct Face
{
	V3 n;
	double d;
	Winding w;
	bool from_clip; /* came from a clipnode plane (emittable boundary) */
};

typedef std::vector<Face> Polytope;

static V3 winding_normal(const Winding &w)
{
	/* Newell's method */
	V3 n = v3(0, 0, 0);
	for (size_t i = 0; i < w.size(); i++)
	{
		const V3 &a = w[i];
		const V3 &b = w[(i + 1) % w.size()];
		n.x += (a.y - b.y) * (a.z + b.z);
		n.y += (a.z - b.z) * (a.x + b.x);
		n.z += (a.x - b.x) * (a.y + b.y);
	}
	return n;
}

static double winding_area(const Winding &w)
{
	V3 n = winding_normal(w);
	return 0.5 * sqrt(vdot(n, n));
}

/* Large quad lying on plane (n,d), wound so geometric normal == n. */
static Winding base_winding(V3 n, double d)
{
	/* pick the major axis, build a basis */
	double ax = fabs(n.x), ay = fabs(n.y), az = fabs(n.z);
	V3 up = (az >= ax && az >= ay) ? v3(1, 0, 0) : v3(0, 0, 1);

	up = vsub(up, vscale(n, vdot(up, n)));
	double ulen = sqrt(vdot(up, up));
	Winding w;
	if (ulen < 1e-9)
		return w;
	up = vscale(up, 1.0 / ulen);

	V3 right = vcross(up, n);
	V3 org = vscale(n, d / vdot(n, n));

	up = vscale(up, NAV_HULL_EXTENT);
	right = vscale(right, NAV_HULL_EXTENT);

	w.push_back(vadd(vsub(org, right), up));
	w.push_back(vadd(vadd(org, right), up));
	w.push_back(vsub(vadd(org, right), up));
	w.push_back(vsub(vsub(org, right), up));

	if (vdot(winding_normal(w), n) < 0)
	{
		Winding rev(w.rbegin(), w.rend());
		w.swap(rev);
	}
	return w;
}

/* Sutherland–Hodgman: keep the part with n·x <= d. Preserves order. */
static Winding clip_winding(const Winding &w, V3 n, double d)
{
	Winding out;
	if (w.size() < 3)
		return out;

	std::vector<double> dist(w.size());
	bool any_front = false, any_back = false;
	for (size_t i = 0; i < w.size(); i++)
	{
		dist[i] = vdot(w[i], n) - d;
		if (dist[i] > NAV_HULL_EPS) any_front = true;
		if (dist[i] < -NAV_HULL_EPS) any_back = true;
	}
	if (!any_front)
		return w; /* fully kept */
	if (!any_back)
		return out; /* fully clipped */

	for (size_t i = 0; i < w.size(); i++)
	{
		size_t j = (i + 1) % w.size();
		double di = dist[i], dj = dist[j];

		if (di <= NAV_HULL_EPS)
			out.push_back(w[i]);
		if ((di < -NAV_HULL_EPS && dj > NAV_HULL_EPS) ||
		    (di > NAV_HULL_EPS && dj < -NAV_HULL_EPS))
		{
			double t = di / (di - dj);
			out.push_back(vadd(w[i], vscale(vsub(w[j], w[i]), t)));
		}
	}
	if (out.size() < 3)
		out.clear();
	return out;
}

struct Builder
{
	hull_t *hull;
	double org[3];
	std::vector<float> verts;
	std::vector<int> tris;
	int emitted;
};

static Builder nav_builder;
static bool nav_builder_active = false;

/* Append one triangle, applying the lift-off correction, entity origin,
   and the 24u origin->feet drop -- same per-vertex transform emit_winding
   uses for a whole face, just for 3 verts instead of a fan. */
static void emit_tri_verts(Builder *b, V3 a, V3 c2, V3 c3, V3 unlift)
{
	V3 tri[3] = {a, c2, c3};
	int base = (int)(b->verts.size() / 3);
	for (int i = 0; i < 3; i++)
	{
		V3 p = vadd(tri[i], unlift);
		b->verts.push_back((float)(p.x + b->org[0]));
		b->verts.push_back((float)(p.y + b->org[1]));
		b->verts.push_back((float)(p.z + b->org[2]) - NAV_HULL_FLOOR_DROP);
	}
	b->tris.push_back(base);
	b->tris.push_back(base + 1);
	b->tris.push_back(base + 2);
	b->emitted++;
}

/* Adaptive point sample for the liquid-hazard check: a floor face coming
   out of the clipnode tree is only split where hull 1 sees a solid/empty
   boundary, and lava/slime never creates one (see carve_leaf), so a floor
   that's part dry and part lava/slime can arrive here as one large,
   unsplit winding (dm4: a dry walkway sharing a face with an adjoining
   slime pool). A single centroid-only hazard check on the WHOLE face
   (the original approach) drops the entire winding the moment any part
   of it touches hazard -- on e1m1/e2m7/e4m5 that nuked large dry floor
   spans (and anything resting on them) sharing one big BSP face with a
   small slime corner. Quaking hazards are lethal to walk into and must
   never be part of a bot's route (nothing in id1 requires crossing one),
   so exclude at LEAF granularity instead: recursively quarter until each
   leaf is small enough that no sample point can hide a hazard patch,
   checking hull 0 at each leaf's centroid, and skip emitting only the
   leaves that actually touch hazard -- the dry remainder of the face
   still becomes ordinary walkable floor. */
static void emit_tri_dry_parts(Builder *b, V3 a, V3 c2, V3 c3, V3 unlift, int depth)
{
	V3 e1 = vsub(c2, a), e2 = vsub(c3, a);
	V3 cr = vcross(e1, e2);
	double area = 0.5 * sqrt(vdot(cr, cr));

	if (area > NAV_HULL_HAZARD_SAMPLE_AREA && depth < NAV_HULL_HAZARD_MAX_DEPTH)
	{
		V3 m12 = vscale(vadd(a, c2), 0.5);
		V3 m23 = vscale(vadd(c2, c3), 0.5);
		V3 m31 = vscale(vadd(c3, a), 0.5);
		emit_tri_dry_parts(b, a, m12, m31, unlift, depth + 1);
		emit_tri_dry_parts(b, m12, c2, m23, unlift, depth + 1);
		emit_tri_dry_parts(b, m31, m23, c3, unlift, depth + 1);
		emit_tri_dry_parts(b, m12, m23, m31, unlift, depth + 1);
		return;
	}

	if (area < NAV_HULL_MIN_AREA)
		return;

	V3 c = vadd(vscale(vadd(vadd(a, c2), c3), 1.0 / 3.0), unlift);
	vec3_t p;
	p[0] = (float)(c.x + b->org[0]);
	p[1] = (float)(c.y + b->org[1]);
	p[2] = (float)(c.z + b->org[2]);
	int contents = SV_PointContents(p);
	if (contents == CONTENTS_LAVA || contents == CONTENTS_SLIME)
		return; /* exclude only this small leaf, never the whole face */

	emit_tri_verts(b, a, c2, c3, unlift);
}

/* Fan-triangulate and append, applying entity origin, the lift-off
   correction, and the 24u origin->feet drop. is_floor routes each fan
   triangle through the liquid-hazard split above instead of a single
   whole-face emission: only faces whose normal points mostly down (i.e.
   floors, which have solid below and open space above) are candidates,
   so walls/ceilings skip the (expensive, and meaningless for a vertical
   or upward-facing face) contents check. */
static void emit_winding(Builder *b, const Winding &w, V3 unlift, bool is_floor)
{
	if (w.size() < 3 || winding_area(w) < NAV_HULL_MIN_AREA)
		return;

	if (is_floor)
	{
		for (size_t i = 1; i + 1 < w.size(); i++)
			emit_tri_dry_parts(b, w[0], w[i], w[i + 1], unlift, 0);
		return;
	}

	int base = (int)(b->verts.size() / 3);
	for (size_t i = 0; i < w.size(); i++)
	{
		V3 p = vadd(w[i], unlift);
		b->verts.push_back((float)(p.x + b->org[0]));
		b->verts.push_back((float)(p.y + b->org[1]));
		b->verts.push_back((float)(p.z + b->org[2]) - NAV_HULL_FLOOR_DROP);
	}
	for (size_t i = 1; i + 1 < w.size(); i++)
	{
		b->tris.push_back(base);
		b->tris.push_back(base + (int)i);
		b->tris.push_back(base + (int)i + 1);
		b->emitted++;
	}
}

/* Push a (lifted) face winding through the clipnode tree and emit only
   the pieces that land in solid leaves — the true hull boundary. */
static void emit_solid_parts(Builder *b, int node_num, const Winding &w, V3 unlift, bool is_floor)
{
	Winding cur = w;
	while (cur.size() >= 3)
	{
		if (node_num < 0)
		{
			if (node_num == CONTENTS_SOLID || node_num == CONTENTS_SKY)
				emit_winding(b, cur, unlift, is_floor);
			return;
		}

		dclipnode_t *node = b->hull->clipnodes + node_num;
		mplane_t *plane = b->hull->planes + node->planenum;
		V3 n = v3(plane->normal[0], plane->normal[1], plane->normal[2]);
		double d = plane->dist;

		/* front: n·x >= d  (children[0]) */
		Winding front = clip_winding(cur, vscale(n, -1.0), -d);
		Winding back = clip_winding(cur, n, d);

		if (front.size() >= 3)
			emit_solid_parts(b, node->children[0], front, unlift, is_floor);
		node_num = node->children[1];
		cur.swap(back);
	}
}

static void carve_leaf(Builder *b, int contents, const Polytope &poly)
{
	if (contents == CONTENTS_SOLID || contents == CONTENTS_SKY)
		return;
	/* This never actually fires: qbsp collapses water/slime/lava into
	   plain CONTENTS_EMPTY in the clip hulls (hull 1/2), so hull 1's
	   clipnode tree can't tell a dry floor from a lava-pit floor.  The
	   real hazard filter is the SV_PointContents probe in emit_winding,
	   which asks hull 0 (the only tree that keeps the distinction).
	   Kept as a cheap no-op guard in case some qbsp variant ever does
	   preserve liquid leaves in the clip hulls. */
	if (contents == CONTENTS_LAVA || contents == CONTENTS_SLIME)
		return;

	for (size_t i = 0; i < poly.size(); i++)
	{
		const Face &f = poly[i];
		if (!f.from_clip || f.w.size() < 3)
			continue;
		if (winding_area(f.w) < NAV_HULL_MIN_AREA)
			continue;

		/* Lift slightly toward the neighbor so tree classification is
		   unambiguous; emit_winding shifts back. */
		double f_n_len = sqrt(vdot(f.n, f.n));
		V3 lift = vscale(f.n, NAV_HULL_FACE_LIFT / f_n_len);
		Winding lifted;
		lifted.reserve(f.w.size());
		for (size_t k = 0; k < f.w.size(); k++)
			lifted.push_back(vadd(f.w[k], lift));

		/* Face normal points out of the empty polytope, into the solid
		   (module doc above) -- a floor has solid BELOW it, so its
		   normal points down, not up. */
		bool is_floor = (f.n.z / f_n_len) < -0.7;
		emit_solid_parts(b, b->hull->firstclipnode, lifted, vscale(lift, -1.0), is_floor);
	}
}

static void carve_node(Builder *b, int node_num, const Polytope &poly)
{
	if (poly.size() < 4)
		return; /* degenerate region, no volume */

	if (node_num < 0)
	{
		carve_leaf(b, node_num, poly);
		return;
	}

	dclipnode_t *node = b->hull->clipnodes + node_num;
	mplane_t *plane = b->hull->planes + node->planenum;
	V3 n = v3(plane->normal[0], plane->normal[1], plane->normal[2]);
	double d = plane->dist;

	Polytope front, back;
	front.reserve(poly.size() + 1);
	back.reserve(poly.size() + 1);

	for (size_t i = 0; i < poly.size(); i++)
	{
		const Face &f = poly[i];
		Winding fw = clip_winding(f.w, vscale(n, -1.0), -d); /* keep n·x >= d */
		Winding bw = clip_winding(f.w, n, d);                /* keep n·x <= d */
		if (fw.size() >= 3)
		{
			Face nf = {f.n, f.d, Winding(), f.from_clip};
			nf.w.swap(fw);
			front.push_back(nf);
		}
		if (bw.size() >= 3)
		{
			Face nb = {f.n, f.d, Winding(), f.from_clip};
			nb.w.swap(bw);
			back.push_back(nb);
		}
	}

	/* Cap each side with the split plane, clipped to the polytope. */
	Winding cap_front = base_winding(vscale(n, -1.0), -d); /* outward -n */
	Winding cap_back = base_winding(n, d);                 /* outward +n */
	for (size_t i = 0; i < poly.size() && cap_front.size() >= 3; i++)
		cap_front = clip_winding(cap_front, poly[i].n, poly[i].d);
	for (size_t i = 0; i < poly.size() && cap_back.size() >= 3; i++)
		cap_back = clip_winding(cap_back, poly[i].n, poly[i].d);

	if (cap_front.size() >= 3)
	{
		Face cf = {vscale(n, -1.0), -d, Winding(), true};
		cf.w.swap(cap_front);
		front.push_back(cf);
	}
	if (cap_back.size() >= 3)
	{
		Face cb = {n, d, Winding(), true};
		cb.w.swap(cap_back);
		back.push_back(cb);
	}

	carve_node(b, node->children[0], front);
	carve_node(b, node->children[1], back);
}

static Polytope box_polytope(const float *mins, const float *maxs)
{
	Polytope poly;
	double lo[3], hi[3];
	for (int i = 0; i < 3; i++)
	{
		lo[i] = mins[i] - NAV_HULL_BOX_PAD;
		hi[i] = maxs[i] + NAV_HULL_BOX_PAD;
	}

	for (int axis = 0; axis < 3; axis++)
	{
		for (int side = 0; side < 2; side++)
		{
			V3 n = v3(0, 0, 0);
			double d;
			double nsign = (side == 0) ? 1.0 : -1.0;
			if (axis == 0) n.x = nsign;
			else if (axis == 1) n.y = nsign;
			else n.z = nsign;
			d = (side == 0) ? hi[axis] : -lo[axis];

			Face f = {n, d, base_winding(n, d), false};
			for (int a2 = 0; a2 < 3 && f.w.size() >= 3; a2++)
			{
				if (a2 == axis) continue;
				V3 cn = v3(0, 0, 0);
				if (a2 == 0) cn.x = 1.0;
				else if (a2 == 1) cn.y = 1.0;
				else cn.z = 1.0;
				f.w = clip_winding(f.w, cn, hi[a2]);
				if (f.w.size() < 3) break;
				if (a2 == 0) cn.x = -1.0;
				else if (a2 == 1) cn.y = -1.0;
				else cn.z = -1.0;
				f.w = clip_winding(f.w, cn, -lo[a2]);
			}
			if (f.w.size() >= 3)
				poly.push_back(f);
		}
	}
	return poly;
}

} /* namespace */

extern "C" void nav_hull_begin(void)
{
	nav_builder.verts.clear();
	nav_builder.tris.clear();
	nav_builder.emitted = 0;
	nav_builder_active = true;
}

extern "C" int nav_hull_add_model(struct model_s *mod, const float *origin)
{
	if (!nav_builder_active || mod == NULL)
		return 0;

	hull_t *hull = &mod->hulls[1];
	if (hull->clipnodes == NULL || hull->planes == NULL)
		return 0;

	nav_builder.hull = hull;
	nav_builder.org[0] = origin ? origin[0] : 0.0;
	nav_builder.org[1] = origin ? origin[1] : 0.0;
	nav_builder.org[2] = origin ? origin[2] : 0.0;

	int before = nav_builder.emitted;
	carve_node(&nav_builder, hull->firstclipnode,
		box_polytope(mod->mins, mod->maxs));
	return nav_builder.emitted - before;
}

extern "C" int nav_hull_end(float **out_verts, int *out_vert_count,
	int **out_tris, int *out_tri_count)
{
	*out_verts = NULL; *out_vert_count = 0;
	*out_tris = NULL;  *out_tri_count = 0;

	if (!nav_builder_active)
		return 0;
	nav_builder_active = false;

	size_t vn = nav_builder.verts.size();
	size_t tn = nav_builder.tris.size();
	if (vn == 0 || tn == 0)
		return 0;

	float *verts = (float *)malloc(vn * sizeof(float));
	int *tris = (int *)malloc(tn * sizeof(int));
	if (!verts || !tris)
	{
		free(verts);
		free(tris);
		return 0;
	}
	memcpy(verts, nav_builder.verts.data(), vn * sizeof(float));
	memcpy(tris, nav_builder.tris.data(), tn * sizeof(int));

	*out_verts = verts;  *out_vert_count = (int)(vn / 3);
	*out_tris = tris;    *out_tri_count = (int)(tn / 3);

	nav_builder.verts.clear();
	nav_builder.tris.clear();
	return 1;
}
