/***********************************************
*                                              *
*      FrikBot Movement + Spatial Awareness    *
*                                              *
***********************************************/

/*

This program is in the Public Domain. My crack legal
team would like to add:

RYAN "FRIKAC" SMITH IS PROVIDING THIS SOFTWARE "AS IS"
AND MAKES NO WARRANTY, EXPRESS OR IMPLIED, AS TO THE
ACCURACY, CAPABILITY, EFFICIENCY, MERCHANTABILITY, OR
FUNCTIONING OF THIS SOFTWARE AND/OR DOCUMENTATION. IN
NO EVENT WILL RYAN "FRIKAC" SMITH BE LIABLE FOR ANY
GENERAL, CONSEQUENTIAL, INDIRECT, INCIDENTAL,
EXEMPLARY, OR SPECIAL DAMAGES, EVEN IF RYAN "FRIKAC"
SMITH HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH
DAMAGES, IRRESPECTIVE OF THE CAUSE OF SUCH DAMAGES.

You accept this software on the condition that you
indemnify and hold harmless Ryan "FrikaC" Smith from
any and all liability or damages to third parties,
including attorney fees, court costs, and other
related costs and expenses, arising out of your use
of this software irrespective of the cause of said
liability.

The export from the United States or the subsequent
reexport of this software is subject to compliance
with United States export control and munitions
control restrictions. You agree that in the event you
seek to export this software, you assume full
responsibility for obtaining all necessary export
licenses and approvals and for assuring compliance
with applicable reexport restrictions.

Any reproduction of this software must contain
this notice in its entirety.

*/

// ---- Progress tracking (single recovery system) ----
.float _progress_best;    // best XY distance to corner seen this goal
.float _progress_stall;   // frames with no improvement
.entity _progress_tgt;    // goal the progress baseline belongs to
.vector _progress_wp;     // corner the progress baseline belongs to
.float _nav_pin;          // next net-displacement check (corner-independent)
.vector _nav_pin_org;     // position at last check
.float _nav_pin_strikes;  // consecutive dead 2s windows
.entity _failed_goal;     // goal abandoned after a stall (engine skips it)
.float _failed_goal_time; // cooldown expiry for _failed_goal
.entity _failed_goal2;     // second-most-recent abandoned goal (engine skips it too)
.float _failed_goal_time2; // cooldown expiry for _failed_goal2
.float _nav_backoff;      // until this time, walk _backoff_dir instead
.vector _backoff_dir;     // retreat direction (give a moving door room)
.entity _plat_link_pl;    // cached nearest-plat lookup for a AI_PLAT_BOTTOM corner
.vector _plat_link_wp;    // corner that cache was computed for

// ---- Debug ----
.float _dbg_notgt_time;   // throttle for no-target logging

// ---- Sense fields ----

.float sense_time;
.float sense_space_time;
.float sense_ground_type;
.float sense_enemy_vis;
.float sense_enemy_dist;
.float sense_enemy_wall;
.float sense_my_size;
.float sense_enemy_size;
.float sense_friendly_block;

// ---- Combat engagement ----

// Arm the pin window: clocks bot_fight reads to time the no-progress
// strikes that abandon a hopeless fight, and to keep a just-acquired
// enemy from being dropped on the very next frame.
void() bot_arm_pin =
{
	self._combat_pin = time + 2;
	self._pin_org = self.origin;
	self._pin_strikes = 0;
};

// Acquire e as the enemy and arm the combat clocks, the way
// bot_find_enemy does — stale clocks make bot_fight drop the target.
void(entity e) bot_engage_mover =
{
	self.enemy = e;
	self.b_react_time = time;
	self.b_combat_time = time;
	bot_arm_pin();
};

// If a fresh traceline put a shootable mover (door/button with health)
// in the lane and we aren't already fighting, gun it open. Returns
// whether it engaged. Caller runs the traceline; we read trace_ent.
float() bot_shoot_lane_mover =
{
	if (self.enemy)
		return FALSE;
	if (trace_ent != world
		&& (trace_ent.classname == "door"
			|| trace_ent.classname == "func_button")
		&& trace_ent.health > 0 && trace_ent.takedamage)
	{
		bot_engage_mover(trace_ent);
		return TRUE;
	}
	return FALSE;
};

// Newton's-method square root — QuakeC has no sqrt builtin and vlen only
// roots a sum of squares. Six iterations from 1.0 cover the fall-time math.
float(float n) bot_sqrt =
{
	local float r, i;
	if (n <= 0)
		return 0;
	r = 1;
	i = 0;
	while (i < 6)
	{
		r = (r + n / r) * 0.5;
		i = i + 1;
	}
	return r;
};

// Mid-flight guidance for a drop. Steer toward the horizontal velocity that
// puts the bot on `corner` when its arc descends back to that height.
// SV_AirAccelerate brakes hard against our motion (~160ups/frame) but only
// nudges forward at 30 — so this erases an overshoot to stick a near landing
// the bot would otherwise sail over, while a too-slow arc still gets the
// forward push. Drops only: a jump must KEEP its launch momentum to clear the
// gap, and braking it just drops the bot short into a re-jump loop.
void(vector corner) bot_air_guide =
{
	local vector to_wp, hv, desired, steer;
	local float dz, disc, t, gap;

	to_wp = corner - self.origin;
	to_wp_z = 0;
	gap = vlen(to_wp);

	// time until the arc falls back through the landing height
	dz = corner_z - self.origin_z;
	disc = self.velocity_z * self.velocity_z - 2 * sv_gravity * dz;
	if (disc < 0)
	{
		// arc never reaches that height — just hold course for the corner
		if (gap > 0.1)
			self.b_wishdir = normalize(to_wp);
		return;
	}
	t = (self.velocity_z + bot_sqrt(disc)) / sv_gravity;
	if (t < 0.05)
		t = 0.05;

	hv = self.velocity;
	hv_z = 0;
	if (gap > 1)
		desired = normalize(to_wp) * (gap / t);
	else
		desired = '0 0 0';
	steer = desired - hv;
	if (vlen(steer) > 1)
		self.b_wishdir = normalize(steer);
	else
		self.b_wishdir = '0 0 0';
};

// ---- Spatial awareness ----

void() bot_sense_space =
{
	if (self.sense_space_time == time)
		return;
	self.sense_space_time = time;

	// ground state (1 trace — used by hazard escape + debug metrics)
	traceline(self.origin, self.origin - '0 0 64', TRUE, self);
	if (trace_fraction < 1)
		self.sense_ground_type = pointcontents(trace_endpos + '0 0 4');
	else
		self.sense_ground_type = CONTENT_EMPTY;
};

// ---- Enemy visibility ----

void() bot_sense_visibility =
{
	local vector spot1, org, dir, v;

	spot1 = self.origin + self.view_ofs;

	if (self.enemy && self.enemy != world)
	{
		org = realorigin(self.enemy);
		self.sense_enemy_dist = vlen(org - self.origin);

		traceline(spot1, org, TRUE, self);
		if (trace_ent == self.enemy || trace_fraction == 1)
			self.sense_enemy_vis = TRUE;
		else
		{
			if (self.enemy.classname == "player")
			{
				traceline(spot1, org + self.enemy.maxs, TRUE, self);
				if (trace_ent == self.enemy || trace_fraction == 1)
					self.sense_enemy_vis = TRUE;
				else
				{
					traceline(spot1, org + self.enemy.mins, TRUE, self);
					if (trace_ent == self.enemy || trace_fraction == 1)
						self.sense_enemy_vis = TRUE;
					else
						self.sense_enemy_vis = FALSE;
				}
			}
			else
				self.sense_enemy_vis = FALSE;
		}

		if (self.sense_enemy_vis)
		{
			dir = normalize(org - self.origin);
			traceline(org, org + dir * 100, TRUE, self.enemy);
			self.sense_enemy_wall = (trace_fraction < 1);
		}
		else
			self.sense_enemy_wall = FALSE;

		if (self.sense_enemy_vis && (teamplay || coop))
		{
			makevectors(self.v_angle);
			v = weapon_range(self.weapon);
			traceline(spot1, spot1 + v_forward * v_z, FALSE, self);
			if (trace_ent.classname == "player")
			{
				if ((trace_ent.team == self.team && teamplay) || coop)
				{
					if (vlen(trace_endpos - spot1) < self.sense_enemy_dist)
						self.sense_friendly_block = TRUE;
					else
						self.sense_friendly_block = FALSE;
				}
				else
					self.sense_friendly_block = FALSE;
			}
			else
				self.sense_friendly_block = FALSE;
		}
		else
			self.sense_friendly_block = FALSE;
	}
	else
	{
		self.sense_enemy_vis = FALSE;
		self.sense_enemy_dist = 0;
		self.sense_enemy_wall = FALSE;
		self.sense_friendly_block = FALSE;
	}
};

// ---- Main sense entry point ----

void() bot_sense =
{
	if (self.sense_time == time)
		return;
	self.sense_time = time;

	self.sense_my_size = bot_size_player(self);
	if (self.enemy && self.enemy != world)
		self.sense_enemy_size = bot_size_player(self.enemy);
	else
		self.sense_enemy_size = 0;

	bot_sense_space();
	bot_sense_visibility();
};

// ---- b_wishdir → movevect ----

void() bot_build_movevect =
{
	local vector vang;
	local float spd;

	self.movevect = '0 0 0';
	if (self.b_wishdir == '0 0 0')
		return;

	vang = self.v_angle;
	vang_x = vang_z = 0;
	makevectors(vang);

	spd = sv_maxspeed * 2;
	self.movevect_x = (self.b_wishdir * v_forward) * spd;
	self.movevect_y = (self.b_wishdir * v_right) * spd;
	// swimming: send the vertical component as upmove so the bot can
	// actually descend/ascend (SV_WaterMove adds cmd.upmove to wishvel)
	if (self.waterlevel > 1)
		self.movevect_z = self.b_wishdir_z * spd;
	else
		self.movevect_z = 0;
};

// ---- Navigation ----

// bot_mark_failed_goal: cool a goal down so the deterministic scorer can't
// immediately re-pick it. Two slots, not one: a single slot only ever
// remembers the LAST failure, so when the top two candidates both fail to
// path/arrive, cooling one just promotes the other into best-pick, which
// then fails and re-cools, evicting the first the moment its own timer
// still has 0-10s left — the two ping-pong forever. Rotating the old
// slot-1 entry into slot-2 (only if it's still live) means both recent
// failures stay skipped at once, so the scorer has to fall through to a
// goal it can actually make progress on.
void(entity g, float dur) bot_mark_failed_goal =
{
	if (self._failed_goal != world && self._failed_goal != g
		&& self._failed_goal_time > time)
	{
		self._failed_goal2 = self._failed_goal;
		self._failed_goal_time2 = self._failed_goal_time;
	}
	self._failed_goal = g;
	self._failed_goal_time = time + dur;
};

// Sample a straight line in 32u steps for lava/slime. The navmesh never
// carves walkable polys over hazard floors, so a corridor never routes
// through one on purpose — but the last-stretch beeline in bot_get()
// below bypasses the mesh entirely, and a goal sitting across a lava
// pool (an item on a mid-pool island with no path in) is close enough
// as the crow flies to trigger it.
float(vector from, vector to) nav_hazard_between =
{
	local vector dir;
	local float dist, step, c;

	dist = vlen(to - from);
	if (dist < 1)
		return FALSE;
	dir = (to - from) * (1 / dist);
	step = 0;
	while (step <= dist)
	{
		c = pointcontents(from + dir * step);
		if (c == CONTENT_LAVA || c == CONTENT_SLIME)
			return TRUE;
		step = step + 32;
	}
	return FALSE;
};

// Opportunistic pickup radius/threshold: nav_find_goal only ever commits
// to ONE item at a time, so a bot beelining across the map for the RL
// walked straight past a shotgun or a health box sitting a few steps off
// its own corridor -- picking it up costs nothing (Quake pickup is a bare
// touch), so it's worth a small steering nudge without touching the
// actual goal/path in the C engine.
float NAV_SIDEGRAB_RADIUS = 150;
float NAV_SIDEGRAB_WANT = 0.3;

// Look for a high-value item near the bot that's roughly on the way to
// this frame's steering corner `wp`. Skips the active goal itself (already
// being pursued), anything behind us (dot product with travel direction),
// and anything a straight line would cross lava/slime to reach.
entity(vector wp) bot_sidegrab_scan =
{
	local entity e, best;
	local float w, bestw, d;
	local vector todir, itemdir;

	todir = wp - self.origin;
	if (vlen(todir) < 1)
		return world;
	todir = normalize(todir);

	best = world;
	bestw = NAV_SIDEGRAB_WANT;
	e = findradius(self.origin, NAV_SIDEGRAB_RADIUS);
	while (e)
	{
		if ((e.flags & FL_ITEM) && e.model != string_null && e != self.target1)
		{
			d = vlen(e.origin - self.origin);
			if (d > 1)
			{
				itemdir = normalize(e.origin - self.origin);
				if ((itemdir * todir) > 0.3 && !nav_hazard_between(self.origin, e.origin))
				{
					w = bot_want(e);
					if (w > bestw)
					{
						bestw = w;
						best = e;
					}
				}
			}
		}
		e = e.chain;
	}
	return best;
};

// bot_get: follow cached path corridor to target item.
// Single-authority steering: navigate() returns corner, bot walks toward it.
// Single recovery: if no progress for 1 second, abandon goal.
void() bot_get =
{
	local vector wp, steer;
	local float link_type;

	if (!self.target1)
		return;

	// Backing off: we were standing in a moving door's lane (secret
	// doors slide through the spot bots queue at, grinding against a
	// waiting crowd forever if we don't move). Step away and let it
	// finish before resuming the corridor.
	if (time < self._nav_backoff)
	{
		self.b_wishdir = self._backoff_dir;
		return;
	}

	wp = navigate(self.origin);
	if (wp == '0 0 0')
	{
		// Corridor says "arrived" (within 32u of the nav target) or died.
		// Arriving is NOT the errand: items need a touch, buttons a press.
		// If the goal entity is right here, walk the last stretch at its
		// center — the progress stall below cools it if we never connect.
		// Anything else is a dead corridor: cool the goal before dropping,
		// or the deterministic scorer re-picks it next frame and the
		// pick/drop loop pins the bot in place at 20Hz.
		local vector gc;
		gc = (self.target1.absmin + self.target1.absmax) * 0.5;
		if (self.target1 != self && vlen(gc - self.origin) < 128
			&& !nav_hazard_between(self.origin, gc))
			wp = gc;
		else
		{
			if (self.target1 != world && self.target1 != self)
				bot_mark_failed_goal(self.target1, 10);
			self.target1 = world;
			return;
		}
	}

	// decode off-mesh link type from Z encoding: z' = z + 10000*(1+type)
	link_type = 0;
	if (wp_z > 9000)
	{
		link_type = rint(wp_z / 10000) - 1;
		wp_z = wp_z - (link_type + 1) * 10000;
	}

	// incidental pickup: only while on plain ground corridor steering --
	// mid-jump/drop/rj/plat/train/door the corner IS the errand, swerving
	// off it risks missing the link entirely.
	if (link_type == 0)
	{
		local entity grab;
		grab = bot_sidegrab_scan(wp);
		if (grab != world)
			wp = grab.origin;
	}

	// single-authority steering: walk toward the nav corner.
	// On ground the vertical is dead weight: corners sit at feet level
	// (up to 24u below origin), and normalizing the 3D vector both
	// starves the horizontal push (near a low corner most of the unit
	// vector points down) and tilts the shoulder probes into the floor.
	// Swimming keeps z — that's the only case with vertical control —
	// but measured from the FEET (origin-24), where corners live: there
	// is no step-up while swimming, so a terrace lip above the feet yet
	// below the origin must steer UP to clear, not level into the lip.
	steer = wp - self.origin;
	if (self.waterlevel < 2)
		steer_z = 0;
	else
		steer_z = steer_z + 24;
	if (vlen(steer) > 0.1)
		self.b_wishdir = normalize(steer);
	else
		self.b_wishdir = '0 0 0';
	if (self.target1 == self)
		self._m_move_mode = MM_ROAM;
	else
		self._m_move_mode = MM_NAV;

	// off-mesh link actions
	if (link_type == AI_DROP)
	{
		if (self.waterlevel < 2 && !(self.flags & FL_ONGROUND))
		{
			// Airborne: guide the landing onto the corner.
			bot_air_guide(wp);
		}
		else
		{
			// Run off the ledge toward the corner. Flatten steering so the
			// corner's -z doesn't starve horizontal speed; swimming keeps
			// the vertical (feet-referenced) to swim to an underwater link.
			local vector flat_to_wp;

			flat_to_wp = wp - self.origin;
			if (self.waterlevel < 2)
				flat_to_wp_z = 0;
			else
				flat_to_wp_z = flat_to_wp_z + 24;
			if (vlen(flat_to_wp) > 0.1)
				self.b_wishdir = normalize(flat_to_wp);
		}
	}
	else if (link_type == AI_JUMP)
	{
		// Run at the edge and jump. Window must be wider than the
		// corridor's 36u link-advance trigger, or the corner changes
		// before the bot ever gets within jumping range.
		local vector jflat;
		local float jdist;

		jflat = wp - self.origin;
		jflat_z = 0;
		jdist = vlen(jflat);
		if (jdist > 0.1)
			self.b_wishdir = normalize(jflat);

		// Don't commit the jump into a blocked lane: a bot body (or
		// wall) between here and the link start just bounces the
		// jumper in place, and airborne frames can't steer around it.
		if (jdist < 64 && (self.flags & FL_ONGROUND))
		{
			traceline(self.origin, wp + '0 0 24', FALSE, self);
			if (trace_fraction == 1)
				self.button2 = TRUE;
		}
	}
	else if (link_type == AI_SUPER_JUMP)
	{
		// No launcher, no rocket jump. Without it the bot just spams a
		// weapon switch that can never land (impulse 7 to a gun it doesn't
		// own) and grinds the link forever. Bail on the goal — the area's
		// rocket-only items go unwanted instead of fatal, and pathing
		// falls back to a route the bot can actually walk.
		// Goal pick only checked health/quad once, at commit time (C side,
		// nav_bot_can_rj). target1 sticks until the errand's done -- a fight
		// picked up along the way can chip health well below the safe
		// margin before the bot ever reaches the launch point, and nothing
		// re-validates that in between. Re-check here too, or a bot that
		// was fine when it picked the goal can still splash itself dead
		// executing a jump it can no longer survive.
		if (!(self.items & IT_ROCKET_LAUNCHER)
			|| self.health <= 75
			|| (self.items & IT_QUAD))
		{
			if (self.target1.flags & FL_ITEM)
				bot_mark_failed_goal(self.target1, 30);
			self.target1 = world;
			return;
		}
		if (self.weapon != IT_ROCKET_LAUNCHER)
			self.impulse = 7;
		else if (self.flags & FL_ONGROUND)
		{
			self.button2 = TRUE;
			self.v_angle_x = self.b_angle_x = 80;
			self.button0 = TRUE;
		}
	}
	else if (link_type == AI_SURFACE)
	{
		// Any water contact counts: a long climb to a distant surface
		// exit can read waterlevel 1-2 for most of the ascent (ledges,
		// shafts that aren't uniformly full-depth), not just 3. Gating
		// on full submersion left the bot with no vertical propulsion
		// at all for most of the climb -- it just stalled treading in
		// place until the goal got abandoned.
		//
		// Pure '0 0 1' thrust only works when the exit is plumb overhead:
		// a crossing whose opening sits off to one side just noses the
		// ceiling forever with no horizontal correction. Keep swimming
		// toward the corner's XY (same corridor authority as everywhere
		// else) and add the upward bias on top, instead of discarding it.
		if (self.waterlevel > 0)
		{
			local vector surf_dir;

			surf_dir = wp - self.origin;
			surf_dir_z = surf_dir_z + 64;
			if (vlen(surf_dir) > 0.1)
				self.b_wishdir = normalize(surf_dir);
			else
				self.b_wishdir = '0 0 1';
			self.button2 = TRUE;
		}
	}
	else if (link_type == AI_PLAT_BOTTOM || link_type == AI_RIDE_TRAIN)
	{
		// Riding: standing on a moving lift/train makes zero XY progress
		// by design — hold position so we don't pace off the edge. The
		// abandon timeout below gets an 8x window for these links (a
		// full wait + ride cycle), but never a free pass: an outright
		// reset here turned every plat-adjacent stall into a permanent
		// pin.
		traceline(self.origin, self.origin - '0 0 64', TRUE, self);
		if (trace_ent != world
			&& (trace_ent.classname == "plat" || trace_ent.classname == "train"))
		{
			if (trace_ent.velocity != '0 0 0')
				self.b_wishdir = '0 0 0';
		}
		else if (link_type == AI_PLAT_BOTTOM)
		{
			// Not aboard. If the plat serving this link is ABOVE the
			// corner we're heading for, hold at the shaft lip — walking
			// to the exact center means dropping into the pit under it
			// (and blocking its return). Position, not state: a bot at
			// the TOP lip must board a top-parked plat or walk off over
			// a lowered one, never stand waiting. The 8x abandon window
			// below covers the wait; a dead plat (targeted, never
			// triggered) still times out.
			local entity pl, bestpl;
			local vector pc;
			local float pd, bestpd;

			// The serving plat is a function of the link corner (wp), which
			// only changes when the corridor advances -- re-scanning every
			// plat in the level every frame while a bot waits at the lip is
			// wasted work. Cache the lookup per corner instead.
			if (self._plat_link_pl == world
				|| self._plat_link_pl.classname != "plat"
				|| vlen(wp - self._plat_link_wp) > 1)
			{
				bestpl = world;
				bestpd = 99999;
				pl = find(world, classname, "plat");
				while (pl != world)
				{
					pc = (pl.absmin + pl.absmax) * 0.5;
					pc_z = wp_z;
					pd = vlen(pc - wp);
					if (pd < bestpd)
					{
						bestpd = pd;
						bestpl = pl;
					}
					pl = find(pl, classname, "plat");
				}
				self._plat_link_pl = bestpl;
				self._plat_link_wp = wp;
			}
			bestpl = self._plat_link_pl;
			pc = (bestpl.absmin + bestpl.absmax) * 0.5;
			pc_z = wp_z;
			bestpd = vlen(pc - wp);
			if (bestpl != world && bestpd < 128
				&& bestpl.absmax_z > wp_z + 32
				&& self.origin_z < bestpl.absmax_z)
			{
				pc = wp - self.origin;
				pc_z = 0;
				if (vlen(pc) < 80)
					self.b_wishdir = '0 0 0';
			}
			// A plat actively moving through this link's shaft: stepping
			// toward it just grinds the bot against its side/underside.
			// Hold until it parks, then board. Moving is transient, so
			// this can't become a standing freeze.
			else if (bestpl != world && bestpd < 96
				&& bestpl.velocity != '0 0 0')
			{
				pc = wp - self.origin;
				pc_z = 0;
				if (vlen(pc) < 96)
					self.b_wishdir = '0 0 0';
			}
		}
	}

	// progress check: not getting closer to corner? abandon.
	// Off-mesh links get 3x timeout (bot may be doing controlled approach).
	{
		local vector flat, rt;
		local float xy_dist, fr, fl;
		flat = wp - self.origin;
		// swimming: progress is 3D, feet-referenced (descending to an
		// underwater corner makes no XY progress at all)
		if (self.waterlevel < 2)
			flat_z = 0;
		else
			flat_z = flat_z + 24;
		xy_dist = vlen(flat);

		// new goal or new corner: reset the baseline, otherwise the
		// previous corner's (small) best distance forces a false stall
		if (self.target1 != self._progress_tgt || vlen(wp - self._progress_wp) > 32)
		{
			self._progress_tgt = self.target1;
			self._progress_wp = wp;
			self._progress_best = 99999;
			self._progress_stall = 0;
		}

		if (xy_dist < self._progress_best - 1)
		{
			self._progress_best = xy_dist;
			self._progress_stall = 0;
		}
		else
			self._progress_stall = self._progress_stall + 1;

		// A shootable door in the lane is an invitation, not a wall:
		// make it the enemy — combat aim already knows how to shoot
		// BSP movers, and opening doors clear takedamage, releasing
		// the lock. (Touch doors open themselves, key doors open on
		// touch if we hold the key; the goal picker never routes us
		// through a door nobody here can open.)
		if (self._progress_stall > 4 && !self.enemy
			&& self.b_wishdir != '0 0 0')
		{
			traceline(self.origin, self.origin + self.b_wishdir * 64, TRUE, self);
			bot_shoot_lane_mover();
		}

		// Shoulder probes: the steering corner can hug an obstacle the
		// center traceline slips past (open door panels, plat edges) —
		// the bot's 32u-wide box grinds on its corner with zero slide.
		// When progress stops, trace both shoulders and veer toward the
		// clear side; wall-slide physics takes over from there.
		// Probes must see entity boxes (nomonsters FALSE): two bots
		// meeting head-on pin each other box-to-box, and a probe that
		// skips SOLID_SLIDEBOX reports "all clear" through the blocker.
		// Jump links recover too: while still walking the approach, a
		// blocked jumper needs the same veer/side-step as flat ground
		// (the jump itself is gated on a clear lane above). Plat/train
		// approaches too: the boarding point can sit past a wall corner
		// the straight-line steer grinds on forever.
		// Drop approaches too: several bots beelining for the same ledge
		// corner box each other in the open room behind it, and with no
		// veer they grind full-speed into each other forever. The
		// ledge-departure itself is untouched: this only fires on-ground,
		// before the bot has committed to running off.
		// Airborne wedges too: a bot can end up pressed flat against a
		// wall while falling (not resting on anything, so FL_ONGROUND
		// never sets) and just grind straight into it under gravity,
		// sliding down the face until it happens to clear the wall's
		// bottom edge. A real jump/drop arc never takes anywhere near 2s,
		// so once _nav_pin_strikes has actually latched (a full realized
		// 2s window of <24u net movement) it's safe to veer even mid-air
		// -- this can't misfire on a legitimate airborne link.
		if (self._progress_stall > 4
			&& (link_type == 0 || link_type == AI_JUMP || link_type == AI_DROP
				|| link_type == AI_PLAT_BOTTOM || link_type == AI_RIDE_TRAIN)
			&& self.b_wishdir != '0 0 0'
			&& ((self.flags & FL_ONGROUND) || self._nav_pin_strikes > 0))
		{
			rt_x = self.b_wishdir_y;
			rt_y = 0 - self.b_wishdir_x;
			rt_z = 0;
			traceline(self.origin + rt * 16,
				self.origin + rt * 16 + self.b_wishdir * 48, FALSE, self);
			fr = trace_fraction;
			traceline(self.origin - rt * 16,
				self.origin - rt * 16 + self.b_wishdir * 48, FALSE, self);
			fl = trace_fraction;
			if (fr < fl - 0.1)
				self.b_wishdir = normalize(self.b_wishdir - rt * 0.8);
			else if (fl < fr - 0.1)
				self.b_wishdir = normalize(self.b_wishdir + rt * 0.8);
			else if (fr < 0.5)
			{
				// Both shoulders blocked at the same depth: a body (or
				// flat wall) dead ahead — asymmetry veer can't fire, so
				// side-step square and let the stall reset once the lane
				// clears. Split the side-step by player slot (colormap
				// is the engine's per-client edict number) so a pile of
				// bots stuck the same way doesn't all step in lockstep
				// and just rotate in place forever, re-forming the same
				// circular jam every time they all turn the identical
				// direction.
				if (self.colormap & 1)
					self.b_wishdir = normalize(rt);
				else
					self.b_wishdir = normalize(rt * -1);
			}
		}

		// Net-displacement pin check: the stall counter above resets
		// whenever the steering corner moves >32u, and at a ledge edge
		// navigate() can flip between two corners forever, each flip
		// resetting the stall before it reaches the limit. Corners lie;
		// the bot's own position doesn't — measure 2s windows like the
		// combat pin does.
		if (time > self._nav_pin)
		{
			local vector nav_moved;
			nav_moved = self.origin - self._nav_pin_org;
			nav_moved_z = 0;
			if (self._nav_pin > 0 && vlen(nav_moved) < 24
				&& self.b_wishdir != '0 0 0')
				self._nav_pin_strikes = self._nav_pin_strikes + 1;
			else
				self._nav_pin_strikes = 0;
			self._nav_pin_org = self.origin;
			self._nav_pin = time + 2;
			// Pinned with a shootable mover toward the corner? Shoot it
			// open instead of abandoning. The stall>4 trigger above never
			// fires here: bouncing off the door keeps inching xy_dist
			// closer, resetting the counter.
			if (self._nav_pin_strikes > 0 && !self.enemy)
			{
				local vector pdir;
				pdir = wp - self.origin;
				pdir_z = 0;
				pdir = normalize(pdir);
				traceline(self.origin, self.origin + pdir * 64, TRUE, self);
				if (bot_shoot_lane_mover())
				{
					self._nav_pin_strikes = 0;
					return;
				}
			}
			if (self._nav_pin_strikes > 1)
			{
				self._nav_pin_strikes = 0;
				self._progress_stall = 0;
				self._progress_best = 99999;
				if (self.target1 != world && self.target1 != self)
					bot_mark_failed_goal(self.target1, 10);
				self.target1 = world;
				return;
			}
		}

		local float stall_limit;
		stall_limit = 1 / sv_ticrate;
		// lift links: zero XY progress is normal while waiting for and
		// riding the plat — budget a full return + ride cycle (~8s)
		// before declaring the goal dead. Surface links get the same
		// budget: a long vertical swim can cover corridor-corner
		// distance slower than dry running, and shouldn't be judged
		// against the same 1s window.
		if (link_type == AI_PLAT_BOTTOM || link_type == AI_RIDE_TRAIN
			|| link_type == AI_SURFACE)
			stall_limit = stall_limit * 8;
		if (self._progress_stall > stall_limit)
		{
			// Probe the lane: a monster-aware trace tells us what's ahead.
			local vector bdir;
			bdir = wp - self.origin;
			bdir_z = 0;
			bdir = normalize(bdir);
			traceline(self.origin, self.origin + bdir * 48, FALSE, self);
			// If it's a shootable mover, gun it open instead of
			// abandoning. The stall>4 hook above misses this case:
			// bouncing off the door keeps inching xy_dist closer,
			// resetting the counter before it reaches 5.
			if (bot_shoot_lane_mover())
			{
				self._progress_stall = 0;
				self._progress_best = 99999;
				return;
			}
			// A door that's MOVING is a door we're standing on: secret
			// doors slide through the queue spot, grinding against the
			// crowd forever if takedamage got cleared mid-open and the
			// slide never finished. Step back, give it room, keep the
			// goal.
			if (trace_ent != world && trace_ent.classname == "door"
				&& trace_ent.velocity != '0 0 0')
			{
				self._nav_backoff = time + 0.8;
				self._backoff_dir = normalize(self.origin
					- (trace_ent.absmin + trace_ent.absmax) * 0.5);
				self._backoff_dir_z = 0;
				self._progress_stall = 0;
				self._progress_best = 99999;
				return;
			}
			self._progress_stall = 0;
			self._progress_best = 99999;
			// Cool the goal down: the scorer is deterministic, so an
			// immediate re-pick chooses the same goal and re-creates the
			// same jam. Cool ANY real goal, not just items — a func_button
			// opener the bot can't reach is re-picked every frame otherwise.
			if (self.target1 != world && self.target1 != self)
				bot_mark_failed_goal(self.target1, 10);
			self.target1 = world;
			return;
		}
	}
};

float(vector dir) frik_walkmove =
{
	self.b_wishdir = dir;
	self.b_wishdir_z = 0;
	return TRUE;
};
