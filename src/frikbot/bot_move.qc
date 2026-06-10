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
.entity _failed_goal;     // goal abandoned after a stall (engine skips it)
.float _failed_goal_time; // cooldown expiry for _failed_goal

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

// bot_get: follow cached path corridor to target item.
// Single-authority steering: navigate() returns corner, bot walks toward it.
// Single recovery: if no progress for 1 second, abandon goal.
void() bot_get =
{
	local vector wp, steer;
	local float link_type;

	if (!self.target1)
		return;

	wp = navigate(self.origin);
	if (wp == '0 0 0')
	{
		self.target1 = world;
		return;
	}

	// decode off-mesh link type from Z encoding: z' = z + 10000*(1+type)
	link_type = 0;
	if (wp_z > 9000)
	{
		link_type = rint(wp_z / 10000) - 1;
		wp_z = wp_z - (link_type + 1) * 10000;
	}

	// single-authority steering: walk toward the nav corner
	steer = wp - self.origin;
	if (vlen(steer) > 0.1)
		self.b_wishdir = normalize(steer);
	else
		self.b_wishdir = '0 0 0';
	if (self.target1 == self)
		self._m_move_mode = MM_ROAM;
	else
		self._m_move_mode = MM_NAV;

	// off-mesh link actions
	if (link_type == 3) // AI_DROP
	{
		// Walk off the ledge at full speed. The link's required_speed is
		// the MINIMUM horizontal speed to clear the gap — never throttle
		// below it (and Quake friction freezes anything under ~100 ups).
		// Flatten steering so the corner's -z doesn't kill horizontal speed.
		local vector flat_to_wp;

		flat_to_wp = wp - self.origin;
		// swimming: keep the vertical component — the drop "edge" is
		// underwater and the bot must swim down to the link end
		if (self.waterlevel < 2)
			flat_to_wp_z = 0;
		if (vlen(flat_to_wp) > 0.1)
			self.b_wishdir = normalize(flat_to_wp);
	}
	else if (link_type == 2) // AI_JUMP
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

		if (jdist < 64 && (self.flags & FL_ONGROUND))
			self.button2 = TRUE;
	}
	else if (link_type == 7) // AI_SUPER_JUMP
	{
		if (self.weapon != 32)
			self.impulse = 7;
		else if (self.flags & FL_ONGROUND)
		{
			self.button2 = TRUE;
			self.v_angle_x = self.b_angle_x = 80;
			self.button0 = TRUE;
		}
	}
	else if (link_type == 8) // AI_SURFACE
	{
		if (self.waterlevel > 2)
		{
			self.b_wishdir = '0 0 1';
			self.button2 = TRUE;
		}
	}
	else if (link_type == 4 || link_type == 5) // AI_PLAT / AI_TRAIN
	{
		traceline(self.origin, self.origin - '0 0 64', TRUE, self);
		if (trace_ent != world && trace_ent.velocity != '0 0 0')
			self.b_wishdir = '0 0 0';
	}
	else if (link_type == 6) // AI_DOORFLAG
	{
		traceline(self.origin, self.origin + self.b_wishdir * 64, TRUE, self);
		if (trace_ent != world && trace_ent.classname == "door")
			if (trace_ent.health > 0)
			{
				self.enemy = trace_ent;
				self.button0 = TRUE;
			}
	}

	// progress check: not getting closer to corner? abandon.
	// Off-mesh links get 3x timeout (bot may be doing controlled approach).
	{
		local vector flat, rt;
		local float xy_dist, fr, fl;
		flat = wp - self.origin;
		// swimming: progress is 3D (descending to an underwater corner
		// makes no XY progress at all)
		if (self.waterlevel < 2)
			flat_z = 0;
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

		// Shoulder probes: the steering corner can hug an obstacle the
		// center traceline slips past (open door panels, plat edges) —
		// the bot's 32u-wide box grinds on its corner with zero slide.
		// When progress stops, trace both shoulders and veer toward the
		// clear side; wall-slide physics takes over from there.
		if (self._progress_stall > 4 && link_type == 0
			&& self.b_wishdir != '0 0 0' && (self.flags & FL_ONGROUND))
		{
			rt_x = self.b_wishdir_y;
			rt_y = 0 - self.b_wishdir_x;
			rt_z = 0;
			traceline(self.origin + rt * 16,
				self.origin + rt * 16 + self.b_wishdir * 48, TRUE, self);
			fr = trace_fraction;
			traceline(self.origin - rt * 16,
				self.origin - rt * 16 + self.b_wishdir * 48, TRUE, self);
			fl = trace_fraction;
			if (fr < fl - 0.1)
				self.b_wishdir = normalize(self.b_wishdir - rt * 0.8);
			else if (fl < fr - 0.1)
				self.b_wishdir = normalize(self.b_wishdir + rt * 0.8);
		}

		if (self._progress_stall > (1 / cvar("sys_ticrate")))
		{
			// TEMP DEBUG: stall position dump
			bprint("STALL ");
			bprint(self.netname);
			bprint(" at ");
			bprint(vtos(self.origin));
			bprint(" wp ");
			bprint(vtos(wp));
			bprint(" lt ");
			bprint(ftos(link_type));
			bprint(" wl ");
			bprint(ftos(self.waterlevel));
			bprint(" og ");
			if (self.flags & FL_ONGROUND)
				bprint("1");
			else
				bprint("0");
			bprint(" tgt ");
			bprint(self.target1.classname);
			bprint("\n");
			self._progress_stall = 0;
			self._progress_best = 99999;
			// Cool the goal down: the scorer is deterministic, so an
			// immediate re-pick chooses the same goal and re-creates
			// the same jam (head-on bot deadlocks, unreachable perches).
			if (self.target1.flags & FL_ITEM)
			{
				self._failed_goal = self.target1;
				self._failed_goal_time = time + 10;
			}
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
