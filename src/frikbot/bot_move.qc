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

	// decode off-mesh link type from Z encoding
	link_type = 0;
	if (wp_z > 9000)
	{
		link_type = wp_z - 10000;
		wp_z = wp_z - 10000 - link_type;
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
		// Controlled drop: slow to required_speed near the edge
		local vector linfo, flat_to_wp;
		local float approach_speed, dist_to_edge;

		linfo = nav_link_info();
		approach_speed = linfo_x;
		if (approach_speed < 10) approach_speed = 10;

		flat_to_wp = wp - self.origin;
		flat_to_wp_z = 0;
		dist_to_edge = vlen(flat_to_wp);

		// Within 64u of edge: scale wishdir to control speed
		if (dist_to_edge < 64)
		{
			local float scale;
			scale = approach_speed / sv_maxspeed;
			if (scale > 1) scale = 1;
			self.b_wishdir = normalize(flat_to_wp) * scale;
		}
	}
	else if (link_type == 2) // AI_JUMP
	{
		// Run at required speed and jump at the edge
		local vector jinfo, jflat;
		local float jdist;

		jinfo = nav_link_info();
		jflat = wp - self.origin;
		jflat_z = 0;
		jdist = vlen(jflat);

		// Jump when close to edge and on ground
		if (jdist < 16 && (self.flags & FL_ONGROUND))
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
	// Skip during off-mesh link traversal (bot is doing controlled approach).
	if (link_type)
	{
		self._progress_stall = 0;
		self._progress_best = 99999;
	}
	{
		local vector flat;
		local float xy_dist;
		flat = wp - self.origin;
		flat_z = 0;
		xy_dist = vlen(flat);

		if (xy_dist < self._progress_best - 1)
		{
			self._progress_best = xy_dist;
			self._progress_stall = 0;
		}
		else
			self._progress_stall = self._progress_stall + 1;

		if (self._progress_stall > (1 / cvar("sys_ticrate")))
		{
			self._progress_stall = 0;
			self._progress_best = 99999;
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
