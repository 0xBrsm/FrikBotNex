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

// ---- Stuck recovery ----
.float _stuck_count;
.vector _stuck_origin;

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

// When a stair waypoint sits on the side edge of the nav ramp, walk along the
// blocking wall until we find the first tread instead of sliding into the side.
float(vector along, vector wall_normal, float floor_z, float dist) bot_stair_probe =
{
	local vector trace_start, trace_end, probe;
	local float rise;

	trace_start = self.origin;
	trace_start_z = trace_start_z - 20;
	trace_end = self.origin + along * dist;
	trace_end_z = trace_end_z - 20;
	traceline(trace_start, trace_end, TRUE, self);
	if (trace_fraction < 1)
		return 0;

	probe = self.origin + along * dist - wall_normal * 24;
	if (pointcontents(probe + '0 0 8') == CONTENT_SOLID)
		return 0;

	trace_start = probe;
	trace_start_z = trace_start_z + 24;
	trace_end = probe;
	trace_end_z = trace_end_z - 40;
	traceline(trace_start, trace_end, TRUE, self);
	if (trace_fraction == 1)
		return 0;

	rise = trace_endpos_z - floor_z;
	if (rise < 6 || rise > 24)
		return 0;

	return rise;
};

vector(vector wall_normal, float step_height) bot_stair_entry_dir =
{
	local vector wall_tangent, trace_start, trace_end, choice;
	local float floor_z, rise1, rise2, align;

	choice = '0 0 0';

	if (!(self.flags & FL_ONGROUND))
		return choice;
	if (step_height <= 4 || step_height >= 128)
		return choice;

	wall_normal_z = 0;
	if (vlen(wall_normal) < 0.1)
		return choice;
	wall_normal = normalize(wall_normal);

	trace_start = self.origin;
	trace_start_z = trace_start_z + 24;
	trace_end = self.origin;
	trace_end_z = trace_end_z - 64;
	traceline(trace_start, trace_end, TRUE, self);
	if (trace_fraction == 1)
		return choice;
	floor_z = trace_endpos_z;

	wall_tangent_x = wall_normal_y;
	wall_tangent_y = wall_normal_x * -1;
	wall_tangent_z = 0;

	align = self.b_wishdir * wall_tangent;

	// sweep along wall at increasing distances to find stair mouth
	local float probe_dist;
	probe_dist = 32;
	while (probe_dist <= 224)
	{
		rise1 = bot_stair_probe(wall_tangent, wall_normal, floor_z, probe_dist);
		rise2 = bot_stair_probe(wall_tangent * -1, wall_normal, floor_z, probe_dist);
		if (rise1 || rise2)
		{
			if (rise1 && !rise2) return wall_tangent;
			if (rise2 && !rise1) return wall_tangent * -1;
			if (align >= 0) return wall_tangent;
			return wall_tangent * -1;
		}
		probe_dist = probe_dist + 32;
	}

	return choice;
};

// ---- Navigation ----

// bot_get: follow cached path corridor to target item
void() bot_get =
{
	local vector wp, steer, diff;
	local float step_height, link_type;

	if (!self.target1)
		return;

	// get next steering corner from dtPathCorridor
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

	// compute direction to corner (3D for facing, 2D for movement)
	steer = wp - self.origin;
	step_height = steer_z;

	// set b_wishdir with full 3D direction for look angle
	if (vlen(steer) > 0.1)
		self.b_wishdir = normalize(steer);
	else
		self.b_wishdir = '0 0 0';
	if (self.target1 == self)
		self._m_move_mode = MM_ROAM;
	else
		self._m_move_mode = MM_NAV;

	// wall avoidance + stair finding
	{
		local vector trace_start, trace_end, wall_normal, slide, stair_dir;
		trace_start = self.origin;
		trace_start_z = trace_start_z - 20;
		trace_end = trace_start + self.b_wishdir * 32;
		traceline(trace_start, trace_end, TRUE, self);
		if (trace_fraction < 1)
		{
			wall_normal = trace_plane_normal;
			stair_dir = bot_stair_entry_dir(wall_normal, step_height);
			if (stair_dir != '0 0 0')
				self.b_wishdir = stair_dir;
			else
			{
				wall_normal_z = 0;
				slide = self.b_wishdir - wall_normal * (self.b_wishdir * wall_normal);
				if (vlen(slide) > 0.1)
					self.b_wishdir = normalize(slide);
			}
		}
	}

	// off-mesh link actions
	if (link_type == 2) // AI_JUMP
	{
		if (self.flags & FL_ONGROUND)
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

	// stuck detection
	diff = self.origin - self._stuck_origin;
	diff_z = 0;
	if (vlen(diff) < 1)
		self._stuck_count = self._stuck_count + 1;
	else
		self._stuck_count = 0;
	self._stuck_origin = self.origin;

	if (self._stuck_count > 5)
	{
		local vector dir;
		dir = self.b_wishdir;
		self.b_wishdir_x = dir_y;
		self.b_wishdir_y = dir_x * -1;

		if (self._stuck_count > 15)
		{
			self._stuck_count = 0;
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
