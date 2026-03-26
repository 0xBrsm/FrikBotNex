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
.float _nav_diag_time;	// next nav diagnostic print time

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

void() frik_movetogoal =
{
	local vector steer, goal_pos, diff;
	local float step_height, path_result;
	local string reason;

	reason = string_null;

	if (self.target1 == world)
	{
		self.b_wishdir = '0 0 0';
		reason = "no_target";

		if (time > self._nav_diag_time)
		{
			self._nav_diag_time = time + 3;
			bprint("["); bprint(self.netname); bprint("] NAV: ");
			bprint(reason);
			bprint(" pos="); bprint(vtos(self.origin));
			bprint("\n");
		}
		return;
	}

	goal_pos = realorigin(self.target1);
	// brush triggers have center high above floor — use floor Z
	if (self.target1.solid == SOLID_TRIGGER)
		goal_pos_z = self.target1.absmin_z;

	if (vlen(goal_pos - self.origin) < 25)
	{
		self.b_wishdir = '0 0 0';
		return;  // close enough — bot_check_lost handles success
	}

	// compute path when goal entity changes
	if (self._nav_goal_ent != self.target1)
	{
		path_result = nav_path_start(goal_pos);
		if (path_result > 0)
			self._nav_goal_ent = self.target1;
		else
		{
			reason = "path_fail";

			if (time > self._nav_diag_time)
			{
				self._nav_diag_time = time + 3;
				bprint("["); bprint(self.netname); bprint("] NAV: ");
				bprint(reason);
				bprint(" tgt="); bprint(self.target1.classname);
				bprint(" pos="); bprint(vtos(self.origin));
				bprint(" goal="); bprint(vtos(goal_pos));
				bprint(" dist="); bprint(ftos(rint(vlen(goal_pos - self.origin))));
				bprint("\n");
			}

			bot_lost(self.target1, FALSE);
			return;
		}
	}

	// get cached steer direction
	steer = nav_path_steer(self.origin);
	if (steer == '0 0 0')
	{
		self._nav_goal_ent = world;
		self.b_wishdir = '0 0 0';
		self._m_move_mode = MM_BEELINE;

		if (vlen(goal_pos - self.origin) > 64)
		{
			reason = "stale";

			if (time > self._nav_diag_time)
			{
				self._nav_diag_time = time + 3;
				bprint("["); bprint(self.netname); bprint("] NAV: ");
				bprint(reason);
				bprint(" tgt="); bprint(self.target1.classname);
				bprint(" pos="); bprint(vtos(self.origin));
				bprint(" goal="); bprint(vtos(goal_pos));
				bprint(" dist="); bprint(ftos(rint(vlen(goal_pos - self.origin))));
				bprint("\n");
			}

			bot_lost(self.target1, FALSE);
		}
		return;
	}

	// Z encodes either height delta or link type (1000 + AI_*)
	step_height = steer_z;
	steer_z = 0;
	self.b_wishdir = steer;
	self._m_move_mode = MM_NAV;

	// off-mesh link traversal (steer_z = 1000 + AI_* link type)
	if (step_height >= 1000)
	{
		local float link_type;
		link_type = step_height - 1000;

		if (link_type == 2) // AI_JUMP: press jump
		{
			if (self.flags & FL_ONGROUND)
				self.button2 = TRUE;
		}
		else if (link_type == 7) // AI_SUPER_JUMP: rocket jump
		{
			if (self.weapon != 32) // IT_ROCKET_LAUNCHER
				self.impulse = 7; // select RL
			else if (self.flags & FL_ONGROUND)
			{
				self.button2 = TRUE; // jump
				self.v_angle_x = self.b_angle_x = 80; // aim down
				self.button0 = TRUE; // fire
			}
		}
		else if (link_type == 8) // AI_SURFACE: swim up
		{
			if (self.waterlevel > 2)
			{
				self.b_wishdir = '0 0 1';
				self.button2 = TRUE;
			}
		}
		else if (link_type == 4 || link_type == 5) // AI_PLAT_BOTTOM / AI_RIDE_TRAIN
		{
			// Check if something is below us (platform/train)
			traceline(self.origin, self.origin - '0 0 64', TRUE, self);
			if (trace_ent != world)
			{
				// On a mover — stop and ride
				if (trace_ent.velocity != '0 0 0')
					self.b_wishdir = '0 0 0';
			}
			// else: mover not here yet, keep walking toward link
		}
		else if (link_type == 6) // AI_DOORFLAG
		{
			// Check if a door is blocking our path
			traceline(self.origin, self.origin + self.b_wishdir * 64, TRUE, self);
			if (trace_ent != world && trace_ent.classname == "door")
			{
				// Door in the way — try to open it
				if (trace_ent.health > 0)
				{
					// Shootable door — target it
					self.enemy = trace_ent;
					self.button0 = TRUE;
				}
				// else: touch-triggered, just walk into it
			}
		}
		// AI_TELELINK (1): just walk in
		// AI_DROP (3): just walk off
		step_height = 0;
	}

	// jump when next waypoint is significantly above us (real ledge, not stairs)
	// engine handles step-up to 18u via SV_WalkMove — only jump for actual ledges
	if (step_height > 30 && self.flags & FL_ONGROUND)
		self.button2 = TRUE;

	// stuck detection: position hasn't changed
	diff = self.origin - self._stuck_origin;
	diff_z = 0;
	if (vlen(diff) < 1)
		self._stuck_count = self._stuck_count + 1;
	else
		self._stuck_count = 0;
	self._stuck_origin = self.origin;

	// stuck recovery: rotate + jump to escape geometry
	if (self._stuck_count > 10)
	{
		local vector dir;
		dir = self.b_wishdir;
		self.b_wishdir_x = dir_y;
		self.b_wishdir_y = dir_x * -1;

		if (self._stuck_count > 20 && self.flags & FL_ONGROUND)
			self.button2 = TRUE;

		if (self._stuck_count > 30)
		{
			self._stuck_count = 0;
			bot_lost(self.target1, FALSE);
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
