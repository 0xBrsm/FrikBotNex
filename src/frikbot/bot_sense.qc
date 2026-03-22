/***********************************************
*                                              *
*         FrikBotNex Spatial Awareness         *
*   "Compute once, read everywhere"            *
*                                              *
***********************************************/

/*

Spatial awareness:

1. bot_sense_space — ground state + forward obstacle detail.
2. bot_sense_visibility — enemy/target LOS, friendly fire.
3. bot_go — set movement direction (feeds velocity pipeline).

*/

// --- Sense system metadata ---
.float sense_time;              // last full update time
.float sense_space_time;        // last geometry probe time

// --- Ground state ---
.float sense_ground_type;       // CONTENT_SOLID, CONTENT_LAVA, etc.
.entity sense_ground_ent;       // entity below (platform/train) or world

// --- Forward probe (velocity-relative, for jump logic) ---
.float sense_fwd_wall_dist;     // distance to wall ahead (0-256)
.float sense_fwd_height;        // height differential of ground ahead
.float sense_fwd_ground_type;   // ground type ahead
.float sense_fwd_gap_width;     // gap width ahead (0 = no gap)
.float sense_fwd_jumpable;      // TRUE if gap/step is jumpable
.vector sense_fwd_normal;       // wall surface normal

// --- Target/enemy visibility cache ---
.float sense_target_vis;
.float sense_enemy_vis;
.float sense_enemy_dist;
.float sense_enemy_wall;
.float sense_my_size;
.float sense_enemy_size;
.float sense_friendly_block;


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_space

Ground state + forward obstacle detail.
Directional safety is handled on-demand by bot_go.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_space =
{
	local vector ang, start, stop, side;
	local float conts, hgt, test;

	if (self.sense_space_time == time)
		return;
	self.sense_space_time = time;

	// ---- ground state (1 trace) ----

	traceline(self.origin, self.origin - '0 0 64', TRUE, self);
	self.sense_ground_ent = trace_ent;
	if (trace_fraction < 1)
		self.sense_ground_type = pointcontents(trace_endpos + '0 0 4');
	else
		self.sense_ground_type = CONTENT_EMPTY;

	// ---- forward probe (velocity-relative, for jump logic) ----

	ang = self.velocity;
	ang_z = 0;

	if (vlen(ang) < 0.1)
	{
		self.sense_fwd_wall_dist = 256;
		self.sense_fwd_height = 0;
		self.sense_fwd_ground_type = CONTENT_SOLID;
		self.sense_fwd_gap_width = 0;
		self.sense_fwd_jumpable = FALSE;
		self.sense_fwd_normal = '0 0 0';
		return;
	}

	ang = normalize(ang);

	// height differential
	start = self.origin + ang * 32;
	start_z = self.origin_z + self.maxs_z;
	stop = start;
	stop_z = self.origin_z + self.mins_z;
	traceline(start, stop - '0 0 256', TRUE, self);

	if (trace_allsolid || trace_startsolid)
		hgt = 0;
	else
		hgt = trace_endpos_z - stop_z;
	self.sense_fwd_height = hgt;

	if (trace_fraction < 1 && !trace_allsolid)
		conts = pointcontents(trace_endpos + '0 0 4');
	else
		conts = CONTENT_EMPTY;
	self.sense_fwd_ground_type = conts;

	// wall distance + gap
	start = self.origin;
	start_z = self.origin_z + self.mins_z - 8;
	stop = start + ang * 256;
	traceline(start, stop, TRUE, self);

	self.sense_fwd_wall_dist = trace_fraction * 256;
	self.sense_fwd_normal = trace_plane_normal;

	if (hgt < 0)
	{
		test = self.sense_fwd_wall_dist;
		if (test > 20)
		{
			side_x = ang_y * -1;
			side_y = ang_x;
			traceline(start - (side * 10), start + (side * 10), TRUE, self);
			if (trace_fraction == 1 && !trace_startsolid)
				self.sense_fwd_gap_width = test;
			else
				self.sense_fwd_gap_width = 0;
		}
		else
			self.sense_fwd_gap_width = 0;
	}
	else
		self.sense_fwd_gap_width = 0;

	// jumpability
	if (self.sense_fwd_gap_width > 0)
	{
		local float jumpable_dist;
		local vector hvel;
		hvel = self.velocity;
		hvel_z = 0;
		jumpable_dist = ((540 / sv_gravity) * vlen(hvel));
		self.sense_fwd_jumpable = (jumpable_dist >= self.sense_fwd_gap_width);
	}
	else if (hgt > 18)
		self.sense_fwd_jumpable = TRUE;
	else
		self.sense_fwd_jumpable = FALSE;

};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_go — "I want to go this direction"

Converts a world-frame yaw to movement keys
relative to the bot's facing direction.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(float desired_yaw) bot_go =
{
	makevectors('0 1 0' * desired_yaw);
	self.b_wishdir = v_forward;
	return TRUE;
};

// ---- b_wishdir → movevect ----
// Converts world-frame desired direction to view-relative movevect.
// Engine's SV_ClientThink reads movevect for friction + acceleration.

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
	if (self.b_aiflags & AI_PRECISION)
		spd = sv_maxspeed * 0.5;

	self.movevect_x = (self.b_wishdir * v_forward) * spd;
	self.movevect_y = (self.b_wishdir * v_right) * spd;
	self.movevect_z = self.b_wishdir_z * spd;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_visibility

Enemy/target LOS, friendly fire, splash potential.
Separate from geometry — this is about entities.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_visibility =
{
	local vector spot1, org, dir, v;

	spot1 = self.origin + self.view_ofs;

	if (self.target1 && self.target1 != world)
	{
		org = realorigin(self.target1);
		traceline(spot1, org, TRUE, self);
		if (trace_ent == self.target1 || trace_fraction == 1)
			self.sense_target_vis = TRUE;
		else
			self.sense_target_vis = FALSE;
	}
	else
		self.sense_target_vis = FALSE;

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


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense

Main entry point. Called once per bot per AI tick.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

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
