/***********************************************
*                                              *
*         FrikBotNex Spatial Awareness         *
*   "Compute once, read everywhere"            *
*                                              *
***********************************************/

/*

Unified spatial awareness. Two passes per AI tick:

1. bot_sense_space — omnidirectional geometry awareness
   8-direction nav fan (wall clearance + floor safety)
   plus ground state and forward obstacle detail.

2. bot_sense_visibility — enemy/target LOS, friendly fire.

*/

// --- Sense system metadata ---
.float sense_time;              // last full update time
.float sense_space_time;        // last geometry probe time

// --- Ground state ---
.float sense_ground_dist;       // distance to floor directly below (0-64)
.float sense_ground_type;       // CONTENT_SOLID, CONTENT_LAVA, etc.
.entity sense_ground_ent;       // entity below (platform/train) or world
.float sense_on_edge;           // true if leading edge is over a dropoff

// --- 8-direction nav fan ---
// Wall clearance (0-256) per compass direction (world frame)
.float sense_dir_n;             // +Y
.float sense_dir_ne;            // +X+Y
.float sense_dir_e;             // +X
.float sense_dir_se;            // +X-Y
.float sense_dir_s;             // -Y
.float sense_dir_sw;            // -X-Y
.float sense_dir_w;             // -X
.float sense_dir_nw;            // -X+Y

// Floor safety per direction (bitmask: bit set = floor is safe)
.float sense_floor_safe;        // 8 bits: N=1, NE=2, E=4, SE=8, S=16, SW=32, W=64, NW=128

// Aggregates
.float sense_openness;          // average clearance / 256 (0=boxed in, 1=open)
.float sense_best_dir;          // best yaw toward goal considering clearance + floor (-1=none)

// --- Forward probe (velocity-relative, for frik_obstacles) ---
.float sense_fwd_wall_dist;     // distance to wall ahead (0-256)
.float sense_fwd_height;        // height differential of ground ahead
.float sense_fwd_ground_type;   // ground type ahead
.float sense_fwd_gap_width;     // gap width ahead (0 = no gap)
.float sense_fwd_jumpable;      // TRUE if gap/step is jumpable
.vector sense_fwd_normal;       // wall surface normal

// --- Hazard probes (velocity-relative) ---
.float sense_hazard_left;       // TRUE if lava/slime left-ahead
.float sense_hazard_right;      // TRUE if lava/slime right-ahead

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

Omnidirectional geometry awareness.
Replaces bot_sense_ground + bot_sense_forward.

Per direction: horizontal trace (clearance) +
trace down at 48u ahead (floor safety).

Plus: ground state, edge detection, forward
obstacle detail for frik_obstacles.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// floor check distance ahead per direction
float FLOOR_CHECK_DIST = 48;
// nav fan trace range
float NAV_RANGE = 256;

// direction vectors (world frame, precomputed)
// N, NE, E, SE, S, SW, W, NW
vector nav_dirs_0, nav_dirs_1, nav_dirs_2, nav_dirs_3;
vector nav_dirs_4, nav_dirs_5, nav_dirs_6, nav_dirs_7;
float nav_dirs_init;

void() nav_init_dirs =
{
	nav_dirs_0 = '0 1 0';
	nav_dirs_1 = '0.707 0.707 0';
	nav_dirs_2 = '1 0 0';
	nav_dirs_3 = '0.707 -0.707 0';
	nav_dirs_4 = '0 -1 0';
	nav_dirs_5 = '-0.707 -0.707 0';
	nav_dirs_6 = '-1 0 0';
	nav_dirs_7 = '-0.707 0.707 0';
	nav_dirs_init = TRUE;
};

// probe one direction: returns clearance, sets floor safety bit
float(vector dir, float bit) nav_probe =
{
	local vector start, check;
	local float dist, conts;

	// wall clearance: horizontal trace at origin
	traceline(self.origin, self.origin + dir * NAV_RANGE, TRUE, self);
	dist = trace_fraction * NAV_RANGE;

	// floor safety: trace down at FLOOR_CHECK_DIST ahead (or wall dist if closer)
	if (dist > FLOOR_CHECK_DIST)
		check = self.origin + dir * FLOOR_CHECK_DIST;
	else if (dist > 16)
		check = self.origin + dir * (dist - 8);
	else
	{
		// too close to wall, floor check at our feet
		self.sense_floor_safe = self.sense_floor_safe | bit;  // assume safe near wall
		return dist;
	}

	check_z = self.origin_z + self.mins_z;
	traceline(check, check - '0 0 64', TRUE, self);

	if (trace_fraction < 1)
	{
		conts = pointcontents(trace_endpos + '0 0 4');
		if (conts >= -3)  // CONTENT_SOLID, CONTENT_WATER, or CONTENT_SKY are >= -3
			self.sense_floor_safe = self.sense_floor_safe | bit;
	}
	// else: no floor = unsafe, bit stays unset

	return dist;
};

void() bot_sense_space =
{
	local vector ang, start, stop, side, check;
	local float conts, hgt, test, total, sped;
	local float goal_yaw, best_score, score, dist, ang_diff;
	local vector goal_dir;

	if (self.sense_space_time == time)
		return;
	self.sense_space_time = time;

	if (!nav_dirs_init)
		nav_init_dirs();

	// ---- ground state ----

	// trace straight down
	traceline(self.origin, self.origin - '0 0 64', TRUE, self);
	self.sense_ground_dist = (1 - trace_fraction) * 64;
	self.sense_ground_ent = trace_ent;
	if (trace_fraction < 1)
		self.sense_ground_type = pointcontents(trace_endpos + '0 0 4');
	else
		self.sense_ground_type = CONTENT_EMPTY;

	// edge detection (velocity-relative)
	if (self.flags & FL_ONGROUND)
	{
		local vector vel;
		vel = self.velocity;
		vel_z = 0;
		sped = vlen(vel);
		if (sped > 0)
		{
			start_x = self.origin_x + vel_x * 16 / sped;
			start_y = self.origin_y + vel_y * 16 / sped;
			start_z = self.origin_z + self.mins_z;
			stop = start;
			stop_z = start_z - 34;
			traceline(start, stop, TRUE, self);
			self.sense_on_edge = (trace_fraction == 1);
		}
		else
			self.sense_on_edge = FALSE;
	}
	else
		self.sense_on_edge = FALSE;

	// ---- 8-direction nav fan ----

	self.sense_floor_safe = 0;  // clear all bits

	self.sense_dir_n  = nav_probe(nav_dirs_0, 1);
	self.sense_dir_ne = nav_probe(nav_dirs_1, 2);
	self.sense_dir_e  = nav_probe(nav_dirs_2, 4);
	self.sense_dir_se = nav_probe(nav_dirs_3, 8);
	self.sense_dir_s  = nav_probe(nav_dirs_4, 16);
	self.sense_dir_sw = nav_probe(nav_dirs_5, 32);
	self.sense_dir_w  = nav_probe(nav_dirs_6, 64);
	self.sense_dir_nw = nav_probe(nav_dirs_7, 128);

	// openness
	total = self.sense_dir_n + self.sense_dir_ne + self.sense_dir_e + self.sense_dir_se
		  + self.sense_dir_s + self.sense_dir_sw + self.sense_dir_w + self.sense_dir_nw;
	self.sense_openness = total / (NAV_RANGE * 8);

	// best direction toward goal (clearance + floor safety + goal alignment)
	self.sense_best_dir = -1;

	// helper: score one direction (defined inline via locals since QC has no closures)
	// scoring: 60% goal alignment + 40% clearance. Requires safe floor + min clearance.
	local float nav_yaws_0, nav_yaws_1, nav_yaws_2, nav_yaws_3;
	local float nav_yaws_4, nav_yaws_5, nav_yaws_6, nav_yaws_7;
	local float nav_dists_0, nav_dists_1, nav_dists_2, nav_dists_3;
	local float nav_dists_4, nav_dists_5, nav_dists_6, nav_dists_7;
	local float nav_bits_0, nav_bits_1, nav_bits_2, nav_bits_3;
	local float nav_bits_4, nav_bits_5, nav_bits_6, nav_bits_7;
	local float i, dir_yaw, dir_dist, dir_bit;
	local float on_hazard;

	nav_yaws_0 = 90;  nav_dists_0 = self.sense_dir_n;  nav_bits_0 = 1;
	nav_yaws_1 = 45;  nav_dists_1 = self.sense_dir_ne; nav_bits_1 = 2;
	nav_yaws_2 = 0;   nav_dists_2 = self.sense_dir_e;  nav_bits_2 = 4;
	nav_yaws_3 = 315; nav_dists_3 = self.sense_dir_se; nav_bits_3 = 8;
	nav_yaws_4 = 270; nav_dists_4 = self.sense_dir_s;  nav_bits_4 = 16;
	nav_yaws_5 = 225; nav_dists_5 = self.sense_dir_sw; nav_bits_5 = 32;
	nav_yaws_6 = 180; nav_dists_6 = self.sense_dir_w;  nav_bits_6 = 64;
	nav_yaws_7 = 135; nav_dists_7 = self.sense_dir_nw; nav_bits_7 = 128;

	on_hazard = (self.sense_ground_type < -3);

	if (self.target1 && self.target1 != world)
	{
		goal_dir = realorigin(self.target1) - self.origin;
		goal_dir_z = 0;
		if (vlen(goal_dir) > 1)
		{
			goal_yaw = vectoyaw(normalize(goal_dir));
			best_score = -1;

			// score all 8 directions
			i = 0;
			while (i < 8)
			{
				if (i == 0) { dir_yaw = nav_yaws_0; dir_dist = nav_dists_0; dir_bit = nav_bits_0; }
				else if (i == 1) { dir_yaw = nav_yaws_1; dir_dist = nav_dists_1; dir_bit = nav_bits_1; }
				else if (i == 2) { dir_yaw = nav_yaws_2; dir_dist = nav_dists_2; dir_bit = nav_bits_2; }
				else if (i == 3) { dir_yaw = nav_yaws_3; dir_dist = nav_dists_3; dir_bit = nav_bits_3; }
				else if (i == 4) { dir_yaw = nav_yaws_4; dir_dist = nav_dists_4; dir_bit = nav_bits_4; }
				else if (i == 5) { dir_yaw = nav_yaws_5; dir_dist = nav_dists_5; dir_bit = nav_bits_5; }
				else if (i == 6) { dir_yaw = nav_yaws_6; dir_dist = nav_dists_6; dir_bit = nav_bits_6; }
				else { dir_yaw = nav_yaws_7; dir_dist = nav_dists_7; dir_bit = nav_bits_7; }

				if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
				{
					ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
					score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4;
					if (score > best_score)
					{
						best_score = score;
						self.sense_best_dir = dir_yaw;
					}
				}
				i = i + 1;
			}
		}
	}

	// hazard escape: if on lava/slime and no direction found, pick ANY safe floor direction
	if (on_hazard && self.sense_best_dir == -1)
	{
		best_score = -1;
		i = 0;
		while (i < 8)
		{
			if (i == 0) { dir_yaw = nav_yaws_0; dir_dist = nav_dists_0; dir_bit = nav_bits_0; }
			else if (i == 1) { dir_yaw = nav_yaws_1; dir_dist = nav_dists_1; dir_bit = nav_bits_1; }
			else if (i == 2) { dir_yaw = nav_yaws_2; dir_dist = nav_dists_2; dir_bit = nav_bits_2; }
			else if (i == 3) { dir_yaw = nav_yaws_3; dir_dist = nav_dists_3; dir_bit = nav_bits_3; }
			else if (i == 4) { dir_yaw = nav_yaws_4; dir_dist = nav_dists_4; dir_bit = nav_bits_4; }
			else if (i == 5) { dir_yaw = nav_yaws_5; dir_dist = nav_dists_5; dir_bit = nav_bits_5; }
			else if (i == 6) { dir_yaw = nav_yaws_6; dir_dist = nav_dists_6; dir_bit = nav_bits_6; }
			else { dir_yaw = nav_yaws_7; dir_dist = nav_dists_7; dir_bit = nav_bits_7; }

			// any safe floor, any clearance > 8
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 8)
			{
				if (dir_dist > best_score)
				{
					best_score = dir_dist;
					self.sense_best_dir = dir_yaw;
				}
			}
			i = i + 1;
		}
	}

	// ---- forward probe (velocity-relative, for frik_obstacles) ----

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
		self.sense_hazard_left = FALSE;
		self.sense_hazard_right = FALSE;
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

	// hazard probes
	if (self.flags & FL_ONGROUND)
	{
		side_x = ang_y * -1;
		side_y = ang_x;

		check = self.origin + ang * 48 + side * 24;
		check_z = self.origin_z + self.mins_z;
		traceline(check, check - '0 0 64', TRUE, self);
		if (trace_fraction < 1)
			self.sense_hazard_left = (pointcontents(trace_endpos + '0 0 4') < -3);
		else
			self.sense_hazard_left = FALSE;

		check = self.origin + ang * 48 - side * 24;
		check_z = self.origin_z + self.mins_z;
		traceline(check, check - '0 0 64', TRUE, self);
		if (trace_fraction < 1)
			self.sense_hazard_right = (pointcontents(trace_endpos + '0 0 4') < -3);
		else
			self.sense_hazard_right = FALSE;
	}
	else
	{
		self.sense_hazard_left = FALSE;
		self.sense_hazard_right = FALSE;
	}
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Sense API — read the nav fan by yaw angle.
Snaps to nearest 45-degree compass direction.
No new traces, just reads cached data.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// snap a yaw to nearest fan index (0-7) and return clearance
float(float yaw) sense_clearance =
{
	local float idx;
	yaw = yaw - floor(yaw / 360) * 360;  // normalize to 0-360
	idx = rint(yaw / 45);
	if (idx > 7) idx = 0;

	if (idx == 0) return self.sense_dir_e;
	if (idx == 1) return self.sense_dir_ne;
	if (idx == 2) return self.sense_dir_n;
	if (idx == 3) return self.sense_dir_nw;
	if (idx == 4) return self.sense_dir_w;
	if (idx == 5) return self.sense_dir_sw;
	if (idx == 6) return self.sense_dir_s;
	if (idx == 7) return self.sense_dir_se;
	return 0;
};

// snap a yaw to nearest fan index and return floor safety
float(float yaw) sense_floor_ok =
{
	local float idx, bit;
	yaw = yaw - floor(yaw / 360) * 360;
	idx = rint(yaw / 45);
	if (idx > 7) idx = 0;

	// bit order: N=1, NE=2, E=4, SE=8, S=16, SW=32, W=64, NW=128
	if (idx == 0) bit = 4;    // E
	if (idx == 1) bit = 2;    // NE
	if (idx == 2) bit = 1;    // N
	if (idx == 3) bit = 128;  // NW
	if (idx == 4) bit = 64;   // W
	if (idx == 5) bit = 32;   // SW
	if (idx == 6) bit = 16;   // S
	if (idx == 7) bit = 8;    // SE

	return (self.sense_floor_safe & bit);
};

// is a direction walkable? (has clearance AND safe floor)
float(float yaw) sense_walkable =
{
	if (sense_clearance(yaw) > 32 && sense_floor_ok(yaw))
		return TRUE;
	return FALSE;
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
