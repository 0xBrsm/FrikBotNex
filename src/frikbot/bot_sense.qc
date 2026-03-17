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

	// best direction: potential field blend of all safe directions
	// each safe direction pulls proportional to its score (alignment + clearance)
	// result is a continuous yaw, not one of 8 discrete angles
	self.sense_best_dir = -1;

	local vector accum, pull;
	local float dir_dist, dir_bit, on_hazard;

	on_hazard = (self.sense_ground_type < -3);
	accum = '0 0 0';

	if (self.target1 && self.target1 != world)
	{
		goal_dir = realorigin(self.target1) - self.origin;
		goal_dir_z = 0;
		if (vlen(goal_dir) > 1)
		{
			goal_yaw = vectoyaw(normalize(goal_dir));

			// accumulate weighted pull from each safe direction
			// N
			dir_dist = self.sense_dir_n; dir_bit = 1;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(90, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_0 * score; }
			// NE
			dir_dist = self.sense_dir_ne; dir_bit = 2;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(45, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_1 * score; }
			// E
			dir_dist = self.sense_dir_e; dir_bit = 4;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(0, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_2 * score; }
			// SE
			dir_dist = self.sense_dir_se; dir_bit = 8;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(315, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_3 * score; }
			// S
			dir_dist = self.sense_dir_s; dir_bit = 16;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(270, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_4 * score; }
			// SW
			dir_dist = self.sense_dir_sw; dir_bit = 32;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(225, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_5 * score; }
			// W
			dir_dist = self.sense_dir_w; dir_bit = 64;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(180, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_6 * score; }
			// NW
			dir_dist = self.sense_dir_nw; dir_bit = 128;
			if ((self.sense_floor_safe & dir_bit) && dir_dist > 32)
			{ ang_diff = fabs(angcomp(135, goal_yaw)); score = (1 - ang_diff / 180) * 0.6 + (dir_dist / NAV_RANGE) * 0.4; accum = accum + nav_dirs_7 * score; }

			if (vlen(accum) > 0.001)
				self.sense_best_dir = vectoyaw(accum);
		}
	}

	// hazard escape: if on lava/slime and no direction found,
	// blend ANY safe-floor direction weighted by clearance
	if (on_hazard && self.sense_best_dir == -1)
	{
		accum = '0 0 0';
		if ((self.sense_floor_safe & 1) && self.sense_dir_n > 8) accum = accum + nav_dirs_0 * self.sense_dir_n;
		if ((self.sense_floor_safe & 2) && self.sense_dir_ne > 8) accum = accum + nav_dirs_1 * self.sense_dir_ne;
		if ((self.sense_floor_safe & 4) && self.sense_dir_e > 8) accum = accum + nav_dirs_2 * self.sense_dir_e;
		if ((self.sense_floor_safe & 8) && self.sense_dir_se > 8) accum = accum + nav_dirs_3 * self.sense_dir_se;
		if ((self.sense_floor_safe & 16) && self.sense_dir_s > 8) accum = accum + nav_dirs_4 * self.sense_dir_s;
		if ((self.sense_floor_safe & 32) && self.sense_dir_sw > 8) accum = accum + nav_dirs_5 * self.sense_dir_sw;
		if ((self.sense_floor_safe & 64) && self.sense_dir_w > 8) accum = accum + nav_dirs_6 * self.sense_dir_w;
		if ((self.sense_floor_safe & 128) && self.sense_dir_nw > 8) accum = accum + nav_dirs_7 * self.sense_dir_nw;

		if (vlen(accum) > 0.001)
			self.sense_best_dir = vectoyaw(accum);
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
