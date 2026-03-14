/***********************************************
*                                              *
*         FrikBotNex Spatial Awareness         *
*   "Compute once, read everywhere"            *
*                                              *
***********************************************/

/*

Unified spatial awareness pass that runs once per bot per AI tick.
Replaces scattered, redundant traceline calls across the bot system
with cached entity fields that all subsystems read from.

See AWARENESS.md for the full design document.

*/

// --- Sense system metadata ---
.float sense_time;              // last full update time (avoid redundant runs)
.float sense_ground_time;       // last ground probe time (used as frame guard)
.float sense_fwd_time;          // last forward probe time (used as frame guard)
.float sense_lateral_time;      // last lateral probe time
.float sense_nav_time;          // last local nav probe time

// --- Ground state ---
// Replaces: frik_recognize_plat() trace, frik_check_hazard() center trace,
//           SV_UserFriction() edge trace
.float sense_ground_dist;       // distance to floor directly below (0-64)
.float sense_ground_type;       // CONTENT_SOLID, CONTENT_LAVA, CONTENT_SLIME, CONTENT_WATER
.entity sense_ground_ent;       // entity below (platform/train) or world
.float sense_on_edge;           // true if leading edge is over a dropoff (edge friction)

// --- Forward probe ---
// Replaces: frik_obstacles() traces (lines 310, 326, 333, 362),
//           frik_check_hazard() center ahead trace
.float sense_fwd_wall_dist;     // distance to wall/obstacle ahead (0-256)
.float sense_fwd_height;        // height differential of ground ahead vs current (-256 to +256)
.float sense_fwd_ground_type;   // ground type ahead (solid/lava/slime/void)
.float sense_fwd_gap_width;     // gap width ahead (0 = no gap)
.float sense_fwd_jumpable;      // TRUE if gap/step is jumpable
.vector sense_fwd_normal;       // wall surface normal (for deflection/wall-following)

// --- Hazard probes (ahead + lateral) ---
// Replaces: frik_check_hazard() left/right traces
.float sense_hazard_left;       // TRUE if lava/slime to the left-ahead
.float sense_hazard_right;      // TRUE if lava/slime to the right-ahead

// --- Lateral clearance (NEW -- enables local navigation) ---
.float sense_left_dist;         // distance to wall on left (0-256)
.float sense_right_dist;        // distance to wall on right (0-256)
.vector sense_left_normal;      // left wall normal
.vector sense_right_normal;     // right wall normal

// --- Target/enemy visibility cache ---
// Replaces: fisible() multi-trace calls, bot_fight_style() LOS check,
//           bot_enemy_near_wall() trace
.float sense_target_vis;        // TRUE if target1 is visible
.float sense_enemy_vis;         // TRUE if enemy is visible
.float sense_enemy_dist;        // distance to enemy
.float sense_enemy_wall;        // TRUE if enemy has wall behind them (splash potential)
.float sense_my_size;           // cached bot_size_player(self) for this frame
.float sense_enemy_size;        // cached bot_size_player(enemy) for this frame
.float sense_friendly_block;    // TRUE if teammate in line of fire

// --- Local navigation fan (NEW -- enables waypoint-free intra-region movement) ---
// 8-direction clearance distances at 45-degree intervals
// Uses world frame (N=+Y, E=+X, etc.) for stability across frames
.float sense_nav_n;             // +Y clearance
.float sense_nav_ne;            // +X+Y clearance
.float sense_nav_e;             // +X clearance
.float sense_nav_se;            // +X-Y clearance
.float sense_nav_s;             // -Y clearance
.float sense_nav_sw;            // -X-Y clearance
.float sense_nav_w;             // -X clearance
.float sense_nav_nw;            // -X+Y clearance
.float sense_nav_best_dir;      // computed best direction toward goal (yaw angle, -1 = none)
.float sense_nav_openness;      // 0.0 = tight corridor, 1.0 = open room


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_ground

Ground state probing: floor distance, floor type,
platform detection, and edge detection.

Traces: 2 (replaces 5-6 scattered traces)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_ground =
{
	local vector start, stop;
	local float conts;

	if (self.sense_ground_time == time)
		return;     // already ran this frame
	self.sense_ground_time = time;

	// Trace 1: straight down from origin to detect floor
	traceline(self.origin, self.origin - '0 0 64', TRUE, self);

	self.sense_ground_dist = (1 - trace_fraction) * 64;
	self.sense_ground_ent = trace_ent;

	// determine ground content type
	if (trace_fraction < 1)
	{
		conts = pointcontents(trace_endpos + '0 0 4');
		self.sense_ground_type = conts;
	}
	else
		self.sense_ground_type = CONTENT_EMPTY;

	// Trace 2: edge detection (leading velocity edge, down 34u)
	// Matches SV_UserFriction logic: trace from leading edge downward
	if (self.flags & FL_ONGROUND)
	{
		local vector vel;
		local float sped;
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
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_forward

Forward obstacle analysis: wall distance, height
differential, gap detection, hazard probing.

Traces: 3-5 (replaces 6-8 scattered traces)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_forward =
{
	local vector ang, start, stop, check, side;
	local float conts, hgt, test;

	if (self.sense_fwd_time == time)
		return;     // already ran this frame
	self.sense_fwd_time = time;

	ang = self.velocity;
	ang_z = 0;

	// need meaningful velocity to probe forward
	if (vlen(ang) < 0.1)
	{
		// no velocity: reset forward sense data
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

	// Trace 1: Forward at head height, then down -- detect height differential
	start = self.origin + ang * 32;
	start_z = self.origin_z + self.maxs_z;
	stop = start;
	stop_z = self.origin_z + self.mins_z;
	traceline(start, stop - '0 0 256', TRUE, self);

	if (trace_allsolid || trace_startsolid)
	{
		self.sense_fwd_height = 0;
		hgt = 0;
	}
	else
	{
		hgt = trace_endpos_z - stop_z;
		self.sense_fwd_height = hgt;
	}

	// get content of ground ahead from trace 1
	if (trace_fraction < 1 && !trace_allsolid)
		conts = pointcontents(trace_endpos + '0 0 4');
	else
		conts = CONTENT_EMPTY;
	self.sense_fwd_ground_type = conts;

	// Trace 2: Forward at feet level -- measure gap and wall distance
	start = self.origin;
	start_z = self.origin_z + self.mins_z - 8;
	stop = start + ang * 256;
	traceline(start, stop, TRUE, self);

	self.sense_fwd_wall_dist = vlen(trace_endpos - start);
	self.sense_fwd_normal = trace_plane_normal;

	// gap detection: if height differential is negative, we may have a gap
	if (hgt < 0)
	{
		test = self.sense_fwd_wall_dist;
		if (test > 20)
		{
			// check gap width with lateral sweep
			side_x = ang_y * -1;
			side_y = ang_x;
			traceline(start - (side * 10), start + (side * 10), TRUE, self);
			if (trace_fraction == 1 && !trace_startsolid)
			{
				// gap is wider than 20 units
				self.sense_fwd_gap_width = test;
			}
			else
				self.sense_fwd_gap_width = 0;
		}
		else
			self.sense_fwd_gap_width = 0;
	}
	else
		self.sense_fwd_gap_width = 0;

	// jumpability: pure math from gap width and velocity
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
		self.sense_fwd_jumpable = TRUE;  // step up, need jump
	else
		self.sense_fwd_jumpable = FALSE;

	// Traces 3-4: Hazard probes (left-ahead, right-ahead)
	// Only when on ground -- skip when airborne
	if (self.flags & FL_ONGROUND)
	{
		side_x = ang_y * -1;
		side_y = ang_x;

		// left-ahead
		check = self.origin + ang * 48 + side * 24;
		check_z = self.origin_z + self.mins_z;
		traceline(check, check - '0 0 64', TRUE, self);
		if (trace_fraction < 1)
		{
			conts = pointcontents(trace_endpos + '0 0 4');
			self.sense_hazard_left = (conts < -3);
		}
		else
			self.sense_hazard_left = FALSE;

		// right-ahead
		check = self.origin + ang * 48 - side * 24;
		check_z = self.origin_z + self.mins_z;
		traceline(check, check - '0 0 64', TRUE, self);
		if (trace_fraction < 1)
		{
			conts = pointcontents(trace_endpos + '0 0 4');
			self.sense_hazard_right = (conts < -3);
		}
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

bot_sense_lateral

Left/right clearance probing. Gives corridor
awareness for dodge maneuvering and local nav.

Traces: 2 (new capability)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_lateral =
{
	local vector ang, left, right, start;

	ang = self.velocity;
	ang_z = 0;

	if (vlen(ang) < 0.1)
	{
		// no velocity: use facing direction
		makevectors(self.v_angle);
		ang = v_forward;
		ang_z = 0;
	}

	ang = normalize(ang);

	// perpendicular vectors
	left_x = ang_y;
	left_y = ang_x * -1;
	right_x = ang_y * -1;
	right_y = ang_x;

	start = self.origin;

	// Trace 1: left clearance
	traceline(start, start + left * 256, TRUE, self);
	self.sense_left_dist = vlen(trace_endpos - start);
	self.sense_left_normal = trace_plane_normal;

	// Trace 2: right clearance
	traceline(start, start + right * 256, TRUE, self);
	self.sense_right_dist = vlen(trace_endpos - start);
	self.sense_right_normal = trace_plane_normal;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_visibility

Cache LOS checks for current enemy and nav
target. Also check friendly fire and enemy
wall proximity for splash damage.

Traces: 2-4 (replaces 6-10 scattered traces)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_visibility =
{
	local vector spot1, org, dir, v;

	spot1 = self.origin + self.view_ofs;

	// --- target1 visibility ---
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

	// --- enemy visibility + distance ---
	if (self.enemy && self.enemy != world)
	{
		org = realorigin(self.enemy);
		self.sense_enemy_dist = vlen(org - self.origin);

		traceline(spot1, org, TRUE, self);
		if (trace_ent == self.enemy || trace_fraction == 1)
			self.sense_enemy_vis = TRUE;
		else
		{
			// corner check for player-sized enemies (matching fisible behavior)
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

		// --- enemy wall proximity (splash potential) ---
		// only compute in combat
		if (self.sense_enemy_vis)
		{
			dir = normalize(org - self.origin);
			traceline(org, org + dir * 100, TRUE, self.enemy);
			self.sense_enemy_wall = (trace_fraction < 1);
		}
		else
			self.sense_enemy_wall = FALSE;

		// --- friendly fire check ---
		// only in combat with teamplay or coop
		if (self.sense_enemy_vis && (teamplay || coop))
		{
			makevectors(self.v_angle);
			v = weapon_range(self.weapon);
			traceline(spot1, spot1 + v_forward * v_z, FALSE, self);
			if (trace_ent.classname == "player")
			{
				if ((trace_ent.team == self.team && teamplay) || coop)
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
	{
		self.sense_enemy_vis = FALSE;
		self.sense_enemy_dist = 0;
		self.sense_enemy_wall = FALSE;
		self.sense_friendly_block = FALSE;
	}
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense_local_nav

8-direction fan traces for intra-region
navigation. Provides clearance distances in
all compass directions plus computed best
direction toward goal and openness score.

Traces: 8 (new capability)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense_local_nav =
{
	local vector start, dir;
	local float total_dist, best_score, goal_yaw, dir_yaw;
	local float score, dist, ang_diff;
	local vector goal_dir;

	start = self.origin;

	// N (+Y)
	dir = '0 1 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_n = vlen(trace_endpos - start);

	// NE (+X+Y)
	dir = '0.707 0.707 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_ne = vlen(trace_endpos - start);

	// E (+X)
	dir = '1 0 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_e = vlen(trace_endpos - start);

	// SE (+X-Y)
	dir = '0.707 -0.707 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_se = vlen(trace_endpos - start);

	// S (-Y)
	dir = '0 -1 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_s = vlen(trace_endpos - start);

	// SW (-X-Y)
	dir = '-0.707 -0.707 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_sw = vlen(trace_endpos - start);

	// W (-X)
	dir = '-1 0 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_w = vlen(trace_endpos - start);

	// NW (-X+Y)
	dir = '-0.707 0.707 0';
	traceline(start, start + dir * 256, TRUE, self);
	self.sense_nav_nw = vlen(trace_endpos - start);

	// compute openness: average clearance normalized to 0-1
	total_dist = self.sense_nav_n + self.sense_nav_ne + self.sense_nav_e
		+ self.sense_nav_se + self.sense_nav_s + self.sense_nav_sw
		+ self.sense_nav_w + self.sense_nav_nw;
	self.sense_nav_openness = total_dist / (256 * 8);

	// compute best direction toward goal
	self.sense_nav_best_dir = -1;

	if (self.target1 && self.target1 != world)
	{
		goal_dir = realorigin(self.target1) - self.origin;
		goal_dir_z = 0;
		if (vlen(goal_dir) > 1)
		{
			goal_dir = normalize(goal_dir);
			goal_yaw = vectoyaw(goal_dir);
			best_score = -1;

			// score each of the 8 directions:
			// weighted combination of (a) alignment with goal and (b) clearance

			// N = yaw 90
			dir_yaw = 90;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_n;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// NE = yaw 45
			dir_yaw = 45;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_ne;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// E = yaw 0
			dir_yaw = 0;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_e;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// SE = yaw 315
			dir_yaw = 315;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_se;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// S = yaw 270
			dir_yaw = 270;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_s;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// SW = yaw 225
			dir_yaw = 225;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_sw;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// W = yaw 180
			dir_yaw = 180;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_w;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }

			// NW = yaw 135
			dir_yaw = 135;
			ang_diff = fabs(angcomp(dir_yaw, goal_yaw));
			dist = self.sense_nav_nw;
			score = (1 - ang_diff / 180) * 0.6 + (dist / 256) * 0.4;
			if (dist > 32 && score > best_score) { best_score = score; self.sense_nav_best_dir = dir_yaw; }
		}
	}
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_sense

Main entry point. Called once per bot per AI
tick from BotAI(), before any decision-making.

Amortizes expensive probes across frames:
- Ground + forward: every frame
- Lateral: every ~2 frames
- Visibility: every frame (combat-critical)
- Local nav fan: every ~3 frames

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_sense =
{
	if (self.sense_time == time)
		return;     // already ran this frame
	self.sense_time = time;

	// cache per-frame size calculations (called 4-6x otherwise)
	self.sense_my_size = bot_size_player(self);
	if (self.enemy && self.enemy != world)
		self.sense_enemy_size = bot_size_player(self.enemy);
	else
		self.sense_enemy_size = 0;

	bot_sense_ground();                      // every frame
	bot_sense_forward();                     // every frame

	if (time > self.sense_lateral_time)      // every 2nd frame
	{
		bot_sense_lateral();
		self.sense_lateral_time = time + 0.028;  // ~2 frames at 72fps
	}

	bot_sense_visibility();                  // every frame (combat-critical)

	if (time > self.sense_nav_time)          // every 3rd frame
	{
		bot_sense_local_nav();
		self.sense_nav_time = time + 0.042;  // ~3 frames at 72fps
	}
};
