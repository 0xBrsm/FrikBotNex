/***********************************************
*                                              *
*            FrikBot Movement AI               *
*     "The slightly better movement AI"        *
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

void() bot_jump =
{
	// TODO check for precision, etc.
	self.button2 = TRUE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_wants_to_drop

Unified check: is the bot intentionally navigating
to a target below? Returns TRUE if the bot should
be allowed to step off the current edge/ledge.

Used by edge detection, hazard scanning, and gap
detection so navigation intent is evaluated in
one place with one set of rules.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_wants_to_drop =
{
	local vector stop, dir, vel;

	if (!self.target1)
		return FALSE;

	stop = realorigin(self.target1);

	// target must be significantly below us
	if ((stop_z - self.origin_z) >= -32)
		return FALSE;

	// target must be roughly in our movement direction
	// (don't walk off the wrong edge)
	dir = stop - self.origin;
	dir_z = 0;
	vel = self.velocity;
	vel_z = 0;
	if (vlen(vel) > 0.1 && vlen(dir) > 0.1)
	{
		if (dir * normalize(vel) < 0)
			return FALSE;  // target is behind us
	}

	return TRUE;
};

float(entity e) bot_can_rj =
{
	// this returns true of the bot can rocket/superjump/hook
	// if your mod doesn't have an RL you can just return FALSE all the time
	// if it has a hook or some other means for the bot to get to high places
	// you can check here for that capability

	// am I dumb?
	if (e.b_skill == 0)
		return FALSE;

	// quad = bad
	if (e.items & 4194304)
		return FALSE;

	// do I have rockets & RL?
	if (!((e.items & 32) && (e.ammo_rockets > 0)))
		return FALSE;

	// do I have pent?
	if (e.items & 1048576)
		return TRUE;

	if (e.health > 50)
		return TRUE;
	else
		return FALSE;
};

float(float flag) frik_recognize_plat =
{
	local entity plat_ent;

	if ((self.classname != "waypoint") && !(self.flags & FL_ONGROUND))
		return FALSE;

	// use sense cache for bot entities; waypoints still trace directly
	if (self.classname == "waypoint")
	{
		traceline(self.origin, self.origin - '0 0 64', TRUE, self);
		plat_ent = trace_ent;
	}
	else
		plat_ent = self.sense_ground_ent;

	if (plat_ent != world)
	{
		if (flag) // affect bot movement too
		{
			if (self.keys & KEY_MOVEUP)
			{
				if (plat_ent.velocity_z > 0)
					self.keys = self.keys & 960; // 960 is all view keys
			}
			else if (self.keys & KEY_MOVEDOWN)
			{
				if (plat_ent.velocity_z < 0)
					self.keys = self.keys & 960;
			}
		}
		return TRUE;
	}
	else
		return FALSE;
};

float(vector sdir) frik_KeysForDir =
{

	local vector keydir;
	local float outkeys, tang;
	outkeys = 0;
	if (sdir_x || sdir_y)
	{
		// Everything is tested against 60 degrees,
		// this allows the bot to overlap the keys
		// 30 degrees on each diagonal  45 degrees 
		// might look more realistic

 		keydir = vectoangles(sdir);
		tang = angcomp(keydir_y, self.v_angle_y);
		if ((tang <= 150) && (tang >= 30))
			outkeys = outkeys + KEY_MOVELEFT;
		else if ((tang >= -150) && (tang <= -30))
			outkeys = outkeys + KEY_MOVERIGHT;
		if (fabs(tang) <= 60)
			outkeys = outkeys + KEY_MOVEFORWARD;
		else if (fabs(tang) >= 120)
			outkeys = outkeys + KEY_MOVEBACK;
	}
	if (sdir_z > 0)
		outkeys = outkeys + KEY_MOVEUP;
	else if (sdir_z < 0)
		outkeys = outkeys + KEY_MOVEDOWN;
	return outkeys;

};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_obstructed

Bot has hit a ledge or wall that he should
manuever around.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void(vector whichway, float danger) frik_obstructed =
{
	local float dist, stuck_time;
	local vector disway, org;

	if (self.b_aiflags & AI_BLIND)
		return;

	if (danger)
	{
		self.b_aiflags = self.b_aiflags | AI_DANGER;
		self.keys = frik_KeysForDir('0 0 0' - whichway);
	}
	if (self.b_aiflags & AI_PRECISION)
		return;

	if (self.b_aiflags & AI_OBSTRUCTED)
	{
		stuck_time = time - self.obs_time;

		// commitment window: don't reset dodge direction too quickly
		// this prevents corner oscillation where hitting alternating
		// walls causes the bot to flip direction every frame
		if (stuck_time < 0.4)
		{
			// within commitment — only upgrade to danger, keep direction
			if (danger && !(self.b_aiflags & AI_DANGER))
				self.b_aiflags = self.b_aiflags | AI_DANGER;
			return;
		}

		// stuck too long (>0.8s) — try the other side
		if (stuck_time > 0.8)
		{
			self.wallhug = !self.wallhug;
			self.obs_time = time;
			return;
		}

		// between 0.4-0.8s: allow fresh direction calculation below
	}

	// compute dodge direction (left or right of obstacle)
	self.obs_dir = whichway;
	self.obs_time = time;

	if (self.target1)
	{
		org = realorigin(self.target1);
		disway_x = whichway_y * -1;
		disway_y = whichway_x;
		dist = vlen(org - (self.origin + disway));
		disway_x = whichway_y;
		disway_y = whichway_x * -1;
		self.wallhug = vlen(org - (self.origin + disway)) > dist;
	}
	else
	{
		disway_x = whichway_y * -1;
		disway_y = whichway_x;
		dist = vlen(disway - self.obs_dir);
		disway_x = whichway_y;
		disway_y = whichway_x * -1;
		self.wallhug = vlen(disway - self.obs_dir) < dist;
	}

	self.b_aiflags = self.b_aiflags | AI_OBSTRUCTED;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_obstacles

Detects small bumps the bot needs to jump over
or ledges the bot should avoid falling in.

Also responsible for jumping gaps.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_check_hazard

Proactive hazard scanning: checks the ground
ahead and to the sides for lava/slime.
Returns TRUE and calls frik_obstructed if
a hazard is detected.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float drop_ok;  // cached result of bot_wants_to_drop() for current frame

float() frik_check_hazard =
{
	local vector ang, side;

	if (!(self.flags & FL_ONGROUND))
		return FALSE;

	ang = self.velocity;
	ang_z = 0;
	if (vlen(ang) < 0.1)
		return FALSE;
	ang = normalize(ang);

	// read from sense cache instead of tracing
	// center-ahead hazard (but defer to navigation intent for drops)
	if (self.sense_fwd_ground_type < -3)
	{
		if (!drop_ok)
		{
			frik_obstructed(ang, TRUE);
			return TRUE;
		}
	}

	side_x = ang_y * -1;
	side_y = ang_x;

	// left-ahead hazard
	if (self.sense_hazard_left)
	{
		frik_obstructed(ang + side, TRUE);
		return TRUE;
	}

	// right-ahead hazard
	if (self.sense_hazard_right)
	{
		frik_obstructed(ang - side, TRUE);
		return TRUE;
	}

	return FALSE;
};

void() frik_obstacles =
{
	local vector ang;
	local float hgt, conts, gap, dist;

	if (!(self.flags & FL_ONGROUND))
		return;
	if (self.b_aiflags & AI_BLIND)
		return;

	// cache bot_wants_to_drop for this frame (called up to 8x otherwise)
	drop_ok = bot_wants_to_drop();

	// refresh safety-critical sense data every physics frame
	// (bot_sense() runs inside BotAI which is AFTER frik_obstacles
	// in the frame, and for skill 0-1 is throttled by ai_time,
	// so the cache can be multiple frames stale — too late for
	// hazard avoidance at full run speed)
	bot_sense_ground();
	bot_sense_forward();

	// proactive hazard scan (lava/slime ahead and to sides)
	if (frik_check_hazard())
		return;

	// edge detection: bot's leading edge is over a dropoff
	// last-resort brake before falling into lava/slime/void
	if (self.sense_on_edge)
	{
		ang = self.velocity;
		ang_z = 0;
		if (vlen(ang) > 0.1)
		{
			conts = self.sense_fwd_ground_type;
			if (conts < -3 || conts == CONTENT_EMPTY)
			{
				// navigating to a target below? trust pathfinding
				if (drop_ok)
					return;

				// lava, slime, or void ahead — hard stop
				frik_obstructed(normalize(ang), TRUE);
				return;
			}
		}
	}

	// read pre-computed sense data instead of tracing
	hgt = self.sense_fwd_height;
	conts = self.sense_fwd_ground_type;
	gap = self.sense_fwd_gap_width;

	if (hgt > 18)
	{
		bot_jump();
		return;
	}
	if (hgt >= 0)
		return;

	// gap detection: use sense cache
	if (gap <= 0)
		return; // no gap or walkable gap

	ang = self.velocity;
	ang_z = 0;

	if (!self.sense_fwd_jumpable) // I can't make it
	{
		if (conts < -3) // bad stuff down there
		{
			if (drop_ok)
				return; // navigation says drop — trust it
			frik_obstructed(normalize(ang), TRUE);
			return;
		}
		else
		{
			if (drop_ok)
				return; // safe to fall toward target
			frik_obstructed(normalize(ang), FALSE);
			return;
		}
	}
	else
	{
		// I can make it -- check for a ledge blocking path
		dist = gap;
		ang = normalize(ang);
		traceline(self.origin, self.origin + (ang * (dist + 20)), TRUE, self);
		if (trace_fraction != 1)
		{
			if (conts < -3)
			{
				if (drop_ok)
					return;
				frik_obstructed(ang, TRUE);
				return;
			}
			else
			{
				if (drop_ok)
					return; // safe to fall toward target
				frik_obstructed(ang, FALSE);
				return;
			}
		}

		if (self.target1)
		{
			// getting further away from my target?
			dist = vlen(self.target1.origin - (ang + self.origin));
			if (dist > vlen(self.target1.origin - self.origin))
			{
				if (conts < -3)
				{
					frik_obstructed(ang, TRUE);
					return;
				}
				else
				{
					frik_obstructed(ang, FALSE);
					return;
				}
			}
		}
	}
	if (hgt < -18)
	{
		if (conts < -3)
		{
			if (drop_ok)
				return;
			frik_obstructed(normalize(self.velocity), TRUE);
			return;
		}
		if (drop_ok)
			return; // safe to fall toward target
		bot_jump();
	}
	// go for it
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

After frik_obstructed, the bot uses the 
following funtion to move "around" the obstacle

I have no idea how well it will work

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() frik_dodge_obstruction =
{
	local vector way, org;
	local float oflags, yaw;

	if (!(self.b_aiflags & AI_OBSTRUCTED))
		return;
	if ((self.b_aiflags & (AI_BLIND | AI_PRECISION)) || !(self.flags & FL_ONGROUND))
	{
		self.b_aiflags = self.b_aiflags - (self.b_aiflags & (AI_OBSTRUCTED | AI_DANGER));
		return;
	}

	// perform a walkmove check to see if the obs_dir is still obstructed

	oflags = self.flags;
	org = self.origin;

	yaw = vectoyaw(self.obs_dir);
	if (walkmove(yaw, 32))
		self.b_aiflags = self.b_aiflags - (self.b_aiflags & (AI_OBSTRUCTED | AI_DANGER));
	else
	{
		if (self.b_aiflags & AI_DANGER)
		{
			way = '0 0 0' - self.obs_dir;
		}
		else if (self.wallhug)
		{
			way_x = self.obs_dir_y * -1;
			way_y = self.obs_dir_x;
		}
		else
		{
			way_x = self.obs_dir_y;
			way_y = self.obs_dir_x * -1;
		}
		self.keys = (self.keys & 960) | frik_KeysForDir(way);
	}

	// fix the bot

	self.origin = org;
	self.flags = oflags;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

movetogoal and walkmove replacements

blah

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_bhop

Bunny hop / strafe jump logic for skill 2+.
Chains jumps on landing and adds air strafe
to gain speed via SV_AirAccelerate.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_bhop =
{
	local float spd, yaw_nudge;
	local vector vel;

	// only skill 2+ bunny hops (aggressive personality: skill 1+ can bhop)
	if (self.b_personality == PERSONALITY_AGGRESSIVE)
	{
		if (self.b_skill < 1)
			return;
	}
	else if (self.b_personality == PERSONALITY_DEFENSIVE || self.b_personality == PERSONALITY_CAMPER)
	{
		// defensive/camper: rarely bhop
		if (self.b_skill < 3)
			return;
	}
	else
	{
		if (self.b_skill < 1)
			return;
	}

	// don't bhop during precision movement, on trains, or in water
	if (self.b_aiflags & (AI_PRECISION | AI_RIDE_TRAIN | AI_WAIT))
		return;
	if (self.waterlevel > 1)
		return;

	// don't bhop when target is very close (prevents overshooting)
	if (self.target1)
	{
		if (vlen(realorigin(self.target1) - self.origin) < 128)
			return;
	}

	vel = self.velocity;
	vel_z = 0;
	spd = vlen(vel);

	if (self.flags & FL_ONGROUND)
	{
		// in a chain - keep jumping if speed is above a lower "stop" threshold
		if (self.b_bhop_land)
		{
			if (spd > 200)
			{
				// human emulation: occasionally fumble a bhop in the chain
				if (random() < self.b_bhop_fumble)
				{
					self.b_bhop_land = 0;
					return;
				}
				bot_jump();
				return;
			}
			else
			{
				// speed dropped too low, break the chain
				self.b_bhop_land = 0;
				return;
			}
		}

		// start a bhop chain if moving fast enough
		// skill 3: initiates aggressively, skill 1-2: needs near-max run speed
		if ((self.b_skill >= 3 && spd > 150) || spd > 280)
		{
			bot_jump();
			self.b_bhop_land = 1;
			// set fumble rate for this chain based on skill
			// skill 1: 25%, skill 2: 15%, skill 3: 5%
			if (self.b_skill >= 3)
				self.b_bhop_fumble = 0.05;
			else if (self.b_skill >= 2)
				self.b_bhop_fumble = 0.15;
			else
				self.b_bhop_fumble = 0.25;
		}
	}
	else if (self.b_bhop_land)
	{
		// in air during bhop - strafe + yaw turn for air acceleration
		// SV_AirAccelerate requires wish direction offset from velocity,
		// achieved by holding strafe and turning into the strafe direction
		if (self.b_strafe_dir == 0)
		{
			if (random() < 0.5)
				self.b_strafe_dir = 1;
			else
				self.b_strafe_dir = -1;
		}
		if (self.b_strafe_dir > 0)
			self.keys = self.keys | KEY_MOVERIGHT;
		else
			self.keys = self.keys | KEY_MOVELEFT;

		// turn view into strafe direction for actual speed gain
		// skill 3: 5-8 deg/frame, skill 2: 3-5 deg/frame
		if (self.b_skill >= 3)
			yaw_nudge = 5 + random() * 3;
		else
			yaw_nudge = 3 + random() * 2;
		self.b_angle_y = self.b_angle_y + self.b_strafe_dir * yaw_nudge;
	}
};

void() frik_movetogoal =
{
	local vector way, start, stop, ang;
	local float g, jitter_chance, jitter_wait;

	if (self.target1 == world)
	{
		makevectors(self.v_angle);
		frik_walkmove(v_forward);
		return;
	}
	way = realorigin(self.target1) - self.origin;
	if (vlen(way) < 25)
	{
		self.keys = self.keys & 960;
		return;
	}

	way = normalize(way);
	self.keys = (self.keys & 960) | frik_KeysForDir(way);

	// human emulation: occasional strafe jitter during navigation
	// humans don't run in perfectly straight lines
	// personality: aggressive bots are fidgety, campers/defensive move clean
	if (!self.enemy && time > self.b_nav_jitter_time)
	{
		if (self.b_personality == PERSONALITY_AGGRESSIVE)
		{	jitter_chance = 0.25; jitter_wait = 0.2 + random() * 0.3; }
		else if (self.b_personality == PERSONALITY_CAMPER)
		{	jitter_chance = 0.06; jitter_wait = 0.5 + random() * 0.8; }
		else if (self.b_personality == PERSONALITY_DEFENSIVE)
		{	jitter_chance = 0.08; jitter_wait = 0.4 + random() * 0.6; }
		else
		{	jitter_chance = 0.15; jitter_wait = 0.3 + random() * 0.5; }

		if (random() < jitter_chance)
		{
			if (random() < 0.5)
				self.keys = self.keys | KEY_MOVELEFT;
			else
				self.keys = self.keys | KEY_MOVERIGHT;
		}
		self.b_nav_jitter_time = time + jitter_wait;
	}

	frik_dodge_obstruction();
	frik_recognize_plat(TRUE);

	// local navigation: when obstructed and sense nav has a viable direction,
	// override movement keys to route around the obstacle spatially
	if (self.b_aiflags & AI_OBSTRUCTED)
	{
		local vector nav_dir;
		local float nav_yaw, stuck_dur;
		local float best_clear, clear;

		stuck_dur = time - self.obs_time;
		nav_yaw = -1;

		if (stuck_dur > 1.0)
		{
			// stuck too long — ignore goal, pick most open direction
			best_clear = self.sense_nav_n; nav_yaw = 90;
			clear = self.sense_nav_ne; if (clear > best_clear) { best_clear = clear; nav_yaw = 45; }
			clear = self.sense_nav_e; if (clear > best_clear) { best_clear = clear; nav_yaw = 0; }
			clear = self.sense_nav_se; if (clear > best_clear) { best_clear = clear; nav_yaw = 315; }
			clear = self.sense_nav_s; if (clear > best_clear) { best_clear = clear; nav_yaw = 270; }
			clear = self.sense_nav_sw; if (clear > best_clear) { best_clear = clear; nav_yaw = 225; }
			clear = self.sense_nav_w; if (clear > best_clear) { best_clear = clear; nav_yaw = 180; }
			clear = self.sense_nav_nw; if (clear > best_clear) { best_clear = clear; nav_yaw = 135; }
		}
		else if (self.sense_nav_best_dir >= 0)
			nav_yaw = self.sense_nav_best_dir;

		if (nav_yaw >= 0)
		{
			makevectors('0 1 0' * nav_yaw);
			nav_dir = v_forward;
			nav_dir_z = 0;
			self.keys = (self.keys & 960) | frik_KeysForDir(nav_dir);
		}
	}

	if (self.b_aiflags & AI_PRECISION)
	{
		g = angcomp(self.v_angle_x, self.b_angle_x);
		if (fabs(g) > 10)
			self.keys = self.keys & 960;
		g = angcomp(self.v_angle_y, self.b_angle_y);
		if (fabs(g) > 10)
			self.keys = self.keys & 960;
	}
	else
		bot_bhop();
};

float(vector weird) frik_walkmove =
{
	// okay so it's not walkmove
	// sue me
 	self.keys = (self.keys & 960) | frik_KeysForDir(weird);

	frik_dodge_obstruction();
	frik_recognize_plat(TRUE);
	if (self.b_aiflags & AI_OBSTRUCTED)
		return FALSE;
	else
		return TRUE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

The "hook" method of navigation. This nav
system is copyrighted 1999 by Ryan "Frika C" 
Smith, keep that in mind when you steal it.

I brought this back because normal roaming
won't work - the bot gets distracted by it's
own waypoints.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() frik_bot_roam =
{
	local vector org, ang, org1;
	local float loopcount, flag, dist;

	loopcount = 26;
	flag = FALSE;
	while((loopcount > 0) && !flag)
	{
		loopcount = loopcount - 1;
		org = self.origin + self.view_ofs;
		ang = self.angles;
		ang_y = frik_anglemod(ang_y - 90 + (random() * 180));
		ang_x = 0; // avoid upward sloping
		makevectors(ang);
		traceline(org, org + v_forward * 2300, TRUE, self);
		if (trace_fraction != 1)
		{
			org1 = trace_endpos;
			ang = normalize(trace_plane_normal);
			ang_z = 0; // avoid upward sloping
			traceline(org1, org1 + (ang * 2300), TRUE, self);
			if ((trace_fraction != 1) && (vlen(trace_endpos - org1) >= 64))
			{
				org = trace_endpos;
				traceline(org, self.origin + self.view_ofs, TRUE, self);
				if (trace_fraction != 1)
				{
					dist = vlen(org1 - org) /2;
					org = org1 + (ang * dist);
					traceline(org, org - '0 0 48', TRUE, self);
					if (trace_fraction != 1)
					{
						SpawnTempWaypoint(org);
						flag = TRUE;
					}
				}
			}
		}
	}
	self.b_angle_y = self.v_angle_y + 10;
	frik_walkmove(v_forward);
};
