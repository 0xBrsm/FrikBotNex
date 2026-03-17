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
	if (e.items & IT_QUAD)
		return FALSE;

	// do I have rockets & RL?
	if (!((e.items & IT_ROCKET_LAUNCHER) && (e.ammo_rockets > 0)))
		return FALSE;

	// do I have pent?
	if (e.items & IT_INVULNERABILITY)
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

	if (danger)
	{
		self.b_aiflags = self.b_aiflags | AI_DANGER;
		// don't write keys here — survival tier handles movement
	}
	if (self.b_aiflags & AI_BLIND)
		return;
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

float frik_drop_ok;  // cached bot_wants_to_drop() for current frik_obstacles frame

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

	// center-ahead hazard (but defer to navigation intent for drops)
	if (self.sense_fwd_ground_type < -3)
	{
		if (!frik_drop_ok)
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

	// cache for this frame (read by frik_check_hazard + gap logic)
	frik_drop_ok = bot_wants_to_drop();

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
				if (frik_drop_ok)
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

	// --- gap but not jumpable ---
	if (!self.sense_fwd_jumpable)
	{
		if (frik_drop_ok)
			return; // navigation says drop — trust it
		frik_obstructed(normalize(ang), (conts < -3));
		return;
	}

	// --- gap and jumpable: check for wall blocking the jump ---
	ang = normalize(ang);
	traceline(self.origin, self.origin + (ang * (gap + 20)), TRUE, self);
	if (trace_fraction != 1)
	{
		if (frik_drop_ok)
			return;
		frik_obstructed(ang, (conts < -3));
		return;
	}

	// getting further from target? treat as obstruction
	if (self.target1)
	{
		dist = vlen(self.target1.origin - (ang + self.origin));
		if (dist > vlen(self.target1.origin - self.origin))
		{
			frik_obstructed(ang, (conts < -3));
			return;
		}
	}

	// clear to jump the gap
	if (hgt < -18)
		bot_jump();
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

movetogoal and walkmove replacements

blah

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() frik_movetogoal =
{
	local vector way;
	local float g;

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

	frik_recognize_plat(TRUE);

	if (self.b_aiflags & AI_PRECISION)
	{
		g = angcomp(self.v_angle_x, self.b_angle_x);
		if (fabs(g) > 10)
			self.keys = self.keys & 960;
		g = angcomp(self.v_angle_y, self.b_angle_y);
		if (fabs(g) > 10)
			self.keys = self.keys & 960;
	}
};

float(vector weird) frik_walkmove =
{
	// okay so it's not walkmove
	// sue me
	local vector org, dir, start, stop;
	local float lookkeys, oflags, yaw, conts;

	lookkeys = self.keys & 960;
	self.keys = lookkeys | frik_KeysForDir(weird);

	if (self.flags & FL_ONGROUND)
	{
		dir = weird;
		dir_z = 0;
		if (vlen(dir) > 0.1)
		{
			dir = normalize(dir);
			start = self.origin + dir * 16;
			start_z = self.origin_z + self.mins_z;
			stop = start - '0 0 40';
			traceline(start, stop, TRUE, self);
			if (trace_fraction == 1)
			{
				self.keys = lookkeys;
				return FALSE;
			}

			conts = pointcontents(trace_endpos + '0 0 4');
			if (conts < -3)
			{
				self.keys = lookkeys;
				return FALSE;
			}
		}
	}

	frik_recognize_plat(TRUE);

	// test if movement in this direction is actually possible
	// callers like bot_combat_strafe use this to flip direction on walls
	org = self.origin;
	oflags = self.flags;
	yaw = vectoyaw(weird);
	if (!walkmove(yaw, 16))
	{
		self.origin = org;
		self.flags = oflags;
		self.keys = lookkeys;
		return FALSE;
	}
	self.origin = org;
	self.flags = oflags;
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
