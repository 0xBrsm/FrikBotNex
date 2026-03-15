/***********************************************
*                                              *
*       FrikBotNex Priority Pipeline           *
*    "One brain, one decision per frame"       *
*                                              *
***********************************************/

/*

Priority-based decision pipeline that replaces the old
scattered key-writing architecture. Each tier either
claims movement (returns TRUE) or passes down. Only one
tier writes to self.keys per frame.

P1: Survival  — hazard avoidance (lava, void, drowning)
P2: Unstuck   — wall/corner escape
P3: Combat    — fight/retreat/pressure/flank + projectile dodge
P4: Navigate  — waypoint following, item pickup, waypoint flags
P5: Idle      — roaming, recreational jumping, sound tracking

*/

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_tier_survival

Immediate hazard avoidance. Checks for lava,
slime, void ahead or underfoot. NEVER skipped
by any flag — survival trumps everything.

Returns TRUE if hazard detected and movement
was overridden.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_tier_survival =
{
	local vector ang;

	if (!(self.flags & FL_ONGROUND))
		return FALSE;

	// drowning — need air NOW
	if (self.waterlevel > 2)
	{
		if (time > (self.air_finished - 2))
		{
			traceline(self.origin, self.origin + '0 0 6800', TRUE, self);
			if (trace_inopen)
			{
				self.keys = KEY_MOVEUP;
				self.button2 = TRUE;
				return TRUE;
			}
		}
	}

	// check sense cache for hazard ahead
	// (refreshed by frik_obstacles every physics frame)
	if (self.b_aiflags & AI_DANGER)
	{
		// hazard flagged by frik_obstacles — reverse away from obs_dir
		self.keys = frik_KeysForDir('0 0 0' - self.obs_dir);
		return TRUE;
	}

	// proactive: lava/slime directly ahead, not yet on edge
	// catch this BEFORE reaching the edge
	if (self.sense_fwd_ground_type < -3 && self.sense_fwd_wall_dist < 64)
	{
		ang = self.velocity;
		ang_z = 0;
		if (vlen(ang) > 0.1 && !bot_wants_to_drop())
		{
			self.keys = frik_KeysForDir('0 0 0' - normalize(ang));
			return TRUE;
		}
	}

	// edge over void/hazard — bot is on edge and ground ahead is bad
	if (self.sense_on_edge)
	{
		local float conts;
		conts = self.sense_fwd_ground_type;
		if (conts < -3 || conts == CONTENT_EMPTY)
		{
			if (!bot_wants_to_drop())
			{
				ang = self.velocity;
				ang_z = 0;
				if (vlen(ang) > 0.1)
				{
					self.keys = frik_KeysForDir('0 0 0' - normalize(ang));
					return TRUE;
				}
			}
		}
	}

	// hazard to sides (lava/slime flanking movement path)
	if (self.sense_hazard_left || self.sense_hazard_right)
	{
		ang = self.velocity;
		ang_z = 0;
		if (vlen(ang) > 0.1)
		{
			local vector side;
			ang = normalize(ang);
			side_x = ang_y * -1;
			side_y = ang_x;

			if (self.sense_hazard_left && !self.sense_hazard_right)
			{
				// hazard on left — dodge right
				self.keys = frik_KeysForDir(ang - side);
				return TRUE;
			}
			else if (self.sense_hazard_right && !self.sense_hazard_left)
			{
				// hazard on right — dodge left
				self.keys = frik_KeysForDir(ang + side);
				return TRUE;
			}
			else
			{
				// hazard both sides — go straight or reverse
				if (self.sense_fwd_ground_type < -3)
				{
					self.keys = frik_KeysForDir('0 0 0' - ang);
					return TRUE;
				}
				// else keep going straight (corridor with hazard on both sides)
			}
		}
	}

	return FALSE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_tier_unstuck

Wall/corner escape. Detects when the bot is
stuck (movement keys pressed but no velocity)
and uses local nav or dodge to escape.

Returns TRUE if stuck and taking evasive action.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_tier_unstuck =
{
	local vector ang, estart;
	local float econts, nav_yaw, stuck_dur;
	local float best_clear, clear;

	if (!(self.flags & FL_ONGROUND))
		return FALSE;

	// check if AI_OBSTRUCTED was set by frik_obstacles in the physics pass
	if (self.b_aiflags & AI_OBSTRUCTED)
	{
		stuck_dur = time - self.obs_time;

		// try walkmove to see if obstruction cleared
		local float oflags;
		local vector org;
		local float yaw;

		oflags = self.flags;
		org = self.origin;
		yaw = vectoyaw(self.obs_dir);

		if (walkmove(yaw, 32))
		{
			// walkmove doesn't detect edges — check for dropoff
			estart = org + self.obs_dir * 16;
			estart_z = org_z + self.mins_z;
			traceline(estart, estart - '0 0 34', TRUE, self);

			if (trace_fraction == 1)
				econts = CONTENT_EMPTY;
			else
				econts = pointcontents(trace_endpos + '0 0 4');

			if ((trace_fraction == 1 || econts < -3) && !bot_wants_to_drop())
			{
				// edge still there — keep dodging
			}
			else
			{
				// clear — obstruction gone
				self.b_aiflags = self.b_aiflags - (self.b_aiflags & (AI_OBSTRUCTED | AI_DANGER));
				self.origin = org;
				self.flags = oflags;
				return FALSE; // not stuck, pass to lower tiers
			}
		}

		self.origin = org;
		self.flags = oflags;

		// still obstructed — use local nav or dodge
		nav_yaw = -1;

		if (stuck_dur > 1.0)
		{
			// stuck too long — pick most open direction regardless of goal
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
			self.keys = frik_KeysForDir(v_forward);
		}
		else
		{
			// no nav data — use wallhug dodge
			local vector way;
			if (self.wallhug)
			{
				way_x = self.obs_dir_y * -1;
				way_y = self.obs_dir_x;
			}
			else
			{
				way_x = self.obs_dir_y;
				way_y = self.obs_dir_x * -1;
			}
			self.keys = frik_KeysForDir(way);
		}
		return TRUE;
	}

	// wall-stuck detection: velocity near-zero but movement keys pressed
	ang = self.velocity;
	ang_z = 0;
	if (vlen(ang) < 10 && (self.keys & 60))
	{
		makevectors(self.v_angle);
		ang = '0 0 0';
		if (self.keys & KEY_MOVEFORWARD) ang = ang + v_forward;
		if (self.keys & KEY_MOVEBACK) ang = ang - v_forward;
		if (self.keys & KEY_MOVELEFT) ang = ang - v_right;
		if (self.keys & KEY_MOVERIGHT) ang = ang + v_right;
		ang_z = 0;
		if (vlen(ang) > 0.1)
		{
			ang = normalize(ang);
			traceline(self.origin, self.origin + ang * 32, TRUE, self);
			if (trace_fraction < 1)
			{
				// wall in intended direction — trigger obstruction
				frik_obstructed(ang, FALSE);
				// don't claim this frame — let obstruction be handled next frame
				// (frik_obstructed needs to set up dodge direction first)
			}
		}
	}

	return FALSE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_tier_combat

Combat decision-making. Handles projectile
dodge as highest-priority combat action, then
delegates to bot_fight_style() for the full
combat state machine.

Returns TRUE if in combat (enemy exists).

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_tier_combat =
{
	if (!self.enemy)
		return FALSE;

	bot_fight_style();
	return TRUE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_tier_navigate

Waypoint navigation and item pickup. Handles
pathfinding, movetogoal, waypoint flag
processing, and special movement states
(trains, platforms, doors).

Returns TRUE if navigating to a target.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_tier_navigate =
{
	if (self.target1)
	{
		frik_movetogoal();
		bot_path();

		// waypoint flag handling that involves movement
		if (self.b_aiflags)
			bot_handle_ai();

		return TRUE;
	}

	// no target — try to find one
	if (waypoint_mode < WM_EDITOR)
	{
		if (self.route_failed)
		{
			if (waypoint_mode == WM_DYNAMIC)
				frik_bot_roam();
			self.route_failed = 0;
			return TRUE;
		}
		else if (!begin_route())
		{
			bot_look_for_crap(FALSE);
			if (self.target1)
				return TRUE; // found something, will navigate next frame
		}
		else
			return TRUE; // route started
	}
	else
	{
		self.b_aiflags = AI_WAIT;
		self.keys = 0;
		return TRUE;
	}

	return FALSE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_tier_idle

Nothing urgent to do. Roam, jump recreationally,
listen for sounds, switch weapons idly.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_tier_idle =
{
	local float jump_chance, snd_dist, snd_best;
	local entity snd_foe;
	local vector snd_view;

	// try to roam somewhere
	frik_bot_roam();

	// idle weapon switching
	if (random() < 0.2)
		if (random() < 0.2)
			bot_weapon_switch(-1);

	// recreational jumping
	if (self.flags & FL_ONGROUND && time > self.b_idle_jump_time)
	{
		if (self.b_personality == PERSONALITY_AGGRESSIVE)
			jump_chance = 0.25;
		else if (self.b_personality == PERSONALITY_CAMPER)
			jump_chance = 0.03;
		else if (self.b_personality == PERSONALITY_DEFENSIVE)
			jump_chance = 0.08;
		else
			jump_chance = 0.12;

		if (random() < jump_chance)
			self.button2 = TRUE;

		self.b_idle_jump_time = time + 0.8 + random() * 2;
	}

	// skill 1+: pre-aim toward nearby sounds
	if (self.b_skill >= 1)
	{
		if (self.b_personality == PERSONALITY_CAMPER)
			snd_best = 1000000;
		else if (self.b_personality == PERSONALITY_DEFENSIVE)
			snd_best = 810000;
		else if (self.b_personality == PERSONALITY_AGGRESSIVE)
			snd_best = 360000;
		else
			snd_best = 640000;
		snd_foe = player_head;
		while (snd_foe)
		{
			if (snd_foe != self && snd_foe.health > 0 && snd_foe.b_sound > time)
			{
				if (!(teamplay && snd_foe.team == self.team) && !coop)
				{
					snd_view = snd_foe.origin - self.origin;
					snd_dist = snd_view_x * snd_view_x + snd_view_y * snd_view_y + snd_view_z * snd_view_z;
					if (snd_dist < snd_best)
					{
						snd_best = snd_dist;
						snd_view = normalize(snd_foe.origin - self.origin);
						snd_view = vectoangles(snd_view);
						snd_view_x = snd_view_x * -1;
						self.b_angle = snd_view;
					}
				}
			}
			snd_foe = snd_foe._next;
		}
	}
};
