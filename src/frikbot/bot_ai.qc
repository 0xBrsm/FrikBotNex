/***********************************************
*                                              *
*            FrikBot General AI                *
*     "The I'd rather be playing Quake AI"     *
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

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Priority pipeline: one brain, one decision per frame.
Each tier either claims self.keys (returns TRUE) or
passes down. Only one tier writes movement per frame.

P1: Survival  -- lava, void, drowning. NOT optional.
P2: Unstuck   -- wall/corner escape
P3: Combat    -- let bot_fight_style handle it
P4: Navigate  -- waypoints, items, doors, trains
P5: Idle      -- roaming, jumping, eavesdropping

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_handle_ai

Waypoint flag handler. Processes AI flags that
involve movement or special actions. 0.10 addition.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_handle_ai =
{
	local entity newt;
	local vector v;

	// AI_WAIT: stop moving, keep looking pretty
	if (self.b_aiflags & AI_WAIT)
		self.keys = self.keys & 960;

	// AI_DOORFLAG: door was here when waypoint spawned
	if (self.b_aiflags & AI_DOORFLAG)
	{
		b_temp3 = self;
		self = self.last_way;
		if (!frik_recognize_plat(FALSE))
		{
			newt = FindThing("door"); // crossfingers
			self = b_temp3;

			if (self.b_aiflags & AI_DOOR_NO_OPEN)
			{
				if (newt.nextthink)
					self.keys = self.keys & 960;
				else
					bot_lost(self.last_way, FALSE);
			}
			else
			{
				if (newt.targetname)
				{
					newt = find(world, target, newt.targetname);
					if (newt.health > 0)
					{
						self.enemy = newt;
						bot_weapon_switch(1);
					}
					else
					{
						target_add(newt);
					}
				}
				self.b_aiflags = self.b_aiflags - AI_DOORFLAG;
			}
		}
		else
			self = b_temp3;
	}

	// AI_JUMP: bunny hop over gap
	if (self.b_aiflags & AI_JUMP)
	{
		if (self.flags & FL_ONGROUND)
		{
			bot_jump();
			self.b_aiflags = self.b_aiflags - AI_JUMP;
		}
	}
	// AI_SUPER_JUMP: rocket jump (the fun kind)
	else if (self.b_aiflags & AI_SUPER_JUMP)
	{
		if (self.weapon != 32)
			self.impulse = 7;
		else if (self.flags & FL_ONGROUND)
		{
			self.b_aiflags = self.b_aiflags - AI_SUPER_JUMP;
			if (bot_can_rj(self))
			{
				bot_jump();
				self.v_angle_x = self.b_angle_x = 80;
				self.button0 = TRUE;
			}
			else
				bot_lost(self.target1, FALSE);
		}
	}

	// AI_SURFACE: swim up if underwater
	if (self.b_aiflags & AI_SURFACE)
	{
		if (self.waterlevel > 2)
		{
			self.keys = KEY_MOVEUP;
			self.button2 = TRUE;
		}
		else
			self.b_aiflags = self.b_aiflags - AI_SURFACE;
	}

	// AI_RIDE_TRAIN: stay on the choo-choo
	if (self.b_aiflags & AI_RIDE_TRAIN)
	{
		if (self.search_time < time + 3)
			self.search_time = time + 3;

		b_temp3 = self;
		self = self.last_way;

		if (!frik_recognize_plat(FALSE))
		{
			self = b_temp3;
			self.keys = self.keys & 960;
		}
		else
		{
			self = b_temp3;
			if (frik_recognize_plat(FALSE))
			{
				v = realorigin(trace_ent) + trace_ent.origin - self.origin;
				v_z = 0;
				if (vlen(v) < 24)
					self.keys = self.keys & 960;
				else
				{
					self.b_aiflags = self.b_aiflags | AI_PRECISION;
					self.keys = frik_KeysForDir(v);
				}
			}
		}
	}

	// AI_PLAT_BOTTOM: waiting for the elevator like it's Monday morning
	if (self.b_aiflags & AI_PLAT_BOTTOM)
	{
		newt = FindThing("plat");
		if (newt.state != 1)
		{
			if (self.search_time < time + 3)
				self.search_time = time + 3;
			v = self.origin - realorigin(newt);
			v_z = 0;
			if (vlen(v) > 96)
				self.keys = self.keys & 960;
			else
				frik_walkmove(v);
		}
		else
			self.b_aiflags = self.b_aiflags - AI_PLAT_BOTTOM;
	}

	// AI_DIRECTIONAL: check if we passed the waypoint
	if (self.b_aiflags & AI_DIRECTIONAL)
	{
		if ((normalize(self.last_way.origin - self.origin) * self.b_dir) > 0.4)
		{
			self.b_aiflags = self.b_aiflags - AI_DIRECTIONAL;
			bot_lost(self.target1, TRUE);
		}
	}

	// AI_SNIPER: find a nice spot, sit down, aim carefully
	if (self.b_aiflags & AI_SNIPER)
	{
		self.b_aiflags = (self.b_aiflags | AI_WAIT | AI_PRECISION) - AI_SNIPER;
	}

	// AI_AMBUSH: surprise! (or not, if they don't show up)
	if (self.b_aiflags & AI_AMBUSH)
	{
		self.b_aiflags = (self.b_aiflags | AI_WAIT) - AI_AMBUSH;

		bot_weapon_switch(-1);

		// patience scales with skill
		if (self.b_skill == 0)
			self.search_time = time + AMBUSH_PATIENCE_SKILL0;
		else if (self.b_skill == 1)
			self.search_time = time + AMBUSH_PATIENCE_SKILL1;
		else if (self.b_skill == 2)
			self.search_time = time + AMBUSH_PATIENCE_SKILL2;
		else
			self.search_time = time + AMBUSH_PATIENCE_SKILL3;

		// personality: campers are patient, aggressive bots get bored fast
		if (self.b_personality == PERSONALITY_CAMPER)
			self.search_time = self.search_time + (self.search_time - time) * 0.5;
		else if (self.b_personality == PERSONALITY_AGGRESSIVE)
			self.search_time = time + (self.search_time - time) * 0.6;

		self.b_ambush_time = time;

		// face the most likely approach direction
		if (self.last_way)
		{
			if (self.last_way.target1 && self.last_way.target1.classname == "waypoint")
			{
				v = normalize(self.last_way.target1.origin - self.origin);
				v = vectoangles(v);
				v_x = v_x * -1;
				self.b_angle = v;
			}
		}
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_path

Follow a route generated by begin_route.
This code works pretty well but can get confused.
(It's been doing its best since 0.10)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_path =
{
	local entity jj, tele;

	bot_check_lost(self.target1);
	if (!self.target1)
	{
		self.keys = 0;
		return;
	}
	if (target_onstack(self.last_way))
		return; // old waypoint still being hunted

	jj = FindRoute(self.last_way);
	if (!jj)
	{
		if (self.target1.current_way != self.last_way)
		{
			if (self.target1.classname != "temp_waypoint")
				if (self.target1.classname != "player")
					bot_lost(self.target1, FALSE);
		}
		return;
	}

	// readahead from next waypoint, point flags from current
	self.b_aiflags = (jj.b_aiflags & AI_READAHEAD_TYPES) | (self.last_way.b_aiflags & AI_POINT_TYPES);
	target_add(jj);
	if (self.last_way)
	{
		if (CheckLinked(self.last_way, jj) == 2)
		{
			tele = FindThing("trigger_teleport"); // probbly the right one
			target_add(tele);
		}
		traceline(self.last_way.origin, jj.origin, FALSE, self);
		if (trace_fraction != 1)
		{
			if (trace_ent.classname == "door" && !(self.b_aiflags & AI_DOOR_NO_OPEN))
			{
				if (trace_ent.owner)
					trace_ent = trace_ent.owner;
				if ((trace_ent.health > 0) && (self.enemy == world))
				{
					self.enemy = trace_ent;
					bot_weapon_switch(1);
					self.b_aiflags = self.b_aiflags | AI_BLIND;
				}
				else if (trace_ent.targetname)
				{
					tele = find(world, target, trace_ent.targetname);
					if (tele.health > 0)
					{
						self.enemy = tele;
						bot_weapon_switch(1);
					}
					else
					{
						target_add(tele);
						self.b_aiflags = self.b_aiflags | AI_BLIND;
						return;
					}
				}
			}
			else if (trace_ent.classname == "func_wall")
			{
				bot_lost(self.target1, FALSE);
				return;
			}
		}
	}
	// directional tracking
	self.b_dir = normalize(jj.origin - self.last_way.origin);
	self.last_way = jj;
};


/*
==============================================
  THE PIPELINE
  "What should I do?" answered five ways.
==============================================
*/


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_ai_survival  (P1)

Hazard avoidance. Drowning, lava, slime, void.
This tier is NEVER skipped. Survival trumps all.

Returns TRUE if we're dodging death.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_ai_survival =
{
	local vector ang;

	// drowning -- need air NOW
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

	// AI_DANGER: something scary in obs_dir, run the other way
	if (self.b_aiflags & AI_DANGER)
	{
		self.keys = frik_KeysForDir('0 0 0' - self.obs_dir);
		return TRUE;
	}

	// lava/slime directly ahead (catch it BEFORE reaching the edge)
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

	// edge over void/hazard
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

	// hazard to sides (lava flanking our path)
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
				self.keys = frik_KeysForDir(ang - side);
				return TRUE;
			}
			else if (self.sense_hazard_right && !self.sense_hazard_left)
			{
				self.keys = frik_KeysForDir(ang + side);
				return TRUE;
			}
			else
			{
				// hazard both sides -- corridor of doom
				if (self.sense_fwd_ground_type < -3)
				{
					self.keys = frik_KeysForDir('0 0 0' - ang);
					return TRUE;
				}
			}
		}
	}

	return FALSE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_ai_unstuck  (P2)

Wall/corner escape. Only runs on ground.
Detects zero-velocity + movement keys as
"stuck" and tries to wiggle free.

Returns TRUE if stuck and dodging.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_ai_unstuck =
{
	local vector ang, estart;
	local float econts, nav_yaw, stuck_dur;
	local float best_clear, clear;

	if (!(self.flags & FL_ONGROUND))
		return FALSE;

	// check if AI_OBSTRUCTED was set by frik_obstacles
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
			// walkmove doesn't detect edges -- check for dropoff
			estart = org + self.obs_dir * 16;
			estart_z = org_z + self.mins_z;
			traceline(estart, estart - '0 0 34', TRUE, self);

			if (trace_fraction == 1)
				econts = CONTENT_EMPTY;
			else
				econts = pointcontents(trace_endpos + '0 0 4');

			if ((trace_fraction == 1 || econts < -3) && !bot_wants_to_drop())
			{
				// edge still there -- keep dodging
			}
			else
			{
				// clear! obstruction gone
				self.b_aiflags = self.b_aiflags - (self.b_aiflags & (AI_OBSTRUCTED | AI_DANGER));
				self.origin = org;
				self.flags = oflags;
				return FALSE;
			}
		}

		self.origin = org;
		self.flags = oflags;

		// still stuck -- use local nav or wallhug
		nav_yaw = -1;

		if (stuck_dur > 1.0)
		{
			// been stuck too long -- pick most open direction
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
			// no nav data -- wallhug dodge
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

	// wall-stuck detection: no velocity but pressing movement keys
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
				// wall in intended direction -- set up dodge for next frame
				frik_obstructed(ang, FALSE);
			}
		}
	}

	return FALSE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_ai_combat  (P3)

Got an enemy? Let bot_fight_style handle it.
Simple delegation, nothing fancy.

Returns TRUE if in combat.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_ai_combat =
{
	if (!self.enemy)
		return FALSE;

	bot_fight_style();
	return TRUE;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_ai_navigate  (P4)

Waypoint following, item pickup, route planning.
The "go somewhere useful" tier.

Returns TRUE if navigating to a target.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_ai_navigate =
{
	if (self.target1)
	{
		frik_movetogoal();
		bot_path();

		if (self.b_aiflags)
			bot_handle_ai();

		return TRUE;
	}

	// no target -- try to find one
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
			bot_pick_goal(FALSE);
			if (self.target1)
				return TRUE;
		}
		else
			return TRUE;
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

bot_ai_idle  (P5)

Nothing to do. Roam around, jump for fun,
listen for footsteps, fiddle with weapons.
The "bored bot" tier.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_ai_idle =
{
	local float jump_chance, snd_dist, snd_best;
	local entity snd_foe;
	local vector snd_view;

	frik_bot_roam();

	// idle weapon switching -- rare, for realism
	if (random() < 0.2)
		if (random() < 0.2)
			bot_weapon_switch(-1);

	// recreational jumping (personality-scaled)
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

	// skill 1+: pre-aim toward nearby enemy sounds
	if (self.b_skill >= 1)
	{
		// hearing range scales with personality
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
					// squared distance (avoid sqrt in hot path)
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


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_angle_set

Point the bot's face at things. Handles
humanized aim (drift, flick, tracking error)
for enemies and simple aim for nav targets.

This is where bots stop looking like robots
and start looking like bad players.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_angle_set =
{
	local float h;
	local vector view, pred_vel, foe_fwd, foe_right;
	local float aim_error, espeed, decay, glance_chance, glance_interval;
	local float drift_sin;

	if (self.enemy)
	{
		if (self.enemy.items & IT_INVISIBILITY)
			if (random() > 0.01)
				return;
		if (self.missile_speed == 0)
			self.missile_speed = 10000;

		// track how long we've been aiming at this enemy
		if (self.b_aim_delay == 0)
		{
			self.b_aim_delay = time;
			// initial aim offset: skill 0 = 25 deg, skill 3 = 3 deg
			if (self.b_skill == 0)
				self.b_aim_offset = 25;
			else if (self.b_skill == 1)
				self.b_aim_offset = 15;
			else if (self.b_skill == 2)
				self.b_aim_offset = 8;
			else
				self.b_aim_offset = 3;

			// flick overshoot on target acquisition
			if (self.b_skill >= 2)
				self.b_aim_flick = (random() - 0.5) * 8;
			else
				self.b_aim_flick = (random() - 0.5) * 16;
			self.b_aim_drift_phase = random() * 6.28;
		}

		if (self.enemy.solid == SOLID_BSP)
		{
			view = realorigin(self.enemy) - self.origin;
		}
		else
		{
			local float levels;
			levels = 3;
			view = self.enemy.origin;
			pred_vel = self.enemy.velocity;

			// skill 2+ opponent model: enhance velocity prediction
			if (self.b_skill >= 2 && self.missile_speed < 2000
				&& self.b_opp_ent == self.enemy)
			{
				foe_fwd = normalize(self.origin - self.enemy.origin);
				foe_right_x = foe_fwd_y * -1;
				foe_right_y = foe_fwd_x;
				foe_right_z = 0;

				if (self.b_opp_strafe > 0.15 || self.b_opp_strafe < -0.15)
					pred_vel = pred_vel + foe_right * self.b_opp_strafe * 100;

				if (self.b_opp_aggro > 0.65)
					pred_vel = pred_vel + foe_fwd * 40;
				else if (self.b_opp_aggro < 0.3)
					pred_vel = pred_vel - foe_fwd * 30;
			}

			while (levels)
			{
				h = vlen(view - self.origin) / self.missile_speed;
				if (self.enemy.flags & FL_ONGROUND)
					view = pred_vel * h + '0 0 -20';
				else
					view = (pred_vel - (sv_gravity * '0 0 1') * h) * h;
				view = self.enemy.origin + view;
				traceline(self.enemy.origin, view, FALSE, self);
				view = trace_endpos;
				levels = levels - 1;
			}
			view = normalize(view - self.origin);
		}
		view = vectoangles(view);
		view_x = view_x * -1;

		// convergence rate: skill 3 ~0.2s, skill 0 ~1.5s
		if (self.b_skill == 0)
			decay = real_frametime * 0.7;
		else if (self.b_skill == 1)
			decay = real_frametime * 1.5;
		else if (self.b_skill == 2)
			decay = real_frametime * 3.0;
		else
			decay = real_frametime * 5.0;
		if (decay > 0.95)
			decay = 0.95;
		self.b_aim_offset = self.b_aim_offset * (1 - decay);

		// floor: minimum persistent jitter per skill
		if (self.b_skill == 0)
		{	if (self.b_aim_offset < 6) self.b_aim_offset = 6; }
		else if (self.b_skill == 1)
		{	if (self.b_aim_offset < 3) self.b_aim_offset = 3; }
		else if (self.b_skill == 2)
		{	if (self.b_aim_offset < 1.5) self.b_aim_offset = 1.5; }
		else
		{	if (self.b_aim_offset < 0.5) self.b_aim_offset = 0.5; }

		// tracking error: fast-moving enemies wobble low-skill aim
		espeed = vlen(self.enemy.velocity);
		if (espeed > 200)
		{
			if (self.b_skill == 0)
				self.b_aim_offset = self.b_aim_offset + espeed * 0.008 * real_frametime * 10;
			else if (self.b_skill == 1)
				self.b_aim_offset = self.b_aim_offset + espeed * 0.004 * real_frametime * 10;
			else if (self.b_skill == 2)
				self.b_aim_offset = self.b_aim_offset + espeed * 0.001 * real_frametime * 10;
		}

		if (self.b_aim_offset > 30)
			self.b_aim_offset = 30;

		// skill 3 hitscan bias toward enemy strafe tendency
		if (self.b_skill >= 3 && self.b_opp_ent == self.enemy
			&& self.missile_speed >= 2000)
		{
			if (self.b_opp_strafe > 0.2 || self.b_opp_strafe < -0.2)
				view_y = view_y - self.b_opp_strafe * 3;
		}

		// --- human emulation ---

		// 1. flick overshoot: snap past target, settle back
		if (self.b_aim_flick != 0)
		{
			view_y = view_y + self.b_aim_flick;
			self.b_aim_flick = self.b_aim_flick * (1 - real_frametime * 4);
			if (fabs(self.b_aim_flick) < 0.3)
				self.b_aim_flick = 0;
		}

		// 2. tracking drift: slow sine wave wobble
		self.b_aim_drift_phase = self.b_aim_drift_phase + real_frametime * 1.8;
		if (self.b_aim_drift_phase > 6.28)
			self.b_aim_drift_phase = self.b_aim_drift_phase - 6.28;
		makevectors(self.b_aim_drift_phase * 57.2958 * '0 1 0');
		drift_sin = v_forward_y;
		if (self.b_skill <= 1)
			view_y = view_y + drift_sin * 2.5;
		else if (self.b_skill == 2)
			view_y = view_y + drift_sin * 1.0;
		else
			view_y = view_y + drift_sin * 0.4;

		// 3. overcorrection: correlated jitter (wiggle around target)
		aim_error = self.b_aim_offset;
		view_y = view_y + (random() - 0.5) * aim_error - self.b_aim_last_yaw_err * 0.3;
		view_x = view_x + (random() - 0.5) * aim_error;
		self.b_aim_last_yaw_err = view_y - self.b_angle_y;

		self.b_angle = view;
	}
	else if (self.target1)
	{
		// no enemy -- reset aim tracking
		self.b_aim_delay = 0;
		self.b_aim_offset = 0;

		view = realorigin(self.target1);
		if (self.target1.flags & FL_ITEM)
			view = view + '0 0 48';
		view = view - (self.origin + self.view_ofs);
		view = vectoangles(view);
		view_x = view_x * -1;

		// slight yaw overshoot during turns (humans don't snap perfectly)
		view_y = view_y + (random() - 0.5) * 5;

		// idle glance: occasionally look to one side while navigating
		if (self.b_idle_glance_yaw != 0)
		{
			view_y = view_y + self.b_idle_glance_yaw;
			self.b_idle_glance_yaw = self.b_idle_glance_yaw * (1 - real_frametime * 3);
			if (fabs(self.b_idle_glance_yaw) < 2)
				self.b_idle_glance_yaw = 0;
		}
		else if (time > self.b_idle_glance_time)
		{
			if (self.b_personality == PERSONALITY_CAMPER)
			{	glance_chance = 0.4; glance_interval = 2 + random() * 3; }
			else if (self.b_personality == PERSONALITY_DEFENSIVE)
			{	glance_chance = 0.3; glance_interval = 3 + random() * 4; }
			else if (self.b_personality == PERSONALITY_AGGRESSIVE)
			{	glance_chance = 0.1; glance_interval = 5 + random() * 5; }
			else
			{	glance_chance = 0.2; glance_interval = 3 + random() * 5; }

			if (random() < glance_chance)
			{
				if (random() < 0.85)
				{
					// normal glance: 30-70 degrees
					if (random() < 0.5)
						self.b_idle_glance_yaw = 30 + random() * 40;
					else
						self.b_idle_glance_yaw = -30 - random() * 40;
				}
				else
				{
					// rare check-behind: 130-170 degrees
					if (random() < 0.5)
						self.b_idle_glance_yaw = 130 + random() * 40;
					else
						self.b_idle_glance_yaw = -130 - random() * 40;
				}
			}
			self.b_idle_glance_time = time + glance_interval;
		}

		self.b_angle = view;
	}
	else
	{
		self.b_aim_delay = 0;
		self.b_aim_offset = 0;
		self.b_angle_x = 0;
	}

	// --- apply view angles ---
	// skill 2+: direct angle set (aim variance handles humanization)
	// skill 0-1: keyboard look for "sluggish" feel
	if (self.b_skill >= 2)
	{
		self.keys = self.keys & 63;
		if (self.b_skill >= 3)
		{
			self.v_angle = self.b_angle;
			while (self.v_angle_x < -180)
				self.v_angle_x = self.v_angle_x + 360;
			while (self.v_angle_x > 180)
				self.v_angle_x = self.v_angle_x - 360;
		}
		// skill 2: b_angle set, mouse emulation in bot_phys.qc
	}
	else if ((self.enemy == world || self.enemy.movetype == MOVETYPE_PUSH) && self.target1.classname != "player")
	{
		// no combat target -- instant turn (don't walk off ledges)
		self.keys = self.keys & 63;
		self.v_angle = self.b_angle;
		while (self.v_angle_x < -180)
			self.v_angle_x = self.v_angle_x + 360;
		while (self.v_angle_x > 180)
			self.v_angle_x = self.v_angle_x - 360;
	}
	else
	{
		// skill 0-1 in combat: keyboard turning (sluggish tracking)
		local float dy, dx;
		if (self.b_angle_x > 180)
			self.b_angle_x = self.b_angle_x - 360;
		self.keys = self.keys & 63;

		dy = angcomp(self.b_angle_y, self.v_angle_y);
		dx = angcomp(self.b_angle_x, self.v_angle_x);
		if (dy > 10)
			self.keys = self.keys | KEY_LOOKLEFT;
		else if (dy < -10)
			self.keys = self.keys | KEY_LOOKRIGHT;
		if (dx < -10)
			self.keys = self.keys | KEY_LOOKUP;
		else if (dx > 10)
			self.keys = self.keys | KEY_LOOKDOWN;
	}
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BotAI

The main event. Called every physics frame
from bot_phys.qc. One question per frame:
"what should I do?"

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/
float stagger_think;

void() BotAI =
{
	// am I dead? mash buttons until respawn
	if (self.health < 1)
	{
		self.button0 = floor(random() * 2);
		self.button2 = 0;
		self.keys = 0;
		self.b_aiflags = 0;
		ClearMyRoute();
		self.target1 = self.target2 = self.target3 = self.target4 = self.enemy = world;
		self.last_way = world;
		self.b_aim_delay = 0;
		self.b_aim_offset = 0;
		self.b_strafe_dir = 0;
		self.b_ambush_time = 0;
		self.b_bhop_land = 0;
		self.b_combat_state = CSTATE_NONE;
		self.b_los_time = 0;
		self.b_aim_flick = 0;
		self.b_aim_last_yaw_err = 0;
		self.b_react_time = 0;
		self.b_wep_commit_time = 0;
		self.b_bhop_fumble = 0;
		self.b_idle_glance_yaw = 0;
		self.b_idle_glance_time = 0;
		self.b_idle_jump_time = 0;
		bot_opp_model_reset();
		return;
	}

	// AI throttle: skill 0-1 only think every 0.05s (staggered)
	if (self.b_skill < 2)
	{
		if (self.ai_time > time)
			return;

		self.ai_time = time + 0.05;
		if (bot_count > 0)
		{
			if ((time - stagger_think) < (0.1 / bot_count))
				self.ai_time = self.ai_time + 0.1 / (2 * bot_count);
		}
		else
			return;
	}
	if (self.view_ofs == '0 0 0')
		bot_start_topic(7);
	stagger_think = time;

	// clear buttons -- subsystems turn them on as needed
	self.button2 = 0;
	self.button0 = 0;

	// unified spatial awareness pass
	bot_sense();

	// enemy/projectile detection BEFORE decisions
	bot_dodge_stuff();

	// route table editing -- special case, not part of pipeline
	if (route_table == self)
	{
		if (busy_waypoints <= 0)
		{
			if (waypoint_mode < WM_EDITOR)
				bot_pick_goal(TRUE);
		}
		self.b_aiflags = 0;
		self.keys = 0;
		bot_angle_set();
		return;
	}

	// --- priority pipeline: first tier to claim wins ---
	if (!bot_ai_survival())
	if (!bot_ai_unstuck())
	if (!bot_ai_combat())
	if (!bot_ai_navigate())
		bot_ai_idle();

	bot_angle_set();

	if (!self.b_aiflags)
		bot_chat();
};
