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
Each tier either claims self.b_wishdir (returns TRUE) or
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
		self.b_wishdir = '0 0 0';

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
					self.b_wishdir = '0 0 0';
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
		if (self.weapon != IT_ROCKET_LAUNCHER)
			self.impulse = 7;
		else if (self.flags & FL_ONGROUND)
		{
			self.b_aiflags = self.b_aiflags - AI_SUPER_JUMP;
			if (bot_can_rj(self))
			{
				bot_jump();
				self.b_angle_x = 80;
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
			self.b_wishdir = '0 0 1';
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
			self.b_wishdir = '0 0 0';
		}
		else
		{
			self = b_temp3;
			if (frik_recognize_plat(FALSE))
			{
				v = realorigin(trace_ent) + trace_ent.origin - self.origin;
				v_z = 0;
				if (vlen(v) < 24)
					self.b_wishdir = '0 0 0';
				else
				{
					self.b_aiflags = self.b_aiflags | AI_PRECISION;
					self.b_wishdir = normalize(v);
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
				self.b_wishdir = '0 0 0';
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

		self.search_time = time + 12 - self.b_skill * 2;

		self.b_ambush_time = time;

		// face the most likely approach direction
		if (self.last_way)
		{
			if (self.last_way.target1 && self.last_way.target1.classname == "waypoint")
			{
				v = vectoangles(self.last_way.target1.origin - self.origin);
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
		self.b_wishdir = '0 0 0';
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


/* tier functions removed — BotAI is now flat */


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
	local float aim_speed, aim_floor, decay;

	if (self.enemy)
	{
		if (self.enemy.items & IT_INVISIBILITY)
			if (random() > 0.01)
				return;
		if (self.missile_speed == 0)
			self.missile_speed = 10000;

		if (self.b_aim_delay == 0)
			self.b_aim_delay = time;

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
			view = view - self.origin;
		}
		view = vectoangles(view);
		view_x = view_x * -1;

		self.b_angle = view;
	}
	else if (self.target1)
	{
		// no enemy -- reset aim tracking
		self.b_aim_delay = 0;

		view = realorigin(self.target1);
		if (self.target1.flags & FL_ITEM)
			view = view + '0 0 48';
		view = view - (self.origin + self.view_ofs);
		view = vectoangles(view);
		view_x = view_x * -1;

		self.b_angle = view;
	}
	else
	{
		self.b_aim_delay = 0;
		self.b_angle_x = 0;
	}

	// --- apply view angles: proportional + velocity smoothing ---
	// mouse_emu stores angular velocity (y=yaw, x=pitch).
	// Proportional step sets the desired velocity (natural decel),
	// but actual velocity blends toward it (smooth accel, no overshoot).
	local float smooth, accel, max_turn;
	local float dyaw, dpitch, want_y, want_p, step_len;

	smooth = 0.03 + self.b_skill * 0.15;
	accel = 0.10 + self.b_skill * 0.15;
	max_turn = (100 + self.b_skill * 250) * sv_ticrate;

	dyaw = self.b_angle_y - self.v_angle_y;
	while (dyaw > 180) dyaw = dyaw - 360;
	while (dyaw < -180) dyaw = dyaw + 360;
	dpitch = self.b_angle_x - self.v_angle_x;
	while (dpitch > 180) dpitch = dpitch - 360;
	while (dpitch < -180) dpitch = dpitch + 360;

	// desired velocity from proportional error
	want_y = dyaw * smooth;
	want_p = dpitch * smooth;

	// blend actual velocity toward desired (smooth accel)
	self.mouse_emu_y = self.mouse_emu_y + (want_y - self.mouse_emu_y) * accel;
	self.mouse_emu_x = self.mouse_emu_x + (want_p - self.mouse_emu_x) * accel;

	// cap velocity
	step_len = vlen(self.mouse_emu_y * '1 0 0' + self.mouse_emu_x * '0 1 0');
	if (step_len > max_turn)
	{
		self.mouse_emu_y = self.mouse_emu_y * (max_turn / step_len);
		self.mouse_emu_x = self.mouse_emu_x * (max_turn / step_len);
	}

	self.v_angle_y = self.v_angle_y + self.mouse_emu_y;
	self.v_angle_x = self.v_angle_x + self.mouse_emu_x;
	while (self.v_angle_x < -180)
		self.v_angle_x = self.v_angle_x + 360;
	while (self.v_angle_x > 180)
		self.v_angle_x = self.v_angle_x - 360;
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

// debug: per-bot movement metrics (accumulated, printed every 10s)
.float _m_stuck;       // frames where position didn't change
.vector _m_last_origin; // previous frame position for stuck detection
.float _m_lava;        // frames on lava/slime
.float _m_nodir;       // frames with sense_best_dir == -1
.float _m_frames;      // total alive frames
.float _m_report_time; // next report time

// lava entry diagnosis: what mode was the bot in when it stepped onto lava?
.float _m_move_mode;   // current movement mode (set before movement)
.float _m_was_lava;    // was on lava last frame? (for transition detect)

.float _m_le_nav;      // lava entries while nav-fan guided
.float _m_le_beeline;  // lava entries while beeline (nodir)
.float _m_le_combat;   // lava entries while fighting
.float _m_le_roam;     // lava entries while roaming/other
float MM_NAV     = 1;
float MM_BEELINE = 2;
float MM_COMBAT  = 3;
float MM_ROAM    = 4;

void() bot_server_spawn =
{
	local float n;
	n = cvar("temp1");
	if (n <= 0) return;
	cvar_set("temp1", "0");
	while (n > 0) { BotConnect(0, 0, cvar("skill")); n = n - 1; }
};

void() bot_debug_metrics =
{
	local string s;
	local vector vel;

	if (self.health < 1)
		return;

	self._m_frames = self._m_frames + 1;

	// stuck = didn't move since last frame (position-based, not velocity)
	vel = self.origin - self._m_last_origin;
	vel_z = 0;
	if (vlen(vel) < 1)
		self._m_stuck = self._m_stuck + 1;
	self._m_last_origin = self.origin;

	local float on_lava;
	on_lava = (self.sense_ground_type < -3);
	if (on_lava)
		self._m_lava = self._m_lava + 1;

	// detect lava entry transition and bucket by movement mode
	if (on_lava && !self._m_was_lava)
	{
		if (self._m_move_mode == MM_NAV) self._m_le_nav = self._m_le_nav + 1;
		else if (self._m_move_mode == MM_BEELINE) self._m_le_beeline = self._m_le_beeline + 1;
		else if (self._m_move_mode == MM_COMBAT) self._m_le_combat = self._m_le_combat + 1;
		else self._m_le_roam = self._m_le_roam + 1;

	}
	self._m_was_lava = on_lava;

	if (self._m_move_mode == MM_BEELINE)
		self._m_nodir = self._m_nodir + 1;

	// report every 10s
	if (time < self._m_report_time)
		return;
	self._m_report_time = time + 10;

	if (self._m_frames < 1)
		return;

	bprint("["); bprint(self.netname); bprint("] ");
	bprint("10s: stuck=");
	s = ftos(rint(100 * self._m_stuck / self._m_frames));
	bprint(s); bprint("% lava=");
	s = ftos(rint(100 * self._m_lava / self._m_frames));
	bprint(s); bprint("% nodir=");
	s = ftos(rint(100 * self._m_nodir / self._m_frames));
	bprint(s); bprint("% hp=");
	s = ftos(rint(self.health));
	bprint(s); bprint(" le:");
	s = ftos(self._m_le_nav); bprint(s); bprint("n/");
	s = ftos(self._m_le_beeline); bprint(s); bprint("b/");
	s = ftos(self._m_le_combat); bprint(s); bprint("c/");
	s = ftos(self._m_le_roam); bprint(s); bprint("r");
	bprint("\n");

	// reset counters
	self._m_stuck = 0;
	self._m_lava = 0;
	self._m_nodir = 0;
	self._m_frames = 0;
	self._m_le_nav = 0;
	self._m_le_beeline = 0;
	self._m_le_combat = 0;
	self._m_le_roam = 0;
};

void() BotAI =
{
	// --- dead: mash respawn ---
	if (self.health < 1)
	{
		self.button0 = floor(random() * 2);
		self.button2 = 0;
		self.b_wishdir = '0 0 0';
		self.b_aiflags = 0;
		ClearMyRoute();
		self.target1 = self.target2 = self.target3 = self.target4 = self.enemy = world;
		self.last_way = world;
		self.b_aim_delay = 0;
		self.mouse_emu = '0 0 0';
		self.b_strafe_dir = 0;
		self.b_ambush_time = 0;
		self.b_combat_time = 0;
		self.b_combat_state = CSTATE_NONE;
		self.b_react_time = 0;
		self.b_wep_commit_time = 0;
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

	self.button2 = 0;
	self.button0 = 0;

	// --- perceive ---
	bot_projectile_scan();
	bot_sense();

	// route table editing (waypoint system housekeeping)
	if (route_table == self)
	{
		if (self.health < 1 || self.search_time < time - 5)
		{
			// dead or stalled — release route table
			route_table = world;
			busy_waypoints = 0;
		}
		else
		{
			if (busy_waypoints <= 0)
				if (waypoint_mode < WM_EDITOR)
					bot_get(TRUE);
			self.b_aiflags = 0;
			self.b_wishdir = '0 0 0';
			bot_angle_set();
			return;
		}
	}

	// --- survive: emergency escape from drowning/lava ---
	if (self.waterlevel > 2 && time > (self.air_finished - 2))
	{
		traceline(self.origin, self.origin + '0 0 6800', TRUE, self);
		if (trace_inopen)
		{
			self.b_wishdir = '0 0 1';
			self.button2 = TRUE;
			bot_angle_set();
			return;
		}
	}

	// --- enemy management: one place, one rule ---
	// validate current enemy
	if (self.enemy)
	{
		if (self.enemy.health <= 0 || !self.enemy.takedamage || bot_nocombat)
			self.enemy = world;
	}
	// only scan when we don't have one (scratch1 disables combat)
	if (!self.enemy && !bot_nocombat)
		bot_find_enemy();

	// --- act: fight, navigate, or roam ---
	if (self.enemy)
	{
		self._m_move_mode = MM_COMBAT;
		bot_fight_style();
	}
	else
	{
		// get a goal if we don't have one
		if (!self.target1)
		{
			if (waypoint_mode < WM_EDITOR)
			{
				if (self.route_failed)
				{
					self._m_move_mode = MM_ROAM;
					if (waypoint_mode == WM_DYNAMIC)
						frik_bot_roam();
					self.route_failed = 0;
				}
				else if (!begin_route())
					bot_get(FALSE);
			}
			else
			{
				self.b_aiflags = AI_WAIT;
				self.b_wishdir = '0 0 0';
			}
		}

		if (self.target1)
		{
			frik_movetogoal();
			bot_path();
			if (self.b_aiflags)
				bot_handle_ai();
		}
		else
		{
			// nothing to do — roam
			self._m_move_mode = MM_ROAM;
			frik_bot_roam();
			if (random() < 0.02)
				if (self.flags & FL_ONGROUND)
					self.button2 = TRUE;
		}
	}

	bot_angle_set();
	bot_debug_metrics();

	if (!self.b_aiflags)
		bot_chat();
};
