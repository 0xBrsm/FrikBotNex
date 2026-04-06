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


// --- shared turn: flick toward b_angle, skill-scaled ---
// Exponential decay: covers (decay) fraction of remaining angle per frame.
// Low skill = sluggish, high skill = snappy flicks.
void() bot_look =
{
	local float decay;
	local float dyaw, dpitch;

	decay = 0.3 + self.b_skill * 0.15; // skill 0=0.3, skill 1=0.45, skill 2=0.6, skill 3=0.75

	dyaw = self.b_angle_y - self.v_angle_y;
	while (dyaw > 180) dyaw = dyaw - 360;
	while (dyaw < -180) dyaw = dyaw + 360;
	dpitch = self.b_angle_x - self.v_angle_x;
	while (dpitch > 180) dpitch = dpitch - 360;
	while (dpitch < -180) dpitch = dpitch + 360;

	self.v_angle_y = self.v_angle_y + dyaw * decay;
	self.v_angle_x = self.v_angle_x + dpitch * decay;
	while (self.v_angle_x < -180)
		self.v_angle_x = self.v_angle_x + 360;
	while (self.v_angle_x > 180)
		self.v_angle_x = self.v_angle_x - 360;
};

// --- combat aim: set b_angle toward enemy with prediction ---
void() bot_aim =
{
	local float h;
	local vector view, pred_vel, foe_fwd, foe_right;

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

// roaming (no goal — wander randomly)
.vector _roam_dir;
.float _roam_change_time;

// navmesh debug visualization
float nav_debug_total;
float nav_debug_idx;
float nav_debug_next;

// debug: per-bot movement metrics (accumulated, printed every 10s)
.float _m_stuck;       // frames where bot wanted to move but couldn't (geometry)
.float _m_idle;        // frames where bot had no movement intent
.vector _m_last_origin; // previous frame position for stuck detection
.float _m_lava;        // frames on lava/slime
.float _m_nodir;       // frames with sense_best_dir == -1
.float _m_frames;      // total alive frames
.float _m_report_time; // next report time

// lava entry tracking
.float _m_was_lava;
.float _m_le_nav, _m_le_beeline, _m_le_combat, _m_le_roam;

// target tracking
.float _m_idle_notgt;  // frames idle with no target
.float _m_idle_roam;   // frames idle with roam target (self)
.float _m_polys_seen;  // unique polys visited (coverage proxy)
.float _m_last_poly;   // last poly ref seen (for change detection)

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

	// stuck = wanted to move but didn't (geometry impact)
	// idle = no movement intent (no goal, between paths, etc.)
	vel = self.origin - self._m_last_origin;
	vel_z = 0;
	if (vlen(vel) < 1)
	{
		if (self.b_wishdir != '0 0 0')
			self._m_stuck = self._m_stuck + 1;
		else
			self._m_idle = self._m_idle + 1;
	}
	self._m_last_origin = self.origin;

	// idle breakdown: no target vs roam
	if (vlen(vel) < 1 && self.b_wishdir == '0 0 0')
	{
		if (!self.target1)
			self._m_idle_notgt = self._m_idle_notgt + 1;
		else if (self.target1 == self)
			self._m_idle_roam = self._m_idle_roam + 1;
	}

	// coverage: count navigate() poly changes as proxy for area explored
	{
		local vector dbg;
		dbg = nav_path_debug(self, -1);
		if (dbg_x != self._m_last_poly)
		{
			self._m_polys_seen = self._m_polys_seen + 1;
			self._m_last_poly = dbg_x;
		}
	}

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
	s = ftos(rint(100 * self._m_stuck / self._m_frames));
	bprint("stk="); bprint(s);
	s = ftos(rint(100 * self._m_idle_notgt / self._m_frames));
	bprint("% no_tgt="); bprint(s);
	s = ftos(rint(100 * self._m_idle_roam / self._m_frames));
	bprint("% roam_idle="); bprint(s);
	s = ftos(rint(100 * self._m_lava / self._m_frames));
	bprint("% lav="); bprint(s);
	bprint("% cov=");
	s = ftos(self._m_polys_seen); bprint(s);
	bprint(" ");
	if (self.target1 == self)
		bprint("tgt=ROAM");
	else if (self.target1)
	{
		bprint("tgt="); bprint(self.target1.classname);
		s = ftos(rint(vlen(self.target1.origin - self.origin)));
		bprint(" d="); bprint(s);
	}
	else
		bprint("tgt=NONE");
	bprint("\n");

	// reset counters
	self._m_stuck = 0;
	self._m_idle = 0;
	self._m_idle_notgt = 0;
	self._m_idle_roam = 0;
	self._m_lava = 0;
	self._m_nodir = 0;
	self._m_frames = 0;
	self._m_le_nav = 0;
	self._m_le_beeline = 0;
	self._m_le_combat = 0;
	self._m_goals_reached = 0;
	self._m_le_roam = 0;
	self._m_polys_seen = 0;
};

void() BotAI =
{
	// --- dead: mash respawn ---
	if (self.health < 1)
	{
		self.button0 = floor(random() * 2);
		self.button2 = 0;
		self.b_wishdir = '0 0 0';
		self.target1 = self.enemy = world;
		self.b_aim_delay = 0;
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
	self.b_wishdir = '0 0 0';

	// --- perceive ---
	bot_projectile_scan();
	bot_sense();

	// --- survive: emergency escape from drowning/lava ---
	if (self.waterlevel > 2 && time > (self.air_finished - 2))
	{
		traceline(self.origin, self.origin + '0 0 6800', TRUE, self);
		if (trace_inopen)
		{
			self.b_wishdir = '0 0 1';
			self.button2 = TRUE;
			self.b_angle = vectoangles(self.b_wishdir);
			bot_look();
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
			if (!item_head)
				map_stuff();
			self.target1 = nav_find_goal();
			if (self.target1)
			{
				// reset progress tracker for new goal
				self._progress_best = 99999;
				self._progress_stall = 0;
			}
			else
			{
				// no goal — walk in a random direction to find new area
				makevectors('0 1 0' * (random() * 360));
				self.b_wishdir = v_forward;
			}
		}

		// clear target if item was picked up (model gone)
		if (self.target1)
			if (self.target1.model == string_null)
				self.target1 = world;

		if (self.target1)
			bot_get();
	}

	// --- set look target ---
	if (self.enemy)
		bot_aim();
	else if (self.b_wishdir != '0 0 0')
	{
		local vector _nav_view;
		_nav_view = vectoangles(self.b_wishdir);
		_nav_view_x = _nav_view_x * -1;
		self.b_angle = _nav_view;
	}
	bot_look();

	bot_debug_metrics();
	bot_chat();
};
