/***********************************************
*                                              *
*             FrikBot Fight Code               *
*      "Because I ain't no Ghandi code"        *
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

// check if distance is within current weapon's effective range
// v must be set to weapon_range(self.weapon) before calling
float(float dist, vector wr) bot_in_weapon_range =
{
	return (dist > wr_y && dist < wr_z);
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_combat_strafe

Handles strafe movement during combat with
periodic direction changes scaled by skill.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_combat_strafe =
{
	local vector org, movedir, right, v;
	local float foedist, strafe_interval;

	org = realorigin(self.enemy);
	foedist = self.sense_enemy_dist;
	makevectors(self.v_angle);
	right = v_right;

	// strafe interval based on skill and range
	// close-range: much faster weaving for skill 1+
	if (foedist < 150 && self.b_skill >= 1)
	{
		strafe_interval = 0.2 + random() * 0.3;
	}
	else if (self.b_skill == 0)
		strafe_interval = 2.0 + random();
	else if (self.b_skill == 1)
		strafe_interval = 1.0 + random();
	else if (self.b_skill == 2)
		strafe_interval = 0.5 + random();
	else
		strafe_interval = 0.3 + random() * 0.5;

	// personality: aggressive strafes faster, defensive slower
	if (self.b_personality == PERSONALITY_AGGRESSIVE)
		strafe_interval = strafe_interval * 0.7;
	else if (self.b_personality == PERSONALITY_DEFENSIVE)
		strafe_interval = strafe_interval * 1.3;

	// initialize strafe direction if not set
	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
		self.b_strafe_time = time + strafe_interval;
	}

	// time to change direction
	if (time > self.b_strafe_time)
	{
		// skill 3 opponent model: counter-strafe (go opposite their tendency)
		if (self.b_skill >= 3 && self.b_opp_ent == self.enemy)
		{
			if (self.b_opp_strafe > 0.4)
				self.b_strafe_dir = -1;		// they go right, we go left
			else if (self.b_opp_strafe < -0.4)
				self.b_strafe_dir = 1;
			else
				self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + strafe_interval;
		}
		// skill 1+: occasional quick fake (short strafe then switch)
		// skill 1: 10% chance, skill 2+: 20% chance
		else if (self.b_skill >= 1 && random() < (0.1 + 0.1 * (self.b_skill >= 2)))
		{
			self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + 0.15 + random() * 0.15;
		}
		else
		{
			self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + strafe_interval;
		}
	}

	// build movement: strafe + range management
	v = weapon_range(self.weapon);
	movedir = right * self.b_strafe_dir;

	if (foedist + 32 < v_x)
		movedir = movedir + normalize(self.origin - org) * 0.5; // too close, back up
	else if (foedist - 32 > v_x)
		movedir = movedir + normalize(org - self.origin) * 0.5; // too far, close in

	if (!frik_walkmove(movedir))
	{
		// obstructed, flip direction immediately
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.1;
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_retreat

Retreat behavior when outgunned. Skill-scaled
from panic freeze to smart angular retreat.
Skill 2+ uses opponent model to dodge away from
enemy's strong side.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_retreat =
{
	local vector away, org, right, movedir, toward_mate;
	local entity mate;

	org = realorigin(self.enemy);
	away = normalize(self.origin - org);
	self.b_retreat = 1;

	// skill 0: panic - barely moves
	if (self.b_skill == 0)
	{
		if (random() < 0.3)
			frik_walkmove(away);
		return;
	}

	// skill 1: run directly away (with teammate bias in teamplay)
	if (self.b_skill == 1)
	{
		if (teamplay)
		{
			mate = bot_nearest_teammate();
			if (mate != world)
			{
				toward_mate = normalize(mate.origin - self.origin);
				// blend: mostly away from enemy, slightly toward teammate
				away = normalize(away + toward_mate * 0.3);
			}
		}
		frik_walkmove(away);
		return;
	}

	// skill 2-3: angular retreat (harder to track)
	makevectors(self.v_angle);
	right = v_right;

	// initialize strafe dir if needed
	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
	}

	// opponent model: retreat away from enemy's strafe direction
	// if they tend to strafe right, we go left to break their tracking
	if (self.b_opp_ent == self.enemy && self.b_skill >= 3)
	{
		if (self.b_opp_strafe > 0.3)
			self.b_strafe_dir = -1;
		else if (self.b_opp_strafe < -0.3)
			self.b_strafe_dir = 1;
	}

	// team coordination: bias retreat toward nearest teammate
	if (teamplay)
	{
		mate = bot_nearest_teammate();
		if (mate != world)
		{
			toward_mate = normalize(mate.origin - self.origin);
			// blend teammate direction into retreat vector
			away = normalize(away + toward_mate * 0.35);
		}
	}

	movedir = normalize(away + right * self.b_strafe_dir * 0.7);

	if (!frik_walkmove(movedir))
	{
		// obstructed, try the other side
		movedir = normalize(away + right * self.b_strafe_dir * -0.7);
		frik_walkmove(movedir);
		self.b_strafe_dir = self.b_strafe_dir * -1;
	}

	// skill 3: seek nearby health while retreating
	if (self.b_skill == 3 && self.health < 50)
	{
		if (!self.target1 || self.target1.classname == "waypoint")
			bot_look_for_crap(FALSE);
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_pressure_move

Aggressive push toward a weakened enemy.
Close distance while strafing unpredictably.
Skill 2+ only.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_pressure_move =
{
	local vector org, toward, right, movedir;
	local float foedist;

	org = realorigin(self.enemy);
	toward = normalize(org - self.origin);
	foedist = self.sense_enemy_dist;
	makevectors(self.v_angle);
	right = v_right;

	// ensure strafe direction is initialized
	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
		self.b_strafe_time = time + 0.3 + random() * 0.4;
	}

	// close in aggressively but not into self-splash range
	if (foedist > 200)
		movedir = toward;
	else if (foedist > 100)
		movedir = toward * 0.3 + right * self.b_strafe_dir * 0.7;
	else
		movedir = right * self.b_strafe_dir;	// close enough, just strafe

	// fast strafe changes when pressuring
	if (time > self.b_strafe_time)
	{
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.3 + random() * 0.4;
	}

	if (!frik_walkmove(movedir))
	{
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.1;
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_flank_move

Circle-strafe to get a better splash angle.
Tries to push the enemy toward walls.
Skill 2+ only.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_flank_move =
{
	local vector org, right, movedir;
	local float foedist;

	org = realorigin(self.enemy);
	foedist = self.sense_enemy_dist;
	makevectors(self.v_angle);
	right = v_right;

	// ensure strafe direction is initialized
	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
		self.b_strafe_time = time + 0.8 + random() * 0.6;
	}

	// circle-strafe: heavy lateral component, slight range maintenance
	movedir = right * self.b_strafe_dir;
	if (foedist > 400)
		movedir = movedir + normalize(org - self.origin) * 0.4;
	else if (foedist < 200)
		movedir = movedir + normalize(self.origin - org) * 0.4;

	// moderate strafe changes
	if (time > self.b_strafe_time)
	{
		// flanking: longer commits to build angle
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.8 + random() * 0.6;
	}

	if (!frik_walkmove(movedir))
	{
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.1;
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_fight_style

Combat state machine driven by utility scoring.

Skill 0-1: simplified path (fight or retreat
based on size comparison, like original logic).

Skill 2+: full utility evaluation picks from
FIGHT, RETREAT, PRESSURE, or FLANK each cycle.
State transitions are smoothed to avoid jitter.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_drop_enemy =
{
	self.enemy = world;
	self.b_strafe_dir = 0;
	self.b_combat_state = CSTATE_NONE;
	self.b_combo_time = 0;
};

void() bot_fight_style =
{
	local vector v, v1, v2, org;
	local float foedist, mysz, foesz;
	local float u_fight, u_retreat, u_pressure, u_flank, u_best;
	local float new_state, cur_score, hyst_margin, hyst_time;

	// --- enemy validity ---
	if (self.enemy.health <= 0)
	{
		bot_drop_enemy();
		return;
	}
	else if (!self.enemy.takedamage)
	{
		bot_drop_enemy();
		return;
	}
	else if (!self.sense_enemy_vis)
	{
		// skill 1+: grace period before dropping enemy on LOS loss
		// prevents combat state thrashing when enemy ducks behind corners
		if (self.b_skill >= 1 && (time - self.b_los_time) < 0.7)
		{
			// lost LOS briefly — keep fighting but don't shoot
			// movement states continue (retreat, strafe, etc.)
		}
		else
		{
			bot_drop_enemy();
			return;
		}
	}
	else
	{
		// have LOS — update timestamp
		self.b_los_time = time;
	}

	// --- update opponent model ---
	bot_opp_model_update();

	org = realorigin(self.enemy);
	makevectors(self.v_angle);

	// --- shooting decision (universal, all states) ---
	foedist = self.sense_enemy_dist;
	v = weapon_range(self.weapon);

	// --- RL->LG combo: execute phase (skill 3, PRESSURE state) ---
	// after firing RL, switch to LG for hitscan damage while rocket flies
	if (self.b_combo_time > 0)
	{
		if (time > self.b_combo_time + 0.6 || self.enemy.health <= 0)
		{
			// combo expired or enemy dead — resume normal weapon logic
			self.b_combo_time = 0;
		}
		else if (self.weapon != IT_LIGHTNING)
		{
			// haven't switched yet — force LG
			self.impulse = 8;
		}
		else
		{
			// LG is active — shoot it, combo is running
			bot_shoot();
		}
	}

	if (bot_in_weapon_range(foedist, v))
	{
		// skip shot if teammate is in the way (from sense cache)
		if (!self.sense_friendly_block)
			bot_shoot();

		// --- RL->LG combo: initiate phase ---
		// skill 3 in PRESSURE: after firing RL at mid range, start combo
		// human emulation: only attempt 40% of the time (humans don't
		// always think to combo, and sometimes fumble the switch)
		if (self.b_skill >= 3 && self.b_combat_state == CSTATE_PRESSURE
			&& self.b_combo_time == 0
			&& self.weapon == IT_ROCKET_LAUNCHER
			&& (self.items & IT_LIGHTNING)	// have LG
			&& self.ammo_cells >= 1
			&& self.waterlevel <= 1	// LG doesn't work underwater
			&& foedist > 200 && foedist < 600
			&& random() < 0.4)
		{
			// rocket is in flight — switch to LG after brief delay
			// (human reaction to own rocket fire)
			self.b_combo_time = time + 0.05 + random() * 0.1;
			self.b_wepswitch_time = time + 1.5; // suppress normal switching
		}
		if (self.b_skill >= 1 && time > self.b_wepswitch_time && self.b_combo_time == 0)
		{
			bot_weapon_switch(foedist);
			self.b_wepswitch_time = time + 1.5;
		}
	}
	else if (self.b_combo_time == 0)
		bot_weapon_switch(foedist);

	self.b_retreat = 0;

	// --- axe-only at range: disengage and look for weapons/ammo ---
	// if we have axe and no switch pending (or switching to axe),
	// and the enemy is beyond melee range, drop combat
	if (self.weapon == IT_AXE && foedist > 64)
	{
		if (self.impulse == 0 || self.impulse == 1)
		{
			bot_drop_enemy();
			return;
		}
	}

	// --- movement constrained by AI flags ---
	if (self.b_aiflags & (AI_PRECISION | AI_BLIND | AI_OBSTRUCTED))
	{
		// constrained: simplified fight-or-retreat only
		if (self.sense_enemy_size > self.sense_my_size + 5)
		{
			bot_retreat();
			if (self.b_skill >= 1)
			{
				bot_weapon_switch(foedist);
				if (bot_in_weapon_range(foedist, v))
					bot_shoot();
			}
			return;
		}
		self.keys = self.keys & 960;
		return;
	}

	// --- projectile dodge takes priority over state movement ---
	if (self.avoid)
	{
		if (self.avoid.velocity)
			v = self.avoid.velocity;
		else
			v = normalize(self.avoid.origin - self.origin);
		v1_x = v_y * -1;
		v1_y = v_x;
		v2_x = v_y;
		v2_y = v_x * -1;
		foedist = vlen(self.avoid.origin - (self.origin + v1));
		if (foedist < vlen(self.avoid.origin - (self.origin + v2)))
			frik_walkmove(v2);
		else
			frik_walkmove(v1);
		return;
	}

	// --- skill 0: classic binary fight/retreat ---
	if (self.b_skill < 1)
	{
		foesz = self.sense_enemy_size;
		mysz = self.sense_my_size + 5;

		if (self.b_personality == PERSONALITY_AGGRESSIVE)
			mysz = mysz + 40;
		else if (self.b_personality == PERSONALITY_DEFENSIVE)
			mysz = mysz - 30;

		if (foesz > mysz)
		{
			self.b_combat_state = CSTATE_RETREAT;
			bot_retreat();
			if (teamplay && random() < 0.02)
			{
				bot_start_topic(5);
				self.b_chattime = 1;
			}
			if (self.b_skill >= 1)
			{
				bot_weapon_switch(foedist);
				if (bot_in_weapon_range(foedist, v))
					bot_shoot();
			}
			return;
		}
		self.b_combat_state = CSTATE_FIGHT;
		if (!(self.enemy.flags & FL_MONSTER))
			bot_combat_strafe();
		return;
	}

	// --- skill 1+: utility-scored state machine ---

	u_fight = bot_util_fight();
	u_retreat = bot_util_retreat();
	u_pressure = bot_util_pressure();
	u_flank = bot_util_flank();

	// find best action
	u_best = u_fight;
	new_state = CSTATE_FIGHT;

	if (u_retreat > u_best)
	{
		u_best = u_retreat;
		new_state = CSTATE_RETREAT;
	}
	if (u_pressure > u_best)
	{
		u_best = u_pressure;
		new_state = CSTATE_PRESSURE;
	}
	if (u_flank > u_best)
	{
		u_best = u_flank;
		new_state = CSTATE_FLANK;
	}

	// hysteresis: require margin to switch states (prevents jitter)
	// human emulation: higher margin = more commitment to decisions
	// skill 3: 10-point margin, 0.5s commitment (crisp reactions)
	// skill 2: 15-point margin, 0.8s commitment (slight tunnel vision)
	// skill 1: 20-point margin, 1.2s commitment (commits hard to decisions)
	if (new_state != self.b_combat_state)
	{
		if (self.b_skill >= 3)
		{	hyst_margin = 10; hyst_time = 0.5; }
		else if (self.b_skill >= 2)
		{	hyst_margin = 15; hyst_time = 0.8; }
		else
		{	hyst_margin = 20; hyst_time = 1.2; }

		if (self.b_combat_state == CSTATE_FIGHT)
			cur_score = u_fight;
		else if (self.b_combat_state == CSTATE_RETREAT)
			cur_score = u_retreat;
		else if (self.b_combat_state == CSTATE_PRESSURE)
			cur_score = u_pressure;
		else if (self.b_combat_state == CSTATE_FLANK)
			cur_score = u_flank;
		else
			cur_score = 0;

		if (u_best < cur_score + hyst_margin && (time - self.b_combat_time) < hyst_time)
			new_state = self.b_combat_state;	// keep current state
	}

	// state transition
	if (new_state != self.b_combat_state)
	{
		// cancel RL->LG combo if leaving PRESSURE
		if (self.b_combat_state == CSTATE_PRESSURE)
			self.b_combo_time = 0;
		self.b_combat_state = new_state;
		self.b_combat_time = time;
	}

	// --- execute current state ---
	if (self.b_combat_state == CSTATE_RETREAT)
	{
		bot_retreat();
		if (teamplay && random() < 0.02)
		{
			bot_start_topic(5);
			self.b_chattime = 1;
		}
		// always shoot while retreating at skill 2+
		bot_weapon_switch(foedist);
		bot_shoot();
	}
	else if (self.b_combat_state == CSTATE_PRESSURE)
	{
		bot_pressure_move();
	}
	else if (self.b_combat_state == CSTATE_FLANK)
	{
		bot_flank_move();
	}
	else
	{
		// CSTATE_FIGHT: standard combat strafe
		if (!(self.enemy.flags & FL_MONSTER))
			bot_combat_strafe();
	}
};
