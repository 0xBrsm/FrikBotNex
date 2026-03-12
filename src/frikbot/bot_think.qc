/***********************************************
*                                              *
*           FrikBot Think Code                 *
*      "The bot's inner monologue"             *
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

.entity avoid;

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Opponent Modeling

Skill 2+ bots track their current enemy's
behavior: weapon, aggression, strafe pattern,
preferred distance. Updated each AI frame
while in combat. The model persists as long
as the same enemy is engaged.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_opp_model_reset =
{
	self.b_opp_ent = world;
	self.b_opp_weapon = 0;
	self.b_opp_aggro = 0.5;
	self.b_opp_strafe = 0;
	self.b_opp_dist = 300;
	self.b_opp_track_time = 0;
};

void() bot_opp_model_update =
{
	local float dist, blend, delta_hp;
	local vector v;

	if (self.b_skill < 2)
		return;
	if (!self.enemy)
		return;

	// new enemy — reset model
	if (self.b_opp_ent != self.enemy)
	{
		bot_opp_model_reset();
		self.b_opp_ent = self.enemy;
		self.b_opp_track_time = time;
		self.b_opp_weapon = self.enemy.weapon;
		return;
	}

	if (time < self.b_opp_track_time + 0.1)
		return;

	// exponential moving average blend factor
	// quick adaptation for skill 3, slower for skill 2
	if (self.b_skill >= 3)
		blend = 0.3;
	else
		blend = 0.15;

	// track weapon changes
	self.b_opp_weapon = self.enemy.weapon;

	// track distance preference
	v = self.enemy.origin - self.origin;
	dist = vlen(v);
	self.b_opp_dist = self.b_opp_dist * (1 - blend) + dist * blend;

	// track aggression: enemy closing distance = aggressive
	// enemy increasing distance = passive
	// normalize: closing at 400ups+ = 1.0, backing at 400ups+ = 0.0
	v = normalize(self.origin - self.enemy.origin);
	delta_hp = self.enemy.velocity * v;	// positive = closing
	delta_hp = delta_hp / 400;			// normalize to 0-1 range
	// human emulation: perception noise on reading enemy movement
	delta_hp = delta_hp + (random() - 0.5) * 0.3;
	if (delta_hp > 1) delta_hp = 1;
	if (delta_hp < -1) delta_hp = -1;
	delta_hp = (delta_hp + 1) * 0.5;		// shift to 0-1
	self.b_opp_aggro = self.b_opp_aggro * (1 - blend) + delta_hp * blend;

	// track strafe tendency from lateral velocity
	makevectors(self.v_angle);
	delta_hp = self.enemy.velocity * v_right;
	// human emulation: noisy read on lateral movement
	delta_hp = delta_hp + (random() - 0.5) * 40;
	if (delta_hp > 50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) + 1 * blend;
	else if (delta_hp < -50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) + (-1) * blend;
	else
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend * 0.5);

	self.b_opp_track_time = time;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Team Coordination (skill 1+ in teamplay)

Bots share information through entity fields:
- .enemy tells teammates who you're fighting
- .b_combat_state tells teammates if you're
  retreating, pressuring, etc.
- .health/.armorvalue for teammate strength

Three integration points:
1. Focus fire: prioritize enemies teammates fight
2. Help retreating: rush enemies chasing teammates
3. Retreat toward teammates for crossfire support

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// count how many nearby teammates are fighting a given enemy
// human emulation: only count allies within awareness range
// (humans can't know what distant teammates are doing)
float(entity foe) bot_team_fighting_enemy =
{
	local entity t;
	local float count, dist;

	if (!teamplay || !foe)
		return 0;

	count = 0;
	t = player_head;
	while (t)
	{
		if (t != self && t.team == self.team && t.health > 0)
		{
			if (t.enemy == foe && t.enemy.health > 0)
			{
				// must be within earshot/sight range to know
				dist = vlen(t.origin - self.origin);
				if (dist < 1000)
					count = count + 1;
			}
		}
		t = t._next;
	}
	return count;
};

// find nearest visible teammate (for retreat direction bias)
entity() bot_nearest_teammate =
{
	local entity t, best;
	local float dist, bestdist;

	best = world;
	bestdist = 100000;
	t = player_head;
	while (t)
	{
		if (t != self && t.team == self.team && t.health > 0)
		{
			dist = vlen(t.origin - self.origin);
			if (dist < bestdist && dist > 64)
			{
				bestdist = dist;
				best = t;
			}
		}
		t = t._next;
	}
	return best;
};

// check if any nearby teammate is retreating from a given enemy
// human emulation: must be close enough to notice teammate's situation
float(entity foe) bot_teammate_retreating_from =
{
	local entity t;

	if (!teamplay || !foe)
		return FALSE;

	t = player_head;
	while (t)
	{
		if (t != self && t.team == self.team && t.health > 0)
		{
			if (t.enemy == foe && t.b_combat_state == CSTATE_RETREAT)
				if (vlen(t.origin - self.origin) < 800)
					return TRUE;
		}
		t = t._next;
	}
	return FALSE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Utility AI — Combat Decision Scoring

Each possible combat action is scored 0-100.
The state machine picks the highest-scoring
action each evaluation cycle. This replaces
the binary fight-or-retreat check.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// how good is staying and fighting right now?
float() bot_util_fight =
{
	local float score, mysz, foesz, health_ratio, wep_adv, allies, fdist;

	score = 50;		// baseline: fighting is the default

	mysz = bot_size_player(self);
	foesz = bot_size_player(self.enemy);

	// health+armor ratio
	health_ratio = mysz / (foesz + 1);
	if (health_ratio > 1.5)
		score = score + 15;		// we're stronger
	else if (health_ratio > 1.0)
		score = score + 5;
	else if (health_ratio < 0.5)
		score = score - 25;		// we're much weaker
	else if (health_ratio < 0.75)
		score = score - 10;

	// weapon advantage
	wep_adv = bot_weapon_score(self.weapon, vlen(self.enemy.origin - self.origin));
	wep_adv = wep_adv - 40;	// center around 0
	score = score + wep_adv * 0.3;

	// aim convergence: if we're aimed in, fighting is better
	if (self.b_aim_offset < 5)
		score = score + 10;

	// personality
	if (self.b_personality == PERSONALITY_AGGRESSIVE)
		score = score + 12;
	else if (self.b_personality == PERSONALITY_DEFENSIVE)
		score = score - 8;

	// opponent model: passive enemy is easier to fight
	if (self.b_skill >= 2 && self.b_opp_ent == self.enemy)
	{
		if (self.b_opp_aggro < 0.3)
			score = score + 8;	// they're running, press advantage
	}

	// team coordination: teammates fighting same enemy = confidence
	if (teamplay && self.b_skill >= 1)
	{
		allies = bot_team_fighting_enemy(self.enemy);
		if (allies >= 1)
			score = score + 12;	// outnumber them, stay and fight
		if (allies >= 2)
			score = score + 8;	// 3v1, definitely fight
	}

	// splash weapons at mid-range with no wall angle: flanking would be better
	if (self.b_skill >= 2 && (self.weapon == 32 || self.weapon == 16))
	{
		fdist = vlen(self.enemy.origin - self.origin);
		if (fdist > 200 && fdist < 600 && !bot_enemy_near_wall())
			score = score - 12;
	}

	if (score < 0) score = 0;
	if (score > 100) score = 100;
	return score;
};

// how much do we want to retreat?
float() bot_util_retreat =
{
	local float score, mysz, foesz, health_ratio, allies;

	score = 20;		// baseline: retreating is not default

	mysz = bot_size_player(self);
	foesz = bot_size_player(self.enemy);

	health_ratio = mysz / (foesz + 1);
	if (health_ratio < 0.4)
		score = score + 40;		// badly outgunned
	else if (health_ratio < 0.6)
		score = score + 25;
	else if (health_ratio < 0.8)
		score = score + 10;
	else if (health_ratio > 1.5)
		score = score - 15;		// we're winning, don't run

	// low health absolute threshold
	if (self.health < 30)
		score = score + 20;
	else if (self.health < 50)
		score = score + 10;

	// enemy has powerup
	if (self.enemy.items & 4194304)	// quad
		score = score + 30;
	if (self.enemy.items & 1048576)	// invuln
		score = score + 40;

	// personality
	if (self.b_personality == PERSONALITY_DEFENSIVE)
		score = score + 15;
	else if (self.b_personality == PERSONALITY_AGGRESSIVE)
		score = score - 20;

	// opponent model: aggressive enemy is scarier
	if (self.b_skill >= 2 && self.b_opp_ent == self.enemy)
	{
		if (self.b_opp_aggro > 0.7)
			score = score + 10;
		// enemy has RL/LG and is closing
		if ((self.b_opp_weapon == 32 || self.b_opp_weapon == 64) && self.b_opp_aggro > 0.6)
			score = score + 10;
	}

	// team coordination: less retreat incentive with teammates nearby
	if (teamplay && self.b_skill >= 1)
	{
		allies = bot_team_fighting_enemy(self.enemy);
		if (allies >= 1)
			score = score - 15;	// teammate has your back
		if (allies >= 2)
			score = score - 10;	// strong numbers advantage
	}

	if (score < 0) score = 0;
	if (score > 100) score = 100;
	return score;
};

// should we rush a weakened enemy?
float() bot_util_pressure =
{
	local float score;

	score = 10;		// baseline: only pressure when clearly winning

	// enemy is weak
	if (self.enemy.health <= 30)
		score = score + 35;
	else if (self.enemy.health <= 50)
		score = score + 20;
	else if (self.enemy.health <= 75)
		score = score + 8;

	// we have a strong weapon
	if (self.weapon == 32 || self.weapon == 64)
		score = score + 15;
	else if (self.weapon == 8)
		score = score + 8;

	// we're healthy
	if (self.health > 80)
		score = score + 10;

	// we have powerup
	if (self.items & 4194304)	// quad
		score = score + 30;

	// personality
	if (self.b_personality == PERSONALITY_AGGRESSIVE)
		score = score + 15;
	else if (self.b_personality == PERSONALITY_DEFENSIVE)
		score = score - 10;
	else if (self.b_personality == PERSONALITY_CAMPER)
		score = score - 15;

	// opponent model: passive enemy is easier to pressure
	if (self.b_skill >= 2 && self.b_opp_ent == self.enemy)
	{
		if (self.b_opp_aggro < 0.3)
			score = score + 12;
	}

	// team coordination: teammates fighting same enemy = pack mentality
	if (teamplay && self.b_skill >= 1)
	{
		if (bot_team_fighting_enemy(self.enemy) >= 1)
			score = score + 10;	// teammate softening them up
	}

	// only skill 1+ can pressure
	if (self.b_skill < 1)
		score = 0;

	if (score < 0) score = 0;
	if (score > 100) score = 100;
	return score;
};

// should we try to get a better angle (splash weapons near walls)?
float() bot_util_flank =
{
	local float score, foedist;

	score = 20;		// baseline: flanking is a valid tactic with splash weapons

	// only useful with splash weapons
	if (self.weapon != 32 && self.weapon != 16)
		return 0;

	foedist = vlen(self.enemy.origin - self.origin);

	// mid-range is ideal for flanking
	if (foedist > 200 && foedist < 600)
		score = score + 15;
	else
		return 0;

	// enemy not already near wall — flanking might push them toward one
	if (!bot_enemy_near_wall())
		score = score + 10;
	else
		score = score - 10;	// already have the angle, just fight

	// we have rockets
	if (self.weapon == 32 && self.ammo_rockets >= 3)
		score = score + 10;

	// skill 2+ only
	if (self.b_skill < 2)
		return 0;

	// personality
	if (self.b_personality == PERSONALITY_CAMPER)
		score = score + 10;

	if (score < 0) score = 0;
	if (score > 100) score = 100;
	return score;
};

float(entity e) bot_size_player =
{
	local float sz;

	sz = e.health + e.armorvalue * e.armortype;
	if (e.weapon == 32)
		sz = sz + 60;
	else if (e.weapon == 64)
		sz = sz + 60;
	else if (e.weapon == 16)
		sz = sz + 50;
	else if (e.weapon == 8)
		sz = sz + 50;
	else if (e.weapon == 4)
		sz = sz + 40;
	else if (e.weapon == 2)
		sz = sz + 40;
	else if (e.weapon == 1)
		sz = sz + 10;
	else if (e.weapon == 4096)
		sz = sz - 50;
	if (e.items & 4194304) // Quad
		sz = sz + 200;
	if (e.items & 1048576) // Invul
		sz = sz + 300;
	if (e.items & 524288) // Invis
		sz = sz + 250;
	if (sz < 1)
		sz = 1;
	return sz;
};

void() bot_dodge_stuff =
{
	local entity foe;
	local float avdist, foesz, flen, tsz;
	local vector v;

	if (waypoint_mode > WM_LOADED)
		return;

	self.avoid = world;


	if (self.enemy)
	{
		v = self.origin - realorigin(self.enemy);

		foesz = bot_size_player(self.enemy);
		foesz = foesz + vlen(v) * 0.5;
	}
	else
		foesz = 9999999;
	avdist = 256;

	foe = find(world, classname, "grenade");
	while(foe)
	{
		flen = vlen(foe.origin - self.origin);
		if (flen < avdist)
		{
			avdist = flen;
			self.avoid = foe;
		}
		foe = find(foe, classname, "grenade");
	}
	if (!self.avoid)
	{
		foe = find(world, classname, "missile");
		while(foe)
		{
			if (foe.owner != self)
			{
				flen = vlen(foe.origin - self.origin);
				if (flen < avdist)
				{
					avdist = flen;
					self.avoid = foe;
				}
			}
			foe = find(foe, classname, "missile");
		}
		if (!self.avoid)
		{
			foe = find(world, classname, "spike");
			while(foe)
			{
				if (foe.owner != self)
				{
					flen = vlen(foe.origin - self.origin);
					if (flen < avdist)
					{
						avdist = flen;
						self.avoid = foe;
					}
				}
				foe = find(foe, classname, "spike");
			}
		}
	}
	if (coop)
	{
		if (!self.enemy)
		{
			foe = findradius(self.origin, 9999);
			while(foe)
			{
				if(foe.flags & FL_MONSTER)
				{
					if(foe.health > 0)
					{
						tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
						if (tsz < foesz)
						{
							if (fisible(foe))
							{
								self.enemy = foe;
								foesz = tsz;
							}
						}
					}
				}
				foe = foe.chain;
			}
		}
	}
	else
	{
		foe = player_head;
		while(foe)
		{
			if(foe != self)
			{
				if (foe.modelindex != 0)
				{
					if (foe.health > 0)
					{
						if (!(teamplay && self.team == foe.team))
						{
							tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
							if (tsz < foesz)
							{
								// ambush: 360-degree awareness, skip FOV check
								if (self.b_ambush_time > 0 || fov(foe) || foe.b_sound > time || self.b_skill == 3)
								{
									if (fisible(foe))
									{
										self.enemy = foe;
										foesz = tsz;
									}
								}
							}
						}
					}
				}
			}
			foe = foe._next;
		}

		// human emulation: reaction delay before engaging new enemies
		// first detection starts a timer; enemy isn't fought until it expires
		if (self.enemy && self.b_skill < 3)
		{
			if (self.b_react_time == 0)
			{
				// first frame we spotted someone — start reaction timer
				if (self.b_skill == 0)
					self.b_react_time = time + 0.3 + random() * 0.4;
				else if (self.b_skill == 1)
					self.b_react_time = time + 0.15 + random() * 0.25;
				else
					self.b_react_time = time + 0.05 + random() * 0.1;
			}
			if (self.b_react_time > time)
			{
				// still reacting — suppress engagement
				self.enemy = world;
			}
			else
			{
				// reaction complete — engage and reset for next time
				self.b_react_time = 0;
			}
		}
		else if (!self.enemy)
		{
			// lost sight during reaction — reset timer
			self.b_react_time = 0;
		}
	}

	// ambush first-strike: if we just spotted an enemy from ambush,
	// snap to target and fire immediately
	if (self.b_ambush_time > 0 && self.enemy)
	{
		// snap aim toward enemy
		v = normalize(realorigin(self.enemy) - self.origin);
		v = vectoangles(v);
		v_x = v_x * -1;
		self.b_angle = v;

		// skill 1+: instant snap aim for first shot
		if (self.b_skill >= 1)
		{
			self.v_angle = v;
		}

		// fire immediately
		bot_weapon_switch(vlen(realorigin(self.enemy) - self.origin));
		bot_shoot();

		// clear ambush state - we've been spotted
		self.b_ambush_time = 0;
		self.b_aiflags = self.b_aiflags - (self.b_aiflags & AI_WAIT);
	}
};




/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

weapon_range

_x "sweet spot range" - try to maintain this range if possible
_y minimum range bot can be to be effective (rl/gl) (move away)
_z maximum range bot can be to be effective (lg/axe) (move in)
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

vector(float wep) weapon_range =
{
	if (wep == 4096) // IT_AXE
		return '48 0 64';
	else if (wep == 1) // IT_SHOTGUN
		//~ ai fix start
		//~ return '128 0 99999';
		return '128 8 3000';
		//~ ai fix end
	else if (wep == 2) // IT_SUPER_SHOTGUN
		//~ ai fix start
		//~ return '128 0 99999';
		return '128 0 3000';
		//~ ai fix end
	else if (wep == 4) // IT_NAILGUN
		return '180 0 3000';
	else if (wep == 8) // IT_SUPER_NAILGUN
		return '180 0 3000';
	else if (wep == 16) // IT_GRENADE_LAUNCHER
		return '180 48 3000';
	else if (wep == 32) // IT_ROCKET_LAUNCHER
		return '180 48 3000';
	else if (wep == 64) // IT_LIGHTNING
		return '350 0 512';
};
/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_enemy_near_wall

Check if enemy has a wall close behind them
(splash damage bonus)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_enemy_near_wall =
{
	local vector dir, org;

	// return cached result if already computed this frame
	if (self.b_wall_time == time)
		return self.b_wall_near;

	org = realorigin(self.enemy);
	dir = normalize(org - self.origin);
	traceline(org, org + dir * 100, TRUE, self.enemy);

	self.b_wall_time = time;
	if (trace_fraction < 1)
	{
		self.b_wall_near = TRUE;
		return TRUE;
	}
	self.b_wall_near = FALSE;
	return FALSE;
};
