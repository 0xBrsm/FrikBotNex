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
	if (delta_hp > 1) delta_hp = 1;
	if (delta_hp < -1) delta_hp = -1;
	delta_hp = (delta_hp + 1) * 0.5;		// shift to 0-1
	self.b_opp_aggro = self.b_opp_aggro * (1 - blend) + delta_hp * blend;

	// track strafe tendency from lateral velocity
	makevectors(self.v_angle);
	delta_hp = self.enemy.velocity * v_right;
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

// count how many teammates are fighting a given enemy
float(entity foe) bot_team_fighting_enemy =
{
	local entity t;
	local float count;

	if (!teamplay || !foe)
		return 0;

	count = 0;
	t = player_head;
	while (t)
	{
		if (t != self && t.team == self.team && t.health > 0)
		{
			if (t.enemy == foe && t.enemy.health > 0)
				count = count + 1;
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

// check if any teammate is retreating from a given enemy
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
	local float score, mysz, foesz, health_ratio, wep_adv, allies;

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
		local float fdist;
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
	local float score, foesz;

	score = 10;		// baseline: only pressure when clearly winning

	foesz = bot_size_player(self.enemy);

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

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_weapon_score

Score a weapon for the current situation.
Higher score = better choice.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(float wep, float brange) bot_weapon_score =
{
	local float score;
	local vector v, wr;

	// base tier scores
	if (wep == 32)
		score = 70; // RL
	else if (wep == 64)
		score = 65; // LG
	else if (wep == 8)
		score = 55; // SNG
	else if (wep == 16)
		score = 50; // GL
	else if (wep == 2)
		score = 45; // SSG
	else if (wep == 4)
		score = 35; // NG
	else if (wep == 1)
		score = 25; // SG
	else if (wep == 4096)
		score = 15; // AXE
	else
		return 0;

	// LG in water = self-discharge, never use it
	if (wep == 64 && self.waterlevel > 0)
		return 0;

	// range fitness - unusable weapons score 0
	v = weapon_range(wep);
	if (brange < v_y || brange > v_z)
		return 0;

	// self-splash danger for RL and GL
	if (wep == 32 || wep == 16)
	{
		if (brange < 120)
			score = score - 60;
		else if (brange < 200)
			score = score - 20;
	}

	// skill 2+ considerations
	if (self.b_skill >= 2)
	{
		// wall proximity bonus for splash weapons
		if (wep == 32 || wep == 16)
		{
			if (self.enemy != world)
			{
				if (bot_enemy_near_wall())
					score = score + 15;
			}
		}

		// water bonus for RL (splash amplified)
		if (wep == 32)
		{
			if (self.waterlevel >= 1)
				score = score + 10;
		}
	}

	// skill 1+ considerations
	if (self.b_skill >= 1)
	{
		if (self.enemy != world)
		{
			// finish weak enemies with cheap weapons
			if (self.enemy.health <= 40)
			{
				if (wep == 1 || wep == 2 || wep == 4096)
					score = score + 20;
				else if (wep == 32 || wep == 16)
					score = score - 15;
			}

			// ammo conservation - save rockets for important fights
			if (wep == 32 && self.ammo_rockets <= 2)
			{
				if (self.enemy.health > 60)
				{
					// still use them against quad/pent enemies
					if (!(self.enemy.items & 4194304) && !(self.enemy.items & 1048576))
						score = score - 25;
				}
			}
		}
	}

	// opponent model weapon adjustments (skill 2+)
	if (self.b_skill >= 2 && self.b_opp_ent == self.enemy)
	{
		// counter-pick: if enemy has RL/LG and is aggressive, prefer LG for hitscan
		if ((self.b_opp_weapon == 32) && self.b_opp_aggro > 0.6)
		{
			if (wep == 64) score = score + 12;	// LG to punish their rush
		}
		// enemy is passive/retreating: splash weapons to flush them out
		if (self.b_opp_aggro < 0.3)
		{
			if (wep == 32 || wep == 16) score = score + 8;
		}
		// enemy at predictable distance: boost weapons that match their range
		if (brange > 0)
		{
			wr = weapon_range(wep);
			// weapon sweet spot matches their preferred distance
			if (fabs(self.b_opp_dist - wr_x) < 100)
				score = score + 5;
		}
	}

	// personality weapon preferences
	if (self.b_personality == PERSONALITY_AGGRESSIVE)
	{
		// aggressive: loves close-range power weapons
		if (wep == 32) score = score + 10; // RL
		else if (wep == 64) score = score + 8; // LG
		else if (wep == 4096) score = score + 5; // AXE (fearless)
		else if (wep == 1 || wep == 4) score = score - 5; // weak weapons
	}
	else if (self.b_personality == PERSONALITY_DEFENSIVE)
	{
		// defensive: prefers range and safety
		if (wep == 8) score = score + 8; // SNG (safe DPS)
		else if (wep == 64) score = score + 5; // LG
		else if (wep == 32 || wep == 16)
		{
			if (brange < 250) // splash risk at close range
				score = score - 10;
		}
	}
	else if (self.b_personality == PERSONALITY_CAMPER)
	{
		// camper: prefers long-range precision weapons
		if (wep == 32) score = score + 5; // RL
		if (wep == 8) score = score + 5; // SNG
		if (wep == 4096) score = score - 10; // hates melee
	}

	if (score < 1)
		score = 1;
	return score;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_weapon_switch

Pick a weapon based on range / ammo using
situational scoring.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void(float brange) bot_weapon_switch =
{
	local float it, best_score, score, best_pulse;

	it = self.items & 127;

	best_score = 0;
	best_pulse = 0;

	// no range = just pick the best overall weapon
	if (brange == -1)
	{
		// simplified: use base tier ordering with ammo check
		if ((self.ammo_rockets >= 1) && (it & 32))
		{	best_pulse = 7; best_score = 70; }
		if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & 64))
		{	if (65 > best_score) { best_pulse = 8; best_score = 65; } }
		if (self.ammo_nails >= 2 && (it & 8))
		{	if (55 > best_score) { best_pulse = 5; best_score = 55; } }
		if ((self.ammo_rockets >= 1) && (it & 16))
		{	if (50 > best_score) { best_pulse = 6; best_score = 50; } }
		if (self.ammo_shells >= 2 && (it & 2))
		{	if (45 > best_score) { best_pulse = 3; best_score = 45; } }
		if (self.ammo_nails >= 1 && (it & 4))
		{	if (35 > best_score) { best_pulse = 4; best_score = 35; } }
		if (self.ammo_shells >= 1 && (it & 1))
		{	if (25 > best_score) { best_pulse = 2; best_score = 25; } }
		if (best_pulse)
			self.impulse = best_pulse;
		return;
	}

	// score each weapon situationally
	if ((self.ammo_rockets >= 1) && (it & 32))
	{
		score = bot_weapon_score(32, brange);
		if (score > best_score) { best_score = score; best_pulse = 7; }
	}
	if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & 64))
	{
		score = bot_weapon_score(64, brange);
		if (score > best_score) { best_score = score; best_pulse = 8; }
	}
	if (self.ammo_nails >= 2 && (it & 8))
	{
		score = bot_weapon_score(8, brange);
		if (score > best_score) { best_score = score; best_pulse = 5; }
	}
	if ((self.ammo_rockets >= 1) && (it & 16))
	{
		score = bot_weapon_score(16, brange);
		if (score > best_score) { best_score = score; best_pulse = 6; }
	}
	if (self.ammo_shells >= 2 && (it & 2))
	{
		score = bot_weapon_score(2, brange);
		if (score > best_score) { best_score = score; best_pulse = 3; }
	}
	if (self.ammo_nails >= 1 && (it & 4))
	{
		score = bot_weapon_score(4, brange);
		if (score > best_score) { best_score = score; best_pulse = 4; }
	}
	if (self.ammo_shells >= 1 && (it & 1))
	{
		score = bot_weapon_score(1, brange);
		if (score > best_score) { best_score = score; best_pulse = 2; }
	}

	if (best_pulse)
		self.impulse = best_pulse;
};

void() bot_shoot = 
{
	// quick little function to stop making him shoot the wrong way ! Argh
	local float g;
	g = angcomp(self.v_angle_x, self.b_angle_x);
	if (fabs(g) > 30)
		return; // argh, too far away
	g = angcomp(self.v_angle_y, self.b_angle_y);
	if (fabs(g) > 30)
		return; // not again!
	self.button0 = TRUE;
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
	foedist = vlen(org - self.origin);
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
		// skill 2+: occasional quick fake (short strafe then switch)
		else if (self.b_skill >= 2 && random() < 0.2)
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
	foedist = vlen(org - self.origin);
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
	foedist = vlen(org - self.origin);
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

void() bot_fight_style =
{
	local vector v, v1, v2, org;
	local float foedist, mysz, foesz;
	local float u_fight, u_retreat, u_pressure, u_flank, u_best;
	local float new_state, cur_score;

	// --- enemy validity ---
	if (self.enemy.health <= 0)
	{
		self.enemy = world;
		self.b_strafe_dir = 0;
		self.b_combat_state = CSTATE_NONE;
		return;
	}
	else if (!self.enemy.takedamage)
	{
		self.enemy = world;
		self.b_strafe_dir = 0;
		self.b_combat_state = CSTATE_NONE;
		return;
	}
	else if (!fisible(self.enemy))
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
			self.enemy = world;
			self.b_strafe_dir = 0;
			self.b_combat_state = CSTATE_NONE;
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
	foedist = vlen(org - self.origin);
	v = weapon_range(self.weapon);

	if (foedist > v_y && foedist < v_z)
	{
		traceline(self.origin + self.view_ofs, self.origin + self.view_ofs + v_forward * v_z, FALSE, self);
		if (vlen(trace_endpos - (self.origin + self.view_ofs)) >= v_y)
		{
			// skip shot if teammate is in the way, but don't freeze movement
			if (trace_ent.classname == "player")
			{
				if (!((trace_ent.team == self.team && teamplay) || (coop)))
					bot_shoot();
			}
			else
				bot_shoot();
		}
		if (self.b_skill >= 1 && time > self.b_wepswitch_time)
		{
			bot_weapon_switch(foedist);
			self.b_wepswitch_time = time + 1.5;
		}
	}
	else
		bot_weapon_switch(foedist);

	self.b_retreat = 0;

	// --- movement constrained by AI flags ---
	if (self.b_aiflags & (AI_PRECISION | AI_BLIND | AI_OBSTRUCTED))
	{
		// constrained: simplified fight-or-retreat only
		if (bot_size_player(self.enemy) > bot_size_player(self) + 5)
		{
			bot_retreat();
			if (self.b_skill >= 1)
			{
				bot_weapon_switch(vlen(org - self.origin));
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

	// --- skill 0-1: classic binary fight/retreat ---
	if (self.b_skill < 2)
	{
		foesz = bot_size_player(self.enemy);
		mysz = bot_size_player(self) + 5;

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
				bot_weapon_switch(vlen(org - self.origin));
				bot_shoot();
			}
			return;
		}
		if (mysz < 140)
			return;
		self.b_combat_state = CSTATE_FIGHT;
		if (!(self.enemy.flags & FL_MONSTER))
			bot_combat_strafe();
		return;
	}

	// --- skill 2+: utility-scored state machine ---

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
	// must beat current state by 10+ points, or 0.5s in current state
	if (new_state != self.b_combat_state)
	{
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

		if (u_best < cur_score + 10 && (time - self.b_combat_time) < 0.5)
			new_state = self.b_combat_state;	// keep current state
	}

	// state transition
	if (new_state != self.b_combat_state)
	{
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
		bot_weapon_switch(vlen(org - self.origin));
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


