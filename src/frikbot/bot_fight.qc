/***********************************************
*                                              *
*           FrikBot Fight Code                 *
*    "Because I ain't no Ghandi code"          *
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

"Know thy enemy" — Opponent Modeling

Skill 1+ bots track enemy behavior: weapon,
aggression, strafe tendency, distance. The
model persists while fighting the same enemy.

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
	local float dist, blend, delta;
	local vector v;

	if (self.b_skill < 1 || !self.enemy)
		return;

	// new enemy — fresh model
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

	// EMA blend: skill 3 adapts fast, lower skills are sluggish
	if (self.b_skill >= 3)
		blend = 0.3;
	else if (self.b_skill >= 2)
		blend = 0.15;
	else
		blend = 0.08;

	self.b_opp_weapon = self.enemy.weapon;

	// distance preference
	v = self.enemy.origin - self.origin;
	dist = vlen(v);
	self.b_opp_dist = self.b_opp_dist * (1 - blend) + dist * blend;

	// aggression: closing = aggressive, backing = passive
	v = normalize(self.origin - self.enemy.origin);
	delta = self.enemy.velocity * v;
	delta = delta / 400;
	delta = delta + (random() - 0.5) * 0.3;	// perception noise
	if (delta > 1) delta = 1;
	if (delta < -1) delta = -1;
	delta = (delta + 1) * 0.5;
	self.b_opp_aggro = self.b_opp_aggro * (1 - blend) + delta * blend;

	// lateral movement tendency
	makevectors(self.v_angle);
	delta = self.enemy.velocity * v_right;
	delta = delta + (random() - 0.5) * 40;		// noisy read
	if (delta > 50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) + 1 * blend;
	else if (delta < -50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) + (-1) * blend;
	else
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend * 0.5);

	self.b_opp_track_time = time;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"The buddy system" — Team Coordination

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

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
			if (t.enemy == foe && t.enemy.health > 0)
				if (vlen(t.origin - self.origin) < 1000)
					count = count + 1;
		t = t._next;
	}
	return count;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Measuring the competition" — Player Sizing

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(entity e) bot_size_player =
{
	local float sz;

	sz = e.health + e.armorvalue * e.armortype;
	if (e.weapon == IT_ROCKET_LAUNCHER || e.weapon == IT_LIGHTNING)
		sz = sz + 60;
	else if (e.weapon == IT_GRENADE_LAUNCHER || e.weapon == IT_SUPER_NAILGUN)
		sz = sz + 50;
	else if (e.weapon == IT_NAILGUN || e.weapon == IT_SUPER_SHOTGUN)
		sz = sz + 40;
	else if (e.weapon == IT_SHOTGUN)
		sz = sz + 10;
	else if (e.weapon == IT_AXE)
		sz = sz - 50;
	if (e.items & IT_QUAD)
		sz = sz + 200;
	if (e.items & IT_INVULNERABILITY)
		sz = sz + 300;
	if (e.items & IT_INVISIBILITY)
		sz = sz + 250;
	if (sz < 1)
		sz = 1;
	return sz;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

weapon_range

_x = sweet spot (try to maintain)
_y = min effective range (move away if closer)
_z = max effective range (move in if farther)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

vector(float wep) weapon_range =
{
	if (wep == IT_AXE)
		return '48 0 64';
	else if (wep == IT_SHOTGUN)
		return '128 8 3000';
	else if (wep == IT_SUPER_SHOTGUN)
		return '128 0 3000';
	else if (wep == IT_NAILGUN)
		return '180 0 3000';
	else if (wep == IT_SUPER_NAILGUN)
		return '180 0 3000';
	else if (wep == IT_GRENADE_LAUNCHER)
		return '180 48 3000';
	else if (wep == IT_ROCKET_LAUNCHER)
		return '180 48 3000';
	else if (wep == IT_LIGHTNING)
		return '350 0 512';
	else
		return '128 0 3000';
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Walls are your friend" — Splash Helpers

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_enemy_near_wall =
{
	return self.sense_enemy_wall;
};

float(float dist, vector wr) bot_in_weapon_range =
{
	return (dist > wr_y && dist < wr_z);
};

float() bot_has_ammo =
{
	local float it;
	it = self.items & 127;
	if ((it & IT_ROCKET_LAUNCHER) && self.ammo_rockets >= 1) return TRUE;
	if ((it & IT_LIGHTNING) && self.ammo_cells >= 1) return TRUE;
	if ((it & IT_GRENADE_LAUNCHER) && self.ammo_rockets >= 1) return TRUE;
	if ((it & IT_SUPER_NAILGUN) && self.ammo_nails >= 2) return TRUE;
	if ((it & IT_NAILGUN) && self.ammo_nails >= 1) return TRUE;
	if ((it & IT_SUPER_SHOTGUN) && self.ammo_shells >= 2) return TRUE;
	if ((it & IT_SHOTGUN) && self.ammo_shells >= 1) return TRUE;
	return FALSE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Guns don't kill people, impulse commands do"
    Weapon Scoring, Switching, and Shooting

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(float wep, float brange) bot_weapon_score =
{
	local float score;
	local vector wr;

	// base tier scores — bigger boom = bigger number
	if (wep == IT_ROCKET_LAUNCHER)
		score = 70;
	else if (wep == IT_LIGHTNING)
		score = 65;
	else if (wep == IT_SUPER_NAILGUN)
		score = 55;
	else if (wep == IT_GRENADE_LAUNCHER)
		score = 50;
	else if (wep == IT_SUPER_SHOTGUN)
		score = 45;
	else if (wep == IT_NAILGUN)
		score = 35;
	else if (wep == IT_SHOTGUN)
		score = 25;
	else if (wep == IT_AXE)
		score = 15;
	else
		return 0;

	// LG in water = shocking yourself. not the fun kind
	if (wep == IT_LIGHTNING && self.waterlevel > 1)
		return 0;

	// range fitness — unusable weapons score zilch
	wr = weapon_range(wep);
	if (brange < wr_y || brange > wr_z)
		return 0;

	// self-splash danger for the boom tubes
	if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
	{
		if (brange < 120)
			score = score - 60;
		else if (brange < 200)
			score = score - 20;
	}

	if (self.b_skill >= 1)
	{
		// wall proximity bonus for splash weapons
		if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
			if (self.enemy != world && bot_enemy_near_wall())
				score = score + 15;

		// water bonus for RL (splash amplified)
		if (wep == IT_ROCKET_LAUNCHER && self.waterlevel >= 1)
			score = score + 10;

		// finish weak enemies with cheap ammo
		if (self.enemy != world && self.enemy.health <= 40)
		{
			if (wep == IT_SHOTGUN || wep == IT_SUPER_SHOTGUN || wep == IT_AXE)
				score = score + 20;
			else if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
				score = score - 15;
		}

		// ammo conservation — save rockets for important fights
		if (wep == IT_ROCKET_LAUNCHER && self.ammo_rockets <= 2)
			if (self.enemy.health > 60)
				if (!(self.enemy.items & IT_QUAD) && !(self.enemy.items & IT_INVULNERABILITY))
					score = score - 25;
	}

	// opponent model counter-picks (skill 1+)
	if (self.b_skill >= 1 && self.b_opp_ent == self.enemy)
	{
		if (self.b_opp_weapon == IT_ROCKET_LAUNCHER && self.b_opp_aggro > 0.6)
			if (wep == IT_LIGHTNING) score = score + 12;
		if (self.b_opp_aggro < 0.3)
			if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
				score = score + 8;
	}

	if (score < 1)
		score = 1;
	return score;
};

void(float brange) bot_weapon_switch =
{
	local float it, best_score, score, best_pulse;

	it = self.items & 127;
	best_score = 0;
	best_pulse = 0;

	// no range = just pick the best overall weapon
	if (brange == -1)
	{
		if ((self.ammo_rockets >= 1) && (it & IT_ROCKET_LAUNCHER))
		{	best_pulse = 7; best_score = 70; }
		if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & IT_LIGHTNING))
		{	if (65 > best_score) { best_pulse = 8; best_score = 65; } }
		if (self.ammo_nails >= 2 && (it & IT_SUPER_NAILGUN))
		{	if (55 > best_score) { best_pulse = 5; best_score = 55; } }
		if ((self.ammo_rockets >= 1) && (it & IT_GRENADE_LAUNCHER))
		{	if (50 > best_score) { best_pulse = 6; best_score = 50; } }
		if (self.ammo_shells >= 2 && (it & IT_SUPER_SHOTGUN))
		{	if (45 > best_score) { best_pulse = 3; best_score = 45; } }
		if (self.ammo_nails >= 1 && (it & IT_NAILGUN))
		{	if (35 > best_score) { best_pulse = 4; best_score = 35; } }
		if (self.ammo_shells >= 1 && (it & IT_SHOTGUN))
		{	if (25 > best_score) { best_pulse = 2; best_score = 25; } }
		if (!best_pulse)
			best_pulse = 1;
		self.impulse = best_pulse;
		return;
	}

	// score each weapon for the current fight
	if ((self.ammo_rockets >= 1) && (it & IT_ROCKET_LAUNCHER))
	{
		score = bot_weapon_score(IT_ROCKET_LAUNCHER, brange);
		if (score > best_score) { best_score = score; best_pulse = 7; }
	}
	if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & IT_LIGHTNING))
	{
		score = bot_weapon_score(IT_LIGHTNING, brange);
		if (score > best_score) { best_score = score; best_pulse = 8; }
	}
	if (self.ammo_nails >= 2 && (it & IT_SUPER_NAILGUN))
	{
		score = bot_weapon_score(IT_SUPER_NAILGUN, brange);
		if (score > best_score) { best_score = score; best_pulse = 5; }
	}
	if ((self.ammo_rockets >= 1) && (it & IT_GRENADE_LAUNCHER))
	{
		score = bot_weapon_score(IT_GRENADE_LAUNCHER, brange);
		if (score > best_score) { best_score = score; best_pulse = 6; }
	}
	if (self.ammo_shells >= 2 && (it & IT_SUPER_SHOTGUN))
	{
		score = bot_weapon_score(IT_SUPER_SHOTGUN, brange);
		if (score > best_score) { best_score = score; best_pulse = 3; }
	}
	if (self.ammo_nails >= 1 && (it & IT_NAILGUN))
	{
		score = bot_weapon_score(IT_NAILGUN, brange);
		if (score > best_score) { best_score = score; best_pulse = 4; }
	}
	if (self.ammo_shells >= 1 && (it & IT_SHOTGUN))
	{
		score = bot_weapon_score(IT_SHOTGUN, brange);
		if (score > best_score) { best_score = score; best_pulse = 2; }
	}
	score = bot_weapon_score(IT_AXE, brange);
	if (score > best_score) { best_score = score; best_pulse = 1; }

	if (best_pulse)
	{
		// weapon commitment: humans don't re-evaluate every frame
		if (self.b_skill < 3 && self.b_wep_commit_time > time)
			return;
		self.impulse = best_pulse;
		if (self.b_skill == 0)
			self.b_wep_commit_time = time + 2.5 + random() * 2;
		else if (self.b_skill == 1)
			self.b_wep_commit_time = time + 1.5 + random();
		else if (self.b_skill == 2)
			self.b_wep_commit_time = time + 0.8 + random() * 0.5;
	}
};

void() bot_shoot =
{
	local float g;
	// don't shoot the wrong way! Argh
	g = angcomp(self.v_angle_x, self.b_angle_x);
	if (fabs(g) > 30)
		return;
	g = angcomp(self.v_angle_y, self.b_angle_y);
	if (fabs(g) > 30)
		return;
	self.button0 = TRUE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Where'd that come from!?" — Enemy Detection

Scans for enemies and incoming projectiles.
Reaction timer gives humans a fighting chance
against lower skill bots.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

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
		foesz = self.sense_enemy_size;
		foesz = foesz + vlen(v) * 0.5;
	}
	else
		foesz = 9999999;

	// scan for incoming projectiles
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

	// coop: hunt monsters instead of players
	if (coop)
	{
		if (!self.enemy)
		{
			foe = findradius(self.origin, 9999);
			while(foe)
			{
				if (foe.flags & FL_MONSTER)
					if (foe.health > 0)
					{
						tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
						if (tsz < foesz)
							if (fisible(foe))
							{
								self.enemy = foe;
								foesz = tsz;
							}
					}
				foe = foe.chain;
			}
		}
	}
	else
	{
		// deathmatch: pick best enemy from player list
		foe = player_head;
		while(foe)
		{
			if (foe != self)
				if (foe.modelindex != 0)
					if (foe.health > 0)
						if (!(teamplay && self.team == foe.team))
						{
							tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
							if (tsz < foesz)
								if (self.b_ambush_time > 0 || fov(foe) || foe.b_sound > time || self.b_skill == 3)
									if (fisible(foe))
									{
										self.enemy = foe;
										foesz = tsz;
									}
						}
			foe = foe._next;
		}

		// reaction delay — humans need a sec to process
		if (self.enemy && self.b_skill < 3)
		{
			if (self.b_react_time == 0)
			{
				if (self.b_skill == 0)
					self.b_react_time = time + 0.3 + random() * 0.4;
				else if (self.b_skill == 1)
					self.b_react_time = time + 0.15 + random() * 0.25;
				else
					self.b_react_time = time + 0.05 + random() * 0.1;
			}
			if (self.b_react_time > time)
				self.enemy = world;
		}
		else if (!self.enemy)
			self.b_react_time = 0;
	}

	// ambush first-strike: snap aim and fire immediately
	if (self.b_ambush_time > 0 && self.enemy)
	{
		v = normalize(realorigin(self.enemy) - self.origin);
		v = vectoangles(v);
		v_x = v_x * -1;
		self.b_angle = v;
		if (self.b_skill >= 1)
			self.v_angle = v;
		bot_weapon_switch(vlen(realorigin(self.enemy) - self.origin));
		bot_shoot();
		self.b_ambush_time = 0;
		self.b_aiflags = self.b_aiflags - (self.b_aiflags & AI_WAIT);
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Should I stay or should I go?"
    Retreat Decision

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() bot_should_retreat =
{
	local float mysz, foesz, ratio;

	mysz = self.sense_my_size;
	foesz = self.sense_enemy_size;
	if (foesz < 1) foesz = 1;
	ratio = mysz / foesz;

	// we have powerup — press the advantage
	if (self.items & IT_QUAD)
		return FALSE;
	if (self.items & IT_INVULNERABILITY)
		return FALSE;

	// enemy has powerup — get out of dodge
	if (self.enemy.items & IT_QUAD)
		return TRUE;
	if (self.enemy.items & IT_INVULNERABILITY)
		return TRUE;

	// no ammo — axe isn't gonna cut it (well, it is, but...)
	if (!bot_has_ammo())
		return TRUE;

	// badly outgunned
	if (ratio < 0.5)
		return TRUE;

	// low absolute health
	if (self.health < 30 && ratio < 0.8)
		return TRUE;

	return FALSE;
};

void() bot_drop_enemy =
{
	self.enemy = world;
	self.b_strafe_dir = 0;
	self.b_combat_state = CSTATE_NONE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Float like a butterfly" — Combat Strafe

Strafe with adaptive range management:
- Winning? Close in aggressively (= pressure)
- Losing? Keep distance (= cautious)
- Splash weapon + enemy near wall? Hold range
  at ideal splash distance (= flank behavior)

Two states do the work of four. Elegance!

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_combat_strafe =
{
	local vector org, movedir, right, v;
	local float foedist, strafe_interval, ideal, ratio;

	org = realorigin(self.enemy);
	foedist = self.sense_enemy_dist;
	makevectors(self.v_angle);
	right = v_right;

	// strafe interval based on skill and range
	if (foedist < 150 && self.b_skill >= 1)
		strafe_interval = 0.2 + random() * 0.3;
	else if (self.b_skill == 0)
		strafe_interval = 2.0 + random();
	else if (self.b_skill == 1)
		strafe_interval = 1.0 + random();
	else if (self.b_skill == 2)
		strafe_interval = 0.5 + random();
	else
		strafe_interval = 0.3 + random() * 0.5;

	// init strafe direction
	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
		self.b_strafe_time = time + strafe_interval;
	}

	// time to switch it up
	if (time > self.b_strafe_time)
	{
		// skill 3: counter-strafe using opponent model
		if (self.b_skill >= 3 && self.b_opp_ent == self.enemy)
		{
			if (self.b_opp_strafe > 0.4)
				self.b_strafe_dir = -1;
			else if (self.b_opp_strafe < -0.4)
				self.b_strafe_dir = 1;
			else
				self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + strafe_interval;
		}
		// skill 1+: occasional quick fake
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

	// adaptive range management — the secret sauce
	v = weapon_range(self.weapon);
	ideal = v_x;

	local float esz;
	esz = self.sense_enemy_size;
	if (esz < 1) esz = 1;
	ratio = self.sense_my_size / esz;

	// winning hard? get in their face (replaces PRESSURE state)
	if (ratio > 1.3)
		ideal = ideal * 0.7;
	// losing? keep your distance
	else if (ratio < 0.6)
		ideal = ideal * 1.3;
	// splash weapon + enemy near wall? hold at splash range (replaces FLANK)
	else if ((self.weapon == IT_ROCKET_LAUNCHER || self.weapon == IT_GRENADE_LAUNCHER)
		&& foedist > 150 && foedist < 500 && bot_enemy_near_wall())
		ideal = foedist;	// hold current range, it's working

	// build movement: strafe + range adjustment
	movedir = right * self.b_strafe_dir;
	if (foedist + 32 < ideal)
		movedir = movedir + normalize(self.origin - org) * 0.5;
	else if (foedist > ideal * 3)
		movedir = normalize(org - self.origin);
	else if (foedist - 32 > ideal)
		movedir = movedir + normalize(org - self.origin) * 0.5;

	if (!frik_walkmove(movedir))
	{
		self.b_strafe_dir = self.b_strafe_dir * -1;
		self.b_strafe_time = time + 0.1;
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

"Run away! Run away!" — Retreat Movement

Angular retreat with skill scaling.
Skill 0: panic freeze. Skill 1: run.
Skill 2+: angular retreat (harder to track).
Team: bias retreat toward teammates.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_retreat =
{
	local vector away, org, right, movedir, toward_mate;
	local entity mate;

	org = realorigin(self.enemy);
	away = normalize(self.origin - org);
	self.b_retreat = 1;

	// skill 0: deer in headlights
	if (self.b_skill == 0)
	{
		if (random() < 0.3)
			frik_walkmove(away);
		return;
	}

	// skill 1: run directly away (with teammate bias)
	if (self.b_skill == 1)
	{
		if (teamplay)
		{
			mate = bot_nearest_teammate();
			if (mate != world)
			{
				toward_mate = normalize(mate.origin - self.origin);
				away = normalize(away + toward_mate * 0.3);
			}
		}
		frik_walkmove(away);
		return;
	}

	// skill 2-3: angular retreat (serpentine!)
	makevectors(self.v_angle);
	right = v_right;

	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5)
			self.b_strafe_dir = 1;
		else
			self.b_strafe_dir = -1;
	}

	// opponent model: break their tracking
	if (self.b_opp_ent == self.enemy && self.b_skill >= 3)
	{
		if (self.b_opp_strafe > 0.3)
			self.b_strafe_dir = -1;
		else if (self.b_opp_strafe < -0.3)
			self.b_strafe_dir = 1;
	}

	// team: retreat toward buddy for crossfire support
	if (teamplay)
	{
		mate = bot_nearest_teammate();
		if (mate != world)
		{
			toward_mate = normalize(mate.origin - self.origin);
			away = normalize(away + toward_mate * 0.35);
		}
	}

	movedir = normalize(away + right * self.b_strafe_dir * 0.7);
	if (!frik_walkmove(movedir))
	{
		movedir = normalize(away + right * self.b_strafe_dir * -0.7);
		frik_walkmove(movedir);
		self.b_strafe_dir = self.b_strafe_dir * -1;
	}

	// skill 3 low health: look for health packs while running
	if (self.b_skill == 3 && self.health < 50)
		if (!self.target1 || self.target1.classname == "waypoint")
			bot_look_for_crap(FALSE);
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_fight_style

The main event. Two states: FIGHT or RETREAT.
Range management adapts to the situation, so
we don't need PRESSURE or FLANK anymore.

Simple 0.5s commit timer prevents jitter.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_fight_style =
{
	local vector v, v1, v2, org;
	local float foedist, has_los;

	// --- enemy still valid? ---
	if (self.enemy.health <= 0 || !self.enemy.takedamage)
	{
		bot_drop_enemy();
		return;
	}

	if (!self.sense_enemy_vis)
	{
		// skill 1+: 0.7s grace period before dropping on LOS loss
		if (self.b_skill >= 1 && (time - self.b_los_time) < 0.7)
			has_los = FALSE;
		else
		{
			bot_drop_enemy();
			return;
		}
	}
	else
	{
		self.b_los_time = time;
		has_los = TRUE;
	}

	// --- update opponent model ---
	bot_opp_model_update();

	org = realorigin(self.enemy);
	foedist = self.sense_enemy_dist;
	v = weapon_range(self.weapon);

	// --- shooting decision ---
	if (bot_in_weapon_range(foedist, v))
	{
		if (has_los && !self.sense_friendly_block && self.sense_fwd_wall_dist >= foedist)
			bot_shoot();

		// periodic weapon switching with skill-based commitment
		if (self.b_skill >= 1 && time > self.b_wepswitch_time)
		{
			bot_weapon_switch(foedist);
			self.b_wepswitch_time = time + 1.5;
		}
	}
	else
		bot_weapon_switch(foedist);

	self.b_retreat = 0;

	// --- axe at range: disengage (no point swinging at air) ---
	if (self.weapon == IT_AXE && foedist > 64)
		if (self.impulse == 0 || self.impulse == 1)
		{
			bot_drop_enemy();
			return;
		}

	// --- projectile dodge takes priority over everything ---
	if (self.avoid)
	{
		if (self.avoid.velocity)
			v = self.avoid.velocity;
		else
			v = normalize(self.avoid.origin - self.origin);
		// perpendicular dodge vectors
		v1_x = v_y * -1;
		v1_y = v_x;
		v2_x = v_y;
		v2_y = v_x * -1;
		// dodge away from the projectile
		if (vlen(self.avoid.origin - (self.origin + v1)) < vlen(self.avoid.origin - (self.origin + v2)))
			frik_walkmove(v2);
		else
			frik_walkmove(v1);
		return;
	}

	// --- state decision with 0.5s commit timer ---
	if (time > self.b_combat_time + 0.5)
	{
		if (bot_should_retreat())
		{
			if (self.b_combat_state != CSTATE_RETREAT)
			{
				self.b_combat_state = CSTATE_RETREAT;
				self.b_combat_time = time;
			}
		}
		else
		{
			if (self.b_combat_state != CSTATE_FIGHT)
			{
				self.b_combat_state = CSTATE_FIGHT;
				self.b_combat_time = time;
			}
		}
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
		// shoot while retreating (skill 1+ already handles weapon switching above)
		if (has_los)
			bot_shoot();
	}
	else
	{
		if (!(self.enemy.flags & FL_MONSTER))
			bot_combat_strafe();
	}
};
