/***********************************************
*                                              *
*           FrikBot Fight Code                 *
*    "Because I ain't no Ghandi code"          *
*                                              *
***********************************************/

/*
This program is in the Public Domain. My crack legal team would like to add:

RYAN "FRIKAC" SMITH IS PROVIDING THIS SOFTWARE "AS IS" AND MAKES NO
WARRANTY, EXPRESS OR IMPLIED, AS TO THE ACCURACY, CAPABILITY, EFFICIENCY,
MERCHANTABILITY, OR FUNCTIONING OF THIS SOFTWARE AND/OR DOCUMENTATION. IN
NO EVENT WILL RYAN "FRIKAC" SMITH BE LIABLE FOR ANY GENERAL, CONSEQUENTIAL,
INDIRECT, INCIDENTAL, EXEMPLARY, OR SPECIAL DAMAGES, EVEN IF RYAN "FRIKAC"
SMITH HAS BEEN ADVISED OF THE POSSIBILITY OF SUCH DAMAGES, IRRESPECTIVE OF
THE CAUSE OF SUCH DAMAGES. Any reproduction of this software must contain
this notice in its entirety.
*/

.entity avoid;

// -=-=- "Know thy enemy" — Opponent Modeling -=-=-

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

	blend = 0.08 + self.b_skill * 0.07;

	self.b_opp_weapon = self.enemy.weapon;
	v = self.enemy.origin - self.origin;
	dist = vlen(v);
	self.b_opp_dist = self.b_opp_dist * (1 - blend) + dist * blend;

	v = normalize(self.origin - self.enemy.origin);
	delta = self.enemy.velocity * v / 400;
	delta = delta + (random() - 0.5) * 0.3;
	if (delta > 1) delta = 1;
	if (delta < -1) delta = -1;
	self.b_opp_aggro = self.b_opp_aggro * (1 - blend) + (delta + 1) * 0.5 * blend;

	makevectors(self.v_angle);
	delta = self.enemy.velocity * v_right + (random() - 0.5) * 40;
	if (delta > 50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) + blend;
	else if (delta < -50)
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend) - blend;
	else
		self.b_opp_strafe = self.b_opp_strafe * (1 - blend * 0.5);
	self.b_opp_track_time = time;
};

// -=-=- "The buddy system" — Team Coordination -=-=-

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
			{ bestdist = dist; best = t; }
		}
		t = t._next;
	}
	return best;
};

// -=-=- "Measuring the competition" — Player Sizing -=-=-

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
	else if (e.weapon == IT_SHOTGUN) sz = sz + 10;
	else if (e.weapon == IT_AXE) sz = sz - 50;
	if (e.items & IT_QUAD) sz = sz + 200;
	if (e.items & IT_INVULNERABILITY) sz = sz + 300;
	if (e.items & IT_INVISIBILITY) sz = sz + 250;
	if (sz < 1) sz = 1;
	return sz;
};

// -=-=- weapon_range: _x=sweet spot _y=min _z=max -=-=-

vector(float wep) weapon_range =
{
	if (wep == IT_AXE) return '48 0 64';
	if (wep == IT_SHOTGUN) return '128 8 3000';
	if (wep == IT_SUPER_SHOTGUN) return '128 0 3000';
	if (wep == IT_NAILGUN) return '180 0 3000';
	if (wep == IT_SUPER_NAILGUN) return '180 0 3000';
	if (wep == IT_GRENADE_LAUNCHER) return '180 48 3000';
	if (wep == IT_ROCKET_LAUNCHER) return '180 48 3000';
	if (wep == IT_LIGHTNING) return '350 0 512';
	return '128 0 3000';
};

// -=-=- "Walls are your friend" — Splash Helpers -=-=-

float(float dist, vector wr) bot_in_weapon_range = { return (dist > wr_y && dist < wr_z); };

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

// -=-=- "Guns don't kill people, impulse commands do" -=-=-

float(float wep, float brange) bot_weapon_score =
{
	local float score;
	local vector wr;

	if (wep == IT_ROCKET_LAUNCHER) score = 70;
	else if (wep == IT_LIGHTNING) score = 65;
	else if (wep == IT_SUPER_NAILGUN) score = 55;
	else if (wep == IT_GRENADE_LAUNCHER) score = 50;
	else if (wep == IT_SUPER_SHOTGUN) score = 45;
	else if (wep == IT_NAILGUN) score = 35;
	else if (wep == IT_SHOTGUN) score = 25;
	else if (wep == IT_AXE) score = 15;
	else return 0;

	if (wep == IT_LIGHTNING && self.waterlevel > 1) return 0;

	wr = weapon_range(wep);
	if (brange < wr_y || brange > wr_z) return 0;

	// self-splash danger for the boom tubes
	if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
	{
		if (brange < 120) score = score - 60;
		else if (brange < 200) score = score - 20;
	}

	if (self.b_skill >= 1)
	{
		// wall bonus for splash weapons
		if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
			if (self.enemy != world && self.sense_enemy_wall)
				score = score + 15;
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
		// ammo conservation
		if (wep == IT_ROCKET_LAUNCHER && self.ammo_rockets <= 2)
			if (self.enemy.health > 60)
				if (!(self.enemy.items & IT_QUAD) && !(self.enemy.items & IT_INVULNERABILITY))
					score = score - 25;
	}

	// opponent model counter-picks
	if (self.b_skill >= 1 && self.b_opp_ent == self.enemy)
	{
		if (self.b_opp_weapon == IT_ROCKET_LAUNCHER && self.b_opp_aggro > 0.6)
			if (wep == IT_LIGHTNING) score = score + 12;
		if (self.b_opp_aggro < 0.3)
			if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
				score = score + 8;
	}

	if (score < 1) score = 1;
	return score;
};

void(float brange) bot_weapon_switch =
{
	local float it, best_score, score, best_pulse;
	it = self.items & 127;
	best_score = 0;
	best_pulse = 0;

	if (brange != -1)
	{	// situational scoring
		if ((self.ammo_rockets >= 1) && (it & IT_ROCKET_LAUNCHER))
		{ score = bot_weapon_score(IT_ROCKET_LAUNCHER, brange); if (score > best_score) { best_score = score; best_pulse = 7; } }
		if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & IT_LIGHTNING))
		{ score = bot_weapon_score(IT_LIGHTNING, brange); if (score > best_score) { best_score = score; best_pulse = 8; } }
		if (self.ammo_nails >= 2 && (it & IT_SUPER_NAILGUN))
		{ score = bot_weapon_score(IT_SUPER_NAILGUN, brange); if (score > best_score) { best_score = score; best_pulse = 5; } }
		if ((self.ammo_rockets >= 1) && (it & IT_GRENADE_LAUNCHER))
		{ score = bot_weapon_score(IT_GRENADE_LAUNCHER, brange); if (score > best_score) { best_score = score; best_pulse = 6; } }
		if (self.ammo_shells >= 2 && (it & IT_SUPER_SHOTGUN))
		{ score = bot_weapon_score(IT_SUPER_SHOTGUN, brange); if (score > best_score) { best_score = score; best_pulse = 3; } }
		if (self.ammo_nails >= 1 && (it & IT_NAILGUN))
		{ score = bot_weapon_score(IT_NAILGUN, brange); if (score > best_score) { best_score = score; best_pulse = 4; } }
		if (self.ammo_shells >= 1 && (it & IT_SHOTGUN))
		{ score = bot_weapon_score(IT_SHOTGUN, brange); if (score > best_score) { best_score = score; best_pulse = 2; } }
		score = bot_weapon_score(IT_AXE, brange);
		if (score > best_score) { best_score = score; best_pulse = 1; }
	}
	else
	{	// no range: simple tier ordering
		if ((self.ammo_rockets >= 1) && (it & IT_ROCKET_LAUNCHER)) { best_pulse = 7; best_score = 70; }
		if (self.waterlevel <= 1 && self.ammo_cells >= 1 && (it & IT_LIGHTNING)) { if (65 > best_score) { best_pulse = 8; best_score = 65; } }
		if (self.ammo_nails >= 2 && (it & IT_SUPER_NAILGUN)) { if (55 > best_score) { best_pulse = 5; best_score = 55; } }
		if ((self.ammo_rockets >= 1) && (it & IT_GRENADE_LAUNCHER)) { if (50 > best_score) { best_pulse = 6; best_score = 50; } }
		if (self.ammo_shells >= 2 && (it & IT_SUPER_SHOTGUN)) { if (45 > best_score) { best_pulse = 3; best_score = 45; } }
		if (self.ammo_nails >= 1 && (it & IT_NAILGUN)) { if (35 > best_score) { best_pulse = 4; best_score = 35; } }
		if (self.ammo_shells >= 1 && (it & IT_SHOTGUN)) { if (25 > best_score) { best_pulse = 2; best_score = 25; } }
		if (!best_pulse) best_pulse = 1;
	}
	if (!best_pulse) return;
	if (self.b_skill < 3 && self.b_wep_commit_time > time) return;
	self.impulse = best_pulse;
	self.b_wep_commit_time = time + 2.5 - self.b_skill * 0.6 + random() * (2.0 - self.b_skill * 0.5);
};

void() bot_shoot =
{
	local float g;
	g = angcomp(self.v_angle_x, self.b_angle_x);
	if (fabs(g) > 30) return;
	g = angcomp(self.v_angle_y, self.b_angle_y);
	if (fabs(g) > 30) return;
	self.button0 = TRUE;
};

// -=-=- Projectile Scan — dodge incoming fire -=-=-

void() bot_projectile_scan =
{
	local entity foe;
	local float avdist, flen;

	if (waypoint_mode > WM_LOADED) return;
	self.avoid = world;
	avdist = 256;

	foe = find(world, classname, "grenade");
	while(foe)
	{
		if (foe.owner != self)
		{
			flen = vlen(foe.origin - self.origin);
			if (flen < avdist) { avdist = flen; self.avoid = foe; }
		}
		foe = find(foe, classname, "grenade");
	}
	if (!self.avoid)
	{
		foe = find(world, classname, "missile");
		while(foe)
		{
			if (foe.owner != self)
			{ flen = vlen(foe.origin - self.origin); if (flen < avdist) { avdist = flen; self.avoid = foe; } }
			foe = find(foe, classname, "missile");
		}
	}
	if (!self.avoid)
	{
		foe = find(world, classname, "spike");
		while(foe)
		{
			if (foe.owner != self)
			{ flen = vlen(foe.origin - self.origin); if (flen < avdist) { avdist = flen; self.avoid = foe; } }
			foe = find(foe, classname, "spike");
		}
	}
};

// -=-=- Enemy Scan — find someone to fight -=-=-
// Only scans when bot has no current enemy.

void() bot_find_enemy =
{
	local entity foe;
	local float foesz, tsz;
	local vector v;

	foesz = 9999999;

	if (coop)
	{
		foe = findradius(self.origin, 9999);
		while(foe)
		{
			if (foe.flags & FL_MONSTER)
				if (foe.health > 0)
				{
					tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
					if (tsz < foesz) if (fisible(foe))
					{ self.enemy = foe; foesz = tsz; }
				}
			foe = foe.chain;
		}
	}
	else
	{
		foe = player_head;
		while(foe)
		{
			if (foe != self && foe.modelindex != 0 && foe.health > 0)
				if (!(teamplay && self.team == foe.team))
				{
					tsz = bot_size_player(foe) + vlen(foe.origin - self.origin) * 0.5;
					if (tsz < foesz)
						if (self.b_ambush_time > 0 || fov(foe) || foe.b_sound > time || self.b_skill == 3)
							if (fisible(foe))
							{ self.enemy = foe; foesz = tsz; }
				}
			foe = foe._next;
		}
	}

	// reaction delay
	if (self.enemy && self.b_skill < 3)
	{
		if (self.b_react_time == 0)
			self.b_react_time = time + 0.35 - self.b_skill * 0.1 + random() * (0.4 - self.b_skill * 0.1);
		if (self.b_react_time > time)
			self.enemy = world;
	}
	else if (!self.enemy)
		self.b_react_time = 0;

	// new enemy: start the LOS grace clock
	if (self.enemy)
		self.b_combat_time = time;

	// ambush first-strike
	if (self.b_ambush_time > 0 && self.enemy)
	{
		local vector org;
		org = realorigin(self.enemy) - self.origin;
		v = vectoangles(normalize(org));
		v_x = v_x * -1;
		self.b_angle = v;
		if (self.b_skill >= 1) self.v_angle = v;
		bot_weapon_switch(vlen(org));
		bot_shoot();
		self.b_ambush_time = 0;
		self.b_aiflags = self.b_aiflags - (self.b_aiflags & AI_WAIT);
	}
};

// -=-=- "Should I stay or should I go?" -=-=-

float() bot_should_retreat =
{
	local float esz, ratio;
	esz = self.sense_enemy_size;
	if (esz < 1) esz = 1;
	ratio = self.sense_my_size / esz;

	if (self.items & IT_QUAD) return FALSE;
	if (self.items & IT_INVULNERABILITY) return FALSE;
	if (self.enemy.items & IT_QUAD) return TRUE;
	if (self.enemy.items & IT_INVULNERABILITY) return TRUE;
	if (!bot_has_ammo()) return TRUE;
	if (ratio < 0.5) return TRUE;
	if (self.health < 30 && ratio < 0.8) return TRUE;
	return FALSE;
};

void() bot_drop_enemy =
{
	self.enemy = world;
	self.b_strafe_dir = 0;
	self.b_combat_state = CSTATE_NONE;
};

// -=-=- "Float like a butterfly" — Combat Strafe -=-=-
// Adaptive range replaces PRESSURE and FLANK states:
//   Winning → close in. Losing → keep distance. Splash angle → hold range.

void() bot_combat_strafe =
{
	local vector org, movedir, right, v;
	local float foedist, strafe_interval, ideal, ratio, esz;

	org = realorigin(self.enemy);
	foedist = self.sense_enemy_dist;
	makevectors(self.v_angle);
	right = v_right;

	if (foedist < 150 && self.b_skill >= 1)
		strafe_interval = 0.2 + random() * 0.3;
	else
		strafe_interval = 2.0 - self.b_skill * 0.55 + random() * (1.0 - self.b_skill * 0.15);

	if (self.b_strafe_dir == 0)
	{
		if (random() < 0.5) self.b_strafe_dir = 1;
		else self.b_strafe_dir = -1;
		self.b_strafe_time = time + strafe_interval;
	}
	if (time > self.b_strafe_time)
	{
		if (self.b_skill >= 3 && self.b_opp_ent == self.enemy)
		{	// counter-strafe
			if (self.b_opp_strafe > 0.4) self.b_strafe_dir = -1;
			else if (self.b_opp_strafe < -0.4) self.b_strafe_dir = 1;
			else self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + strafe_interval;
		}
		else if (self.b_skill >= 1 && random() < (0.1 + 0.1 * (self.b_skill >= 2)))
		{	// quick fake
			self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + 0.15 + random() * 0.15;
		}
		else
		{
			self.b_strafe_dir = self.b_strafe_dir * -1;
			self.b_strafe_time = time + strafe_interval;
		}
	}

	// adaptive range — the secret sauce
	v = weapon_range(self.weapon);
	ideal = v_x;
	esz = self.sense_enemy_size;
	if (esz < 1) esz = 1;
	ratio = self.sense_my_size / esz;

	if (ratio > 1.3) ideal = ideal * 0.7;			// winning: pressure
	else if (ratio < 0.6) ideal = ideal * 1.3;		// losing: keep distance
	else if ((self.weapon == IT_ROCKET_LAUNCHER || self.weapon == IT_GRENADE_LAUNCHER)
		&& foedist > 150 && foedist < 500 && self.sense_enemy_wall)
		ideal = foedist;							// splash angle: hold

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

// -=-=- "Run away! Run away!" — Retreat -=-=-
// Skill 0: panic. Skill 1: run. Skill 2+: serpentine. Team: run to buddy.

void() bot_retreat =
{
	local vector away, org, right, movedir;
	local entity mate;

	org = realorigin(self.enemy);
	away = normalize(self.origin - org);


	if (self.b_skill == 0)
	{ if (random() < 0.3) frik_walkmove(away); return; }

	// team: bias toward nearest buddy
	if (teamplay)
	{
		mate = bot_nearest_teammate();
		if (mate != world)
			away = normalize(away + normalize(mate.origin - self.origin) * 0.3);
	}
	if (self.b_skill == 1)
	{ frik_walkmove(away); return; }

	// skill 2-3: angular retreat
	makevectors(self.v_angle);
	right = v_right;
	if (self.b_strafe_dir == 0)
	{ if (random() < 0.5) self.b_strafe_dir = 1; else self.b_strafe_dir = -1; }

	if (self.b_opp_ent == self.enemy && self.b_skill >= 3)
	{
		if (self.b_opp_strafe > 0.3) self.b_strafe_dir = -1;
		else if (self.b_opp_strafe < -0.3) self.b_strafe_dir = 1;
	}

	movedir = normalize(away + right * self.b_strafe_dir * 0.7);
	if (!frik_walkmove(movedir))
	{
		movedir = normalize(away + right * self.b_strafe_dir * -0.7);
		frik_walkmove(movedir);
		self.b_strafe_dir = self.b_strafe_dir * -1;
	}

	if (self.b_skill == 3 && self.health < 50)
		if (!self.target1 || self.target1.classname == "waypoint")
			bot_get(FALSE);
};

// -=-=- bot_fight_style — The Main Event -=-=-
// Two states: FIGHT or RETREAT. Adaptive range management handles
// what used to need PRESSURE and FLANK. 0.5s commit timer prevents jitter.

void() bot_fight_style =
{
	local vector v, v1, v2;
	local float foedist, has_los;

	has_los = self.sense_enemy_vis;

	// lost LOS? give up after 1s (b_react_time doubles as last-seen tracker here)
	if (has_los)
		self.b_react_time = time;
	else if ((time - self.b_react_time) > 1.0)
	{ bot_drop_enemy(); return; }

	bot_opp_model_update();
	foedist = self.sense_enemy_dist;
	v = weapon_range(self.weapon);

	if (bot_in_weapon_range(foedist, v))
	{
		if (has_los && !self.sense_friendly_block)
			bot_shoot();
		if (self.b_skill >= 1 && time > self.b_wepswitch_time)
		{ bot_weapon_switch(foedist); self.b_wepswitch_time = time + 1.5; }
	}
	else
		bot_weapon_switch(foedist);



	if (self.weapon == IT_AXE && foedist > 64)
		if (self.impulse == 0 || self.impulse == 1)
		{ bot_drop_enemy(); return; }

	// projectile dodge takes priority
	if (self.avoid)
	{
		if (self.avoid.velocity) v = self.avoid.velocity;
		else v = normalize(self.avoid.origin - self.origin);
		v1_x = v_y * -1; v1_y = v_x;
		v2_x = v_y; v2_y = v_x * -1;
		if (vlen(self.avoid.origin - (self.origin + v1)) < vlen(self.avoid.origin - (self.origin + v2)))
			frik_walkmove(v2);
		else
			frik_walkmove(v1);
		return;
	}

	// state decision with 0.5s commit timer
	if (time > self.b_combat_time + 0.5)
	{
		if (bot_should_retreat())
		{ if (self.b_combat_state != CSTATE_RETREAT) { self.b_combat_state = CSTATE_RETREAT; self.b_combat_time = time; } }
		else
		{ if (self.b_combat_state != CSTATE_FIGHT) { self.b_combat_state = CSTATE_FIGHT; self.b_combat_time = time; } }
	}

	if (self.b_combat_state == CSTATE_RETREAT)
	{
		bot_retreat();
		if (teamplay && random() < 0.02)
		{ bot_start_topic(5); self.b_chattime = 1; }
		if (has_los && !self.sense_friendly_block && bot_in_weapon_range(foedist, v))
			bot_shoot();
	}
	else
	{
		if (!(self.enemy.flags & FL_MONSTER))
			bot_combat_strafe();
	}
};
