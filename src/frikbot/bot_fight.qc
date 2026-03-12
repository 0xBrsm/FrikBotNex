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
	org = realorigin(self.enemy);
	dir = normalize(org - self.origin);
	traceline(org, org + dir * 100, TRUE, self.enemy);
	if (trace_fraction < 1)
		return TRUE;
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
	local vector v;

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
		// skill 2+: occasional quick fake (short strafe then switch)
		if (self.b_skill >= 2 && random() < 0.2)
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

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_retreat =
{
	local vector away, org, right, movedir;

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

	// skill 1: run directly away
	if (self.b_skill == 1)
	{
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

Bot_fight_style

Core combat decision loop: shoot, move, or
retreat based on situation and skill.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() bot_fight_style =
{
	local vector v, v1, v2, org;
	local float foedist, mysz, foesz;

	if (self.enemy.health <= 0)
	{
		self.enemy = world;
		self.b_strafe_dir = 0;
		return;
	}
	else if (!self.enemy.takedamage)
	{
		self.enemy = world;
		self.b_strafe_dir = 0;
		return;
	}
	else if (!fisible(self.enemy))
	{
		self.enemy = world;
		self.b_strafe_dir = 0;
		return;
	}

	org = realorigin(self.enemy);
	makevectors(self.v_angle);

	// decide if I should shoot
	foedist = vlen(org - self.origin);
	v = weapon_range(self.weapon);

	if (foedist > v_y && foedist < v_z)
	{
		traceline(self.origin + self.view_ofs, self.origin + self.view_ofs + v_forward * v_z, FALSE, self);
		if (vlen(trace_endpos - (self.origin + self.view_ofs)) >= v_y)
		{
			// try to avoid shooting teammates
			if (trace_ent.classname == "player")
				if ((trace_ent.team == self.team && teamplay) || (coop))
					return;
			bot_shoot();
		}
	}
	else
		bot_weapon_switch(foedist);

	self.b_retreat = 0;

	if (!(self.b_aiflags & (AI_PRECISION | AI_BLIND | AI_OBSTRUCTED)))
	{
		foesz = bot_size_player(self.enemy);
		mysz = bot_size_player(self) + 5;

		// personality adjusts retreat threshold
		if (self.b_personality == PERSONALITY_AGGRESSIVE)
			mysz = mysz + 40; // fights even when outgunned
		else if (self.b_personality == PERSONALITY_DEFENSIVE)
			mysz = mysz - 30; // retreats earlier

		if (foesz > mysz)
		{
			bot_retreat();
			if (teamplay)
			{
				if (random() < 0.02)
				{
					bot_start_topic(5);
					self.b_chattime = 1;
				}
			}
			return;
		}
		else if (mysz < 140)
			return;
		else if (self.avoid)
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
		}
		else if (!(self.enemy.flags & FL_MONSTER))
		{
			bot_combat_strafe();
		}
	}
	else
	{
		foesz = bot_size_player(self.enemy);
		mysz = bot_size_player(self) + 5;

		if (foesz > mysz)
		{
			bot_retreat();
			return;
		}
		else if (mysz < 140)
			return;
		self.keys = self.keys & 960;
	}
};


