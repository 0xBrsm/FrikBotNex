/***********************************************
*                                              *
*           FrikBot Shoot Code                 *
*  "Guns don't kill people, impulse commands do" *
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

	// LG in water = self-discharge, never use it
	// Quake discharge triggers at waterlevel > 1 (waist deep or more)
	if (wep == IT_LIGHTNING && self.waterlevel > 1)
		return 0;

	// range fitness - unusable weapons score 0
	v = weapon_range(wep);
	if (brange < v_y || brange > v_z)
		return 0;

	// self-splash danger for RL and GL
	if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
	{
		if (brange < 120)
			score = score - 60;
		else if (brange < 200)
			score = score - 20;
	}

	// skill 1+ considerations: environment-aware weapon scoring
	if (self.b_skill >= 1)
	{
		// wall proximity bonus for splash weapons
		if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
		{
			if (self.enemy != world)
			{
				if (bot_enemy_near_wall())
					score = score + 15;
			}
		}

		// water bonus for RL (splash amplified)
		if (wep == IT_ROCKET_LAUNCHER)
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
				if (wep == IT_SHOTGUN || wep == IT_SUPER_SHOTGUN || wep == IT_AXE)
					score = score + 20;
				else if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
					score = score - 15;
			}

			// ammo conservation - save rockets for important fights
			if (wep == IT_ROCKET_LAUNCHER && self.ammo_rockets <= 2)
			{
				if (self.enemy.health > 60)
				{
					// still use them against quad/pent enemies
					if (!(self.enemy.items & IT_QUAD) && !(self.enemy.items & IT_INVULNERABILITY))
						score = score - 25;
				}
			}
		}
	}

	// opponent model weapon adjustments (skill 2+)
	if (self.b_skill >= 1 && self.b_opp_ent == self.enemy)
	{
		// counter-pick: if enemy has RL/LG and is aggressive, prefer LG for hitscan
		if ((self.b_opp_weapon == IT_ROCKET_LAUNCHER) && self.b_opp_aggro > 0.6)
		{
			if (wep == IT_LIGHTNING) score = score + 12;
		}
		// enemy is passive/retreating: splash weapons to flush them out
		if (self.b_opp_aggro < 0.3)
		{
			if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER) score = score + 8;
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
		if (wep == IT_ROCKET_LAUNCHER) score = score + 10; // RL
		else if (wep == IT_LIGHTNING) score = score + 8;
		else if (wep == IT_AXE) score = score + 5;
		else if (wep == IT_SHOTGUN || wep == IT_NAILGUN) score = score - 5;
	}
	else if (self.b_personality == PERSONALITY_DEFENSIVE)
	{
		// defensive: prefers range and safety
		if (wep == IT_SUPER_NAILGUN) score = score + 8;
		else if (wep == IT_LIGHTNING) score = score + 5;
		else if (wep == IT_ROCKET_LAUNCHER || wep == IT_GRENADE_LAUNCHER)
		{
			if (brange < 250) // splash risk at close range
				score = score - 10;
		}
	}
	else if (self.b_personality == PERSONALITY_CAMPER)
	{
		// camper: prefers long-range precision weapons
		if (wep == IT_ROCKET_LAUNCHER) score = score + 5; // RL
		if (wep == IT_SUPER_NAILGUN) score = score + 5;
		if (wep == IT_AXE) score = score - 10;
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
			best_pulse = 1; // axe as last resort
		if (best_pulse)
			self.impulse = best_pulse;
		return;
	}

	// score each weapon situationally
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
	// axe: always available, but weapon_range limits it to melee (< 64 units)
	score = bot_weapon_score(IT_AXE, brange);
	if (score > best_score) { best_score = score; best_pulse = 1; }

	if (best_pulse)
	{
		// human emulation: weapon commitment and lazy switching
		// skill 0-2 sometimes stick with current weapon even if not optimal
		if (self.b_skill < 3 && self.b_wep_commit_time > time)
		{
			// committed to current weapon — skip switch
			return;
		}
		self.impulse = best_pulse;
		// lock weapon for a bit (humans don't constantly re-evaluate)
		if (self.b_skill == 0)
			self.b_wep_commit_time = time + 2.5 + random() * 2;
		else if (self.b_skill == 1)
			self.b_wep_commit_time = time + 1.5 + random() * 1;
		else if (self.b_skill == 2)
			self.b_wep_commit_time = time + 0.8 + random() * 0.5;
	}
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
