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
		if (self.waterlevel < 1 && self.ammo_cells >= 1 && (it & 64))
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
