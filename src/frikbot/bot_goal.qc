/***********************************************
*                                              *
*            FrikBot Goal System               *
*      "I want it all and I want it now"       *
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

// ----- target_onstack: is this thing on my stack? returns slot or FALSE -----

float(entity scot) target_onstack =
{
	if (scot == world)
		return FALSE;
	else if (self.target1 == scot)
		return 1;
	else if (self.target2 == scot)
		return 2;
	else if (self.target3 == scot)
		return 3;
	else if (self.target4 == scot)
		return 4;
	else
		return FALSE;
};

// ----- target_add: shove something onto the LIFO stack -----

void(entity ent) target_add =
{
	if (ent == world)
		return;
	if (target_onstack(ent))
		return;
	self.target4 = self.target3;
	self.target3 = self.target2;
	self.target2 = self.target1;
	self.target1 = ent;
	self.search_time = time + 5;
};

// ----- target_drop: yank it off, shift the rest down -----

void(entity ent) target_drop =
{
	local float tg;
	tg = target_onstack(ent);
	if (tg == 1)
	{
		self.target1 = self.target2;
		self.target2 = self.target3;
		self.target3 = self.target4;
		self.target4 = world;
	}
	else if (tg == 2)
	{
		self.target2 = self.target3;
		self.target3 = self.target4;
		self.target4 = world;
	}
	else if (tg == 3)
	{
		self.target3 = self.target4;
		self.target4 = world;
	}
	else if (tg == 4)
		self.target4 = world;
};

// ----- bot_lost: we lost our target. deal with it -----

void(entity targ, float success) bot_lost =
{
	if (!targ)
		return;

	target_drop(targ);
	if (targ.classname == "waypoint")
		targ.b_sound = targ.b_sound - (targ.b_sound & ClientBitFlag(self.b_clientno));

	if (!success)
	{
		// total reset -- find somewhere new to go
		self.target1 = self.target2 = self.target3 = self.target4 = world;
		self.last_way = FindWayPoint(self.current_way);
		ClearMyRoute();
		self.b_aiflags = 0;
		self.b_ambush_time = 0;
	}
	else
	{
		if (targ.classname == "item_artifact_invisibility")
			if (self.items & IT_INVISIBILITY)
				bot_start_topic(3);
		if (targ.flags & FL_ITEM)
		{
			if (targ.model == string_null)
				targ._last = world;
			else
				targ._last = self;
		}
	}
	// NOTE: do NOT set targ.search_time on items here.
	// that caused 10s stalls where bots ignored nearby pickups. sue me.
};

// ----- bot_check_lost: should I give up? spoiler: probably yes -----

void(entity targ) bot_check_lost =
{
	local vector dist;
	if (targ == world)
		return;

	if (targ.flags & FL_ITEM)
	{
		if (vlen(targ.origin - self.origin) < 32)
			bot_lost(targ, TRUE);
		else if (targ.model == string_null)
			bot_lost(targ, TRUE);
	}
	else if (targ.classname == "waypoint")
	{
		if (!(self.b_aiflags & (AI_SNIPER | AI_AMBUSH)))
		{
			if (self.b_aiflags & AI_PRECISION)
			{
				if (vlen(targ.origin - self.origin) < 24)
					bot_lost(targ, TRUE);
			}
			else if (vlen(targ.origin - self.origin) < 32)
				bot_lost(targ, TRUE);
		}
	}
	else if (targ.classname == "temp_waypoint")
	{
		if (vlen(targ.origin - self.origin) < 32)
			bot_lost(targ, TRUE);
	}
	else if (targ.classname == "player")
	{
		if (targ.health <= 0)
			bot_lost(targ, TRUE);
		else if ((coop) || (teamplay && targ.team == self.team))
		{
			if (vlen(targ.origin - self.origin) < 128)
			{	bot_lost(targ, TRUE); return;	}
			else if (!fisible(targ))
				bot_lost(targ, FALSE);
		}
	}
	else if (targ.classname == "func_button")
	{
		if (targ.frame)
		{
			bot_lost(targ, TRUE);
			if (self.enemy == targ)
				self.enemy = world;
		}
	}
	else if ((targ.movetype == MOVETYPE_NONE) && (targ.solid == SOLID_TRIGGER))
	{
		if (targ.nextthink >= time)
			bot_lost(targ, TRUE);
	}

	// target way above head, on ground, no platform = give up
	if ((targ.origin_z - self.origin_z) > 64)
	{
		dist = targ.origin - self.origin;
		dist_z = 0;
		if (vlen(dist) < 32)
			if (self.flags & FL_ONGROUND)
				if (!frik_recognize_plat(FALSE))
					bot_lost(targ, FALSE);
	}
	else if (targ.classname == "train")
	{
		if (frik_recognize_plat(FALSE))
			bot_lost(targ, TRUE);
	}
	if (time > self.search_time)
		bot_lost(targ, FALSE);
};

// ----- bot_want: "how bad do I want this?" returns 0-100 -----

float(entity thing) bot_want =
{
	// already grabbed or on cooldown? pass
	if (thing.flags & FL_ITEM)
	{
		if (thing.model == string_null)	return 0;
		if (thing.search_time > time)	return 0;
	}

	// powerups -- always juicy
	if (thing.classname == "item_artifact_super_damage")	return 65;
	if (thing.classname == "item_artifact_invulnerability")	return 65;
	if (thing.classname == "item_artifact_invisibility")	return 65;

	// health
	if (thing.classname == "item_health")
	{
		if (thing.spawnflags & 2) // megahealth
		{
			if (self.health >= 200) return 0;
			return 55;
		}
		if (self.health <= 30)	return 70;
		if (self.health <= 50)	return 55;
		if (self.health <= 80)	return 35;
		return 5;
	}

	// armor -- only if it's an upgrade
	if (thing.classname == "item_armorInv")
	{
		if (self.armortype < 0.8 || self.armorvalue < 150) return 60;
		return 0;
	}
	if (thing.classname == "item_armor2")
	{
		if (self.armortype < 0.6 || (self.armortype == 0.6 && self.armorvalue < 100)) return 50;
		return 0;
	}
	if (thing.classname == "item_armor1")
	{
		if (self.armortype < 0.3 || (self.armortype == 0.3 && self.armorvalue < 50)) return 25;
		return 0;
	}

	// weapons -- want what we ain't got
	if (thing.classname == "weapon_rocketlauncher")
	{
		if (!(self.items & IT_ROCKET_LAUNCHER))	return 60;
		if (self.ammo_rockets < 5)		return 30;
		return 0;
	}
	if (thing.classname == "weapon_lightning")
	{
		if (!(self.items & IT_LIGHTNING))	return 55;
		if (self.ammo_cells < 10)		return 25;
		return 0;
	}
	if (thing.classname == "weapon_grenadelauncher")
	{
		if (!(self.items & IT_GRENADE_LAUNCHER))	return 45;
		return 0;
	}
	if (thing.classname == "weapon_supernailgun")
	{
		if (!(self.items & IT_SUPER_NAILGUN))	return 40;
		return 0;
	}
	if (thing.classname == "weapon_nailgun")
	{
		if (!(self.items & IT_NAILGUN))		return 30;
		return 0;
	}
	if (thing.classname == "weapon_supershotgun")
	{
		if (!(self.items & IT_SUPER_SHOTGUN))	return 25;
		return 0;
	}

	// backpacks -- free ammo is free ammo
	if (thing.model == "progs/backpack.mdl")	return 35;

	// envirosuit -- meh
	if (thing.classname == "item_artifact_envirosuit")	return 20;

	// monsters -- worth a detour
	if ((thing.flags & FL_MONSTER) && thing.health > 0)	return 45;

	return 0;
};

// ----- bot_pick_goal: find the best thing to go after -----
// scope TRUE = full map (13000 units, pathfind to it)
// scope FALSE = nearby (500 units, line of sight required)

void(float scope) bot_pick_goal =
{
	local entity foe, best;
	local float want, bestwant;

	if (scope)
		foe = findradius(self.origin, 13000);
	else
		foe = findradius(self.origin, 500);

	bestwant = 0;
	best = world;
	while (foe)
	{
		want = bot_want(foe);
		if (want > 0 && !scope)
			if (!sisible(foe))
				want = 0;
		if (want > bestwant)
		{
			bestwant = want;
			best = foe;
		}
		foe = foe.chain;
	}
	if (best == world)
		return;
	if (target_onstack(best))
		return;

	target_add(best);
	if (scope)
		bot_get_path(best, FALSE);
};
