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

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Map Stuff — item registry

Classifies every item ONCE at map load.
Caches objective facts on each entity:
what resource, which pool, how much it gives.
All string comparisons happen here, nowhere else.
Rebuilt every 10s to catch backpacks.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// ---- resource types ----
float RES_HEALTH	= 1;
float RES_ARMOR		= 2;
float RES_WEAPON	= 3;
float RES_AMMO		= 4;
float RES_POWERUP	= 5;

// ---- resource pools ----
float POOL_HEALTH	= 1;
float POOL_ARMOR	= 2;
float POOL_SHELLS	= 3;
float POOL_NAILS	= 4;
float POOL_ROCKETS	= 5;
float POOL_CELLS	= 6;
float POOL_QUAD		= 7;
float POOL_PENT		= 8;
float POOL_RING		= 9;
float POOL_SUIT		= 10;

// ---- pool capacities (engine constants) ----
float CAP_HEALTH	= 100;
float CAP_ARMOR_EHP	= 160;	// RA effective HP: 200 * 0.8
float CAP_SHELLS	= 100;
float CAP_NAILS		= 200;
float CAP_ROCKETS	= 100;
float CAP_CELLS		= 100;

// ---- pickup amounts (engine constants) ----
float GIVE_HEALTH_NORMAL = 25;
float GIVE_HEALTH_MEGA	= 100;
float GIVE_WEP_SHELLS	= 5;
float GIVE_WEP_NAILS	= 30;
float GIVE_WEP_ROCKETS	= 5;
float GIVE_WEP_CELLS	= 15;
float GIVE_SHELLS_SM	= 20;	float GIVE_SHELLS_LG	= 40;
float GIVE_NAILS_SM	= 25;	float GIVE_NAILS_LG	= 50;
float GIVE_ROCKETS_SM	= 5;	float GIVE_ROCKETS_LG	= 10;
float GIVE_CELLS_SM	= 6;	float GIVE_CELLS_LG	= 12;

// ---- powerup duration ----
float POWERUP_DUR	= 30;

// ---- reward profiles ----
// called at bot spawn to set per-bot weights

void() act_normal =
{
	self.w_survival = 1.0;
	self.w_combat = 1.0;
	self.w_scavenge = 0.5;
	self.w_quad = 0.8;
	self.w_pent = 0.8;
	self.w_ring = 0.4;
};

// ---- respawn times (engine constants) ----
float RESPAWN_ITEM	= 20;
float RESPAWN_AMMO	= 30;
float RESPAWN_QUAD	= 60;
float RESPAWN_MEGA	= 120;
float RESPAWN_PENT	= 300;
float RESPAWN_RING	= 300;
float RESPAWN_SUIT	= 60;

void(entity it) map_classify =
{
	if (it.item_res)
		return;

	// powerups
	if (it.classname == "item_artifact_super_damage")
	{ it.item_res = RES_POWERUP; it.item_pool = POOL_QUAD; it.item_mag = 1.0; it.item_respawn_dur = RESPAWN_QUAD; return; }
	if (it.classname == "item_artifact_invulnerability")
	{ it.item_res = RES_POWERUP; it.item_pool = POOL_PENT; it.item_mag = 1.0; it.item_respawn_dur = RESPAWN_PENT; return; }
	if (it.classname == "item_artifact_invisibility")
	{ it.item_res = RES_POWERUP; it.item_pool = POOL_RING; it.item_mag = 1.0; it.item_respawn_dur = RESPAWN_RING; return; }
	if (it.classname == "item_artifact_envirosuit")
	{ it.item_res = RES_POWERUP; it.item_pool = POOL_SUIT; it.item_mag = 0.15; it.item_respawn_dur = RESPAWN_SUIT; return; }

	// health
	if (it.classname == "item_health")
	{
		it.item_res = RES_HEALTH; it.item_pool = POOL_HEALTH;
		if (it.spawnflags & 2)
		{ it.item_mag = GIVE_HEALTH_MEGA / CAP_HEALTH; it.item_respawn_dur = RESPAWN_MEGA; }
		else
		{ it.item_mag = GIVE_HEALTH_NORMAL / CAP_HEALTH; it.item_respawn_dur = RESPAWN_ITEM; }
		return;
	}

	// armor (magnitude = effective HP / EHP cap)
	if (it.classname == "item_armorInv")
	{ it.item_res = RES_ARMOR; it.item_pool = POOL_ARMOR; it.item_mag = 1.0; it.item_respawn_dur = RESPAWN_ITEM; return; }
	if (it.classname == "item_armor2")
	{ it.item_res = RES_ARMOR; it.item_pool = POOL_ARMOR; it.item_mag = 0.5625; it.item_respawn_dur = RESPAWN_ITEM; return; }
	if (it.classname == "item_armor1")
	{ it.item_res = RES_ARMOR; it.item_pool = POOL_ARMOR; it.item_mag = 0.1875; it.item_respawn_dur = RESPAWN_ITEM; return; }

	// weapons (magnitude = bonus ammo / pool cap)
	if (it.classname == "weapon_rocketlauncher")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_ROCKETS; it.item_mag = GIVE_WEP_ROCKETS / CAP_ROCKETS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "weapon_lightning")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_CELLS; it.item_mag = GIVE_WEP_CELLS / CAP_CELLS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "weapon_grenadelauncher")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_ROCKETS; it.item_mag = GIVE_WEP_ROCKETS / CAP_ROCKETS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "weapon_supernailgun")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_NAILS; it.item_mag = GIVE_WEP_NAILS / CAP_NAILS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "weapon_nailgun")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_NAILS; it.item_mag = GIVE_WEP_NAILS / CAP_NAILS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "weapon_supershotgun")
	{ it.item_res = RES_WEAPON; it.item_pool = POOL_SHELLS; it.item_mag = GIVE_WEP_SHELLS / CAP_SHELLS; it.item_respawn_dur = RESPAWN_AMMO; return; }

	// ammo boxes
	if (it.classname == "item_rockets")
	{ it.item_res = RES_AMMO; it.item_pool = POOL_ROCKETS; it.item_mag = ((it.spawnflags & 1) ? GIVE_ROCKETS_LG : GIVE_ROCKETS_SM) / CAP_ROCKETS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "item_cells")
	{ it.item_res = RES_AMMO; it.item_pool = POOL_CELLS; it.item_mag = ((it.spawnflags & 1) ? GIVE_CELLS_LG : GIVE_CELLS_SM) / CAP_CELLS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "item_spikes")
	{ it.item_res = RES_AMMO; it.item_pool = POOL_NAILS; it.item_mag = ((it.spawnflags & 1) ? GIVE_NAILS_LG : GIVE_NAILS_SM) / CAP_NAILS; it.item_respawn_dur = RESPAWN_AMMO; return; }
	if (it.classname == "item_shells")
	{ it.item_res = RES_AMMO; it.item_pool = POOL_SHELLS; it.item_mag = ((it.spawnflags & 1) ? GIVE_SHELLS_LG : GIVE_SHELLS_SM) / CAP_SHELLS; it.item_respawn_dur = RESPAWN_AMMO; return; }

	// backpacks (dynamic)
	if (it.model == "progs/backpack.mdl")
	{ it.item_res = RES_AMMO; it.item_pool = 0; it.item_mag = 0.25; it.item_respawn_dur = 0; return; }
};

void() map_stuff =
{
	local entity e, prev;
	item_head = world;
	prev = world;

	e = nextent(world);
	while (e)
	{
		if (e.item_res && e.item_respawn_dur == 0)
			e.item_res = 0;
		map_classify(e);
		if (e.item_res)
		{
			if (prev)
				prev._item_next = e;
			else
				item_head = e;
			e._item_next = world;
			prev = e;
		}
		e = nextent(e);
	}
	item_reg_time = time;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_stuff — how full is the bot on this
item's resource? 0 = empty/don't own, 1 = full.

For weapons: 0 if don't own, ammo level if do.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(float pool) bot_has_weapon_for =
{
	if (pool == POOL_ROCKETS)	return (self.items & IT_ROCKET_LAUNCHER) || (self.items & IT_GRENADE_LAUNCHER);
	if (pool == POOL_CELLS)		return (self.items & IT_LIGHTNING);
	if (pool == POOL_NAILS)		return (self.items & IT_SUPER_NAILGUN) || (self.items & IT_NAILGUN);
	if (pool == POOL_SHELLS)	return (self.items & IT_SUPER_SHOTGUN) || (self.items & IT_SHOTGUN);
	return FALSE;
};

float(entity thing) bot_stuff =
{
	local float remaining;

	// weapons: 0 if don't own, ammo level if do
	if (thing.item_res == RES_WEAPON)
		if (!bot_has_weapon_for(thing.item_pool))
			return 0;

	// powerups: normalized time remaining (0 = don't have, 1 = just grabbed)
	if (thing.item_pool == POOL_QUAD)
	{ remaining = self.super_damage_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (thing.item_pool == POOL_PENT)
	{ remaining = self.invincible_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (thing.item_pool == POOL_RING)
	{ remaining = self.invisible_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (thing.item_pool == POOL_SUIT)
	{ remaining = self.radsuit_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }

	if (thing.item_pool == POOL_HEALTH)	return self.health / CAP_HEALTH;
	if (thing.item_pool == POOL_ARMOR)	return (self.armorvalue * self.armortype) / CAP_ARMOR_EHP;
	if (thing.item_pool == POOL_ROCKETS)	return self.ammo_rockets / CAP_ROCKETS;
	if (thing.item_pool == POOL_CELLS)	return self.ammo_cells / CAP_CELLS;
	if (thing.item_pool == POOL_NAILS)	return self.ammo_nails / CAP_NAILS;
	if (thing.item_pool == POOL_SHELLS)	return self.ammo_shells / CAP_SHELLS;
	return 0;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_want — reward profile

How much does this bot care about this item?
Returns a weight based on two axes of bot state:

survival = hp * (0.5 + ap * 0.5)
combat   = best_weapon_tier * ammo_for_best

Each category's weight = own_deficit * axis_deficit * param.
Sibling resources contribute context without overriding.

Three tunable params: A (survival), B (combat), C (powerups).

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(entity thing) bot_want =
{
	local float w;
	local float hp, ap, survival;
	local float tier, ammo, combat;

	// ---- bot state: two axes ----

	hp = self.health / CAP_HEALTH;
	if (hp > 1) hp = 1;
	ap = (self.armorvalue * self.armortype) / CAP_ARMOR_EHP;
	if (ap > 1) ap = 1;
	survival = hp * (0.5 + ap * 0.5);

	tier = 0;
	ammo = 0;
	if (self.items & IT_ROCKET_LAUNCHER)
	{	tier = 1.0; ammo = self.ammo_rockets / CAP_ROCKETS; }
	else if (self.items & IT_LIGHTNING)
	{	tier = 1.0; ammo = self.ammo_cells / CAP_CELLS; }
	else if (self.items & IT_GRENADE_LAUNCHER)
	{	tier = 0.5; ammo = self.ammo_rockets / CAP_ROCKETS; }
	else if (self.items & IT_SUPER_NAILGUN)
	{	tier = 0.5; ammo = self.ammo_nails / CAP_NAILS; }
	else if (self.items & IT_NAILGUN)
	{	tier = 0.15; ammo = self.ammo_nails / CAP_NAILS; }
	else if (self.items & IT_SUPER_SHOTGUN)
	{	tier = 0.15; ammo = self.ammo_shells / CAP_SHELLS; }
	combat = tier * ammo;

	// ---- category weight (0-1) ----

	if (thing.item_res == RES_HEALTH)
		w = (1 - hp) * (1 - survival) * self.w_survival;
	else if (thing.item_res == RES_ARMOR)
		w = (1 - ap) * (1 - survival) * self.w_survival;
	else if (thing.item_res == RES_WEAPON)
		w = (1 - tier) * (1 - combat) * self.w_combat;
	else if (thing.item_res == RES_AMMO)
	{
		if (thing.item_pool == 0)
			w = (1 - combat) * self.w_scavenge;
		else if (bot_has_weapon_for(thing.item_pool))
			w = (1 - ammo) * (1 - combat) * self.w_combat;
		else
			w = (1 - combat) * self.w_scavenge * 0.1;
	}
	else if (thing.item_res == RES_POWERUP)
	{
		if (thing.item_pool == POOL_QUAD)
			w = combat * survival * self.w_quad;
		else if (thing.item_pool == POOL_PENT)
		{
			w = 1 - survival;
			if (combat > w) w = combat;
			w = w * self.w_pent;
		}
		else if (thing.item_pool == POOL_RING)
			w = combat * survival * self.w_ring;
		else
			w = 0.1;
	}
	else
		return 0;

	return w;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_get — pick the best item to go after

Iterates the item registry. Scores each item:
  score = raw_contribution * bot_want

raw = mag / (bot_stuff + mag)   — objective
bot_want                         — subjective profile

scope TRUE = full map + pathfind
scope FALSE = nearby (500u) + LOS

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void(float scope) bot_get =
{
	local entity it, best;
	local float score, bestscore, raw, has, mag, dist;

	if (time > item_reg_time + 10)
		map_stuff();

	bestscore = 0;
	best = world;
	it = item_head;

	while (it)
	{
		dist = vlen(it.origin - self.origin);

		if (!scope)
			if (dist > 500)
			{ it = it._item_next; continue; }

		// availability check
		score = 0;
		if (it.item_res)
		{
			if (it.flags & FL_ITEM)
			{
				if (it.model == string_null)
				{
					// picked up — stamp respawn time
					if (it.item_respawn_at == 0 && it.item_respawn_dur > 0)
						it.item_respawn_at = time + it.item_respawn_dur;
				}
				else
				{
					it.item_respawn_at = 0;
					// raw contribution * profile weight * proximity
					mag = it.item_mag;
					has = bot_stuff(it);
					raw = mag / (has + mag);
					score = raw * bot_want(it) * (400 / (400 + dist));
				}
			}
		}

		if (score > 0 && !scope)
			if (!sisible(it))
				score = 0;

		if (score > bestscore)
		{
			bestscore = score;
			best = it;
		}
		it = it._item_next;
	}

	if (best == world)
		return;
	if (target_onstack(best))
		return;

	target_add(best);
	if (scope)
	{
		bot_get_path(best, FALSE);
		self.b_aiflags = self.b_aiflags | AI_WAIT;
	}
};
