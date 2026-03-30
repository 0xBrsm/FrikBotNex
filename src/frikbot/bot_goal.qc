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
	{
		it.item_res = RES_AMMO; it.item_pool = POOL_ROCKETS; it.item_respawn_dur = RESPAWN_AMMO;
		if (it.spawnflags & 1) it.item_mag = GIVE_ROCKETS_LG / CAP_ROCKETS;
		else it.item_mag = GIVE_ROCKETS_SM / CAP_ROCKETS;
		return;
	}
	if (it.classname == "item_cells")
	{
		it.item_res = RES_AMMO; it.item_pool = POOL_CELLS; it.item_respawn_dur = RESPAWN_AMMO;
		if (it.spawnflags & 1) it.item_mag = GIVE_CELLS_LG / CAP_CELLS;
		else it.item_mag = GIVE_CELLS_SM / CAP_CELLS;
		return;
	}
	if (it.classname == "item_spikes")
	{
		it.item_res = RES_AMMO; it.item_pool = POOL_NAILS; it.item_respawn_dur = RESPAWN_AMMO;
		if (it.spawnflags & 1) it.item_mag = GIVE_NAILS_LG / CAP_NAILS;
		else it.item_mag = GIVE_NAILS_SM / CAP_NAILS;
		return;
	}
	if (it.classname == "item_shells")
	{
		it.item_res = RES_AMMO; it.item_pool = POOL_SHELLS; it.item_respawn_dur = RESPAWN_AMMO;
		if (it.spawnflags & 1) it.item_mag = GIVE_SHELLS_LG / CAP_SHELLS;
		else it.item_mag = GIVE_SHELLS_SM / CAP_SHELLS;
		return;
	}

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

bot_has — how full is the bot on this item's resource?
0 = empty/don't own, 1 = full/capped.

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

float(entity it) bot_has =
{
	local float remaining;

	if (it.item_res == RES_WEAPON)
		if (!bot_has_weapon_for(it.item_pool))
			return 0;

	if (it.item_pool == POOL_QUAD)
	{ remaining = self.super_damage_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (it.item_pool == POOL_PENT)
	{ remaining = self.invincible_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (it.item_pool == POOL_RING)
	{ remaining = self.invisible_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }
	if (it.item_pool == POOL_SUIT)
	{ remaining = self.radsuit_finished - time; if (remaining <= 0) return 0; return remaining / POWERUP_DUR; }

	if (it.classname == "item_health" && (it.spawnflags & 2))
		return self.health / 250;
	if (it.item_pool == POOL_HEALTH)	return self.health / CAP_HEALTH;
	if (it.item_pool == POOL_ARMOR)		return (self.armorvalue * self.armortype) / CAP_ARMOR_EHP;
	if (it.item_pool == POOL_ROCKETS)	return self.ammo_rockets / CAP_ROCKETS;
	if (it.item_pool == POOL_CELLS)		return self.ammo_cells / CAP_CELLS;
	if (it.item_pool == POOL_NAILS)		return self.ammo_nails / CAP_NAILS;
	if (it.item_pool == POOL_SHELLS)	return self.ammo_shells / CAP_SHELLS;
	return 0;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

bot_want — how much does the bot want this item?

Complete desire score (0-1), irrespective of distance.
Combines: what the item gives (mag), what the bot has
(bot_has), bot state (survival/combat axes), and
personality weights.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(entity it) bot_want =
{
	local float has, need, w;
	local float hp, ap, survival;
	local float best_tier, best_ammo, combat;

	// bot state: two axes
	hp = self.health / CAP_HEALTH;
	if (hp > 1) hp = 1;
	ap = (self.armorvalue * self.armortype) / CAP_ARMOR_EHP;
	if (ap > 1) ap = 1;
	survival = hp * (0.5 + ap * 0.5);

	best_tier = 0;
	best_ammo = 0;
	if (self.items & IT_ROCKET_LAUNCHER)
	{	best_tier = 1.0; best_ammo = self.ammo_rockets / CAP_ROCKETS; }
	else if (self.items & IT_LIGHTNING)
	{	best_tier = 1.0; best_ammo = self.ammo_cells / CAP_CELLS; }
	else if (self.items & IT_GRENADE_LAUNCHER)
	{	best_tier = 0.5; best_ammo = self.ammo_rockets / CAP_ROCKETS; }
	else if (self.items & IT_SUPER_NAILGUN)
	{	best_tier = 0.5; best_ammo = self.ammo_nails / CAP_NAILS; }
	else if (self.items & IT_NAILGUN)
	{	best_tier = 0.15; best_ammo = self.ammo_nails / CAP_NAILS; }
	else if (self.items & IT_SUPER_SHOTGUN)
	{	best_tier = 0.15; best_ammo = self.ammo_shells / CAP_SHELLS; }
	combat = best_tier * best_ammo;

	// how much of this resource does the bot already have?
	has = bot_has(it);
	if (has < 0) has = 0;
	if (has > 1) has = 1;

	// objective need: what the item gives vs what the bot has
	if (it.item_res == RES_ARMOR)
		need = it.item_mag - has;
	else
		need = it.item_mag * (1 - has) * (1 - has);
	if (need < 0) need = 0;

	// subjective weight: personality × situational urgency
	if (it.item_res == RES_HEALTH)
	{
		local float item_survival;
		item_survival = has * (0.5 + ap * 0.5);
		w = (1 - has) * (1 - item_survival) * self.w_survival;
	}
	else if (it.item_res == RES_ARMOR)
		w = (1 - ap) * (1 - survival) * self.w_survival;
	else if (it.item_res == RES_WEAPON)
		w = (1 - best_tier) * (1 - combat) * self.w_combat;
	else if (it.item_res == RES_AMMO)
	{
		if (it.item_pool == 0)
			w = (1 - combat) * self.w_scavenge;
		else if (bot_has_weapon_for(it.item_pool))
			w = (1 - has) * (1 - combat) * self.w_combat;
		else
			w = (1 - combat) * self.w_scavenge * 0.1;
	}
	else if (it.item_res == RES_POWERUP)
	{
		if (it.item_pool == POOL_QUAD)
			w = combat * survival * self.w_quad;
		else if (it.item_pool == POOL_PENT)
		{
			w = 1 - survival;
			if (combat > w) w = combat;
			w = w * self.w_pent;
		}
		else if (it.item_pool == POOL_RING)
			w = combat * survival * self.w_ring;
		else
			w = 0.1;
	}
	else
		return 0;

	return need * w;
};

/* bot_get is now in bot_move.qc — follows cached path.
   Goal selection moved to nav_find_goal (C builtin #86). */
