// FrikBotNex — evolved from FrikBot X (v0.10.2) by Ryan "FrikaC" Smith
// Original FrikBot code is public domain. See LICENSE for details.

// ----- entity fields -----
.float	ishuman;
.float	b_frags, b_shirt, b_pants;
.float	ai_time, b_sound, missile_speed;
.float	portal_time, b_skill;
.float	b_num, b_chattime;
.float b_strafe_time, b_strafe_dir;
.float b_aim_delay;
.float b_ambush_time;
.float b_wepswitch_time;
.float b_react_time;
.float b_wep_commit_time;
.float b_combat_state;
.float b_combat_time;
.float _m_goals_reached;

// goal weights (set by act_normal at spawn)
.float w_survival, w_combat, w_scavenge;
.float w_quad, w_pent, w_ring;

// item registry (cached on item entities)
.float item_res, item_pool, item_mag;
.float item_respawn_dur, item_respawn_at;
.entity _item_next;

// opponent modeling
.entity b_opp_ent;
.float b_opp_weapon, b_opp_aggro, b_opp_strafe;
.float b_opp_dist, b_opp_track_time;

// combat states
float CSTATE_NONE    = 0;
float CSTATE_FIGHT   = 1;
float CSTATE_RETREAT = 2;

.entity	target1, target2, target3, target4;
.entity	_next, _last;
.vector	b_angle, mouse_emu;
.vector	movevect;
.vector	b_wishdir;

// movement modes (for metrics)
.float _m_move_mode;
float MM_NAV     = 1;
float MM_BEELINE = 2;
float MM_COMBAT  = 3;
float MM_ROAM    = 4;

.float _botcam_hud_time;	// throttle for botcam debug overlay
.float _botcam_beam_time;	// faster refresh for botcam beam temp ents
.float _stuck_count;		// forward decl (defined in bot_move.qc)

// ---- globals ----
entity	item_head;
float	item_reg_time;
float	active_clients;
float	max_clients;
float	bot_count;
float	sv_gravity, sv_maxspeed, sv_ticrate;
float	bot_nocombat;
entity	b_temp1, b_temp2, b_temp3;
entity	player_head;
float	_bot_pending_profiles;
float	_bot_colors_pants, _bot_colors_shirt;

// ---- prototypes ----
void()				ClientConnect;
void()				ClientDisconnect;
void()				SetNewParms;
float(float clientno)		ClientBitFlag;
void(float whichteam, float whatbot, float whatskill) BotConnect;
void(entity bot)		BotDisconnect;
void(float clientno)		BotInvalidClientNo;
float(vector weird)		frik_walkmove;
void()				frik_movetogoal;
void()				bot_build_movevect;

// navmesh builtins (Detour pathfinding)
float()					nav_ready = #80;
vector(vector pos, vector target)	nav_move = #81;
float(vector goal)			nav_path_start = #82;
float(vector start, vector goal)	nav_route_cost = #83;
vector(vector pos)			nav_path_steer = #84;
vector(entity bot, float index)	nav_path_debug = #85;
float		nav_booted;

.entity _nav_goal_ent;	// entity we last computed a path to

// ai & misc
float()				PickARandomBotProfile;
string()				PickARandomName;
float(entity targ)		fov;
float(float y1, float y2)	angcomp;
float(entity targ)		fisible;
vector(entity ent)		realorigin;
void(entity ent)			target_drop;
void(entity ent)			target_add;
void()				KickABot;
void()				BotImpulses;
void(entity targ, float success) bot_lost;
string(float r)			BotName;
float(float v)			frik_anglemod;
void() bot_chat;
void(float tpic) bot_start_topic;

// bot_fight.qc — combat, weapons, enemy detection
void() bot_opp_model_update;
void() bot_opp_model_reset;
entity() bot_nearest_teammate;
float(entity e) bot_size_player;
vector(float wep) weapon_range;
float(float wep, float brange) bot_weapon_score;
void(float brange) bot_weapon_switch;
void() bot_shoot;
void() bot_projectile_scan;
void() bot_find_enemy;
void() bot_fight_style;
void() bot_combat_strafe;
void() bot_retreat;
void() bot_drop_enemy;

// bot_goal.qc — item registry + goal system
void() map_stuff;
void() act_normal;
float(entity thing) bot_stuff;
float(entity thing) bot_want;
void(float scope) bot_get;

// bot_ai.qc — main AI loop
void() bot_handle_ai;
void() bot_server_spawn;
void() BotAI;


// ----------Commands---------
// aliases for builtins overridden by bot.qc
void(entity e, float chan, string samp, float vol, float atten) frik_sound = #8;
vector(entity e, float sped)	frik_aim = #44;

//----------------------------------------------------------------------------

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Function overrides. Bots are real engine clients
now — most wrappers just call the builtin directly.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/
vector(entity e, float sped) aim =
{
	e.missile_speed = sped;
	return frik_aim(e, sped);
};

void(entity e, float chan, string samp, float vol, float atten) sound =
{
	frik_sound(e, chan, samp, vol, atten);
	if (samp != "items/inv3.wav")
	{
		if (e.classname == "player")
			e.b_sound = time + 1;
		else if (other.classname == "player")
			other.b_sound = time + 1;
	}
};
/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Bot Cam, see what the bot sees (or any other player)

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float() botcam =
{
	if (self.classname != "botcam")
		return FALSE;
	if (!self.enemy || self.enemy == world)
	{
		// target gone — exit botcam
		msg_entity = self;
		WriteByte (MSG_ONE, 5);
		WriteEntity (MSG_ONE, self);
		PutClientInServer();
		return TRUE;
	}
	// PVS: keep spectator near target so world renders correctly
	setorigin(self, self.enemy.origin);

	// HUD state
	self.items = self.enemy.items;
	self.weapon = self.enemy.weapon;
	self.weaponmodel = self.enemy.weaponmodel;
	self.currentammo = self.enemy.currentammo;
	self.weaponframe = self.enemy.weaponframe;
	self.ammo_shells = self.enemy.ammo_shells;
	self.ammo_nails = self.enemy.ammo_nails;
	self.ammo_rockets= self.enemy.ammo_rockets;
	self.ammo_cells = self.enemy.ammo_cells;
	self.view_ofs = self.enemy.view_ofs;
	self.health = self.enemy.health;
	self.armorvalue = self.enemy.armorvalue;
	self.deadflag = self.enemy.deadflag;

	// zero these — engine sends them to our client separately
	self.punchangle = '0 0 0';
	self.dmg_take = 0;
	self.dmg_save = 0;

	// angles: use fixangle for smooth engine-native sync
	// (no WriteAngle quantization jitter)
	self.v_angle = self.enemy.v_angle;
	self.angles = self.enemy.v_angle;
	self.fixangle = TRUE;

	self.modelindex = 0;
	self.impulse = 0;

	// --- botcam debug HUD ---
	if (time > self._botcam_hud_time)
	{
		local entity b;
		local string mode, s1;
		local vector info;

		b = self.enemy;
		self._botcam_hud_time = time + 1;

		if (b._m_move_mode == MM_NAV) mode = "NAV";
		else if (b._m_move_mode == MM_BEELINE) mode = "NOPATH";
		else if (b._m_move_mode == MM_COMBAT) mode = "COMBAT";
		else mode = "IDLE";

		info = nav_path_debug(b, -1);

		if (b.target1 && b.target1 != world)
		{
			s1 = ftos(rint(vlen(realorigin(b.target1) - b.origin)));
			centerprint(self, mode, " ", b.target1.classname, " d:", s1);
		}
		else
			centerprint(self, mode, " no target");
	}

	return TRUE;
};

void() botcam_u =
{
	local entity start;

	// cycle to next trackable bot (skip self and other botcams)
	if (self.classname != "botcam")
		self.enemy = player_head;
	else
	{
		self.enemy = self.enemy._next;
		if (!self.enemy)
			self.enemy = player_head;
	}
	start = self.enemy;
	while (self.enemy == self || self.enemy.classname == "botcam")
	{
		self.enemy = self.enemy._next;
		if (!self.enemy)
			self.enemy = player_head;
		if (self.enemy == start)
			break; // looped all the way around
	}
	// if we only found self/botcams, no valid target
	if (self.enemy == self || self.enemy.classname == "botcam")
	{
		sprint(self, "No one left to track!\n");
		return;
	}
	self.classname = "botcam";
	self.solid = SOLID_NOT;
	self.movetype = MOVETYPE_NONE;
	self.takedamage = DAMAGE_NO;
	if (!self.enemy.ishuman)
	{
		self.enemy.dmg_take = 0;
		self.enemy.dmg_save = 0;
	}
	// set viewport to target (persists until changed)
	msg_entity = self;
	WriteByte (MSG_ONE, 5);
	WriteEntity (MSG_ONE, self.enemy);
	sprint(self, "Now tracking ");
	sprint(self, self.enemy.netname);
	sprint(self, "\n");
};



/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Stuff mentioned up top
it just links the bot into the mod

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() ClientInRankings =
{
	local float cno;
	local entity t;

	cno = self.colormap - 1;
	if (cno >= 0 && cno < max_clients)
		BotInvalidClientNo (cno);

	// guard against double-insertion — if already in list, remove first
	t = player_head;
	while (t)
	{
		if (t == self)
		{
			// already in list — unlink before re-inserting
			if (player_head == self)
				player_head = self._next;
			if (self._next)
				self._next._last = self._last;
			if (self._last)
				self._last._next = self._next;
			self._next = world;
			self._last = world;
			break;
		}
		t = t._next;
	}

	if (player_head)
		player_head._last = self;

	self._next = player_head;
	self._last = world;
	player_head = self;

	if (parm11)
	{
		self.ishuman = FALSE;
		self.b_skill = parm10;
		self.b_num = parm12;
		if (self.b_num < 1 || self.b_num > 16)
			self.b_num = 13;
		self.netname = BotName(self.b_num);
		self.b_pants = _bot_colors_pants;
		self.b_shirt = _bot_colors_shirt;
		_bot_pending_profiles = _bot_pending_profiles
			- (_bot_pending_profiles & ClientBitFlag(self.b_num - 1));
		bot_count = bot_count + 1;
		active_clients = active_clients | ClientBitFlag(cno);
		act_normal();	// initialize goal weights
		return;
	}

	// human player
	active_clients = active_clients | ClientBitFlag(cno);
	self.ishuman = TRUE;
};


void() ClientDisconnected =
{
	// exit botcam cleanly before disconnect
	if (self.classname == "botcam")
	{
		self.classname = "player";
		self.enemy = world;
	}

	if (player_head == self)
		player_head = self._next;
	if (self._next)
		self._next._last = self._last;
	if (self._last)
		self._last._next = self._next;

	active_clients = active_clients - (active_clients & ClientBitFlag(self.colormap - 1));
};
/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BotPreFrame & BotPostFrame, used to make the
bot easier to install

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/
float () BotPreFrame =
{
	if (self.colormap < 1)
		return TRUE;
	if (self.ishuman)
	{
		if (self.classname == "botcam")
			return TRUE;
	}
	// track frag changes for chat
	if (self.b_frags != self.frags)
	{
		if (self.b_frags > self.frags)
		{
			if (pointcontents(self.origin) == CONTENT_LAVA)
				bot_start_topic(10);
			else
				bot_start_topic(9);
		}
		else
			bot_start_topic(2);
		self.b_frags = self.frags;
	}
	// return FALSE — let PlayerPreThink run for bots (water, powerups, etc.)
	return FALSE;
};
float () BotPostFrame =
{
	if (self.colormap < 1)
		return TRUE;
	if (self.ishuman)
	{

		// editor display removed (bot_ed.qc not compiled)

		BotImpulses();

		if (botcam())
			return TRUE;
	}
	return FALSE;
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Bot Chat code

The rest of this code is in bot_misc.qc

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/
void(string h) BotSay = // simulate talking by composing a 'chat' message
{
	WriteByte(MSG_ALL, 8);
	WriteByte(MSG_ALL, 1);
	WriteString(MSG_ALL, self.netname);
	WriteByte(MSG_ALL, 8);
	WriteByte(MSG_ALL, 2);
	WriteString(MSG_ALL, h);
};
void() BotSayInit =
{
	WriteByte(MSG_ALL, 8);
	WriteByte(MSG_ALL, 1);
	WriteString(MSG_ALL, self.netname);
};
void(string h) BotSay2 =
{
	WriteByte(MSG_ALL, 8);
	WriteByte(MSG_ALL, 2);
	WriteString(MSG_ALL, h);
};
void(string h) BotSayTeam = 
{
	local entity t;
	if (!teamplay)
		return;
	t = player_head;
	while(t)
	{
		if (t.team == self.team)
		{
			msg_entity = t;
			WriteByte(MSG_ONE, 8);
			WriteByte(MSG_ONE, 1);
			WriteByte(MSG_ONE, 40);
			WriteString(MSG_ONE, self.netname);
			WriteByte(MSG_ONE, 8);
			WriteByte(MSG_ONE, 2);
			WriteByte(MSG_ONE, 41);
			WriteString(MSG_ONE, h);
		}
		t = t._next;
	}
};
/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BotInit

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/


void() BotInit =
{
	local entity ent;


	// count client slots
	ent = nextent(world);
	max_clients = 0;
	while(ent != world)
	{
		max_clients = max_clients + 1;
		ent = nextent(ent);
	}
	if (max_clients > 16)
		max_clients = 16;

	precache_model("progs/s_light.spr");
	precache_model("progs/s_bubble.spr");

};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Rankings 'utilities'. Written by Alan Kivlin,
this code just fools clients by sending precisely
the same network messages as when a real player
signs on to the server.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/



float(float clientno) ClientBitFlag =
{
	// bigger, but faster
	if (clientno == 0)
		return 1;
	else if (clientno == 1)
		return 2;
	else if (clientno == 2)
		return 4;
	else if (clientno == 3)
		return 8;
	else if (clientno == 4)
		return 16;
	else if (clientno == 5)
		return 32;
	else if (clientno == 6)
		return 64;
	else if (clientno == 7)
		return 128;
	else if (clientno == 8)
		return 256;
	else if (clientno == 9)
		return 512;
	else if (clientno == 10)
		return 1024;
	else if (clientno == 11)
		return 2048;
	else if (clientno == 12)
		return 4096;
	else if (clientno == 13)
		return 8192;
	else if (clientno == 14)
		return 16384;
	else if (clientno == 15)
		return 32768;
	return 0;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BotConnect and related functions.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/				
entity(float num) GetClientEntity =
{
	local entity upsy;
	upsy = world;
	num = num + 1;
	while (num > 0)
	{
		num = num - 1;
		upsy = nextent(upsy);
	} 
	return upsy;
};

void(float whichteam, float whatbot, float whatskill) BotConnect =
{
	whatskill = rint(whatskill);
	if (whatskill > 3) whatskill = 3;
	else if (whatskill < 0) whatskill = 0;

	whatbot = rint(whatbot);
	if (whatbot < 1 || whatbot > 16)
		whatbot = PickARandomBotProfile();

	_bot_pending_profiles = _bot_pending_profiles | ClientBitFlag(whatbot - 1);

	// queue engine-side connection by name
	localcmd("addbot ");
	localcmd(BotName(whatbot));
	localcmd(" ");
	localcmd(ftos(whatskill));
	localcmd("\n");
};

void(entity bot) BotDisconnect =
{
	local entity uself;
	uself = self;
	self = bot;

	bot_count = bot_count - 1;
	active_clients = active_clients - (active_clients & ClientBitFlag(self.colormap - 1));
	self.colormap = 0;
	ClientDisconnect();

	self = uself;
};
/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

BotInvalidClientNo
kicks a bot if a player connects and takes the bot's space

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void(float clientno) BotInvalidClientNo =
{
	local entity bot;

	bot = GetClientEntity(clientno);
	if(bot != self && !bot.ishuman && (active_clients & ClientBitFlag(clientno)))
	{
		if (active_clients & ClientBitFlag(self.colormap - 1))
		{
			BotDisconnect(bot);
			active_clients = active_clients | ClientBitFlag(self.colormap - 1);
			BotConnect(0, bot.b_num, bot.b_skill); 
			return;
		}
	}
};

void() BotFrame =
{
	local float num;

	bot_server_spawn();
	// cache cvars used by bot AI/sense
	sv_maxspeed = cvar("sv_maxspeed");
	sv_gravity = cvar("sv_gravity");
	sv_ticrate = cvar("sys_ticrate");
	bot_nocombat = cvar("bot_nocombat");
	// trigger navmesh build on first frame (all entities spawned by now)
	if (!nav_booted)
	{
		nav_ready();
		nav_booted = TRUE;
	}
	
	self = nextent(world);
	num = 0;
	while (num < max_clients)
	{
		if (self.ishuman == FALSE)
		{
			if (active_clients & ClientBitFlag(num))
			{
				BotAI();
				if (self.v_angle_x > 80) self.v_angle_x = 80;
				else if (self.v_angle_x < -70) self.v_angle_x = -70;
				self.v_angle_z = 0;
				self.v_angle_y = frik_anglemod(self.v_angle_y);
				bot_build_movevect();
			}
		}
		self = nextent(self);
		num = num + 1;
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Bot Impulses. Allows the player to perform bot
related functions.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() BotImpulses =
{
	local float f;

	if (self.impulse == 100)
	{
		f = cvar("skill");
		BotConnect(0, 0, f);
	}
	else if (self.impulse == 101)
	{
		f = cvar("skill");
		BotConnect(1, 0, f);
	}
	else if (self.impulse == 102)
		KickABot();
	else if (self.impulse == 103)
		botcam_u();
	// impulse 104 (waypoint editor) removed — bot_ed.qc not compiled
	else
		return;

	self.impulse = 0;
};



