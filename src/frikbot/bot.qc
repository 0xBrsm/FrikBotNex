
/*
======================================
FrikBot X (Version 0.10.2)
======================================

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

======================================
These installation instructions only apply to Normal Quake (as does this
entire file). For QuakeWorld, please refer to bot_qw.qc

--------------------------------------
To install on a new mod, do all this:
--------------------------------------
Place all included bot*.qc files in the subdirectory "frikbot"
in your source folder, then...

* Add the following lines to progs.src right after the defs.qc line
frikbot/bot.qc
frikbot/bot_way.qc
frikbot/bot_think.qc
frikbot/bot_shoot.qc
frikbot/bot_fight.qc
frikbot/bot_ai.qc
frikbot/bot_misc.qc
frikbot/bot_phys.qc
frikbot/bot_move.qc
frikbot/bot_ed.qc

--------------------------------------
* Comment out the following functions in defs.qc
sound, stuffcmd, sprint, aim, centerprint, setspawnparms
WriteByte, WriteChar, WriteShort, WriteLong, WriteCoord
WriteAngle, WriteString, WriteEntity
--------------------------------------
* Add this to worldspawn() in world.qc, right at the very top, before InitBodyQue();
BotInit();  // FrikBot
--------------------------------------
* Add this line to StartFrame() in world.qc, at the very top
BotFrame(); // FrikBot
--------------------------------------
* Add these two lines to PlayerPreThink in client.qc at the very top
if (BotPreFrame()) // FrikBot
	return;
--------------------------------------
* Add this line to PlayerPostThink in client.qc at the very top
if (BotPostFrame()) // FrikBot
	return;
--------------------------------------
* Add the following line to the very top of Client Connect in client.qc
ClientInRankings(); // FrikBot
--------------------------------------
* Add these lines to the very top of ClientDisconnect in client.qc
ClientDisconnected(); // FrikBot
--------------------------------------

To install the waypoints:

add

waypoints/map_dm1.qc
waypoints/map_dm2.qc
waypoints/map_dm3.qc
waypoints/map_dm4.qc
waypoints/map_dm5.qc
waypoints/map_dm6.qc

To progs.src, immediately after defs.qc

To remove these waypoints:

Delete the code in bot_map_load, located below.
*/

void() bot_map_load =
{
	// place your qc loaded waypoints here

	if (mapname == "dm1")
		map_dm1();
	else if (mapname == "dm2")
		map_dm2();
	else if (mapname == "dm3")
		map_dm3();
	else if (mapname == "dm4")
		map_dm4();
	else if (mapname == "dm5")
		map_dm5();
	else if (mapname == "dm6")
		map_dm6();
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Variables and shtuff

bot.qc has become pretty much a header file
for all variables in the bot...

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

// ----- entity fields ---
.float	keys, ishuman;
.float	b_frags, b_shirt, b_pants;
.float	ai_time, b_sound, missile_speed;
.float	portal_time, b_skill;
.float	b_aiflags, b_num, b_chattime;
.float route_failed, dyn_flags, dyn_time;
.float dyn_plat;
.float b_strafe_time, b_strafe_dir;
.float b_aim_delay;
.float b_ambush_time;
.float b_wepswitch_time;
.float b_react_time;		// delay before engaging newly detected enemy
.float b_wep_commit_time;	// locked into current weapon until this time

// combat state machine
.float b_combat_state;
.float b_combat_time;		// when we entered current state

// reward profile (per-bot weights, set at spawn)
.float w_survival;
.float w_combat;
.float w_scavenge;
.float w_quad;
.float w_pent;
.float w_ring;

// item registry (cached on item entities at map load)
.float item_res;		// resource type: RES_HEALTH, RES_ARMOR, etc.
.float item_pool;		// which pool: POOL_SHELLS, POOL_NAILS, etc.
.float item_mag;		// magnitude: what this gives / pool capacity (0-1)
.float item_respawn_dur;	// seconds to respawn
.float item_respawn_at;		// time when this item returns (0 = available)
.entity _item_next;		// item registry linked list

// opponent modeling (skill 2+)
.entity b_opp_ent;		// which enemy we're tracking
.float b_opp_weapon;		// enemy's current weapon
.float b_opp_aggro;		// aggression estimate (0.0 = passive, 1.0 = rush)
.float b_opp_strafe;		// enemy strafe tendency (-1 to +1)
.float b_opp_dist;		// enemy preferred distance
.float b_opp_track_time;	// last opponent model update

// combat states (state machine)
float CSTATE_NONE	= 0;	// not in combat
float CSTATE_FIGHT	= 1;	// standard engage (strafe + shoot)
float CSTATE_RETREAT	= 2;	// outgunned, falling back
.entity	temp_way, last_way;
.entity	target1, target2, target3, target4;
.entity	_next, _last;
.entity	current_way;
.vector	b_angle, mouse_emu, b_dir;
.vector	movevect;
.vector	b_wishdir;	// desired world-frame movement direction (unit vector or '0 0 0')
.vector dyn_dest;

// --------defines-----

// these are aiflags for waypoints
// some overlap to the bot
float AI_TELELINK_1	= 1; // link type
float AI_TELELINK_2	= 2; // link type
float AI_TELELINK_3	= 4; // link type
float AI_TELELINK_4	= 8; // link type
float AI_DOORFLAG		= 16; // read ahead
float AI_PRECISION	= 32; // read ahead + point
float AI_SURFACE		= 64; // point 
float AI_BLIND		= 128; // read ahead + point
float AI_JUMP		= 256; // point + ignore
float AI_DIRECTIONAL	= 512; // read ahead + ignore
float AI_PLAT_BOTTOM	= 1024; // read ahead 
float AI_RIDE_TRAIN	= 2048; // read ahead 
float AI_SUPER_JUMP	= 4096; // point + ignore + route test
float AI_SNIPER		= 8192; // point type 
float AI_AMBUSH		= 16384; // point type
float AI_DOOR_NO_OPEN	= 32768; // read ahead
float AI_DIFFICULT	= 65536; // route test
float AI_TRACE_TEST	= 131072; // route test

// these are flags for bots/players (dynamic/editor flags)
float AI_HOLD_SELECT	= 2;
float AI_WAIT		= 4;


// addition masks
float AI_POINT_TYPES 	= 29152;
float AI_READAHEAD_TYPES	= 36528;
float AI_IGNORE_TYPES	= 4864;

float WM_UNINIT		= 0;
float WM_DYNAMIC		= 1;
float WM_LOADING		= 2;
float WM_LOADED		= 3;
float WM_EDITOR		= 4;
float WM_EDITOR_DYNAMIC	= 5;
float WM_EDITOR_DYNLINK	= 6;


float OPT_SAVEBOTS	= 1;
float OPT_NOCHAT	= 2;

// -------globals-----
entity	item_head;		// head of item registry linked list
float	item_reg_time;		// last time registry was rebuilt

float 	active_clients;
float		max_clients;
float		bot_count, b_options;
float		waypoint_mode;
float		waypoints, direct_route;
float		sv_gravity, sv_maxspeed, sv_ticrate;
float		bot_nocombat;	// scratch1 cvar: 1=pacifist mode
entity	fixer;
entity	route_table;
entity	b_temp1, b_temp2, b_temp3;
entity 	player_head, way_head;
float		busy_waypoints;

// pending bot connection queue (set by BotConnect, consumed by ClientConnected)
// pending bot connection
float		_bot_pending_skill;
float		_bot_pending_count;		// how many bots awaiting ClientConnect
float		_bot_colors_pants, _bot_colors_shirt;

// -------ProtoTypes------
// external
void()				ClientConnect;
void()				ClientDisconnect;
void()				SetNewParms;

// rankings
float(float clientno)		ClientBitFlag;
void(float whichteam, float whatbot, float whatskill) BotConnect;
void(entity bot)			BotDisconnect;
void(float clientno)		BotInvalidClientNo;

// waypointing
void()				DynamicWaypoint;
entity(vector org)		make_waypoint;
void()				ClearAllWays;
void()				FixWaypoints;
float()				begin_route;
void()				bot_path;
void()				bot_handle_ai;
void(entity this, float direct)			bot_get_path;
void()				WaypointThink;
entity(entity start)				FindWayPoint;

// physics & movement
float(entity e)			bot_can_rj;
void()				bot_jump;
void()				frik_bot_roam;
float(vector weird)		frik_walkmove;
void()				frik_movetogoal;
float(float flag)			frik_recognize_plat;

// sense API
void()				bot_sense_space;
float(float desired_yaw)	bot_go;
void()				bot_build_movevect;

// ai & misc
string()				PickARandomName;
float(entity targ)		fov;
float(float y1, float y2)	angcomp;
float(entity targ1, entity targ2)		wisible;
float(entity targ)		sisible;
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
void() bot_path;
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

	// detect bot vs human:
	// - reconnecting bot: ishuman=FALSE (set by DecodeLevelParms via parm11)
	// - new bot: _bot_pending_count > 0
	// - human: neither
	if (self.ishuman == FALSE && _bot_pending_count > 0)
	{
		// new bot — first connection
				self.netname = PickARandomName();
		self.b_pants = _bot_colors_pants;
		self.b_shirt = _bot_colors_shirt;
		self.b_skill = _bot_pending_skill;
		bot_count = bot_count + 1;
		active_clients = active_clients | ClientBitFlag(cno);
		_bot_pending_count = _bot_pending_count - 1;
		return;
	}
	if (self.ishuman == FALSE)
	{
		// reconnecting bot (changelevel) — engine restored name/colors
		// from client_t, DecodeLevelParms restored skill from parm10
				bot_count = bot_count + 1;
		active_clients = active_clients | ClientBitFlag(cno);
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
	DynamicWaypoint();
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


void(entity e1, entity e2, float flag) DeveloperLightning =
{
	// used to show waypoint links for debugging
	WriteByte (MSG_BROADCAST, 23);
	if (flag)
		WriteByte (MSG_BROADCAST, 6);
	else
		WriteByte (MSG_BROADCAST, 13);
	WriteEntity (MSG_BROADCAST, e2);
	WriteCoord (MSG_BROADCAST, e1.origin_x);
	WriteCoord (MSG_BROADCAST, e1.origin_y);
	WriteCoord (MSG_BROADCAST, e1.origin_z);
	WriteCoord (MSG_BROADCAST, e2.origin_x);
	WriteCoord (MSG_BROADCAST, e2.origin_y);
	WriteCoord (MSG_BROADCAST, e2.origin_z); 
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
	local float pending;

	whatskill = rint(whatskill);
	if (whatskill > 3) whatskill = 3;
	else if (whatskill < 0) whatskill = 0;
	_bot_pending_skill = whatskill;
	_bot_pending_count = _bot_pending_count + 1;

	// trigger engine connection via bot net driver
	pending = cvar("bot_pending");
	cvar_set("bot_pending", ftos(pending + 1));
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
			bot.colormap = 0;
			BotDisconnect(bot);
			active_clients = active_clients | ClientBitFlag(self.colormap - 1);
			BotConnect(0, bot.b_num, bot.b_skill); 
			return;
		}
	}
};

/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

Waypoint Loading from file

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/
void() LoadWaypoint = 
{
	local vector org;
	local entity tep;
	local float r;
	org_x = cvar("saved1");
	org_y = cvar("saved2");
	org_z = cvar("saved3");
	
	tep = make_waypoint(org);

	r = cvar("saved4");

	tep.b_aiflags = floor(r / 4);
	tep.b_pants = cvar("scratch1");
	tep.b_skill = cvar("scratch2");
	tep.b_shirt = cvar("scratch3");
	tep.b_frags = cvar("scratch4");
};


void() WaypointWatch =
{
	// Waypoint Baywatch
	local float bigboobs;
	local string h;
	if (framecount < 4)
		return;
	if (max_clients < 2)
		return;
	if (waypoint_mode != WM_UNINIT)
	{
		bigboobs = cvar("saved4");
		if (bigboobs != 0)
		{
			if ((bigboobs & 3) == 1)
				ClearAllWays();
			else if ((bigboobs & 3) == 3)
			{
				FixWaypoints();
				h = ftos(b_options);
				cvar_set("saved1", h);
				cvar_set("saved4", "0");
				cvar_set("scratch1", "0");
				waypoint_mode = WM_LOADED;
				return;
			}
			LoadWaypoint();
			waypoint_mode = WM_LOADING;
			cvar_set("saved4", "0");
		}
	}
	else
	{
		b_options = cvar("saved1");
		cvar_set ("saved4", "0");
		if (max_clients > 1)
		{
			localcmd("exec maps/");
			localcmd(mapname);
			localcmd(".way\n");
			waypoint_mode = WM_DYNAMIC;
			bot_map_load();
		}
		else
			waypoint_mode = WM_LOADED;
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
	bot_nocombat = cvar("scratch1");
	
	self = nextent(world);
	num = 0;
	while (num < max_clients)
	{
		if (self.ishuman == FALSE)
		{
			if (active_clients & ClientBitFlag(num))
			{
				BotAI();			// decide direction → b_wishdir + b_angle
				// view angles: bot_angle_set applies turn speed limit
				if (self.v_angle_x > 80) self.v_angle_x = 80;
				else if (self.v_angle_x < -70) self.v_angle_x = -70;
				self.v_angle_z = 0;
				self.v_angle_y = frik_anglemod(self.v_angle_y);
				bot_build_movevect();	// b_wishdir → movevect
				// engine reads movevect in SV_RunClients → SV_ClientThink
			}
		}
		self = nextent(self);
		num = num + 1;
	}
	WaypointWatch();
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



