/*
Bot network driver.

Provides fake network connections so the engine processes bots
identically to real players. The driver keeps a small queue of
bot_connect requests, mirrors the stock signon reply sequence,
and then emits normal clc_move packets from QC-populated fields.
*/

#include "quakedef.h"
#include "net_bot.h"
#include "nav_bot.h"

// Driver index in net_drivers[]. Set by Bot_Init.
static int bot_driverlevel = -1;

// Bot cvars (registered early so autoexec/commandline can set them)
static cvar_t bot_nocombat_cvar = {"bot_nocombat", "1"};
static cvar_t bot_fixedgoal_cvar = {"bot_fixedgoal", "0"};

// QC field offsets (cached on first use via ED_FindField).
static int movevect_ofs = -1;

#define BOT_MAX_PENDING	MAX_SCOREBOARD
#define BOT_SIGNON_WAIT		0
#define BOT_SIGNON_PRESPAWN	1
#define BOT_SIGNON_SPAWN		2
#define BOT_SIGNON_BEGIN		3
#define BOT_SIGNON_DONE		4

typedef struct
{
	int profile;
	int skill;
} bot_request_t;

typedef struct
{
	int signon_reply;
	int profile;
	int skill;
} bot_client_t;

static bot_request_t bot_pending[BOT_MAX_PENDING];
static int bot_pending_head = 0;
static int bot_pending_tail = 0;
static int bot_pending_count = 0;

extern ddef_t *ED_FindField (char *name);
extern client_t *host_client;

static int Bot_FindField (char *name)
{
	ddef_t *def = ED_FindField (name);
	return def ? def->ofs : 0;
}

static bot_client_t *Bot_ClientData (qsocket_t *sock)
{
	if (!sock || sock->driver != bot_driverlevel)
		return NULL;
	return (bot_client_t *)sock->driverdata;
}

static void Bot_ProfileInfo (int profile, char **name, int *shirt, int *pants)
{
	*name = "FrikBot";
	*shirt = 2;
	*pants = 11;

	switch (profile)
	{
	case 1:  *name = "Vincent";  *shirt = 0;  *pants = 11; break;
	case 2:  *name = "Bishop";   *shirt = 3;  *pants = 1;  break;
	case 3:  *name = "Nomad";    *shirt = 2;  *pants = 13; break;
	case 4:  *name = "Hudson";   *shirt = 6;  *pants = 7;  break;
	case 5:  *name = "Lore";     *shirt = 6;  *pants = 12; break;
	case 6:  *name = "Servo";    *shirt = 4;  *pants = 4;  break;
	case 7:  *name = "Gort";     *shirt = 5;  *pants = 2;  break;
	case 8:  *name = "Kryten";   *shirt = 3;  *pants = 10; break;
	case 9:  *name = "Pimp Bot"; *shirt = 4;  *pants = 9;  break;
	case 10: *name = "Max";      *shirt = 7;  *pants = 4;  break;
	case 11: *name = "Marvin";   *shirt = 11; *pants = 3;  break;
	case 12: *name = "Erwin";    *shirt = 12; *pants = 13; break;
	case 13: *name = "FrikBot";  *shirt = 2;  *pants = 11; break;
	case 14: *name = "Krosis";   *shirt = 2;  *pants = 0;  break;
	case 15: *name = "Gypsy";    *shirt = 9;  *pants = 8;  break;
	case 16: *name = "Hal";      *shirt = 10; *pants = 5;  break;
	}
}

static int Bot_ProfileByName (const char *name)
{
	int i;
	char *n;
	int s, p;

	for (i = 1; i <= 16; i++)
	{
		Bot_ProfileInfo (i, &n, &s, &p);
		if (!Q_strcasecmp (name, n))
			return i;
	}
	return 0;
}

// addbot [skill]        — random profile
// addbot <name> [skill] — specific profile by name
static void Bot_AddBot_f (void)
{
	int profile = 0;
	int skill = 1;
	int argc = Cmd_Argc();
	const char *arg1;

	if (bot_pending_count >= BOT_MAX_PENDING)
	{
		Con_Printf ("addbot: queue full\n");
		return;
	}

	if (argc >= 2)
	{
		arg1 = Cmd_Argv(1);
		profile = Bot_ProfileByName (arg1);
		if (profile)
		{
			// addbot <name> [skill]
			if (argc >= 3)
				skill = atoi (Cmd_Argv(2));
		}
		else
		{
			// addbot <skill>
			skill = atoi (arg1);
			profile = 0;
		}
	}

	if (skill < 0) skill = 0;
	else if (skill > 3) skill = 3;

	bot_pending[bot_pending_tail].profile = profile;
	bot_pending[bot_pending_tail].skill = skill;
	bot_pending_tail = (bot_pending_tail + 1) % BOT_MAX_PENDING;
	bot_pending_count++;
}

// ---- driver interface ----

int Bot_Init (void)
{
	bot_driverlevel = net_driverlevel;
	Cmd_AddCommand ("addbot", Bot_AddBot_f);
	Cvar_RegisterVariable (&bot_nocombat_cvar);
	Cvar_RegisterVariable (&bot_fixedgoal_cvar);
	Nav_RegisterBuiltins ();
	return 0;
}

void Bot_Listen (qboolean state) { }
void Bot_SearchForHosts (qboolean xmit) { }
qsocket_t *Bot_Connect (char *host) { return NULL; }

qsocket_t *Bot_CheckNewConnections (void)
{
	qsocket_t *sock;
	bot_client_t *botdata;
	bot_request_t *req;
	int save_driverlevel;

	if (bot_pending_count <= 0)
		return NULL;

	save_driverlevel = net_driverlevel;
	net_driverlevel = bot_driverlevel;
	sock = NET_NewQSocket ();
	net_driverlevel = save_driverlevel;
	if (!sock)
		return NULL;

	botdata = Z_Malloc (sizeof(*botdata));
	req = &bot_pending[bot_pending_head];
	botdata->profile = req->profile;
	botdata->skill = req->skill;
	botdata->signon_reply = BOT_SIGNON_WAIT;

	bot_pending_head = (bot_pending_head + 1) % BOT_MAX_PENDING;
	bot_pending_count--;

	sock->driverdata = botdata;
	Q_strcpy (sock->address, "bot");
	sock->socket = 0;
	sock->canSend = true;
	return sock;
}

int Bot_GetMessage (qsocket_t *sock)
{
	bot_client_t *botdata;
	edict_t *ent;
	float *mv;
	char *name;
	char cmd[64];
	int shirt;
	int pants;

	if (!sock->canSend)
		return 0;
	sock->canSend = false;

	botdata = Bot_ClientData (sock);
	if (!botdata)
		return 0;

	SZ_Clear (&net_message);

	if (botdata->signon_reply != BOT_SIGNON_DONE)
	{
		switch (botdata->signon_reply)
		{
		case BOT_SIGNON_PRESPAWN:
			MSG_WriteByte (&net_message, clc_stringcmd);
			MSG_WriteString (&net_message, "prespawn");
			botdata->signon_reply = BOT_SIGNON_WAIT;
			return 1;

		case BOT_SIGNON_SPAWN:
			Bot_ProfileInfo (botdata->profile, &name, &shirt, &pants);

			if (host_client)
			{
				host_client->spawn_parms[9] = botdata->skill;
				host_client->spawn_parms[10] = 1;
				host_client->spawn_parms[11] = botdata->profile;
			}

			MSG_WriteByte (&net_message, clc_stringcmd);
			snprintf (cmd, sizeof(cmd), "name \"%s\"\n", name);
			MSG_WriteString (&net_message, cmd);

			MSG_WriteByte (&net_message, clc_stringcmd);
			snprintf (cmd, sizeof(cmd), "color %d %d\n", shirt, pants);
			MSG_WriteString (&net_message, cmd);

			MSG_WriteByte (&net_message, clc_stringcmd);
			MSG_WriteString (&net_message, "spawn");
			botdata->signon_reply = BOT_SIGNON_WAIT;
			return 1;

		case BOT_SIGNON_BEGIN:
			MSG_WriteByte (&net_message, clc_stringcmd);
			MSG_WriteString (&net_message, "begin");
			botdata->signon_reply = BOT_SIGNON_DONE;
			return 1;
		}
		return 0;
	}

	ent = host_client->edict;
	if (!ent)
		return 0;

	if (movevect_ofs == -1)
		movevect_ofs = Bot_FindField ("movevect");
	if (!movevect_ofs)
		return 0;

	mv = (float *)((int *)&ent->v + movevect_ofs);

	MSG_WriteByte (&net_message, clc_move);
	MSG_WriteFloat (&net_message, sv.time);
	MSG_WriteAngle (&net_message, ent->v.v_angle[0]);
	MSG_WriteAngle (&net_message, ent->v.v_angle[1]);
	MSG_WriteAngle (&net_message, ent->v.v_angle[2]);
	MSG_WriteShort (&net_message, (int)mv[0]);
	MSG_WriteShort (&net_message, (int)mv[1]);
	MSG_WriteShort (&net_message, (int)mv[2]);
	MSG_WriteByte (&net_message, (int)ent->v.button0
	                           | ((int)ent->v.button2 << 1));
	MSG_WriteByte (&net_message, (int)ent->v.impulse);

	return 1;
}

int Bot_SendMessage (qsocket_t *sock, sizebuf_t *data)
{
	bot_client_t *botdata;
	int i;
	int stage;
	int latest_stage;

	botdata = Bot_ClientData (sock);
	latest_stage = 0;
	if (botdata)
	{
		for (i = 0; i < data->cursize - 1; i++)
		{
			if (data->data[i] != svc_signonnum)
				continue;
			stage = data->data[i + 1];
			if (stage >= 1 && stage <= 3 && stage > latest_stage)
				latest_stage = stage;
		}
		if (latest_stage)
			botdata->signon_reply = latest_stage;
	}

	sock->canSend = true;
	return 1;
}

int Bot_SendUnreliableMessage (qsocket_t *sock, sizebuf_t *data)
{
	sock->canSend = true;
	return 1;
}

qboolean Bot_CanSendMessage (qsocket_t *sock)
{
	sock->canSend = true;
	return true;
}

qboolean Bot_CanSendUnreliableMessage (qsocket_t *sock) { return true; }

void Bot_Close (qsocket_t *sock)
{
	if (sock && sock->driverdata)
	{
		Z_Free (sock->driverdata);
		sock->driverdata = NULL;
	}
}

void Bot_Shutdown (void)
{
	bot_pending_head = 0;
	bot_pending_tail = 0;
	bot_pending_count = 0;
}
