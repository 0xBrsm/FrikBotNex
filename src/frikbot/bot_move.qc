/***********************************************
*                                              *
*            FrikBot Movement AI               *
*     "The slightly better movement AI"        *
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

void() bot_jump =
{
	self.button2 = TRUE;
};

float(entity e) bot_can_rj =
{
	if (e.b_skill == 0)
		return FALSE;
	if (e.items & IT_QUAD)
		return FALSE;
	if (!((e.items & IT_ROCKET_LAUNCHER) && (e.ammo_rockets > 0)))
		return FALSE;
	if (e.items & IT_INVULNERABILITY)
		return TRUE;
	if (e.health > 50)
		return TRUE;
	return FALSE;
};

float(float flag) frik_recognize_plat =
{
	local entity plat_ent;

	if ((self.classname != "waypoint") && !(self.flags & FL_ONGROUND))
		return FALSE;

	if (self.classname == "waypoint")
	{
		traceline(self.origin, self.origin - '0 0 64', TRUE, self);
		plat_ent = trace_ent;
	}
	else
		plat_ent = self.sense_ground_ent;

	if (plat_ent != world)
	{
		if (flag)
		{
			if (self.b_wishdir_z > 0)
			{
				if (plat_ent.velocity_z > 0)
					self.b_wishdir = '0 0 0';
			}
			else if (self.b_wishdir_z < 0)
			{
				if (plat_ent.velocity_z < 0)
					self.b_wishdir = '0 0 0';
			}
		}
		return TRUE;
	}
	else
		return FALSE;
};





/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_movetogoal

Navigate toward target using the nav fan.
sense_best_dir picks the best walkable
direction toward the goal. No reactive
wall dodging needed.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() frik_movetogoal =
{
	local vector way;
	local float g, hgt, gap;

	if (self.target1 == world)
	{
		makevectors(self.v_angle);
		frik_walkmove(v_forward);
		return;
	}

	way = realorigin(self.target1) - self.origin;
	if (vlen(way) < 25)
	{
		self.b_wishdir = '0 0 0';
		return;
	}

	way_z = 0;
	if (bot_go(vectoyaw(way)))
		self._m_move_mode = MM_NAV;
	else
		self._m_move_mode = MM_BEELINE;

	// jump logic: step-ups and safe gap crossings
	if (self.flags & FL_ONGROUND)
	{
		hgt = self.sense_fwd_height;
		gap = self.sense_fwd_gap_width;

		if (hgt > 18)
		{
			// step up — only jump if landing is safe
			if (self.sense_fwd_ground_type >= -3)
				bot_jump();
		}
		else if (gap > 0 && hgt < -18 && self.sense_fwd_jumpable)
		{
			// gap — only jump if other side isn't lava
			if (self.sense_fwd_ground_type >= -3)
				bot_jump();
		}
	}

	frik_recognize_plat(TRUE);

	if (self.b_aiflags & AI_PRECISION)
	{
		g = angcomp(self.v_angle_x, self.b_angle_x);
		if (fabs(g) > 10)
			self.b_wishdir = '0 0 0';
		g = angcomp(self.v_angle_y, self.b_angle_y);
		if (fabs(g) > 10)
			self.b_wishdir = '0 0 0';
	}
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_walkmove

Try to move in a direction. Checks sense API
first — returns FALSE if blocked or unsafe.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

float(vector weird) frik_walkmove =
{
	return bot_go(vectoyaw(weird));
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

frik_bot_roam — hook navigation for roaming

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() frik_bot_roam =
{
	local vector org, ang, org1;
	local float loopcount, flag, dist;

	loopcount = 26;
	flag = FALSE;
	while((loopcount > 0) && !flag)
	{
		loopcount = loopcount - 1;
		org = self.origin + self.view_ofs;
		ang = self.angles;
		ang_y = frik_anglemod(ang_y - 90 + (random() * 180));
		ang_x = 0;
		makevectors(ang);
		traceline(org, org + v_forward * 2300, TRUE, self);
		if (trace_fraction != 1)
		{
			org1 = trace_endpos;
			ang = normalize(trace_plane_normal);
			ang_z = 0;
			traceline(org1, org1 + (ang * 2300), TRUE, self);
			if ((trace_fraction != 1) && (vlen(trace_endpos - org1) >= 64))
			{
				org = trace_endpos;
				traceline(org, self.origin + self.view_ofs, TRUE, self);
				if (trace_fraction != 1)
				{
					dist = vlen(org1 - org) /2;
					org = org1 + (ang * dist);
					traceline(org, org - '0 0 48', TRUE, self);
					if (trace_fraction != 1)
					{
						SpawnTempWaypoint(org);
						flag = TRUE;
					}
				}
			}
		}
	}
	self.b_angle_y = self.v_angle_y + 10;
	frik_walkmove(v_forward);
};
