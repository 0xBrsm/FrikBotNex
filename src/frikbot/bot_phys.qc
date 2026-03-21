/***********************************************
*                                              *
*                FrikBot Physics               *
*        Velocity computation for bots         *
*                                              *
*  Engine handles actual movement via patched   *
*  SV_Physics_Client (SV_WalkMove, gravity,    *
*  water, step-ups, collision). This file just  *
*  computes velocity from bot input (movevect). *
*                                              *
***********************************************/

/*
This program is in the Public Domain.
*/

// ---- input: b_wishdir → movevect ----
// Converts world-frame desired direction to view-relative movevect.
// Checks AI flags for stop/precision. SV_ClientThink clamps to sv_maxspeed.

void() bot_build_movevect =
{
	local vector vang;
	local float spd;

	self.movevect = '0 0 0';

	if (self.b_wishdir == '0 0 0')
		return;

	// project world direction onto view axes (yaw only, matching SV_ClientThink)
	vang = self.v_angle;
	vang_x = vang_z = 0;
	makevectors(vang);

	spd = sv_maxspeed;
	if (self.b_aiflags & AI_PRECISION)
		spd = spd * 0.5;

	self.movevect_x = (self.b_wishdir * v_forward) * spd;
	self.movevect_y = (self.b_wishdir * v_right) * spd;
	self.movevect_z = self.b_wishdir_z * spd;
};


/*
-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=

SV_ClientThink — compute velocity from movevect

Called once per bot per frame from BotFrame.
Reads self.movevect (set by bot_build_movevect) and
self.v_angle (set by bot_angle_set), applies
Quake friction + acceleration to self.velocity.

Engine then moves the bot via SV_WalkMove.

-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=-=
*/

void() SV_ClientThink =
{
	local vector wishvel, vangle;
	local vector vel, start, stop;
	local float sped, friction, newspeed;
	local float addspeed, wishspd, accelspeed, currentspeed;

	if (self.movetype == MOVETYPE_NONE)
		return;
	if (self.health <= 0)
		return;

	// angles → entity display
	self.v_angle_z = 0;
	self.angles_z = 0;
	vangle = self.v_angle + self.punchangle;
	if (!self.fixangle)
	{
		self.angles_x = (vangle_x / -3);
		self.angles_y = vangle_y;
	}
	else
	{
		self.v_angle = self.angles;
		self.fixangle = 0;
	}

	// waterjump: override velocity
	if (self.flags & FL_WATERJUMP)
	{
		if (time > self.teleport_time || !self.waterlevel)
		{
			self.flags = self.flags - (self.flags & FL_WATERJUMP);
			self.teleport_time = 0;
		}
		self.velocity_x = self.movedir_x;
		self.velocity_y = self.movedir_y;
		return;
	}

	// water movement
	if ((self.waterlevel >= 2) && (self.movetype != MOVETYPE_NOCLIP))
	{
		makevectors(self.v_angle);
		wishvel = v_right * self.movevect_y + v_forward * self.movevect_x;
		if (self.movevect == '0 0 0')
			wishvel_z = wishvel_z - 60;
		else
			wishvel_z = wishvel_z + self.movevect_z;
		wishspd = vlen(wishvel);
		if (wishspd > sv_maxspeed)
		{
			wishvel = (sv_maxspeed / wishspd) * wishvel;
			wishspd = sv_maxspeed;
		}
		wishspd = wishspd * 0.7;
		sped = vlen(self.velocity);
		if (sped)
		{
			newspeed = sped - (real_frametime * sped * sv_friction);
			if (newspeed < 0)
				newspeed = 0;
			self.velocity = self.velocity * (newspeed / sped);
		}
		else
			newspeed = 0;
		if (!wishspd)
			return;
		addspeed = wishspd - newspeed;
		if (addspeed <= 0)
			return;
		wishvel = normalize(wishvel);
		accelspeed = sv_accelerate * wishspd * real_frametime;
		if (accelspeed > addspeed)
			accelspeed = addspeed;
		self.velocity = self.velocity + accelspeed * wishvel;
		return;
	}

	// air/ground movement
	vangle = self.v_angle;
	vangle_x = vangle_z = 0;
	makevectors(vangle);
	if (time < self.teleport_time && (self.movevect_x < 0))
		self.movevect_x = 0;
	wishvel = v_right * self.movevect_y + v_forward * self.movevect_x;
	if (self.movetype != MOVETYPE_WALK)
		wishvel_z = self.movevect_z;
	else
		wishvel_z = 0;
	if (vlen(wishvel) > sv_maxspeed)
		wishvel = normalize(wishvel) * sv_maxspeed;

	if (self.movetype == MOVETYPE_NOCLIP)
	{
		self.velocity = wishvel;
		return;
	}

	if (self.flags & FL_ONGROUND)
	{
		// ground: friction + accelerate
		vel = self.velocity;
		vel_z = 0;
		sped = vlen(vel);
		vel = self.velocity;
		if (sped)
		{
			start_x = stop_x = self.origin_x + vel_x / (sped * 16);
			start_y = stop_y = self.origin_y + vel_y / (sped * 16);
			start_z = self.origin_z + self.mins_z;
			stop_z = start_z - 34;
			traceline(start, stop, TRUE, self);
			if (trace_fraction == 1)
				friction = sv_friction * 2;
			else
				friction = sv_friction;
			if (sped < sv_stopspeed)
				newspeed = sped - real_frametime * sv_stopspeed * friction;
			else
				newspeed = sped - real_frametime * sped * friction;
			if (newspeed < 0)
				newspeed = 0;
			newspeed = newspeed / sped;
			self.velocity_y = vel_y * newspeed;
			self.velocity_x = vel_x * newspeed;
		}

		wishspd = vlen(wishvel);
		if (wishspd < 0.001)
			return;
		wishvel = wishvel * (1.0 / wishspd);
		currentspeed = self.velocity * wishvel;
		addspeed = wishspd - currentspeed;
		if (addspeed <= 0)
			return;
		accelspeed = sv_accelerate * wishspd * real_frametime;
		if (accelspeed > addspeed)
			accelspeed = addspeed;
		self.velocity = self.velocity + accelspeed * wishvel;
	}
	else
	{
		// air: capped acceleration
		wishspd = vlen(wishvel);
		if (wishspd < 0.001)
			return;
		wishvel = wishvel * (1.0 / wishspd);
		if (wishspd > 30)
			wishspd = 30;
		currentspeed = self.velocity * wishvel;
		addspeed = wishspd - currentspeed;
		if (addspeed <= 0)
			return;
		accelspeed = sv_accelerate * wishspd * real_frametime;
		if (accelspeed > addspeed)
			accelspeed = addspeed;
		self.velocity = self.velocity + accelspeed * wishvel;
	}
};
