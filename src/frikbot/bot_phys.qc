/***********************************************
*                                              *
*                FrikBot Physics               *
*        Velocity computation for bots         *
*                                              *
*  Engine handles actual movement via patched   *
*  SV_Physics_Client (SV_WalkMove, gravity,    *
*  water, step-ups, collision). This file just  *
*  computes velocity from bot input.            *
*                                              *
***********************************************/

/*
This program is in the Public Domain.
*/

// ---- input: view angles from bot AI ----
// bot_angle_set writes self.b_angle with skill-based aim.
// We just apply it directly, like a real mouse client would.

void() CL_KeyMove =
{
	self.v_angle = self.b_angle;
	if (self.v_angle_x > 80)
		self.v_angle_x = 80;
	else if (self.v_angle_x < -70)
		self.v_angle_x = -70;
	self.v_angle_z = 0;
	self.v_angle_y = frik_anglemod(self.v_angle_y);
};


// ---- velocity: friction + acceleration ----

void() SV_UserFriction =
{
	local vector vel, start, stop;
	local float sped, friction, newspeed;

	vel = self.velocity;
	vel_z = 0;
	sped = vlen(vel);
	vel = self.velocity;

	if (!sped)
		return;

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
};

void(vector wishvel) SV_AirAccelerate =
{
	local float addspeed, wishspd, accelspeed, currentspeed;

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
};

void(vector wishvel) SV_Accelerate =
{
	local float addspeed, wishspd, accelspeed, currentspeed;

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
};


// ---- movement dispatch ----

void() SV_WaterJump =
{
	if (time > self.teleport_time || !self.waterlevel)
	{
		self.flags = self.flags - (self.flags & FL_WATERJUMP);
		self.teleport_time = 0;
	}
	self.velocity_x = self.movedir_x;
	self.velocity_y = self.movedir_y;
};

void() SV_WaterMove =
{
	local vector wishvel;
	local float wishspeed, addspeed, cspeed, newspeed;
	makevectors(self.v_angle);
	wishvel = v_right * self.movevect_y + v_forward * self.movevect_x;
	if (self.movevect == '0 0 0')
		wishvel_z = wishvel_z - 60;
	else
		wishvel_z = wishvel_z + self.movevect_z;
	wishspeed = vlen(wishvel);
	if (wishspeed > sv_maxspeed)
	{
		wishvel = (sv_maxspeed / wishspeed) * wishvel;
		wishspeed = sv_maxspeed;
	}
	wishspeed = wishspeed * 0.7;
	cspeed = vlen(self.velocity);
	if (cspeed)
	{
		newspeed = cspeed - (real_frametime * cspeed * sv_friction);
		if (newspeed < 0)
			newspeed = 0;
		self.velocity = self.velocity * (newspeed / cspeed);
	}
	else
		newspeed = 0;
	if (!wishspeed)
		return;
	addspeed = wishspeed - newspeed;
	if (addspeed <= 0)
		return;
	wishvel = normalize(wishvel);
	cspeed = sv_accelerate * wishspeed * real_frametime;
	if (cspeed > addspeed)
		cspeed = addspeed;
	self.velocity = self.velocity + cspeed * wishvel;
};

void() SV_AirMove =
{
	local vector wishvel, vangle;

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
		self.velocity = wishvel;
	else if (self.flags & FL_ONGROUND)
	{
		SV_UserFriction();
		SV_Accelerate(wishvel);
	}
	else
		SV_AirAccelerate(wishvel);
};

void() SV_ClientThink =
{
	local vector vangle;

	if (self.movetype == MOVETYPE_NONE)
		return;
	if (self.health <= 0)
		return;
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
	if (self.flags & FL_WATERJUMP)
	{
		SV_WaterJump();
		return;
	}
	if ((self.waterlevel >= 2) && (self.movetype != MOVETYPE_NOCLIP))
	{
		SV_WaterMove();
		return;
	}
	SV_AirMove();
};
