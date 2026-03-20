/***********************************************
*                                              *
*                FrikBot Physics               *
*        Client movement emulation for bots    *
*                                              *
*         Special Thanks to: Asdf, Frog        *
*             Alan "Strider" Kivlin            *
*                                              *
***********************************************/

/*
This program is in the Public Domain.
*/

// ---- input: keys/movevect → view angles ----

void() CL_KeyMove =
{
	local float anglespeed;
	local vector view;
	if (self.keys != self.oldkeys)
	{
		self.movevect = '0 0 0';
		self.movevect_y = self.movevect_y + (350 * ((self.keys & KEY_MOVERIGHT) > 0));
		self.movevect_y = self.movevect_y - (350 * ((self.keys & KEY_MOVELEFT) > 0));
		self.movevect_x = self.movevect_x + (200 * ((self.keys & KEY_MOVEFORWARD) > 0));
		self.movevect_x = self.movevect_x - (200 * ((self.keys & KEY_MOVEBACK) > 0));
		self.movevect_z = self.movevect_z + (200 * ((self.keys & KEY_MOVEUP) > 0));
		self.movevect_z = self.movevect_z - (200 * ((self.keys & KEY_MOVEDOWN) > 0));
		if (!(self.b_aiflags & AI_PRECISION))
			self.movevect = self.movevect * 2;
	}
	self.oldkeys = self.keys;

	if (self.b_skill != 2)
	{
		anglespeed = 1.5 * real_frametime;
		self.v_angle_y = self.v_angle_y + anglespeed * ((self.keys & KEY_LOOKLEFT) > 0) * 140;
		self.v_angle_y = self.v_angle_y - anglespeed * ((self.keys & KEY_LOOKRIGHT) > 0) * 140;
		self.v_angle_x = self.v_angle_x - anglespeed * ((self.keys & KEY_LOOKUP) > 0) * 150;
		self.v_angle_x = self.v_angle_x + anglespeed * ((self.keys & KEY_LOOKDOWN) > 0) * 150;
	}
	else
	{
		view_x = angcomp(self.b_angle_x, self.v_angle_x);
		view_y = angcomp(self.b_angle_y, self.v_angle_y);
		if (vlen(view) > 30)
		{
			self.mouse_emu = self.mouse_emu + (view * 30);
			if (vlen(self.mouse_emu) > 180)
				self.mouse_emu = normalize(self.mouse_emu) * 180;
		}
		else if (real_frametime > 0.0001)
			self.mouse_emu = view * (1 / real_frametime);
		else
			self.mouse_emu = view;
		self.v_angle = self.v_angle + self.mouse_emu * real_frametime;
	}
	if (self.v_angle_x > 80)
		self.v_angle_x = 80;
	else if (self.v_angle_x < -70)
		self.v_angle_x = -70;
	if (self.v_angle_z > 50)
		self.v_angle_z = 50;
	else if (self.v_angle_z < -50)
		self.v_angle_z = -50;
	self.v_angle_y = frik_anglemod(self.v_angle_y);
};


// ---- velocity: friction, acceleration, clamping ----

void() SV_CheckVelocity =
{
	if (self.velocity_x != self.velocity_x) self.velocity_x = 0;
	if (self.velocity_y != self.velocity_y) self.velocity_y = 0;
	if (self.velocity_z != self.velocity_z) self.velocity_z = 0;
	if (self.velocity_x > 2000) self.velocity_x = 2000;
	if (self.velocity_x < -2000) self.velocity_x = -2000;
	if (self.velocity_y > 2000) self.velocity_y = 2000;
	if (self.velocity_y < -2000) self.velocity_y = -2000;
	if (self.velocity_z > 2000) self.velocity_z = 2000;
	if (self.velocity_z < -2000) self.velocity_z = -2000;
};

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

void(float scale) SV_AddGravity =
{
	self.velocity_z = self.velocity_z - (scale * sv_gravity * real_frametime);
};


// ---- movement dispatch: water, air, ground ----

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
	wishvel_z = 0;
	if (vlen(wishvel) > sv_maxspeed)
		wishvel = normalize(wishvel) * sv_maxspeed;
	if (self.flags & FL_ONGROUND)
	{
		SV_UserFriction();
		SV_Accelerate(wishvel);
	}
	else
		SV_AirAccelerate(wishvel);
	SV_CheckVelocity();
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


// ---- proxy: engine physics bridge ----

void() RemoveThud =
{
	local entity oself;
	if (self.flags & FL_ONGROUND)
		self.flags = self.flags - FL_ONGROUND;
	if (other == world || other == self.owner)
		return;
	if (self.owner.solid == SOLID_NOT)
		return;
	oself = other;
	other = self.owner;
	self = oself;
	if (self.solid == SOLID_BSP)
		if (self.touch)
			self.touch();
};

void() SV_CheckOnGround =
{
	local vector org;
	org = self.origin;
	if ((self.waterlevel == 3) && (self.movetype == MOVETYPE_WALK))
		self.flags = self.flags - (self.flags & FL_ONGROUND);
	else if (!droptofloor())
		self.flags = self.flags - (self.flags & FL_ONGROUND);
	else if (org_z - self.origin_z < 2)
		self.flags = self.flags | FL_ONGROUND;
	else
		self.flags = self.flags - (self.flags & FL_ONGROUND);
	setorigin(self, org);
};

void() PostPhysics =
{
	self = self.owner;
	self.velocity = self.velocity - self.phys_obj.dest1 + self.phys_obj.velocity;
	SV_CheckOnGround();
	PlayerPostThink();
	BotAI();
	self.dmg_take = self.dmg_save = 0;
};

void() SV_FlyMove =
{
	if (self.phys_obj == world)
	{
		self.phys_obj = find(world, classname, "phys_obj");
		while (self.phys_obj.owner != self)
		{
			self.phys_obj = find(self.phys_obj, classname, "phys_obj");
			if (self.phys_obj == world)
				error("No physics entity spawned!\nMake sure BotInit was called\n");
		}
		// one-time proxy setup
		setmodel(self.phys_obj, string_null);
		self.phys_obj.movetype = MOVETYPE_FLY;
		self.phys_obj.solid = SOLID_TRIGGER;
		self.phys_obj.touch = RemoveThud;
		setsize(self.phys_obj, self.mins, self.maxs);
	}

	// per-frame update
	self.phys_obj.origin = self.origin;
	self.phys_obj.dest1 = self.phys_obj.velocity = self.velocity;
	self.phys_obj.velocity_z = self.phys_obj.velocity_z + sv_gravity * real_frametime;
	self.phys_obj.flags = 0;
	self.phys_obj.think = PostPhysics;
	self.phys_obj.nextthink = time;
};


// ---- main dispatch ----

void() SV_Physics_Client =
{
	PlayerPreThink();

	if (self.movetype == MOVETYPE_NONE)
	{
		PlayerPostThink();
		BotAI();
	}
	else if (self.movetype == MOVETYPE_WALK)
	{
		if (!(self.waterlevel > 1) && (!(self.flags & FL_WATERJUMP)))
			SV_AddGravity(1);
		SV_FlyMove();
	}
	else if (self.movetype == MOVETYPE_NOCLIP)
	{
		self.origin = self.origin + real_frametime * self.velocity;
		PlayerPostThink();
		BotAI();
	}
	else
		error("SV_Physics_Client: Bad Movetype (BOT)");
};
