/***********************************************
*                                              *
*           FrikBot Movement Input             *
*  Converts AI direction to engine movevect    *
*                                              *
*  Engine handles everything else: friction,   *
*  acceleration, gravity, collision, step-ups  *
*  via SV_ClientThink + SV_WalkMove.           *
*                                              *
***********************************************/

/*
This program is in the Public Domain.
*/

// ---- b_wishdir → movevect ----
// Converts world-frame desired direction to view-relative movevect.
// Engine's SV_ClientThink reads movevect for friction + acceleration.

void() bot_build_movevect =
{
	local vector vang;
	local float spd;

	self.movevect = '0 0 0';

	if (self.b_wishdir == '0 0 0')
		return;

	// Engine's SV_ClientThink expects view-relative speeds:
	//   forwardmove = how fast forward/back
	//   sidemove = how fast left/right
	//   upmove = how fast up/down
	// It does its own makevectors(v_angle) to convert to world velocity.
	// So we project b_wishdir (world) onto view axes to get the right
	// forward/side decomposition.
	vang = self.v_angle;
	vang_x = vang_z = 0;
	makevectors(vang);

	spd = sv_maxspeed * 2;	// overshoot so engine clamp hits maxspeed
	if (self.b_aiflags & AI_PRECISION)
		spd = sv_maxspeed * 0.5;

	self.movevect_x = (self.b_wishdir * v_forward) * spd;
	self.movevect_y = (self.b_wishdir * v_right) * spd;
	self.movevect_z = self.b_wishdir_z * spd;
};
