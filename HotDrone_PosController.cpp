/*************************************************************
 * Copyright @ Dullerud's Autonomous Lab
 * University of Illinois Urbana Champaign
 * 
 * Author @ Bicheng Zhang <viczhang1990@gmail.com>
 * 
 * <---3D Simulation Framework for Autonomous Vehicles-->
 *
 *
 *            ********                  *****
 *             -A---               --A---
 *                
 *                      U U
 *                      ^
 *
 *
 *
 ************************************************************/

#include <time.h>

#include "HotDrone_PosController.h"
//global


HotDrone_PosController::HotDrone_PosController()
{
	this->last_t = clock();

	time_past = 0;

	_x_integral = 0;
	_y_integral = 0;
	_z_integral = 0;

	_vx_integral = 0;
	_vy_integral = 0;
	_vz_integral = 0;

	_last_pos_z = 0;
	_integrated_h_err = 0;
	_limit_thrust_int = 0.05;
	_thrust_feedforward = 0.69;




	this->Kp_vxy = HotDrone_PosController_VEL_P;
	this->Ki_vxy = HotDrone_PosController_VEL_I;
	this->Kd_vxy = HotDrone_PosController_VEL_D;


	_height_p = HOTDRONE_POSCONTROLLER_HEIGHT_P;
	_height_i = HOTDRONE_POSCONTROLLER_HEIGHT_I;
	_height_d = HOTDRONE_POSCONTROLLER_HEIGHT_D;

	_pos_p = HOTDRONE_POSCONTROLLER_POS_P;
	_pos_i = HOTDRONE_POSCONTROLLER_POS_I;
	_pos_d = HOTDRONE_POSCONTROLLER_POS_D;

	_limit_roll = HOTDRONE_POSCONTROLLER_LIMIT_ROLL;
	_limit_pitch = HOTDRONE_POSCONTROLLER_LIMIT_PITCH;

	_limit_thrust_upper = HOTDRONE_POSCONTROLLER_LIMIT_THRUSTER_UPPER;
	_limit_thrust_lower = HOTDRONE_POSCONTROLLER_LIMIT_THRUSTER_LOWER;
	_limit_height_err = HOTDRONE_POSCONTROLLER_LIMIT_HEIGHT_ERR;
	_thrust_feedforward = HOTDRONE_POSCONTROLLER_THRUST_FEEDFORWARD;

	_limit_pos_err = HOTDRONE_POSCONTROLLER_LIMIT_POS_ERR;
}

/*
 * WARNING: Error dt
 *
 *
 *
 */
void 
	HotDrone_PosController::update(const vehicle_local_pos_sp_t pos_sp, const vehicle_local_pos_t pos, 
												const vehicle_local_vel_t vel, const hotdrone_att_t att, hotdrone_att_sp_t &att_sp)
{
	
	clock_t cur_t = clock();
	
	float dt = (float) (cur_t - last_t)*TIME_SCALE;


	//printf("dt:%0.2f,z_sp:%0.2f, z:%0.2f\n", dt, -pos_sp.z, -pos.z);
	// Control x
	float err_pos_x = saturate(pos_sp.x - pos.x , _limit_pos_err, -_limit_pos_err);
	_x_integral += err_pos_x * dt;
	float cntrl_x_sig = _pos_p*err_pos_x + _pos_i*_x_integral + _pos_d*vel.vx;
	float x_rate_err = saturate(cntrl_x_sig - vel.vx, _limit_pos_err, -_limit_pos_err);
	_vx_integral += x_rate_err*dt;
	float ang_ref_x = -1 * (Kp_vxy*x_rate_err + Ki_vxy*_x_integral + Kd_vxy*vel.vx);

	//printf("t:%0.2f,x:%0.2f,vx:%0.4f,P:%0.4f,I:%0.4f,D:%0.4f,ang_ref:%0.3f\n", 
		//time_past,pos.x,vel.vx, Kp_vxy*x_rate_err, Ki_vxy*x_integral, Kd_vxy*vel.vx, ang_ref_x);

	// Control y
	float err_pos_y = saturate(pos_sp.y - pos.y , _limit_pos_err, -_limit_pos_err);
	_y_integral += err_pos_y * dt;
	float cntrl_y_sig = _pos_p*err_pos_y + _pos_i*_y_integral + _pos_d*vel.vy;
	float y_rate_err = saturate(cntrl_y_sig - vel.vy, _limit_pos_err, -_limit_pos_err);
	_vy_integral += y_rate_err*dt;
	float ang_ref_y =  (Kp_vxy*y_rate_err - Ki_vxy*_y_integral + Kd_vxy*vel.vy);



	float err_pos_z = saturate(pos.z - pos_sp.z, _limit_height_err, -_limit_height_err);
	float thrust_p = err_pos_z * _height_p;
	float thrust_d = vel.vz * _height_d;
	_z_integral += err_pos_z * dt;
	float thrust_i = _z_integral * _height_i;
	float thrust_control = _thrust_feedforward + thrust_p + thrust_d + thrust_i;
	thrust_control = saturate(thrust_control, _limit_thrust_upper, _limit_thrust_lower);

	//printf("thrust_sp:%0.4f, p:%0.4f, i:%0.4f, d:%0.4f\n", thrust_control, thrust_p, thrust_i, thrust_d);

	last_t = cur_t;

	att_sp.roll= ang_ref_y = saturate(ang_ref_y, _limit_roll, -_limit_roll);
	att_sp.pitch= ang_ref_x = saturate(ang_ref_x, _limit_pitch, -_limit_pitch);
	att_sp.throttle = thrust_control;

	//printf("roll_sp:%0.2f\n", att_sp.roll);
	time_past += dt;

	//printf("t:%0.2f,z_sp:%0.3f, z:%0.3f,vz:%0.3f,thr_sp:%0.3f\n", 
		//time_past, pos_sp.z, pos.z, vel.vz, throttle_ref);



}



float 
	HotDrone_PosController::saturate(float x, float uplimit, float lowlimit)
{
	if(x > uplimit) x=uplimit;
	else if(x < lowlimit) x=lowlimit;
	return x;
}