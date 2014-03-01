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

#ifndef _HOTDRONE_POSCONTROLLER_H_
#define _HOTDRONE_POSCONTROLLER_H_


#include <vector>
#include <math.h>

#include "Hotdrone_Structs.h"
#include "Vehicle_Structs.h"
#include "Parameters.h"
#include "HotDrone_Motor.h"

using namespace std;

#define HotDrone_PosController_VEL_P 0.1f
#define HotDrone_PosController_VEL_I 0.000f
#define HotDrone_PosController_VEL_D 0.000f
#define HOTDRONE_POSCONTROLLER_LIMIT_VEL_ERR 10.0f
#define HOTDRONE_POSCONTROLLER_LIMIT_VEL_INT 10.0f







#define HOTDRONE_POSCONTROLLER_POS_P 0.15f
#define HOTDRONE_POSCONTROLLER_POS_I 0.000f
#define HOTDRONE_POSCONTROLLER_POS_D 0.3f
#define HOTDRONE_POSCONTROLLER_LIMIT_POS_INT 10.0f
#define HOTDRONE_POSCONTROLLER_LIMIT_POS_ERR 10.0f

#define HOTDRONE_POSCONTROLLER_HEIGHT_P 0.15f
#define HOTDRONE_POSCONTROLLER_HEIGHT_I 0.0001f
#define HOTDRONE_POSCONTROLLER_HEIGHT_D 0.30f
#define HOTDRONE_POSCONTROLLER_LIMIT_HEIGHT_ERR 5.0f
#define HOTDRONE_POSCONTROLLER_LIMIT_HEIGHT_INT 10.0f



#define HOTDRONE_POSCONTROLLER_LIMIT_THRUSTER_UPPER 0.8f //L_TH_U
#define HOTDRONE_POSCONTROLLER_LIMIT_THRUSTER_LOWER 0.4f //L_TH_L
#define HOTDRONE_POSCONTROLLER_THRUST_FEEDFORWARD 0.4f //0.69
#define HOTDRONE_POSCONTROLLER_LIMIT_ROLL 0.4f //rad
#define HOTDRONE_POSCONTROLLER_LIMIT_PITCH 0.4f //rad
#define HotDrone_PosController_ANGLE_REF_LIMIT 0.2356 // rad, 13.5 in degree


class HotDrone_PosController{
public:
	HotDrone_PosController();
	HotDrone_PosController(float* Kp,float* Ki,float* Kd);
	void update(const vehicle_local_pos_sp_t pos_sp, const vehicle_local_pos_t pos, 
											const vehicle_local_vel_t vel, const hotdrone_att_t att, hotdrone_att_sp_t &hotdrone_att_sp);

private:
	float saturate(float x, float uplimit, float lowlimit);

	//bool _standby_phase, _landing_phase, _operation_phase;


	float Kp_vxy, Ki_vxy, Kd_vxy;

	float _limit_roll, _limit_pitch;

	float _pos_p, _pos_i, _pos_d;
	float _height_p, _height_i, _height_d;
	float _limit_thrust_upper, _limit_thrust_lower;
	float _limit_height_err;
	

	float _limit_pos_err;

	clock_t last_t;
	float time_past;

	float _x_integral;
	float _y_integral;
	float _z_integral;

	float _vx_integral;
	float _vy_integral;
	float _vz_integral;

	//awkward variables
	float _last_pos_z;
	float _integrated_h_err;
	float _limit_thrust_int;
	float _thrust_feedforward;

};


# endif 
