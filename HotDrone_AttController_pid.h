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

#ifndef _HOTDRONE_ATTCONTROLLER_PID_H_
#define _HOTDRONE_ATTCONTROLLER_PID_H_
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


#include "Hotdrone_Structs.h"
#include "pid.h"
#include <time.h>

#define HOTDRONE_ATTCONTROLLER_PID_YAW_P 2.0f
#define HOTDRONE_ATTCONTROLLER_PID_YAW_I 0.15f
#define HOTDRONE_ATTCONTROLLER_PID_YAW_D 0.0f

#define HOTDRONE_ATTCONTROLLER_PID_ATT_P 6.8f //6.8f
#define HOTDRONE_ATTCONTROLLER_PID_ATT_I 0.0f
#define HOTDRONE_ATTCONTROLLER_PID_ATT_D 0.0f



class HotDrone_AttController_pid {
public:
	HotDrone_AttController_pid();
	~HotDrone_AttController_pid();

	void update(const hotdrone_att_sp_t att_sp, const hotdrone_att_t att, 
							hotdrone_att_rate_sp_t &att_rates_sp, bool control_yaw_position, bool reset_integral);

private:

	float _yaw_p, _yaw_i, _yaw_d;
	float _att_p, _att_i, _att_d;

	clock_t _last_run;
	uint64_t _last_input;

	int _motor_skip_counter;

	PID_t _pitch_controller;
	PID_t _roll_controller;


	bool _initialized;
	float _yaw_error;

};


#endif

