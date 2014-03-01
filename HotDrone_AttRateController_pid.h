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

#ifndef _HOTDRONE_ATTRATECONTROLLER_PID_H_
#define _HOTDRONE_ATTRATECONTROLLER_PID_H_

#include <time.h>

#include "pid.h"
#include "HotDrone_Motor.h"
#include "Hotdrone_Structs.h"


#define HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_P 0.3f
#define HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_I 0.005f
#define HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_D 0.2f

#define HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_P 0.09f
#define HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_I 0.002f
#define HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_D 0.0f

class HotDrone_AttRateController_pid {
public:
	HotDrone_AttRateController_pid();
	~HotDrone_AttRateController_pid();

	void update(const hotdrone_att_rate_sp_t att_rate_sp,
						   const hotdrone_att_t att, hotdrone_actuator_sp_t &actuators_sp, bool reset_integral);

private:
	float wrap_pi(float angle);
	float saturate(float x, float uplimit, float lowlimit);
	float _yawrate_p, _yawrate_i, _yawrate_d;
	float _attrate_p, _attrate_i, _attrate_d;

	clock_t _last_run;
	uint64_t _last_input;

	int _motor_skip_counter;

	PID_t _pitch_rate_controller;
	PID_t _roll_rate_controller;


	bool _initialized;
	float _yaw_error;

};


#endif
