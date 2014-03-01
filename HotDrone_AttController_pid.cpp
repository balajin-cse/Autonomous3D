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

#include <stdint.h>
#include <math.h>
#include <float.h>
#include <stdio.h>

#include "HotDrone_AttController_pid.h"
#include "Parameters.h"

HotDrone_AttController_pid::HotDrone_AttController_pid()
{
	_yaw_p = HOTDRONE_ATTCONTROLLER_PID_YAW_P;
	_yaw_i = HOTDRONE_ATTCONTROLLER_PID_YAW_I;
	_yaw_d = HOTDRONE_ATTCONTROLLER_PID_YAW_D;

	_att_p = HOTDRONE_ATTCONTROLLER_PID_ATT_P;
	_att_i = HOTDRONE_ATTCONTROLLER_PID_ATT_I;
	_att_d = HOTDRONE_ATTCONTROLLER_PID_ATT_D;

	_last_run = clock();
	_last_input = 0;

	_motor_skip_counter = 0;
	_initialized = false;

	_yaw_error = 0;
}

HotDrone_AttController_pid::~HotDrone_AttController_pid()
{


}


void HotDrone_AttController_pid::update(const hotdrone_att_sp_t att_sp,
				 const hotdrone_att_t att, hotdrone_att_rate_sp_t &att_rates_sp, bool control_yaw_position, bool reset_integral)
{
	
	
	//printf("roll:%0.3f, roll_r:%0.3f, pitch:%0.3f, pitch_rate:%0.3f\n", 
		//	att.roll, att.roll_rate, att.pitch, att.pitch_rate);

	float deltaT = (float)(clock() - _last_run) *TIME_SCALE*TIME_SCALE;
	_last_run = clock();

	if (_last_input != att_sp.timestamp) {
		_last_input = att_sp.timestamp;
	}


	// initialize the pid controllers when the function is called for the first time 
	if (_initialized == false) {

		pid_init(&_pitch_controller, _att_p, _att_i, _att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);
		pid_init(&_roll_controller, _att_p, _att_i, _att_d, 1000.0f, 1000.0f, PID_MODE_DERIVATIV_SET, 0.0f);

		_initialized = true;
	}

	//load new parameters with lower rate
	if (_motor_skip_counter % 500 == 0) {
		// update parameters from storage 
		//parameters_update(&h, &p);

		// apply parameters 
		pid_set_parameters(&_pitch_controller, _att_p, _att_i, _att_d, 1000.0f, 1000.0f);
		pid_set_parameters(&_roll_controller, _att_p, _att_i, _att_d, 1000.0f, 1000.0f);
	}

	// reset integrals if needed 
	if (reset_integral) {
		pid_reset_integral(&_pitch_controller);
		pid_reset_integral(&_roll_controller);
		//TODO pid_reset_integral(&yaw_controller);
	}

	// calculate current control outputs 

	// control pitch (forward) output 
	att_rates_sp.pitch_rate = pid_calculate(&_pitch_controller, att_sp.pitch,
						att.pitch, att.pitch_rate, deltaT);
	//att_rates_sp.pitch_rate = 0;

	// control roll (left/right) output 
	att_rates_sp.roll_rate = pid_calculate(&_roll_controller, att_sp.roll ,
				       att.roll, att.roll_rate, deltaT);

	if (control_yaw_position) {
		// control yaw rate 
		// TODO use pid lib

		// positive error: rotate to right, negative error, rotate to left (NED frame)
		// yaw_error = _wrap_pi(att_sp->yaw_body - att->yaw);

		_yaw_error = att_sp.yaw - att.yaw;

		if (_yaw_error > PI) { //WARNING: subject to bug
			_yaw_error -= PI;

		} else if (_yaw_error < -PI) {
			_yaw_error += PI;
		}

		att_rates_sp.yaw_rate = _yaw_p * (_yaw_error) - (_yaw_d * att.yaw_rate);
	} else {
		att_rates_sp.yaw_rate = 0;
	}

	att_rates_sp.thrust = att_sp.throttle;
	//att_rates_sp.thrust = 4.38256; //hack
    //need to update the timestamp now that we've touched rates_sp
    att_rates_sp.timestamp = clock();

	//printf("roll_sp:%0.4f, roll:%0.4f, roll_rate:%0.4f,roll_rate_sp:%0.4f\n", att_sp.roll, 
		//att.roll, att.roll_rate, att_rates_sp.roll_rate);

	_motor_skip_counter++;
}