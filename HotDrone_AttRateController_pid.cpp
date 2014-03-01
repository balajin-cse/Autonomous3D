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

#include "HotDrone_AttRateController_pid.h"
#include "Parameters.h"



HotDrone_AttRateController_pid::HotDrone_AttRateController_pid()
{

	_yawrate_p = HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_P;
	_yawrate_i = HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_I;
	_yawrate_d = HOTDRONE_ATTRATECONTROLLER_PID_YAWRATE_D;

	_attrate_p = HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_P;
	_attrate_i = HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_I;
	_attrate_d = HOTDRONE_ATTRATECONTROLLER_PID_ATTRATE_D;

	_last_run = clock();
	_last_input = 0;

	_motor_skip_counter = 0;
	_initialized = false;

	_yaw_error = 0;

}

HotDrone_AttRateController_pid::~HotDrone_AttRateController_pid()
{



}


void HotDrone_AttRateController_pid::update(const hotdrone_att_rate_sp_t att_rate_sp,
						   const hotdrone_att_t att, hotdrone_actuator_sp_t &actuators_sp, bool reset_integral)
{
	

	const float deltaT = (float)(clock() - _last_run)*TIME_SCALE*TIME_SCALE;


	if (_last_input != att_rate_sp.timestamp) {
		_last_input = att_rate_sp.timestamp;
	}

	//last_run = hrt_absolute_time();


	// initialize the pid controllers when the function is called for the first time 
	if (_initialized == false) {
		//parameters_init(&h);
		//parameters_update(&h, &p);
		//initialized = true;

		pid_init(&_pitch_rate_controller, _attrate_p, _attrate_i, _attrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);
		pid_init(&_roll_rate_controller, _attrate_p, _attrate_i, _attrate_d, 1.0f, 1.0f, PID_MODE_DERIVATIV_CALC_NO_SP, 0.003f);

	}

	// load new parameters with lower rate 
	if (_motor_skip_counter % 2500 == 0) {
		// update parameters from storage 
		//parameters_update(&h, &p);
		pid_set_parameters(&_pitch_rate_controller, _attrate_p, _attrate_i, _attrate_d, 1.0f, 1.0f);
		pid_set_parameters(&_roll_rate_controller,  _attrate_p, _attrate_i, _attrate_d, 1.0f, 1.0f);
	}

	// reset integrals if needed 
	if (reset_integral) {
		pid_reset_integral(&_pitch_rate_controller);
		pid_reset_integral(&_roll_rate_controller);
		// TODO pid_reset_integral(&yaw_rate_controller);
	}

	// control pitch (forward) output 
	float pitch_control = pid_calculate(&_pitch_rate_controller, att_rate_sp.pitch_rate,
					    att.pitch_rate, 0.0f, deltaT);
	//float pitch_control = 0;

	// control roll (left/right) output 
	float roll_control = pid_calculate(&_roll_rate_controller, att_rate_sp.roll_rate,
					   att.roll_rate, 0.0f, deltaT);

	// control yaw rate / //XXX use library here
	//printf("yaw_p:%0.4f, yaw_rate_sp:%0.3f, yaw_rate:%0.3f\n", 
		//_yawrate_p, att_rate_sp.yaw_rate, att.yaw_rate);
	//float yaw_rate_control = _yawrate_p * (att_rate_sp.yaw_rate - att.yaw_rate);
	float yaw_rate_control = 0;
	// increase resilience to faulty control inputs 
	if (!_finite(yaw_rate_control)) {
		yaw_rate_control = 0.0f;
		//warnx("rej. NaN ctrl yaw");
	}

	//printf("deltaT:%0.3f, roll_cntrl:%0.3f, thrust:%0.3f\n", deltaT, roll_control, att_rate_sp.control);
	


	//printf("roll_rate_sp:%0.3f,roll:%0.3f,roll_rate:%0.3f,roll_contrl:%0.3f,throttle:%0.2f\n", 
		//	att_rate_sp.roll_rate, att.roll, att.roll_rate, roll_control, att_rate_sp.control);

	//printf("cntrl_sig:%0.4f,P:%0.3f,I:%0.3f,throttle:%0.2f\n", 
		//		cntrl_sig[0], att_err[0]*Kp_Ang[0], att_int[0]*Ki_Ang[0], cntrl_sig[3]);

	//printf("cntrl_sig0:%0.4f, sig1:%0.4f, sig2:%0.4f, sig3:%0.4f\n", 
		//	cntrl_sig[0], cntrl_sig[1], cntrl_sig[2], cntrl_sig[3]);

	//printf("motor1:%0.2f, motor2:%0.2f, motor3:%0.2f, motor4:%0.2f\n", 
		//	att_cntrl_s.motor_cntrl_sp[0],att_cntrl_s.motor_cntrl_sp[1],att_cntrl_s.motor_cntrl_sp[2],att_cntrl_s.motor_cntrl_sp[3]);

	actuators_sp.control[0] = roll_control;
	actuators_sp.control[1] = pitch_control;
	actuators_sp.control[2] = yaw_rate_control;
	actuators_sp.control[3] = att_rate_sp.thrust;

	//actuators_sp.control[0] = saturate(actuators_sp.control[0], MaxTHRUST_SINGLE_MOTOR, 0);
	//actuators_sp.control[1] = saturate(actuators_sp.control[1], MaxTHRUST_SINGLE_MOTOR, 0);
	//actuators_sp.control[2] = saturate(actuators_sp.control[2], MaxTHRUST_SINGLE_MOTOR, 0);
	//actuators_sp.control[3] = saturate(actuators_sp.control[3], MaxTHRUST_SINGLE_MOTOR, 0);

	//printf("thrust1:%0.3f, thrust2:%0.3f, thrust3:%0.3f, thrust4:%0.3f\n", 
		//	actuators_sp.control[0], actuators_sp.control[1], actuators_sp.control[2], actuators_sp.control[3]);

	_motor_skip_counter++;
}

float HotDrone_AttRateController_pid::wrap_pi(float angle){
	if(angle > PI){
		angle = angle - 2*PI;
	}
	if(angle < -PI){
		angle = angle + 2*PI;
	}
	return angle;
}

float HotDrone_AttRateController_pid::saturate(float x, float uplimit, float lowlimit){
	if(x>uplimit) x=uplimit;
	if(x<lowlimit) x=lowlimit;
	return x;
}