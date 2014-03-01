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

#ifndef _HOTDRONE_MOTOR_H_
#define _HOTDRONE_MOTOR_H_


#include <math.h>
#include "Parameters.h"
#include "Hotdrone_Structs.h"

#define HOTDRONE_MOTOR_PROPGAIN 1.2e-7
#define HOTDRONE_MOTOR_MAXTHRUST 20 // unit:N STUB
#define HOTDRONE_MOTOR_MAXTHRUST_SINGLE_MOTOR 5 // unit:N

#define HOTDRONE_MOTOR_FULL_CONTROL_START_POINT 0.25f
#define HOTDRONE_MOTOR_THRUST_MIN  0.02f			// 2% minimum thrust, original 0.2
#define HOTDRONE_MOTOR_THURST_MAX 1.0f			// 100% max thrust, original 1.0
#define HOTDRONE_MOTOR_SCALING 510.0f			// 100% thrust equals a value of 510 which works, 512 leads to cutoff */

#define HOTDRONE_MOTOR_L 0.1778  //
#define HOTDRONE_MOTOR_D 1e-7 //


class HotDrone_Motor {
public:
	HotDrone_Motor();
	void update(const hotdrone_actuator_sp_t actuators);

	void reset();
	float* get_motor_rpm() {return _motor_rpm;}
	float* get_old_angle();
	float* get_delta_angle();
	float* get_global_force();

private:
	float fmaxf_self_implement(float x, float y);

	float _rpm_to_thrust(const float rpm);
	float _thrust_to_rpm(const float thrust);

	float _get_total_thrust();
	
	/*HotDrone Motor Parameters*/
	float _Prop_gain;
	float _Max_thrust;

	float _thrust_min;
	float _thrust_max;
	float _scaling;
	float _startpoint_full_control;

	float _L;
	float _D;

	float _motor_thrust[4];
	float _motor_rpm[4];
	float _motor_pwm[4];

	float _old_angle[4];
	float _delta_angle[4];



};

#endif