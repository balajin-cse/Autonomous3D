#include <stdio.h>

#include "HotDrone_Motor.h"

HotDrone_Motor::HotDrone_Motor() {
	_Prop_gain = HOTDRONE_MOTOR_PROPGAIN;

	_L = HOTDRONE_MOTOR_L;
	_D = HOTDRONE_MOTOR_D;

	_thrust_min = HOTDRONE_MOTOR_THRUST_MIN;
	_thrust_max = HOTDRONE_MOTOR_THURST_MAX;
	_scaling = HOTDRONE_MOTOR_SCALING;

	_startpoint_full_control = HOTDRONE_MOTOR_FULL_CONTROL_START_POINT;

	reset();
}

/*
 *
 */
void
	HotDrone_Motor::reset() 
{
	for (int i=0; i<4; i++) {
		_motor_thrust[i] = 0;
		_motor_rpm[i] = 0;
		_motor_pwm[i] = 0;
		_old_angle[i] = 0;
		_delta_angle[i] = 0;
	}
}


void 
	HotDrone_Motor::update(const hotdrone_actuator_sp_t actuators)
{
	
	float roll_control = actuators.control[0];
	float pitch_control = actuators.control[1];
	float yaw_control = actuators.control[2];
	float thrust_control = actuators.control[3]; // normalized
	float motor_calc[4];



	
	float min_gas = _thrust_min * _scaling;	// value range sent to motors, minimum */
	float max_gas = _thrust_max * _scaling;	// value range sent to motors, maximum */


	float output_band = 0.0f;


	/* linearly scale the control inputs from 0 to startpoint_full_control */
	if (thrust_control < _startpoint_full_control) {
		output_band = thrust_control/_startpoint_full_control; // linear from 0 to 1
	} else {
		output_band = 1.0f;
	}

	roll_control *= output_band;
	pitch_control *= output_band;
	yaw_control *= output_band;

	//thrust_control *= 10/4; //hack


	//add the yaw, nick and roll components to the basic thrust //TODO:this should be done by the mixer

	// FRONT (MOTOR 1)
	motor_calc[0] = thrust_control + (roll_control / 2 + pitch_control / 2 - yaw_control);
	// RIGHT (MOTOR 2)
	motor_calc[1] = thrust_control + (-roll_control / 2 + pitch_control / 2 + yaw_control);
	// BACK (MOTOR 3)
	motor_calc[2] = thrust_control + (-roll_control / 2 - pitch_control / 2 - yaw_control);
	// LEFT (MOTOR 4)
	motor_calc[3] = thrust_control + (roll_control / 2 - pitch_control / 2 + yaw_control);



	/* if one motor is saturated, reduce throttle */
	//float saturation = fmaxf_self_implement(fmaxf_self_implement(motor_calc[0], motor_calc[1]),
	//								fmaxf_self_implement(motor_calc[2], motor_calc[3])) - _thrust_max;
	float saturation = -1; //hack


	if (saturation > 0.0f) {

		/* reduce the motor thrust according to the saturation */
		thrust_control -= saturation;

		// FRONT (MOTOR 1)
		motor_calc[0] = thrust_control + (roll_control / 2 + pitch_control / 2 - yaw_control);
		// RIGHT (MOTOR 2)
		motor_calc[1] = thrust_control + (-roll_control / 2 + pitch_control / 2 + yaw_control);
		// BACK (MOTOR 3)
		motor_calc[2] = thrust_control + (-roll_control / 2 - pitch_control / 2 - yaw_control);
		// LEFT (MOTOR 4)
		motor_calc[3] = thrust_control + (roll_control / 2 - pitch_control / 2 + yaw_control);
	}

	for (int i = 0; i<4; i++) {
		_motor_thrust[i] = motor_calc[i] * 2.5;
	}




	/* set the motor values */

	/* scale up from 0..1 to 10..510) */
	_motor_pwm[0] = (uint16_t) (motor_calc[0] * (float)((max_gas - min_gas) + min_gas));
	_motor_pwm[1] = (uint16_t) (motor_calc[1] * (float)((max_gas - min_gas) + min_gas));
	_motor_pwm[2] = (uint16_t) (motor_calc[2] * (float)((max_gas - min_gas) + min_gas));
	_motor_pwm[3] = (uint16_t) (motor_calc[3] * (float)((max_gas - min_gas) + min_gas));

	/* Failsafe logic for min values - should never be necessary */
	_motor_pwm[0] = (_motor_pwm[0] > 0) ? _motor_pwm[0] : min_gas;
	_motor_pwm[1] = (_motor_pwm[1] > 0) ? _motor_pwm[1] : min_gas;
	_motor_pwm[2] = (_motor_pwm[2] > 0) ? _motor_pwm[2] : min_gas;
	_motor_pwm[3] = (_motor_pwm[3] > 0) ? _motor_pwm[3] : min_gas;

	/* Failsafe logic for max values - should never be necessary */
	_motor_pwm[0] = (_motor_pwm[0] <= max_gas) ? _motor_pwm[0] : max_gas;
	_motor_pwm[1] = (_motor_pwm[1] <= max_gas) ? _motor_pwm[1] : max_gas;
	_motor_pwm[2] = (_motor_pwm[2] <= max_gas) ? _motor_pwm[2] : max_gas;
	_motor_pwm[3] = (_motor_pwm[3] <= max_gas) ? _motor_pwm[3] : max_gas;

	/*
	TODO: Need Motor pwm to motor thrust relationship
	for (int i=0; i< 4; i++) {
		if (_motor_pwm[i]>100) {
			_motor_thrust[i] = (_motor_pwm[i]-100)/50;
		} else {
			_motor_thrust[i] = 0;
		}

	}*/


	//printf("motor_tr1:%0.3f, 2:%0.3f, 3:%0.3f, 4:%0.3f, saturation:%0.3f\n", 
	//	_motor_thrust[0], _motor_thrust[1],_motor_thrust[2],_motor_thrust[3], saturation); 

	_motor_rpm[0] = _thrust_to_rpm(_motor_thrust[0]);
	_motor_rpm[1] = _thrust_to_rpm(_motor_thrust[1]);
	_motor_rpm[2] = _thrust_to_rpm(_motor_thrust[2]);
	_motor_rpm[3] = _thrust_to_rpm(_motor_thrust[3]);


	//printf("roll_cntrl:%0.3f, pitch_cntrl:%0.3f, yaw_cntrl:%0.3f, thrust:%0.3f, sat:%0.3f\n", 
		//	roll_control, pitch_control, yaw_control, thrust_control, saturation);
	//printf("thrust_sp:%0.3f, motor1:%0.3f, motor2:%0.3f, motor3:%0.3f, motor1:%0.4f\n", 
		//	motor_thrust, _motor_thrust[0], _motor_thrust[1], _motor_thrust[2], _motor_thrust[3]);

}

float HotDrone_Motor::fmaxf_self_implement(float x, float y) {
	if (x > y) return x;
	else return y;
}


float
	HotDrone_Motor::_get_total_thrust()
{
	float MotorThr[4];
	float MotorRadPS[4];
	for(int i=0; i<4; i++){
		MotorThr[i] = _rpm_to_thrust(_motor_rpm[i]);
		MotorRadPS[i] = 2*PI/60*_motor_rpm[i];
	}

	return -(MotorThr[0]+MotorThr[1]+MotorThr[2]+MotorThr[3]);
}

/*
 *
 */
float* 
	HotDrone_Motor::get_global_force() 
{
	//MotorRPM: motor RPM 1 to 4, input of the function
	//Quad_Config: Config of x or +, input of the function
	//U: Main force/torque vector  U[0] throtle, U[1] roll, U[2] pitch, U[3] yaw
	
	float U[4];
	float MotorRadPS[4];
	for(int i=0;i<4;i++){
		MotorRadPS[i] = 2*PI/60*_motor_rpm[i];
	}

	U[0] = -(_motor_thrust[0]+_motor_thrust[1]+_motor_thrust[2]+_motor_thrust[3]);



	//if(false){
	//	U[1] = l*(MotorThr[3]-MotorThr[1]);
	//	U[2] = l*(MotorThr[0]-MotorThr[2]);
	//}
	//else{
	U[1] =1/sqrt_2*_L*(_motor_thrust[0]-_motor_thrust[1]-_motor_thrust[2]+_motor_thrust[3]);
	U[2] =1/sqrt_2*_L*(_motor_thrust[0]+_motor_thrust[1]-_motor_thrust[2]-_motor_thrust[3]);
	//}

	U[3] = _D*(pow(MotorRadPS[1],2)+pow(MotorRadPS[3],2)-pow(MotorRadPS[0],2)-pow(MotorRadPS[2],2)); 

	//printf("motor says-> U:0:%0.2f\n", U[0]);

	//printf("motor_thrust0:%0.3f, motor_thrust1:%0.3f, motor_thrust2:%0.3f, motor_thrust3:%0.3f\n", 
		//	motor_thrust[0], motor_thrust[1], motor_thrust[2], motor_thrust[3]);

	

	return U;

}

float 
	HotDrone_Motor::_rpm_to_thrust(const float rpm)
{
	//printf("prog_gain:%0.2f, rpm:%0.2f, prop_exp:%0.2f, GRAVITY:%0.2f\n", 
		//	prop_gain, rpm, prop_exp, GRAVITY);
	return(_Prop_gain*powf(rpm,2)*GRAVITY/1000);
}


float 
	HotDrone_Motor::_thrust_to_rpm(const float thrust)
{
	return sqrt(thrust*1000/GRAVITY/_Prop_gain);
}


float*
	HotDrone_Motor::get_old_angle()
{
	return _old_angle;
}


float*
	HotDrone_Motor::get_delta_angle()
{
	_delta_angle[0] = _motor_rpm[0]*0.001*60;
	_delta_angle[1] = _motor_rpm[1]*0.001*60;
	_delta_angle[2] = _motor_rpm[2]*0.001*60;
	_delta_angle[3] = _motor_rpm[3]*0.001*60;

	for (int i=0;i<4;i++){
		_old_angle[i] += _delta_angle[i];
	}
	return _delta_angle;
}

