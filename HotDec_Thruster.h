#ifndef _HOTDEC_THRUSTER_H_
#define _HOTDEC_THRUSTER_H_

#include <time.h>
#include "Parameters.h"
#include "HotDec_Structs.h"

//Inputs f[1-4]: local forces f1,f2,f3,f4 of each thruster 
//Inputs Theta:  Yaw angle theta of Hovercraft
//Outputs F:[1-3]	 global force Fx,Fy,Torque

#define HOTDEC_THRUSTER_FW_GAIN 2560000 //Force to angular speed gain
#define HOTDEC_THRUSTER_RR_GAIN 9.5493 // Rad/s to RPM
#define HOTDEC_THRUSTER_KP 10
#define HOTDEC_THRUSTER_KI 0.001
#define HOTDEC_THRUSTER_KD 0.01

#define HOTDEC_THRUSTER_A1 -8.6
#define HOTDEC_THRUSTER_B1 1
#define HOTDEC_THRUSTER_C1 20868760.0
#define HOTDEC_THRUSTER_VsqrtoF 20868760.0

#define HOTDEC_THRUSTER_Ts 0.001
#define HOTDEC_THRUSTER_PWM_TIMER_PERIOD 14993

#define HOTDEC_THRUSTER_R 0.13 //hotdec center to thruster

class HotDec_Thruster {
public:
	HotDec_Thruster();
	~HotDec_Thruster();
	void update(const hotdec_thrust_sp_t thrust_sp, const float theta);
	float* get_global_force(float theta);
private:
	void control_and_plant(float *f_ref,float* f_out);

	float _thrusts[4];
	int _noise_counter;

	clock_t _last_t;
	float _time_past;

	float _Fw_gain;
	float _RR_gain;
	float _Kp;
	float _Ki;
	float _Kd;
	float _A1;
	float _B1;
	float _C1;
	float _VsqrtoF;
	float _PWM_timer_period;
	float _R;


	float _f_act[4];
	float _error[4];
	float _deriv[4];
	float _integral[4];
	float _error_old[4];
	float _cmp[4]; //???????? 
	float _x_old[4];
};

#endif
