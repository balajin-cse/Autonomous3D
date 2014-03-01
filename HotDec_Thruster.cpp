#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "HotDec_Thruster.h"



HotDec_Thruster::HotDec_Thruster()
{
	_last_t = clock();

	_Fw_gain = HOTDEC_THRUSTER_FW_GAIN;
	_RR_gain = HOTDEC_THRUSTER_RR_GAIN;
	_Kp = HOTDEC_THRUSTER_KP;
	_Ki = HOTDEC_THRUSTER_KI;
	_Kd = HOTDEC_THRUSTER_KD;
	_A1 = HOTDEC_THRUSTER_A1;
	_B1 = HOTDEC_THRUSTER_B1;
	_C1 = HOTDEC_THRUSTER_C1;
	_VsqrtoF = HOTDEC_THRUSTER_VsqrtoF;
	_PWM_timer_period = HOTDEC_THRUSTER_PWM_TIMER_PERIOD;
	_R = HOTDEC_THRUSTER_R;

	for (int i=0; i<4; i++) {
		_thrusts[i] = 0;
		_f_act[i] = 0;
		_error[i] = 0;
		_deriv[i] = 0;
		_integral[i] = 0;
		_error_old[i] = 0;
		_cmp[i] = 0; //???????? 
		_x_old[i] = 0;
	
	}
	
	_noise_counter = 0;

}

HotDec_Thruster::~HotDec_Thruster()
{

}



/*
 * Input: f: thruster reference
 *        theta: angle
 * Output: Global forces: Fx, Fy, Torque
 */
void 
	HotDec_Thruster::update(const hotdec_thrust_sp_t thrust_sp, const float theta)
{
	clock_t cur_t = clock();
	float dt = (float)(cur_t - _last_t)*TIME_SCALE;
	_time_past += dt;
	_last_t = cur_t;
	
	float ref[4];
    float ang_vel[4];
	float error[4];
	float deriv[4];
	float cmp_del[4]={0,0,0,0};

	float u[4];
	float x[4];


	for(int i=0;i<4;i++){
		
		ref[i]=_RR_gain*(sqrt(_Fw_gain*thrust_sp.U[i]));

		if(ref[i]> (float)15000) ref[i]=15000;
		if(ref[i]< (float) -15000) ref[i]-=-15000;

		if(fabs(ref[i])<261) { //Dead Zone of Reference
			ref[i]=0;
			ang_vel[i]=0;
			_integral[i]=0;
			_error_old[i]=0;
			_cmp[i]=0;
		}

		ang_vel[i]=_RR_gain*(sqrt(_f_act[i])); //Feedback from actual force

		if(ang_vel[i]<= 3000) {ang_vel[i]=0;} //Dead Zone of actual
		else if(ang_vel[i]>=16000) {ang_vel[i]=16000;}//Dead Zone of actual

		error[i]=ref[i]-ang_vel[i];
		_integral[i]+= dt*(error[i]+_error_old[i])/2;
		deriv[i]= (error[i]-_error_old[i])*(1/dt);

		_error_old[i]=error[i];

		cmp_del[i]=_Kp*(error[i]+_Ki*_integral[i]+_Kd*deriv[i]);
		_cmp[i]+=cmp_del[i];

		if(_cmp[i]>_PWM_timer_period) {_cmp[i]=_PWM_timer_period;}
		else if(_cmp[i]<0) {_cmp[i]=0;}

		u[i]= 2*(_cmp[i]/_PWM_timer_period);	

		x[i]=_x_old[i]+_A1*dt*_x_old[i]+_B1*dt*u[i]; //Discrete case of xdot=Ax+Bu
		_x_old[i]=x[i];
		_f_act[i]=_C1*x[i];// This is the direct output from thursters, so feedback

		_thrusts[i]=x[i]*8.1581 -0.02;

		if(_thrusts[i]>10) _thrusts[i]=10;
		if(_thrusts[i]<0) _thrusts[i]=0;

	}
	return;

}



float*
	HotDec_Thruster::get_global_force(float theta)
{	
	float f_med[2]={0,0};
	float ret[3];

	for(int i=0;i<4;i++){
		if(_thrusts[i]>2.25) _thrusts[i]=2.25;
		if(0==_noise_counter%10){
			_thrusts[i]=_thrusts[i]+0.029+0.0001*(rand()%20); // Noise 0.03 plus minus 0.001
		}
		if(0==_noise_counter%100){
			_thrusts[i]=_thrusts[i]+ 0.002*(rand()%10);
		}
	}

	_noise_counter++;

	f_med[0]=(_thrusts[0]+_thrusts[2]-_thrusts[1]-_thrusts[3])*cos(PI/4);
	f_med[1]=(_thrusts[0]+_thrusts[1]-_thrusts[2]-_thrusts[3])*cos(PI/4);

	ret[0]= f_med[0]*cos(theta) -f_med[1]*sin(theta);
	ret[1]= f_med[0]*sin(theta) +f_med[1]*cos(theta);
	ret[2]=(_thrusts[0]+_thrusts[3]-_thrusts[1]-_thrusts[2])*_R;

	return ret;
}



