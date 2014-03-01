#include <math.h>
#include <stdlib.h>
#include <stdio.h>
#include "Parameters.h"
#include "HotDec_Controller.h"

HotDec_Controller::HotDec_Controller()
{

}

HotDec_Controller::~HotDec_Controller()
{

}


// control inputs in global cordinate
// M=2.4KG VERSION , wo Minimum subtract , hc_bt = 0.08, hc_br = 0.01 , hc_J = 0.0608;//
/* velocity profile is not good
Fx =  7.5798  *(ref[0]-x[0])+   11.1803  *(ref[1]-x[1]);
Fy =  7.5798* (ref[2]-x[2])+    11.1803    *(ref[3]-x[3]);
Tau = 0.7990 *(ref[4]-x[4])+    0.4472 *(ref[5]-x[5]);
*/

// M=2.4KG VERSION , wo Minimum subtract , hc_bt = 0.08, hc_br = 0.01 , hc_J = 0.0608;//

/*Inputs ref[1-6]: reference of Xdot,X,Ydot,Y,Thetadot,Theta
 *Inputs x[1-6]:   sensor/kalman filter value of Xdot,X,Ydot,Y,Thetadot,Theta
 *Outputs thruster setpoint:  local forces for thruster  
 */
 void 
	 HotDec_Controller::update(const hotdec_state_sp_t state_sp, const hotdec_plant_state_t state, 
																	hotdec_thrust_sp_t &thrust_sp)
{

	//double r=0.2254;   //hotdec center to thruster
	float r=0.13;   //hotdec center to thruster
	float Fx=0;
	float Fy=0;
	float Tau=0;
	float fx=0;
	float fy=0;
	float tau=0;

	float f1=0;
	float f2=0;
	float f3=0;
	float f4=0;

	float f_min=0;


	Fx = 2.7*(state_sp.vx-state.vx)+ 1 *(state_sp.x-state.x); //2.7 ,1

	Fy = 2.7*(state_sp.vy-state.vy)+ 1 *(state_sp.y-state.y); //2.7,1

	Tau = 0.02225*(state_sp.theta_rate-state.theta_rate)+ 0.0005*(state_sp.theta-state.theta);


	// global force to body force
	fx=Fx*cos(state.theta)+ Fy*sin(state.theta);   //1.5708 = pi/2  : modified 0401
	fy=-Fx*sin(state.theta)+ Fy*cos(state.theta);
	tau=Tau;

	// assign body foce to each thruster force
	if(fx>=0){  // modified 0401
		f1=f1+(fx/sqrt_2);
		f3=f3+(fx/sqrt_2);
	}
	else if(fx<0){
		f2=f2+(fabs(fx)/sqrt_2);
		f4=f4+(fabs(fx)/sqrt_2); 
	}

	if(fy>=0){
		f1=f1+(fy/sqrt_2);
		f2=f2+(fy/sqrt_2);
	}

	else if(fy<0){
		f3=f3+(fabs(fy)/sqrt_2);
		f4=f4+(fabs(fy)/sqrt_2);
	}
   

	if(tau>=0){
		f1=f1+(tau/(2*r));
		f4=f4+(tau/(2*r));
	}
	else if(tau<0){
		f2=f2+fabs(tau)/(2*r);
		f3=f3+fabs(tau)/(2*r);
	}


	if (f1<=f2){
		f_min=f1;
	}
	else{
		f_min=f2;
	}

	if (f3<=f_min){
		f_min=f3;
	}
	else{
		f_min=f4;
	}
       
	thrust_sp.U[0] = f1;
	thrust_sp.U[1] = f2;
	thrust_sp.U[2] = f3;
	thrust_sp.U[3] = f4;
	
}
