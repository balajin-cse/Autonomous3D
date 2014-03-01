//Hovercraft Model, called every 0.1 s
#include <math.h>
#include <stdio.h>
#include "HotDec_Plant.h"
#include "Parameters.h"


//A,B,C matrices are hovercraft models. 


//Change values of rows and columns accordingly if A,B,C dimensions are changed
//unsigned int RowA=6;
//unsigned int ColumnA=6;
//unsigned int RowB=6;
//unsigned int ColumnB=3;
//unsigned int RowC=3;
//unsigned int ColumnC=6;



HotDec_Plant::HotDec_Plant() 
{
	_last_t = clock();
	_time_past = 0;
	zero_plant_state(_X);
	zero_plant_state(_X_i);
	zero_plant_state(_X_old);

	_hc_m = HOTDEC_HC_M;
	_hc_j = HOTDEC_HC_J;
	_hc_bt = HOTDEC_HC_BT;
	_hc_br = HOTDEC_HC_BR;

	float A[6][6]={{-_hc_bt/_hc_m,0,0,0,0,0},
				{1,0,0,0,0,0},
				{0,0,-_hc_bt/_hc_m,0,0,0},
				{0,0,1,0,0,0},
				{0,0,0,0,-_hc_br/_hc_j,0},
				{0,0,0,0,1,0}};

	float B[6][3]={{1/_hc_m,0,0},{0,0,0},{0,1/_hc_m,0},{0,0,0},{0,0,1/_hc_j},{0,0,0}};
	float C[3][6]={{0,1,0,0,0,0},{0,0,0,1,0,0},{0,0,0,0,0,1}};

}

HotDec_Plant::~HotDec_Plant()
{

}

/*
 * U: Fx, Fy, Torque
 * Y: x, y, theta
 */
void 
	HotDec_Plant::update(const float *u)
{	
	clock_t cur_t = clock();
	float dt = (float)(cur_t - _last_t)*TIME_SCALE;
	_time_past += dt;
	_last_t = cur_t;

	float x[6];
	float y_med[3]={0,0,0};
	float f_stop[3]={0,0,0}; //Friction if stopped
	float f_disk=0; //Friction if stick to disk


	if (fabs(_X_old.vx) <= 0.005) f_stop[0] = 0;
	else if (_X_old.vx>0.005) f_stop[0]= 0.9;
	else if (_X_old.vx<-0.005) f_stop[0]= -0.9;

	if (fabs(_X_old.vy) <= 0.005) f_stop[1] = 0;
	else if (_X_old.vy>0.005) f_stop[1]= 0.9;
	else if (_X_old.vy<-0.005) f_stop[1]= -0.9;

	if (fabs(_X_old.theta_rate) <= 0.005) f_stop[2] = 0;
	else if (_X_old.theta_rate>0.005) f_stop[2]= 0.9;
	else if (_X_old.theta_rate<-0.005) f_stop[2]= -0.9;



	//Both Algorithm applies simplest discrete transformation xdot = (x-_X_old)/dT
	//I used the first one for now because it is faster in calculation 
	//but if A,B,C matrices are changed using second one is easier

//-------------------------------- Algorithm 1  ---------------------------------------

	//This algorithm is the simplification of  xdot = A*x + B*u, y=C*x after plugging in values of A,B,C and computing
	//This is straight forward but requires preliminary calculation

	
	_X.vx =_X_old.vx + _X_old.vx * (-_hc_bt/_hc_m)*dt+(u[0]*(1/_hc_m)-f_stop[0]*(1/_hc_m))*dt;
	_X.x =_X_old.x + _X_old.vx * dt;
	_X.vy =_X_old.vy + _X_old.vy * (-_hc_bt/_hc_m)*dt+(u[1]*(1/_hc_m)-f_stop[1]*(1/_hc_m))*dt;
	_X.y =_X_old.y + _X_old.vy * dt;
	_X.theta_rate =_X_old.theta_rate + _X_old.theta_rate * (-_hc_br/_hc_j)*dt+(u[2]*(1/_hc_j)-f_stop[2]*(1/_hc_j))*dt;
	_X.theta =_X_old.theta + _X_old.theta_rate * dt;
	/*
	if(ThrusterStop){
		if (_X.vx*_X_old.vx < 0) _X.vx = 0;
		if (_X.vy*_X_old.vy < 0) _X.vy = 0;
		if (_X.theta_rate*_X_old.theta_rate < 0) _X.theta_rate = 0;
	}*/

	_X_old.vx = _X.vx;
	_X_old.x = _X.x;
	_X_old.vy = _X.vy;
	_X_old.y = _X.y;
	_X_old.theta_rate = _X.theta_rate;
	_X_old.theta = _X.theta;

	//printf("dt:%0.4f, x:%0.4f, y:%0.4f, time_past:%0.4f\n", dt, _X.x, _X.y, _time_past);


//-------------------------------- Algorithm 2 ------------------------------------------

	//This algorithm is to compute xdot=A*x+B*u, y=C*x directly 
	//It is easy in terms of writing but takes more time to compute
	//It gives more precision error than the first one after long time
	//I do this in case states are changed, you can simply change A,B,C matrix 

	/*for(i=0;i<RowA;i++){
		x[i]=x_old[i];
		for(j=0;j<ColumnA;j++){
			x[i]+=Ts_Gum*(A[i][j])*x_old[j];
			if(j<ColumnB) x[i]+=Ts_Gum*(B[i][j])*u[j];
		}
		x_old[i]=x[i];	
	}

	for(k=0;k<RowC;k++){
		for(i=0;i<RowA;i++) y_med[k]+=(C[k][i])*x[i];
		y[k]=y_med[k];
	}*/
//---------------------------------------------------------------------------------


}


void 
	HotDec_Plant::reset_plant_state(hotdec_plant_state_t &X)
{
	X.vx = 0;
	X.x = 0;
	X.vy = 0;
	X.y = 0;
	X.theta_rate = 0;
	X.theta = 0;

}
void 
	HotDec_Plant::zero_plant_state(hotdec_plant_state_t &X) 
{
	X.vx = 0;
	X.x = 0;
	X.vy = 0;
	X.y = 0;
	X.theta_rate = 0;
	X.theta = 0;
}



