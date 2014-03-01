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

#include "HotDrone_Plant.h"
#include <math.h>
#include <stdio.h>




HotDrone_Plant::HotDrone_Plant(unsigned int config) {
	this->last_t = clock();
	plant_config = config;
	zero_plant_state(plant_s);
	zero_plant_state(plant_s_dot);
	zero_plant_state(plant_s_old);

	Mass = HOTDRONE_MASS;
	Ixx=0.00204016;
	Iyy=0.00156771;
	Izz=0.00351779;
	J_M=2.0e-6;	
	J_P=1.5e-6;	
	J_MP=(J_M + J_P);
	J_TP=J_MP;
	J_gain=5e6;
}


hotdrone_plant_state_t 
	HotDrone_Plant::update(const float* thrust) 
{
	//printf("yahoo, I'm %d, z:%0.2f, thrust0:%0.3f\n", this->name, this->plant_s.z, thrust[0]);
	//printf("what becomes? tr[0]:%0.2f", thrust[0]);

	clock_t cur_t = clock();

	float dt = (float) (cur_t - last_t)*TIME_SCALE;




		//printf("pre: dt:%0.5f,z:%0.2f,vz:%0.2f,az:%0.2f\n",
		//dt, plant_s.z, plant_s.vz, plant_s_dot.vz);

	plant_s_dot.u = plant_s.v*plant_s.r-plant_s.w*plant_s.q-GRAVITY*sin(plant_s.theta);													//u_dot  = v*r - w*q -  g*sin(theta)
	plant_s_dot.v = plant_s.w*plant_s.p-plant_s.r*plant_s.u+GRAVITY*cos(plant_s.theta)*sin(plant_s.phi);								//v_dot  = w*p - r*u + g*cos(theta)*sin(phi)
	plant_s_dot.w =plant_s.u*plant_s.q-plant_s.v*plant_s.p+GRAVITY*cos(plant_s.theta)*cos(plant_s.phi)+thrust[0]/Mass;				//w_dot = u*q - v*p + g*cos(theta)*cos(phi) + thrust1/m

	/*
	X_dot[3] = (Iyy-Izz)/Ixx* plant_s.q*plant_s.r  -  plant_s.q *J_TP/Ixx*Omega + thrust[1]/Ixx;        //p_dot     = (Iyy-Izz)/Ixx*q*r - J_TP/Ixx*q*Omega + thrust2/Ixx;
	X_dot[4] = (Izz-Ixx)/Iyy *plant_s.p*plant_s.r + J_TP/Iyy* plant_s.p *Omega + thrust[2]/Iyy;        //q_dot     = (Izz-Ixx)/Iyy*p*r + J_TP/Iyy*p*Omega + thrust3/Iyy;
	X_dot[5] = (Ixx-Iyy)/Izz *plant_s.p*plant_s.q  + thrust[3]/Izz;											    //r_dot     = (Ixx-Iyy)/Izz*p*q                    + thrust4/Izz;
	*/
	plant_s_dot.p = thrust[1]/Ixx;
	plant_s_dot.q = thrust[2]/Iyy;
	plant_s_dot.r = thrust[3]/Izz;

	plant_s_dot.x = plant_s.vx;
	plant_s_dot.vx = (sin(plant_s.psi)*sin(plant_s.phi)+cos(plant_s.psi)*sin(plant_s.theta)*cos(plant_s.phi))*thrust[0]/Mass;   //	x_ddot = (sin(psi)*sin(phi)  + cos(psi)*sin(theta)*cos(phi))*thrust1/m;
	plant_s_dot.y = plant_s.vy;
	plant_s_dot.vy = (-cos(plant_s.psi)*sin(plant_s.phi)+sin(plant_s.psi)*sin(plant_s.theta)*cos(plant_s.phi))*thrust[0]/Mass;   //    	y_ddot = (-cos(psi)*sin(phi) + sin(psi)*sin(theta)*cos(phi))*thrust1/m;
	plant_s_dot.z = plant_s.vz;
	plant_s_dot.vz =  GRAVITY +cos(plant_s.theta)*cos(plant_s.phi)*thrust[0]/Mass;													  //	z_ddot = g + cos(theta)*cos(phi)*thrust1/m;
//	printf("Evil g:%0.2f, theta:%0.2f,phi:%0.2f,thrust:%0.2f, mass:%0.2f, az:%0.2f", 
	//		GRAVITY, plant_s.theta, plant_s.phi, thrust[0], Mass, plant_s_dot.vz);

	plant_s_dot.phi = plant_s.p+sin(plant_s.phi)*tan(plant_s.theta)*plant_s.q+cos(plant_s.phi)*tan(plant_s.theta)*plant_s.r;			//phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
	plant_s_dot.theta = cos(plant_s.phi)*plant_s.q-sin(plant_s.phi)*plant_s.r;														//theta_dot =     cos(phi)*q  - sin(phi)*r;
	plant_s_dot.psi = sin(plant_s.phi)/cos(plant_s.theta)*plant_s.q+cos(plant_s.phi)/cos(plant_s.theta)*plant_s.r;					//psi_dot   =     sin(phi)/cos(theta)*q + cos(phi)/cos(theta)*r;

	
	// Integration
	plant_s_old.u = plant_s.u = plant_s_old.u + plant_s_dot.u*dt;
	plant_s_old.v = plant_s.v = plant_s_old.v + plant_s_dot.v*dt;
	plant_s_old.w = plant_s.w = plant_s_old.w + plant_s_dot.w*dt;
	plant_s_old.p = plant_s.p = plant_s_old.p + plant_s_dot.p*dt;
	plant_s_old.q = plant_s.q = plant_s_old.q + plant_s_dot.q*dt;
	plant_s_old.r = plant_s.r = plant_s_old.r + plant_s_dot.r*dt;
	plant_s_old.x = plant_s.x = plant_s_old.x + plant_s_dot.x*dt;
	//printf("I'm %d, dt:%0.5f,s_x:%0.2f,s_old_x:%0.2f,s_dot_x:%0.2f\n",
		//this->name, dt, plant_s.x, plant_s_old.x, plant_s_dot.x);
	plant_s_old.vx = plant_s.vx = plant_s_old.vx + plant_s_dot.vx*dt;
	plant_s_old.y = plant_s.y = plant_s_old.y + plant_s_dot.y*dt;
	plant_s_old.vy = plant_s.vy = plant_s_old.vy + plant_s_dot.vy*dt;

	plant_s_old.z = plant_s.z = plant_s_old.z + plant_s_dot.z*dt;
	//printf("I'm %d, dt:%0.5f,s_z:%0.2f,s_old_z:%0.2f,s_dot_z:%0.2f\n",
		//this->name, dt, plant_s.z, plant_s_old.z, plant_s_dot.z);
	plant_s_old.vz = plant_s.vz = plant_s_old.vz + plant_s_dot.vz*dt;

	if (plant_s.z > 0) { // assume landing or crashing
		reset_plant_state(plant_s_old);
		reset_plant_state(plant_s_dot);
		reset_plant_state(plant_s);
	}

	plant_s_old.phi = plant_s.phi = plant_s_old.phi + plant_s_dot.phi*dt;
	if (plant_s.phi > PI) plant_s_old.phi = plant_s.phi = plant_s.phi - 2*PI;
	else if (plant_s.phi < -PI ) plant_s_old.phi = plant_s.phi = plant_s.phi + 2*PI;

	plant_s_old.theta = plant_s.theta = plant_s_old.theta + plant_s_dot.theta*dt;
	if (plant_s.theta > PI) plant_s_old.theta = plant_s.theta = plant_s.theta - 2*PI;
	else if (plant_s.theta < -PI ) plant_s_old.theta = plant_s.theta = plant_s.theta + 2*PI;

	plant_s_old.psi = plant_s.psi = plant_s_old.psi + plant_s_dot.psi*dt;
	if (plant_s.psi > PI) plant_s_old.psi = plant_s.psi = plant_s.psi - 2*PI;
	else if (plant_s.psi < -PI) plant_s_old.psi = plant_s.psi = plant_s.psi + 2*PI;




	//printf("thrust0:%0.2f, thrust1:%0.6f, p:%0.6f\n", thrust[0], thrust[1], plant_s.p);

	

	this->last_t = cur_t;

	return plant_s;
}



void 
	HotDrone_Plant::zero_plant_state(hotdrone_plant_state_t &plant_state) 
{
	plant_state.u = 0;
	plant_state.v = 0;
	plant_state.w = 0;
	plant_state.p = 0;
	plant_state.q = 0;
	plant_state.r = 0;
	plant_state.x = 0;
	plant_state.vx = 0;
	plant_state.y = 0;
	plant_state.vy = 0;
	plant_state.z = 0;
	plant_state.vz = 0;
	plant_state.phi = 0;
	plant_state.theta = 0;
	plant_state.psi = 0;
}

/*
 * Zero out everything except x position and y position
 *
 */
void 
	HotDrone_Plant::reset_plant_state(hotdrone_plant_state_t &plant_state) 
{
	plant_state.u = 0;
	plant_state.v = 0;
	plant_state.w = 0;
	plant_state.p = 0;
	plant_state.q = 0;
	plant_state.r = 0;
	plant_state.vx = 0;
	plant_state.vy = 0;
	plant_state.z = 0;
	plant_state.vz = 0;
	plant_state.phi = 0;
	plant_state.theta = 0;
	plant_state.psi = 0;
}


void
	HotDrone_Plant::set_x(float x)
{
	plant_s.x = x;
	plant_s_old.x = x;
}

void
	HotDrone_Plant::set_y(float y)
{
	plant_s.y = y;
	plant_s_old.y = y;
}

void
	HotDrone_Plant::set_z(float z)
{
	plant_s.z = z;
	plant_s_old.z = z;
}