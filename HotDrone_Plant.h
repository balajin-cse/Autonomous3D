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

#ifndef _HOTDRONE_PLANT_H_
#define _HOTDRONE_PLANT_H_

#include <time.h>
#include "HotDrone_Motor.h"
#include "Hotdrone_Structs.h"
#include "Parameters.h"


#define HOTDRONE_MASS 0.4472 //




class HotDrone_Plant {
public:
	HotDrone_Plant(unsigned int config);
	hotdrone_plant_state_t update(const float* thrust);

	float get_u() {return plant_s.u;}
	float get_v() {return plant_s.v;}
	float get_w() {return plant_s.w;}
	float get_p() {return plant_s.p;}
	float get_q() {return plant_s.q;}
	float get_r() {return plant_s.r;}
	float get_x() {return plant_s.x;}
	float get_vx() {return plant_s.vx;}
	float get_y() {return plant_s.y;}
	float get_vy() {return plant_s.vy;}
	float get_z() {return plant_s.z;}
	float get_vz() {return plant_s.vz;}
	float get_phi() {return plant_s.phi;}
	float get_theta() {return plant_s.theta;}
	float get_psi() {return plant_s.psi;}

	void set_x(float x);
	void set_y(float y);
	void set_z(float z);



	hotdrone_plant_state_t get_plant_state_all() {return plant_s;}

private:
	clock_t last_t;
	hotdrone_plant_state_t plant_s;
	hotdrone_plant_state_t plant_s_dot;
	hotdrone_plant_state_t plant_s_old;
	void update_states(float *U, float* motor_rpm);
	void reset_plant_state(hotdrone_plant_state_t &plant_state);
	void zero_plant_state(hotdrone_plant_state_t &plant_state);

	unsigned int plant_config;
	float Mass; //Quadrotor weight with 1350mAh battery [kg]
	float Ixx;		//Ixx is moment of inertia around roll axis  [kg*m^2]
	float Iyy;		//Iyy is moment of inertia around pitch axis [kg*m^2]
	float Izz;		//Izz is moment of inertia around yaw axis   [kg*m^2]
	float J_M;				//Motor moment of inertia, Turnigy 2211-2000 [Nm*s^2]
	float J_P;				//Propeller moment of inertia, GWS EP 5030   [Nm*s^2]
	float J_MP;
	float J_TP;      //J_TP = J_MP because motor and prop are connected directly
	float J_gain;       //gain used to adjust the pole position of the motor TF

};


#endif