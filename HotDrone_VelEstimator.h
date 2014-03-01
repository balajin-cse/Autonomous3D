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

#ifndef _HOTDRONE_VELESTIMATOR_H_
#define _HOTDRONE_VELESTIMATOR_H_

#include <vector>
#include <time.h>
#include <math.h>
#include "Hotdrone_Structs.h"
#include "Vehicle_Structs.h"

using namespace std;

#define X_POLE 15
#define Y_POLE 15
#define VX_POLE 25
#define VY_POLE 25



class HotDrone_VelEstimator{
public:
	HotDrone_VelEstimator();

	vehicle_local_vel_t get_vehicle_local_vel();
	
	void estimate_velocity_zTransform(const vehicle_local_pos_t pos, const float hackvz, 
													vehicle_local_vel_t &vehicle_local_vel);
	
private:
	float vx_s;
	float vy_s;
	float vz_s;

	float x_pole;
	float y_pole;
	float vx_pole;
	float vy_pole;


	float pos_old[2];
	float vel_old[2];
	float pos_fil_old[2];

	clock_t last_t;

};


# endif 