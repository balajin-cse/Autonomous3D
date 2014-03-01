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

#include <time.h>

#include "HotDrone_VelEstimator.h"
#include "Parameters.h"


HotDrone_VelEstimator::HotDrone_VelEstimator() 
{
	vx_s = 0;
	vy_s = 0;
	vz_s = 0;

	x_pole = X_POLE;
	y_pole = Y_POLE;
	vx_pole = VX_POLE;
	vy_pole = VY_POLE;

	pos_old[0] = pos_old[1] = 0;
	vel_old[0] = vel_old[1] = 0;
	pos_fil_old[0] = pos_fil_old[1] = 0;

	last_t = clock();

}


vehicle_local_vel_t
	HotDrone_VelEstimator::get_vehicle_local_vel()
{
	vehicle_local_vel_t ret;
	ret.vx = vx_s;
	ret.vy = vy_s;

	return ret;
}




/*
 * S domain to Z domain using bilinear transform.
 *
 *
 *
 */
void
	HotDrone_VelEstimator::estimate_velocity_zTransform(const vehicle_local_pos_t pos, const float hackvz, 
																vehicle_local_vel_t &vehicle_local_vel)
{	
	clock_t cur_t = clock();
	float dt = (float)(cur_t - last_t)*TIME_SCALE;
	
	//static float old_pos_z = 0;

	float PosFil[2]={0,0};
	float PosDelta[2] ={0,0};

	PosDelta[0] = pos.x-pos_old[0];
	PosDelta[1] = pos.y-pos_old[1];

	vx_s = (2*vx_pole*PosDelta[0] - (x_pole*dt-2)*vel_old[0])/(vx_pole*dt+2);
	vy_s = (2*vy_pole*PosDelta[1] - (y_pole*dt-2)*vel_old[1])/(vy_pole*dt+2);

	PosFil[0] = ((2-x_pole*dt)*pos_fil_old[0]+x_pole*dt*(pos.x+pos_old[0]))/(2+x_pole*dt);
	PosFil[1] = ((2-y_pole*dt)*pos_fil_old[1]+y_pole*dt*(pos.y+pos_old[1]))/(2+y_pole*dt);

	pos_old[0]=pos.x;
	pos_old[1]=pos.y;
	vel_old[0]=vx_s;
	vel_old[1]=vy_s;

	//vz_s = -(old_pos_z - pos.z)/dt; // very rough estimation
	vz_s = hackvz;

	last_t = cur_t;

	vehicle_local_vel.vx = vx_s;
	vehicle_local_vel.vy = vy_s;
	vehicle_local_vel.vz = vz_s;
}

