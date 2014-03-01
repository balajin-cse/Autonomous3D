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

#include "Kinect.h"
#include "GaussianNoise.h"
#include "Parameters.h"


Kinect::Kinect() {
	kin_x = 0;
	kin_y = 0;
}

void 
Kinect::reset() {
	kin_x = 0;
	kin_y = 0;
}

vehicle_local_pos_t
Kinect::get_vehicle_pos() 
{
	vehicle_local_pos_t ret;
	ret.x = kin_x;
	ret.y = kin_y;
	return ret;
}

void 
	Kinect::update(float x, float y, vehicle_local_pos_t &vehicle_pos) 
{
	static float Std_Kinect[2]={0.025,0.007};
	float noise[2];
	noise[0]= getGaussian(0,Std_Kinect[0]);
	noise[1]= getGaussian(0,Std_Kinect[1]);
	kin_x=x-0.1153*x+0.297+noise[0];
	kin_y=y+0.3637*y-1.1203+noise[1];

	//vehicle_pos.x = kin_x;
	//vehicle_pos.y = kin_y;
	vehicle_pos.x = x; //hack
	vehicle_pos.y = y;
}