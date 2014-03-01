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

#include "HotDrone_Sonar.h"
#include "GaussianNoise.h"

HotDrone_Sonar::HotDrone_Sonar() {
	_height = 0;
}


void
HotDrone_Sonar::reset() {
	_height = 0;
}



void 
	HotDrone_Sonar::update(const float vehicle_height, vehicle_local_pos_t &vehicle_pos) {//data input in the unit of m
	static float Std_n_Alti_in = 0.0002;
	static float Std_n_Alti_out = 0.0017;
	
	float temp = vehicle_height;
	if((temp<0.05)&&(temp>-0.26)){
		temp = -0.27+getGaussian(0,Std_n_Alti_in);
	}
	else if(temp<=-0.26){
		temp = 1.2257*temp + 0.0441 +getGaussian(0,Std_n_Alti_out);
	}
	else {}
	_height = temp;

	//vehicle_pos.z = height_s;

	vehicle_pos.z = vehicle_height; //hack

}