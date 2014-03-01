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
#include <stdio.h>

#include "HotDrone_Gyroscope.h"	
#include "GaussianNoise.h"
#include "Parameters.h"

HotDrone_Gyroscope::HotDrone_Gyroscope() 
{
	roll_s = 0;
	pitch_s = 0;
	yaw_s = 0;
}

void 
	HotDrone_Gyroscope::reset() 
{
	roll_s = 0;
	pitch_s = 0;
	yaw_s = 0;
}


void 
	HotDrone_Gyroscope::update(const float roll, const float pitch, const float yaw,
								const float roll_rate, const float pitch_rate, const float yaw_rate,
									hotdrone_att_t &hotdrone_att) 
{
	static float Std_n_Gyro[3] = {0.00125,0.0015,0.0009}; // roll, pitch, yaw
	static float Std_BRW[3]={9.13e-6, 2.66e-5, 1.5e-5};
	static float Bias[3] = {8.83e-4,-4.97e-5,-5.67e-4};

	static clock_t last_t = clock();

	clock_t cur_t = clock();

	float dt = (float) (cur_t - last_t)*TIME_SCALE;

	float gyro[3]={roll,pitch,yaw};
	float noise[3], walkBias[3];
	// noise model
	for(int i=0;i<3;i++){
		noise[i]= getGaussian(0, Std_n_Gyro[i]);
		walkBias[i] = getGaussian(0,Std_BRW[i]);
		gyro[i] = gyro[i] + Bias[i] + noise[i] + walkBias[i];
		if(1==i) gyro[i]+= -6.22e-5*dt;
	}
	roll_s = gyro[0];
	pitch_s = gyro[1];
	yaw_s = gyro[2];

	//hotdrone_att.roll = roll_s;
	//hotdrone_att.pitch = pitch_s;
	//hotdrone_att.yaw = yaw_s;

	hotdrone_att.roll = roll; // hack!!
	hotdrone_att.pitch = pitch;
	hotdrone_att.yaw = yaw;
	hotdrone_att.roll_rate = roll_rate;
	hotdrone_att.pitch_rate = pitch_rate;
	hotdrone_att.yaw_rate = yaw_rate;

	//printf("hotdrone_att.roll_rate:%0.6f, roll_rate:%0.6f\n", 
		//hotdrone_att.roll_rate, roll_rate);
}