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
#include "HotDrone_Accelerometer.h"
#include "GaussianNoise.h"

HotDrone_Accelerometer::HotDrone_Accelerometer() {
	reset();
}

void 
	HotDrone_Accelerometer::reset() {
	acc = 0;
}




void 
	HotDrone_Accelerometer::update(const float acc) {
	//float noise = getGaussian(0,Std_n);
	//return (TrueData + noise);

}