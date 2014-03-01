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

#ifndef _HOTDRONE_ACCELEROMETER_H_
#define _HOTDRONE_ACCELEROMETER_H_

#include "Hotdrone_Structs.h"

class HotDrone_Accelerometer {
public:
	HotDrone_Accelerometer();
	void reset();
	float get_acc() {return acc;}
	void update(const float acc);

private:
	float acc;

};



#endif