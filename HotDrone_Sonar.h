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

#ifndef _HOTDRONE_SONAR_H_
#define _HOTDRONE_SONAR_H_

#include "Hotdrone_Structs.h"
#include "Vehicle_Structs.h"

class HotDrone_Sonar {
public:
	HotDrone_Sonar();
	void reset();
	float get_height() {return _height;}
	void update(const float vehicle_height, vehicle_local_pos_t &vehicle_pos);
private:
	float _height;

};

#endif