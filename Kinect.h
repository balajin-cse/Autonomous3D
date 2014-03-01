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

#ifndef _KINECT_H_
#define _KINECT_H_

#include "Vehicle_Structs.h"



class Kinect {
public:
	Kinect();
	void reset();
	float get_x() {return kin_x;}
	float get_y() {return kin_y;}
	vehicle_local_pos_t get_vehicle_pos();
	void update(float x, float y, vehicle_local_pos_t &vehicle_pos);

private:
	float kin_x;
	float kin_y;


};

#endif