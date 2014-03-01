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

#ifndef _HOTDRONE_GYROSCOPE_H_
#define _HOTDRONE_GYROSCOPE_H_

#include <time.h>

#include "Hotdrone_Structs.h"

class HotDrone_Gyroscope {
public:
	
	HotDrone_Gyroscope();
	void reset();
	float get_roll() {return roll_s;}
	float get_pitch() {return pitch_s;}
	float get_yaw() {return yaw_s;}
	void update(const float roll, const float pitch, const float yaw,
								const float roll_rate, const float pitch_rate, const float yaw_rate,
									hotdrone_att_t &hotdrone_att);

private:
	clock_t timestamp;
	float roll_s;
	float pitch_s;
	float yaw_s;

};

#endif