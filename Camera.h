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

#ifndef _CAMERA_H_
#define _CAMERA_H_

#include "Vehicle_Structs.h"
#include "HotDec_Structs.h"

class Camera {
public:
	Camera();
	~Camera();
	void update(float x, float y, vehicle_local_pos_t &local_pos);
	void update(float x, float y, float theta, vehicle_local_pos_t &local_pos, 
												hotdec_att_t &att);

private:


};

#endif