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

#ifndef _VEHICLE_H_
#define _VEHICLE_H_

#include <irrlicht.h>

class Vehicle {
public:

protected:
	/*---Vehicle state---*/
	int id;
	bool armed;	
	/*---3D---*/

	irr::scene::ISceneNode* vehicle_node;


};



#endif