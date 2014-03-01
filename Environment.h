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


#ifndef _ENVIRONMENT_H_
#define _ENVIRONMENT_H_

#include <irrlicht.h>

#include "Vehicle.h"
#include "HotDrone.h"

#define ENVIRON_PAINT_SCENE_PERIOD 1 //milisecond


class Environment {
public:
	Environment();
	~Environment();
	int setup_background();
	void attach_hotdrone(HotDrone* hotdrone);
	void run();
	void stop();

	bool is_running();

	irr::IrrlichtDevice* device;
	irr::video::IVideoDriver* driver;	// video driver
	irr::scene::ISceneManager* smgr; // scene manager
	irr::scene::ICameraSceneNode* cam;

private:

	void paint_scene();
	bool running;


};



#endif