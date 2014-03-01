/*************************************************************
 * Copyright @ Dullerud's Autonomous Lab
 * University of Illinois Urbana Champaign
 * 
 * Author @ Bicheng Zhang <viczhang1990@gmail.com>
 * 
 * <---3D Simulation Framework for Autonomous Vehicles-->
 *
 *
 *            ********                *******
 *             -A---               --A---
 *                
 *                      U U
 *                      ^
 *
 *
 *
 ************************************************************/

#ifndef _VEHICLE_STRUCTS_H_
#define _VEHICLE_STRUCTS_H_

#include <time.h>

/*
 *
 */
struct vehicle_local_vel_t {
	clock_t timestamp;
	float vx;
	float vy;
	float vz;

	float vx_body;
	float vy_body;
	float vz_body;

	void init() {vx=vy=vz=vx_body=vy_body=vz_body=0;timestamp=clock();} 
};


/*
 *
 */
struct vehicle_local_pos_sp_t {
	clock_t timestamp;
	float x;
	float y;
	float z;

	void init() {x=y=z=0;timestamp=clock();}
};

/*
 *
 */
struct vehicle_local_pos_t {
	clock_t timestamp;
	float x;
	float y;
	float z;

	void init() {x=y=z=0;timestamp=clock();}
};

#endif