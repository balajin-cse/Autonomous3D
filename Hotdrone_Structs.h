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

#ifndef _HOTDRONE_STRUCTS_H_
#define _HOTDRONE_STRUCTS_H_

#include <stdint.h>
#include <time.h>

struct hotdrone_manual_sp_t {
	float throttle;
	void init() {throttle=0;}
};

/*
 *
 */
struct hotdrone_motor_data_t {
	float motor_rpm[4];

	uint64_t timestamp;
};

/*
 *
 */


struct hotdrone_actuator_sp_t {
	float control[4]; // normalized
	uint64_t timestamp;
	void init() {control[0]=control[1]=control[2]=control[3]=timestamp=0;}
};

struct hotdrone_att_rate_sp_t {
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float thrust;

	uint64_t timestamp;
	void init() {roll_rate=pitch_rate=yaw_rate=thrust=timestamp=0;}

};

struct hotdrone_att_rate_t {
	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	float altitude_rate;

	uint64_t timestamp;

	void init() {roll_rate=pitch_rate=yaw_rate=altitude_rate=timestamp=0;}

};


struct hotdrone_att_t {
	float roll;
	float pitch;
	float yaw;

	float roll_rate;
	float pitch_rate;
	float yaw_rate;
	
	float roll_acc;
	float pitch_acc;
	float yaw_acc;

	uint64_t timestamp;

	void init() {roll=pitch=yaw=roll_rate=pitch_rate=yaw_rate=roll_acc=pitch_acc=yaw_acc=timestamp=0;}

};

/*
 *
 */
struct hotdrone_att_sp_t {
	float roll;
	float pitch;
	float yaw;
	float throttle;

	clock_t timestamp;

	void init() {roll=pitch=yaw=throttle=timestamp=0;}
};







/*
 *
 */
struct hotdrone_plant_state_t {
	float u; // vel in body x
	float v; // vel in body y
	float w; // vel in body z
	float p; // roll angle rate in body frame
	float q; // pitch angle rate in body frame
	float r; // yaw angle rate in body frame
	float x; // x position in earth frame
	float vx; // x velocity in earth frame
	float y; // y position in earth frame
	float vy; // y velocity in earth frame 
	float z; // z position in earth frame
	float vz; // z velocity in earth frame
	float phi; // roll angle in earth frame
	float theta; // pitch angle in earth frame
	float psi; // yaw angle in earth frame

	uint64_t timestamp;
};


#endif