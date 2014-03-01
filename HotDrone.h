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

#ifndef _HOTDRONE_H_
#define _HOTDRONE_H_


#include <math.h>
#include <stdio.h>
#include <thread>
#include <iostream>
#include <irrlicht.h>
#include "Parameters.h"

#include "HotDrone_AttController_pid.h"
#include "HotDrone_AttRateController_pid.h"
#include "HotDrone_PosController.h"
#include "HotDrone_VelEstimator.h"
#include "HotDrone_Accelerometer.h"
#include "HotDrone_Gyroscope.h"
#include "HotDrone_Sonar.h"
#include "HotDrone_OpticalFlow.h"
#include "HotDrone_Plant.h"
#include "HotDrone_Motor.h"
#include "Hotdrone_structs.h"
#include "Vehicle.h"
#include "Kinect.h"

using namespace std;

#define HOTDRONE_PLANT_REFRESH_PERIOD 5 //millisecond
#define HOTDRONE_ATT_CONTROLLER_UPDATE_PERIOD 5 //milisecond
#define HOTDRONE_ATT_RATE_CONTROLLER_UPDATE_PERIOD 5 //milisecond
#define HOTDRONE_POS_CONTROLLER_UPDATE_PERIOD 5 //milisecond
#define HOTDRONE_ATT_ESTIMATOR_UPDATE_PERIOD 50 //milisecond
#define HOTDRONE_VEL_ESTIMATOR_UPDATE_PERIOD 50 //milisecond
#define HOTDRONE_SENSOR_UPDATE_PERIOD 50 //milisecond
#define HOTDRONE_KINECT_UPDATE_PERIOD 50 // millisecond
#define HOTDRONE_MOTOR_UPDATE_PERIOD 10 // millisecond
#define HOTDRONE_3DMODEL_REFRESH_PERIOD 5 //millisecond

#define HOTDRONE_POS_CONTROLLER_ENABLED true
#define HOTDRONE_ATT_CONTROLLER_ENABLED true

#define HOTDRONE_PLANT_CONFIG 0 // quadrotor



class HotDrone : public Vehicle {
public:
	/*---functions---*/
	HotDrone(int number);
	void deploy(float x, float y);
	void deploy(float x, float y, float z);
	~HotDrone();
	void arm();
	void disarm();
	void moveTo(float* time_current);
	void moveTo(float x, float y);
	void moveTo(float x, float y, float z);
	void riseTo(float h);
	void setRoll(float degree);
	void setPitch(float degree);
	void setYaw(float degree);
	void circle(float* center, float rad, float freq, float t);

	void set_manual_throttle(float throttle);

	void construct_3dmodel(irr::scene::ISceneManager* smgr);

private:	

	/*---state variables---*/
	bool _armed;
	bool _standby_phase;
	bool _landing_phase;
	bool _operation_phase;

	/*---manual setpoint---*/
	hotdrone_manual_sp_t _manual_sp;

	/*---thread varaibles---*/
	thread* plant_thrd;
	thread* motor_thrd;
	thread* estimator_thrd;
	thread* _att_controller_thrd;
	thread* _att_rate_controller_thrd;
	thread* pos_controller_thrd;
	thread* sensor_thrd;
	thread* threeDmodel_thrd;
	thread* autonomous_thrd;

	/*---Plant---*/
	HotDrone_Plant* plant;
	/*---Motor---*/
	HotDrone_Motor* motors;
	/*---Sensors---*/
	HotDrone_Accelerometer* acc_sensor;
	HotDrone_Gyroscope* gyro_sensor;
	HotDrone_Sonar* sonar_sensor;

	/*---Offboard sensors---*/
	Kinect* kinect;

	/*---Controllers---*/
	//HotDrone_AttController* att_controller;
	HotDrone_AttController_pid* _att_controller;
	HotDrone_AttRateController_pid* _att_rate_controller;
	HotDrone_PosController* _pos_controller;

	/*---Estimators---*/
	HotDrone_VelEstimator* vel_estimator;

	void run_plant();
	void run_motor();
	void run_estimator();
	void _run_att_controller();
	void _run_att_rate_controller();
	void run_pos_controller();
	void run_sensor();
	void run_3dmodel();
	void _run_autonomous();
	void update_3dmodel();
	
	/*---Vehicle state---*/

	/*---Data---*/
	hotdrone_actuator_sp_t _actuator_sp;
	hotdrone_att_sp_t _att_sp; // roll, pitch, yaw, altitude
	hotdrone_att_rate_sp_t _att_rate_sp;
	vehicle_local_pos_sp_t _local_pos_sp;
	vehicle_local_pos_t _local_pos;
	vehicle_local_vel_t _local_vel;
	hotdrone_att_t _att;


	/*---3D model---*/
	irr::scene::IAnimatedMeshSceneNode* propeller_1;	  //front-left
	irr::scene::IAnimatedMeshSceneNode* propeller_2;	  //front-right
	irr::scene::IAnimatedMeshSceneNode* propeller_3;	  //rear-left
	irr::scene::IAnimatedMeshSceneNode* propeller_4;	  //rear-right


};

#endif