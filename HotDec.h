#ifndef _HOTDEC_H_
#define _HOTDEC_H_

#include <math.h>
#include <time.h>
#include <stdio.h>
#include <thread>
#include <iostream>
#include <irrlicht.h>

#include "Vehicle.h"
#include "Camera.h"
#include "HotDec_Plant.h"
#include "HotDec_Estimator.h"
#include "HotDec_Controller.h"
#include "HotDec_Thruster.h"

using namespace std;

#define normal_wireless_delay 0.003
#define abnormal_wireless_delay 0.4
#define camera_delay 0.033

#define HOTDEC_PLANT_REFRESH_PERIOD 1 //millisecond
#define HOTDEC_CONTROLLER_UPDATE_PERIOD 100 //milisecond
#define HOTDEC_ESTIMATOR_UPDATE_PERIOD 10 //milisecond
#define HOTDEC_CAMERA_UPDATE_PERIOD 10 // millisecond
#define HOTDEC_THRUSTER_UPDATE_PERIOD 1 // millisecond
#define HOTDEC_3DMODEL_REFRESH_PERIOD 10 //millisecond


class HotDec : public Vehicle {
public:
	HotDec(int vehicle_id);
	~HotDec();
	void arm();
	void disarm();
	void moveTo(float x, float y);

	void construct_3dmodel(irr::scene::ISceneManager* smgr);


private:
	void delay(float *signal_in, float *clock, float *random_num, float *pkg_out);
	void update_3dmodel();

	void _run_plant();
	void _run_camera();
	void _run_thruster();
	void _run_estimator();
	void _run_controller();
	void _run_3dmodel();

	Camera* _camera;
	HotDec_Plant* _plant;
	HotDec_Controller* _controller;
	HotDec_Estimator* _estimator;
	HotDec_Thruster* _thruster;

	thread* _camera_thrd;
	thread* _plant_thrd;
	thread* _thruster_thrd;
	thread* _estimator_thrd;
	thread* _controller_thrd;
	thread* _threeDmodel_thrd;

	clock_t _last_t;

	hotdec_plant_state_t _esti_state;
	vehicle_local_pos_t _local_pos;
	hotdec_att_t _att;
	hotdec_state_sp_t _state_sp;
	hotdec_thrust_sp_t _thrust_sp;

	irr::scene::ISceneNode* _thruster_1;
	irr::scene::ISceneNode* _thruster_2;
	irr::scene::ISceneNode* _thruster_3;
	irr::scene::ISceneNode* _thruster_4;
	irr::scene::ISceneNode* _thruster_5;
	irr::scene::IAnimatedMesh* _blade;
	irr::scene::IAnimatedMeshSceneNode* _blade_1;
	irr::scene::IAnimatedMeshSceneNode* _blade_2;
	irr::scene::IAnimatedMeshSceneNode* _blade_3;
	irr::scene::IAnimatedMeshSceneNode* _blade_4;
	irr::scene::IAnimatedMeshSceneNode* _blade_5;



};




#endif