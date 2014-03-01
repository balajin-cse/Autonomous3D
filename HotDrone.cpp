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

#include <process.h>
#include <Windows.h>
#include <time.h>

#include "HotDrone.h"
#include "Parameters.h"





#pragma comment(lib, "Irrlicht.lib")

/*
HotDrone*
HotDrone::create() const{

	HotDrone* ret = new HotDrone();
	plant = new HotDrone_Plant(0);
	motors = new HotDrone_Motor();
	att_controller = new HotDrone_AttController();
	pos_controller = new HotDrone_PosController();
	vel_estimator = new HotDrone_VelEstimator();
	
	acc_sensor = new HotDrone_Accelerometer();
	gyro_sensor = new HotDrone_Gyroscope();
	sonar_sensor = new HotDrone_Sonar();
	kinect = new Kinect();


	//Roll, pitch, yaw and throttle
	//Roll: [-45 45] from -45 to 45 degrees
	//Pitch: [-45 45] from -45 to 45 degrees
	//Yaw: 0 yaw hold
	//Throttle:  around 500 throttle hold
	//Need to change this later


}*/

HotDrone::HotDrone(int number)
{
	this->id = number;
	_local_pos.init();
	_local_pos_sp.init();
	_local_vel.init();
	_att.init();
	_att_sp.init();
	_actuator_sp.init();
	_manual_sp.init();

	this->_armed = false;

	this->plant = new HotDrone_Plant(0);
	this->motors = new HotDrone_Motor();
	_att_controller = new HotDrone_AttController_pid();
	_att_rate_controller = new HotDrone_AttRateController_pid();
	_pos_controller = new HotDrone_PosController();
	this->vel_estimator = new HotDrone_VelEstimator();
	
	this->acc_sensor = new HotDrone_Accelerometer();
	this->gyro_sensor = new HotDrone_Gyroscope();
	this->sonar_sensor = new HotDrone_Sonar();
	this->kinect = new Kinect();

	_operation_phase = false;
	_standby_phase = true;
	_landing_phase = false;


	//Roll, pitch, yaw and throttle
	//Roll: [-45 45] from -45 to 45 degrees
	//Pitch: [-45 45] from -45 to 45 degrees
	//Yaw: 0 yaw hold
	//Throttle:  around 500 throttle hold
	//Need to change this later
}

void
	HotDrone::_run_autonomous() 
{
	float height_sp = _local_pos_sp.z;

	clock_t cur_t = clock();
	clock_t last_t = cur_t;

	while (_armed) {

		cur_t = clock();

		float dt = (float) (cur_t - last_t)*TIME_SCALE;

		// Manual interrupt, controlling height
		if ((_operation_phase && _manual_sp.throttle >= -0.2f) || 
								(_standby_phase && _manual_sp.throttle > 0.2f))
		{
			// manually trigger to fly up
			_operation_phase = true;
			height_sp = 3;
			_local_pos_sp.z = -height_sp;
		}
		else if (_landing_phase || (_operation_phase && _manual_sp.throttle < 0-.2f))
		{
			// manually trigger to land
			_landing_phase = true;
			_operation_phase = false;
			height_sp -= 0.05;

			_local_pos_sp.z = -height_sp;

			if (-_local_pos.z <= 0.0f || height_sp <= 0.0f) {
				_standby_phase = true;
				_landing_phase = false;
			}
		} else {
			// No manual interrupt, ignore the case
		}

		// X-Y position autonomous control
		_local_pos_sp.x = _local_pos.x;
		_local_pos_sp.y = _local_pos.y;

		printf("dt:%0.2f, height_sp:%0.2f, height:%0.2f\n", dt, -_local_pos_sp.z, -_local_pos.z);

	} //endof while
}



void 
	HotDrone::arm() 
{
	this->_armed = true;
	this->plant_thrd = new thread(&HotDrone::run_plant, this);
	this->estimator_thrd = new thread(&HotDrone::run_estimator, this);
	if (HOTDRONE_ATT_CONTROLLER_ENABLED) _att_controller_thrd = new thread(&HotDrone::_run_att_controller, this);
	if (HOTDRONE_POS_CONTROLLER_ENABLED) this->pos_controller_thrd = new thread(&HotDrone::run_pos_controller, this);
	if (HOTDRONE_ATT_CONTROLLER_ENABLED) this->_att_rate_controller_thrd = new thread(&HotDrone::_run_att_rate_controller, this);
	this->sensor_thrd = new thread(&HotDrone::run_sensor, this);
	this->threeDmodel_thrd = new thread(&HotDrone::run_3dmodel, this);
	this->motor_thrd = new thread(&HotDrone::run_motor, this);
	this->autonomous_thrd = new thread(&HotDrone::_run_autonomous, this);

}

void 
	HotDrone::disarm() 
{
	this->_armed = false;
	this->plant_thrd->join();
	this->sensor_thrd->join();
	this->estimator_thrd->join();
	if (HOTDRONE_ATT_CONTROLLER_ENABLED) this->_att_controller_thrd->join();
	if (HOTDRONE_POS_CONTROLLER_ENABLED) this->pos_controller_thrd->join();
	if (HOTDRONE_ATT_CONTROLLER_ENABLED) this->_att_rate_controller_thrd->join();
	this->threeDmodel_thrd->join();
	this->motor_thrd->join();
	this->autonomous_thrd->join();

	//delete plant_thrd, sensor_thrd, estimator_thrd, controller_thrd, threeDmodel_thrd;
}


void
	HotDrone::set_manual_throttle(float throttle)
{
	_manual_sp.throttle = throttle;
}


void 
	HotDrone::moveTo(float x, float y)
{
	this->_local_pos_sp.x = x;
	this->_local_pos_sp.y = y;

}

void
	HotDrone::moveTo(float x, float y, float z) 
{
	this->_local_pos_sp.x = x;
	this->_local_pos_sp.y = y;
	this->_local_pos_sp.z = -z;
}

void 
	HotDrone::circle(float* center, float rad, float freq, float t)
{
	this->_local_pos_sp.x = center[0]-rad+rad*cos(freq*t);
	this->_local_pos_sp.y = center[1]+rad*sin(freq*t);
}

void 
	HotDrone::riseTo(float h)
{
	this->_local_pos_sp.z = -h;
}

void
	HotDrone::setRoll(float degree) 
{
	this->_att_sp.roll = degree*PI/180;

}

void
	HotDrone::setPitch(float degree) 
{
	this->_att_sp.pitch = degree*PI/180;

}

void
	HotDrone::setYaw(float degree) 
{
	this->_att_sp.yaw = degree*PI/180;

}

void
	HotDrone::deploy(float x, float y)
{
	this->plant->set_x(x);
	this->plant->set_y(y);
	this->_local_pos_sp.x = x;
	this->_local_pos_sp.y = y;
}

void
	HotDrone::deploy(float x, float y, float z)
{
	this->plant->set_x(x);
	this->plant->set_y(y);
	this->plant->set_z(z);
	this->_local_pos_sp.x = x;
	this->_local_pos_sp.y = y;
	this->_local_pos_sp.z = z;
}



/*
 *
 *
 */
void 
	HotDrone::run_plant()
{
	while (_armed) {
		float* force = this->motors->get_global_force();
		
		float global_force[4] = {force[0], force[1],force[2], force[3]};
		//printf("I'm %d, m1:%0.2f, m2:%0.2f, m3:%0.2f, m4:%0.2f\n",this->name, thrust2[0], thrust2[1], thrust2[2], thrust2[3]);
		this->plant->update(global_force);
		Sleep(HOTDRONE_PLANT_REFRESH_PERIOD);
	}
}


void 
	HotDrone::run_sensor()
{
	while (_armed) {
		this->acc_sensor->update(0); //stub
		//printf("p:%0.6f\n", plant->get_p());
		this->gyro_sensor->update(this->plant->get_phi(), this->plant->get_theta(), this->plant->get_psi(), 
			this->plant->get_p(), this->plant->get_q(), this->plant->get_r(), _att); //stub
		this->sonar_sensor->update(this->plant->get_z(), this->_local_pos); //stub
		this->kinect->update(this->plant->get_x(), this->plant->get_y(), this->_local_pos);
		
		Sleep(HOTDRONE_SENSOR_UPDATE_PERIOD);
	}

}

void 
	HotDrone::run_estimator() 
{
	while (_armed) {
		//TODO: assuming perfect network channel
		//New Channel object mimic network data transmission
		this->vel_estimator->estimate_velocity_zTransform(this->_local_pos, this->plant->get_vz(), this->_local_vel); 
		Sleep(HOTDRONE_VEL_ESTIMATOR_UPDATE_PERIOD);

	}
}

void 
	HotDrone::_run_att_controller() 
{
	while (_armed) {
		//printf("roll:%0.2f, pitch:%0.2f, yaw:%0.2f, height:%0.2f\n", 
			//_att.roll*180/PI, _att.pitch*180/PI, _att.yaw*180/PI, _local_pos.z*-1); 
		_att_controller->update(_att_sp, _att, _att_rate_sp, true, false);
		//printf("yohoo~ I'm %d, att_cntrl_sig->m1:%0.2f, m2:%0.2f, m3:%0.2f, m4:%0.2f\n", 
			//this->name, att_cntrl_sig.motor_cntrl_sp[0], att_cntrl_sig.motor_cntrl_sp[1], att_cntrl_sig.motor_cntrl_sp[2], att_cntrl_sig.motor_cntrl_sp[3]);
		
		Sleep(HOTDRONE_POS_CONTROLLER_UPDATE_PERIOD);
	}
			 
}

void
	HotDrone::_run_att_rate_controller()
{
	while (_armed) {
		_att_rate_controller->update(_att_rate_sp, _att, _actuator_sp, false);
		//printf("yohoo~ I'm %d, att_cntrl_sig->m1:%0.2f, m2:%0.2f, m3:%0.2f, m4:%0.2f\n", 
			//this->name, att_cntrl_sig.motor_cntrl_sp[0], att_cntrl_sig.motor_cntrl_sp[1], att_cntrl_sig.motor_cntrl_sp[2], att_cntrl_sig.motor_cntrl_sp[3]);
		
		Sleep(HOTDRONE_ATT_CONTROLLER_UPDATE_PERIOD);
	}


}

void
	HotDrone::run_pos_controller() 
{
	while (_armed) {
		//printf("x:%0.2f,y:%0.2f,roll:%0.3f,pitch:%0.3f,yaw:%0.3f,height:%0.3f\n", 
			//	_local_pos.x, _local_pos.y, _att.roll*180/PI, _att.pitch*180/PI, _att.yaw*180/PI, -_local_pos.z);
		//printf("yohoo~I'm %d, z_sp:%0.2f,sonar_z:%0.2f,plant_z:%0.2f\n", this->name, this->local_pos_sp.z, this->sonar_sensor->get_height(), this->plant->get_z());
		//WARNING: update replaced by update2
		_pos_controller->update(_local_pos_sp, _local_pos, _local_vel, _att, _att_sp);
		//printf("pos_cntrl_sig->att_sp_roll:%0.2f, pitch:%0.2f, yaw:%0.2f, throttle%0.2f\n", 
			//att_sp.roll, att_sp.pitch, att_sp.yaw, att_sp.throttle);
		//("yohoo~I'm %d, z_sp:%0.2f, z:%0.2f, throttle_sp:%0.4f\n", 
			//this->name, this->local_pos_sp.z, this->local_pos.z, this->att_sp.throttle);

		Sleep(HOTDRONE_ATT_CONTROLLER_UPDATE_PERIOD);
	}

}





void 
	HotDrone::run_motor() 
{
	while (_armed) {
		this->motors->update(_actuator_sp);
		//printf("yohoo~ I'm %d, motor rpm->m1:%0.2f, m2:%0.2f, m3:%0.2f, m4:%0.2f\n", 
			//this->name, thrust[0], thrust[1], thrust[2], thrust[3]);
		
		Sleep(HOTDRONE_MOTOR_UPDATE_PERIOD);
	}
}

void 
	HotDrone::run_3dmodel() 
{
	while (_armed) {
		this->update_3dmodel();
		Sleep(HOTDRONE_3DMODEL_REFRESH_PERIOD);
	}

}

void 
	HotDrone::update_3dmodel() 
{
		//drone
		/*---Update Drone1---*/
		//px4drone->setPosition(core::vector3df(pos[1][0]*100,-10*GlobalPos[2],pos[1][1]*100));

	//printf("kin_x:%0.2f,kin_y:%0.2f,height:%0.2f\n",kinect->get_x(),  kinect->get_y(), sonar_sensor->get_height());

	this->vehicle_node->setPosition(irr::core::vector3df(100*this->_local_pos.x, -10*this->_local_pos.z+6.4,100*this->_local_pos.y)); 
	this->vehicle_node->setRotation(irr::core::vector3df(this->_att.roll*180/PI, this->_att.yaw*180/PI,this->_att.pitch*180/PI));

	//printf("x:%0.2f,y:%0.2f,z:%0.2f,vx:%0.2f,vy:%0.2f,vz:%0.2f,roll:%0.2f,pitch:%0.2f,yaw:%0.2f\n",
	//	local_pos.x, local_pos.y, local_pos.z, local_vel.vx, local_vel.vy, local_vel.vz, att.roll, att.pitch, att.yaw);

	//printf("roll:%0.2f,pitch:%0.2f,yaw:%0.2f,throttle:%0.2f,roll_sp:%0.2f,pitch_sp:%0.2f,throttle_sp:%0.2f\n",
	//	att.roll, att.pitch, att.yaw, att.throttle, att_sp.roll, att_sp.pitch, att_sp.throttle);

	float* delta_angle = motors->get_delta_angle();
	float* old_angle = motors->get_old_angle();



	this->propeller_1->setRotation(irr::core::vector3df(0,-old_angle[0]-delta_angle[0],0));
	this->propeller_2->setRotation(irr::core::vector3df(0,old_angle[1]+delta_angle[1],0));
	this->propeller_3->setRotation(irr::core::vector3df(0,-old_angle[2]-delta_angle[2],0));
	this->propeller_4->setRotation(irr::core::vector3df(0,old_angle[3]+delta_angle[3],0));


	

	//printf("Yohooo! Im'm:%d updated at x:%0.2f, y:%0.2f, z:%0.2f, z_sp:%0.2f\n", name, local_pos.x, local_pos.y, local_pos.z, local_pos_sp.z);

	//0722
	//Goal->setPosition(core::vector3df(ref[0][1]*100,50+10*sin(lalala),ref[0][3]*100));
	//Goal2->setPosition(core::vector3df(ref[1][1]*100,50+10*sin(lalala),ref[1][3]*100));

	//halo
	//halo_hotd -> setPosition(core::vector3df(pos[0][0]*100+100*sin(lalala),2.5,pos[0][1]*100+100*cos(lalala)));
	//halo_drone -> setPosition(core::vector3df(100*GlobalPos[0]+100*sin(-lalala),-10*GlobalPos[2],100*GlobalPos[1]+100*cos(-lalala)));

	//hoop_hotd -> setPosition(core::vector3df(pos[0][0]*100,2.5,pos[0][1]*100));
	//hoop_drone -> setPosition(core::vector3df(100*GlobalPos[0],-10*GlobalPos[2],100*GlobalPos[1]));




}


void 
	HotDrone::construct_3dmodel(irr::scene::ISceneManager* smgr) 
{
	

		
	/*---Extract object---*/
	irr::scene::IAnimatedMesh* drone_body = smgr->getMesh("../../Media/HotDrone/drone_body.obj");		//Declare an Animated Mesh temp and assign quadrator model to it
	irr::scene::IAnimatedMesh* propeller = smgr->getMesh("../../Media/HotDrone/rotationalpartf.obj");
	//irr::scene::IAnimatedMesh* kinect_body = smgr->getMesh("../../Media/Kinect/kinect.obj");
	//irr::scene::IAnimatedMesh* goaly = smgr->getMesh("../../Media/Miscellance/goal.obj");
	//irr::scene::IAnimatedMesh* halo_dot_body = smgr->getMesh("../../Media/Miscellance/halodot.obj");
	//irr::scene::IAnimatedMesh* mark_body = smgr->getMesh("../../Media/Miscellance/mark_body.obj");
	//irr::scene::IAnimatedMesh* hoop_body = smgr->getMesh("../../Media/Miscellance/hoop_body.obj");
	
	
	/*---Compose Drone---*/
	this->vehicle_node = smgr->addMeshSceneNode(drone_body->getMesh(0),0,01,
			irr::core::vector3df(0,0,0),irr::core::vector3df(0,0,0),irr::core::vector3df(0.1f,0.1f,0.1f));					//Declare a scene Node Drone_body and assign temp to it
	//--------Four propellers-----------
	this->propeller_1 = smgr->addAnimatedMeshSceneNode(propeller);	  //front-left
	this->propeller_2 = smgr->addAnimatedMeshSceneNode(propeller);	  //front-right
	this->propeller_3 = smgr->addAnimatedMeshSceneNode(propeller);	  //rear-left
	this->propeller_4 = smgr->addAnimatedMeshSceneNode(propeller);	  //rear-right

	//--------***Positions are well calibrated***-----------
	this->propeller_1->setParent(vehicle_node); 
	this->propeller_1->setPosition(irr::core::vector3df(81.5,10.5,122.0));
	this->propeller_2->setParent(vehicle_node); 
	this->propeller_2->setPosition(irr::core::vector3df(80.0,10.5,-131.5));
	this->propeller_3->setParent(vehicle_node); 
	this->propeller_3->setPosition(irr::core::vector3df(-179,10.5,121.00));
	this->propeller_4->setParent(vehicle_node); 
	this->propeller_4->setPosition(irr::core::vector3df(-179,10.5,-129.5));


}



HotDrone::~HotDrone() 
{
	delete this->plant;
	delete this->motors;
	delete this->acc_sensor;
	delete this->gyro_sensor;
	delete this->sonar_sensor;
	delete this->vel_estimator;
	delete _att_controller;
	delete _att_rate_controller;
	delete _pos_controller;
}


