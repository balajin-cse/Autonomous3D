#include <math.h>
#include <stdlib.h>
#include <Windows.h>
#include <stdio.h>
#include "HotDec.h"


#define DSPSampleTime 0.001
#define GumSampleTime 0.1
#define NoiseSampleTime 5

HotDec::HotDec(int vehicle_id)
{
	_esti_state.init();
	_local_pos.init();
	_att.init();
	_state_sp.init();
	_thrust_sp.init();

	this->id = vehicle_id;
	this->armed = false;
	this->_plant = new HotDec_Plant();
	this->_thruster = new HotDec_Thruster();
	this->_controller = new HotDec_Controller();
	this->_estimator = new HotDec_Estimator();
	this->_camera = new Camera();


}

HotDec::~HotDec()
{
	delete _plant;
	delete _thruster;
	delete _controller;
	delete _estimator;
	delete _camera;
	delete _plant_thrd;
	delete _thruster_thrd;
	delete _estimator_thrd;
	delete _controller_thrd;
	delete _threeDmodel_thrd;
}

void 
	HotDec::moveTo(float x, float y)
{
	_state_sp.x = x;
	_state_sp.y = y;
}

void 
	HotDec::arm()
{
	_last_t = clock();
	this->armed = true;
	_plant_thrd = new thread(&HotDec::_run_plant, this);
	_camera_thrd = new thread(&HotDec::_run_camera, this);
	_thruster_thrd = new thread(&HotDec::_run_thruster, this);
	_estimator_thrd = new thread(&HotDec::_run_estimator, this);
	_controller_thrd = new thread(&HotDec::_run_controller, this);
	_threeDmodel_thrd = new thread(&HotDec::_run_3dmodel, this);
}

void 
	HotDec::disarm()
{
	this->armed = false;
	_plant_thrd->join();
	_thruster_thrd->join();
	_estimator_thrd->join();
	_controller_thrd->join();
	_threeDmodel_thrd->join();
}

void
	HotDec::_run_camera()
{
	while (armed) {
		_camera->update(_plant->get_plant_x(), _plant->get_plant_y(), _plant->get_plant_theta(), 
						_local_pos, _att);
		Sleep(HOTDEC_CAMERA_UPDATE_PERIOD); // stub

	}

}

void 
	HotDec::_run_plant()
{
	while (armed) {
		//TODO: eliminate deep copy here
		float* global_force = _thruster->get_global_force(_plant->get_plant_theta());
		float tmp[3] = {global_force[0], global_force[1], global_force[2]};
		//printf("force:0:%0.2f, 1:%0.2f, 2:%0.2f, 3:%0.2f\n", global_force[0], 
			//		global_force[1],global_force[2]);
		_plant->update(tmp);
		Sleep(HOTDEC_PLANT_REFRESH_PERIOD); //stub

	}

}

void 
	HotDec::_run_thruster()
{
	while (armed) {
		float* ref; //stub
		_thruster->update(_thrust_sp, _att.theta);
		Sleep(HOTDEC_THRUSTER_UPDATE_PERIOD); //stub

	}

}

void 
	HotDec::_run_3dmodel()
{
	while (armed) {
		update_3dmodel();
		Sleep(HOTDEC_3DMODEL_REFRESH_PERIOD); //stub
	}

}

void 
	HotDec::_run_controller()
{
	while (armed) {
		_controller->update(_state_sp, _plant->get_plant_state(), _thrust_sp);
		//printf("x_sp%0.2f, y_sp:%0.2f, thruster sp:1:%0.2f, 2:%0.2f, 3:%0.2f, 4:%0.2f\n", 
			//_state_sp.x, _state_sp.y, _thrust_sp.U[0], _thrust_sp.U[1], _thrust_sp.U[2], _thrust_sp.U[3]);
		Sleep(HOTDEC_CONTROLLER_UPDATE_PERIOD); //stub
	
	}

}

void 
	HotDec::_run_estimator()
{
	while (armed) {
		float* global_force = _thruster->get_global_force(_att.theta);
		float tmp[4] = {global_force[0], global_force[1], global_force[2], global_force[3]};
		_estimator->update(_local_pos, _att, tmp, _esti_state);
		Sleep(HOTDEC_ESTIMATOR_UPDATE_PERIOD); //stub
	}

}

void
	HotDec::update_3dmodel()
{
	clock_t time_past = (float)(clock() - _last_t)*TIME_SCALE*10;
	_blade_1->setRotation(irr::core::vector3df(0,0,time_past*5000));
	_blade_2->setRotation(irr::core::vector3df(0,0,time_past*5000));
	_blade_3->setRotation(irr::core::vector3df(0,0,time_past*5000));
	_blade_4->setRotation(irr::core::vector3df(0,0,time_past*5000));
	_blade_5->setRotation(irr::core::vector3df(0,0,time_past*5000));

	vehicle_node->setPosition(irr::core::vector3df(_local_pos.x*100,2.5,_local_pos.y*100)); //-300 before
	vehicle_node->setRotation(irr::core::vector3df(0,(_att.theta/PI*180),0));

	//printf("x:%0.4f, y:%0.4f\n", _local_pos.x, _local_pos.y);
}

void
	HotDec::construct_3dmodel(irr::scene::ISceneManager* smgr) 
{
	//material.setTexture(0, driver->getTexture("../../3Dmodel/HotDec_3.jpg"));					//Assign outUV_1(pic) to material
	//material.Lighting = true;
	//material_2.setTexture(0,driver->getTexture("../../3Dmodel/RP_HOTDEC_ALLWHITE.jpg"));
	//material_2.Lighting = true;
	irr::scene::IAnimatedMesh* temp = smgr->getMesh("../../3Dmodel/HotDec.obj");						//Declare an Animated Mesh temp and assign HotDec to it
	//scene::IAnimatedMesh* temp_2 = smgr->getMesh("../../3Dmodel/HotDec_2.obj");	
	vehicle_node =smgr->addMeshSceneNode(temp->getMesh(0));					//Declare a scene Node vehicle_node and assign temp to it
	//vehicle_node->getMaterial(0) = material;	
	//scene::ISceneNode* vehicle_node_2=smgr->addMeshSceneNode(temp->getMesh(0));
	//vehicle_node_2->getMaterial(0) =  material_2;


	temp = smgr->getMesh("../../3Dmodel/thruster.obj");											//Assign thruster(obj) to temp
	_thruster_1=smgr->addMeshSceneNode(temp->getMesh(0));						//Declare SceneNode thurster_1
	_thruster_1->setParent(vehicle_node);															//Fix thruster_1 to vehicle_node
	_thruster_1->setPosition(irr::core::vector3df(0,0,0));											//Set Position of thruster_1

	_thruster_2=smgr->addMeshSceneNode(temp->getMesh(0));
	_thruster_2->setParent(vehicle_node);
	_thruster_2->setPosition(irr::core::vector3df(22.003,0,-2.817));
	_thruster_2->setRotation(irr::core::vector3df(0,44.668,0));

    _thruster_3=smgr->addMeshSceneNode(temp->getMesh(0));
	_thruster_3->setParent(vehicle_node);
	_thruster_3->setRotation(irr::core::vector3df(0,-44.668,0));

    _thruster_4=smgr->addMeshSceneNode(temp->getMesh(0));
	_thruster_4->setParent(vehicle_node);
	_thruster_4->setPosition(irr::core::vector3df(-23.003,0,1.9));
	_thruster_4->setRotation(irr::core::vector3df(0,-135,0));
	//thruster_4_2->setParent(vehicle_node_2);
	//thruster_4_2->setPosition(core::vector3df(-23.003,0,1.9));
	//thruster_4_2->setRotation(core::vector3df(0,-135,0));

    _thruster_5=smgr->addMeshSceneNode(temp->getMesh(0));
	//scene::ISceneNode* thruster_5_2=smgr->addMeshSceneNode(temp->getMesh(0));
   // thruster_5->getMaterial(0) = material;
	//thruster_5_2->getMaterial(0) = material;
	_thruster_5->setParent(vehicle_node);
	_thruster_5->setPosition(irr::core::vector3df(23.003,0,2.9));
	_thruster_5->setRotation(irr::core::vector3df(0,135,0));
	//thruster_5_2->setParent(vehicle_node_2);
	//thruster_5_2->setPosition(core::vector3df(23.003,0,2.9));
	//thruster_5_2->setRotation(core::vector3df(0,135,0));


	_blade = smgr->getMesh("../../3Dmodel/blade.obj");						//Declare Animated Mesh blade and assign blade(obj) to it

    _blade_1 = smgr->addAnimatedMeshSceneNode(_blade);				//Declare Node blade_1 and assign blade to it												//Assign material to blade_1
	_blade_1->setParent(vehicle_node);															//Fix blade_1 to vehicle_node
	_blade_1->setPosition(irr::core::vector3df(0,6.35,-16.38));										//Set Position of blade_1								//Set Position of blade_1

    _blade_2 = smgr->addAnimatedMeshSceneNode(_blade);
	_blade_2->setParent(_thruster_2);
	_blade_2->setPosition(irr::core::vector3df(0,6.35,-16.38));


	_blade_3 = smgr->addAnimatedMeshSceneNode(_blade);
	_blade_3->setParent(_thruster_3);
	_blade_3->setPosition(irr::core::vector3df(0,6.35,-16.38));


	_blade_4 = smgr->addAnimatedMeshSceneNode(_blade);
	_blade_4->setParent(_thruster_4);
	_blade_4->setPosition(irr::core::vector3df(0,6.35,-16.38));

    _blade_5 = smgr->addAnimatedMeshSceneNode(_blade);
	_blade_5->setParent(_thruster_5);
	_blade_5->setPosition(irr::core::vector3df(0,6.35,-16.38));


    temp = smgr->getMesh("../../3Dmodel/chargeboard.obj");										//Assign chargeboard(obj) to temp



}

void 
	HotDec::delay(float *signal_in, float *clock, float *random_num, float *pkg_out)
{
	static int i;
	static int j;
	int k;
	static float timer;
	static float restart;
	static int restart_flag;
 	static struct package
	{	
		float time;
		float pkg[3];
	} buff[30];
	static float clock_old;
	
	/*initialization*/
	if (clock[0] == 0)
	{
		timer = -0.033;
		restart = 0;
		restart_flag = 0;
		i=0;
		j=0;
		clock_old = 0;
	}
	
	/*sample data every 33 ms*/
	
	if (clock[0] - 0.033 > timer) 
	{
		if (i > 29) i = 0;
		buff[i].time = clock[0];
		for (k=0;k<3;k++)
			buff[i].pkg[k] = signal_in[k];
		i++;
		timer += 0.033;
	}

	/*every 10 seconds there is a restart*/
	if ((clock[0]-10.) > restart)
	{
		restart = clock[0];
		restart_flag = 1;
	}

	if (restart_flag)
	{
		if (clock[0] - abnormal_wireless_delay - camera_delay - random_num[0] > buff[j].time)
		{
			for (k=0; k<3; k++)
				pkg_out[k] = buff[j].pkg[k];
			j++;
			if (j > 29) j = 0;
			restart_flag = 0;
		}
	}
	else
	{
		if ((clock[0] - normal_wireless_delay - camera_delay - random_num[0] > buff[j].time) && (clock[0] - clock_old) > 0.001)
		{
			for (k=0; k<3; k++)
				pkg_out[k] = buff[j].pkg[k];
			j++;
			if (j > 29) j = 0;
			clock_old = clock[0];
		}
	}

}



/*
void SimMain(float *time_current,float *ref,float *out,float *debug,int ThrusterStop){

	static float time_old_1=0;
	static float time_old_2=0;
	static float time_old_3=-1;

	 float F_ref[3];
	 float f_out[4];
	static float theta[1]={0};

	static float f[4]={0,0,0,0};
	static float F[3];

	 float ModelPos[3];
	 float random[1]={0};
	 float SensorPos[3];
	 float dummydebug[3];

	 float KFinput[6];
	 float KFoutput[6];

	static unsigned char DSPflag=0,LQRflag=0;

	unsigned int i;
    unsigned int j;



//------------------------Noise Generation-------------------------------------
		if( (time_current[0]-time_old_3)>NoiseSampleTime){
			random[0]=0.034+ 0.001*(rand()%35);
			time_old_3=time_current[0];	
		}

//-----------------------------------------------------------------------------
	if( (time_current[0]-time_old_1)>DSPSampleTime){ //comes in here every DSP sampletime

			ThrusterModel(f,theta,F,f_out);
			time_old_1=time_current[0];


			if(1==LQRflag){//Comes in here only after function "LQR_lsh" is called
				DSPflag=1;//Set up flag indicating output of function "ThursterModel" ready to use
				LQRflag=0;
			}
	}

	if( (time_current[0]-time_old_2)>GumSampleTime){//comes in here every Gumstick sampletime 

		LQR_lsh(ref,out,f,F_ref,ThrusterStop);
			
		LQRflag=1;//Flag indicating function "LQR_lsh" is called
		
		if(1==DSPflag){//Only use output of function "ThursterModel" after "LQR_lsh" is called

			Hovercraft(F,ModelPos,ThrusterStop);

			//delay(ModelPos,time_current,random,SensorPos,dummydebug);

			//SensorPos[2]= (SensorPos[2]*PI)/180.0;
			theta[0]= ModelPos[2];//Update theta

			for(i=0;i<6;i++){
				if(i<3){
					KFinput[i]=F[i];
				}
				else{ 
					KFinput[i]=ModelPos[(i-3)];
				}
			}

			KF_lsh(KFinput,KFoutput);

			for(i=0;i<6;i++){
				if(0==(i%2)){
					out[i]=KFoutput[i];
				}
				else{
					out[i]=ModelPos[(i-1)/2];
				}	
			}
			DSPflag=0;//Clear DSPflag;
		}

		time_old_2=time_current[0];
	}

	debug[0]=F_ref[0];
	debug[1]=F_ref[1];
	
	debug[2]=F[0];
	debug[3]=F[1];

}
*/
