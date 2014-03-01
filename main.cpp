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













/*-------------------------------------------------------------------------------------------------------------------------------------------------
*
*						Autonomous Vehicle 3D
*
*						Bicheng Zhang
*			
*						Prof. Geir E. Dullerud
*				
*						Networked Autonomous Lab
*
*						University of Illinois Urbana Champaign
*
----------------------------------------------------------------------------------------------------------------------------------------------------*/













#include <iostream>
#include <stdlib.h>
#include <stdio.h>
#include <irrlicht.h>
#include <math.h>
#include <time.h>
#include "HotDrone.h"
#include "HotDec.h"
#include "Parameters.h"


//#pragma comment(lib, "Irrlicht.lib")
   





int main() {	

	/* select video driver */
	irr::video::E_DRIVER_TYPE driverType = irr::video::EDT_DIRECT3D9;

	printf("Please select the driver you want for this example:\n"\
		" (a) Direct3D 9.0c\n (b) Direct3D 8.1\n (c) OpenGL 1.5\n"\
		" (d) Software Renderer\n (e) Burning's Software Renderer\n"\
		" (f) NullDevice\n (otherKey) exit\n\n");

	char choice;
	std::cin >> choice;

	switch(choice)
	{
		case 'a': driverType = irr::video::EDT_DIRECT3D9;break;
		case 'b': driverType = irr::video::EDT_DIRECT3D8;break;
		case 'c': driverType = irr::video::EDT_OPENGL;   break;
		case 'd': driverType = irr::video::EDT_SOFTWARE; break;
		case 'e': driverType = irr::video::EDT_BURNINGSVIDEO;break;
		case 'f': driverType = irr::video::EDT_NULL;     break;
		default: return 1;
	}

	/* create irrlich device */

	//IrrlichtDevice* device = createDevice(driverType,core::dimension2d<u32>(800,600),16, false, false, false, 0); //Create brunch "device"
	irr::IrrlichtDevice* device = irr::createDevice(driverType, 
						irr::core::dimension2d<irr::u32>(800,600),16, false, false, false, 0);


	if (device == 0)
		return 1; // could not create selected driver.

	irr::video::IVideoDriver* driver = device->getVideoDriver();						//Create driver for device
	irr::scene::ISceneManager* smgr = device->getSceneManager();						//Creat Scene Manager "smgr" for device

	//device->setInputReceivingSceneManager(smgr);


	irr::scene::ICameraSceneNode* cam = smgr->addCameraSceneNodeFPS(0, 100.0f, 0.1f);  // Declare Camera Scene Node"cam"
	cam->setPosition(irr::core::vector3df(50,150,-50));	 // Set camera position

	
	
	printf("Initialize environment finished\n");

	    /*CIrrEventReceiver* recv = new CIrrEventReceiver();
	device->setEventReceiver(recv);*/

	//RTSCamera* cam = new RTSCamera(device,smgr->getRootSceneNode(),smgr,-1,1000.0f,10.0f,10.0f);
    //cam->setPosition(vector3df(-200,100,-400));

	/* create irrlich camera*/


	/* set up background texture */
	irr::video::SMaterial material,material_2;																	//Declare "material"
	irr::video::ITexture* pTexture;
	
	pTexture = driver->getTexture("../../Media/background/floor/lab_floor.jpg"); //Declare texture "pTexture", assign floor(pic) to it
	material.setTexture(0, pTexture);															//assign ptexture to material
	material.Lighting = true;
	irr::scene::IAnimatedMesh* space = smgr->getMesh("../../Media/background/floor/space.obj");						//Declare Animated Mesh space, assign sapce(obj)
	smgr->getMeshManipulator()->makePlanarTextureMapping(space->getMesh(0), 0.008f);
	irr::scene::ISceneNode* nodespace = 0;															//Declare scene Node space "nodespace"
	nodespace = smgr->addAnimatedMeshSceneNode(space);											//Assign space to nodespace
	nodespace->getMaterial(0) = material;														//assign material to nodespace

	irr::scene::ITriangleSelector* selector = 0;														//Declare a selector

	//floor
	irr::scene::ISceneNode* nodespace1 = 0;															//Declare another node space "nodespace1"
	if (space) nodespace1 = smgr->addMeshSceneNode(space->getMesh(0));	
	nodespace1->getMaterial(0) = material;														//assign material to nodespace1
	nodespace1->setPosition(irr::core::vector3df(300,0,300));
	nodespace->setVisible(false);																//set nodespace as unvisible
	
	//wall paper
	pTexture = driver->getTexture("../../Media/background/floor/lab_floor.jpg"); //Declare texture "pTexture", assign floor(pic) to it
	material.setTexture(0, pTexture);															//assign ptexture to material
	material.Lighting = true;

	//front wall
	irr::scene::ISceneNode* nodespace2 = 0;															//Declare another node space "nodespace1"
	if (space) nodespace2 = smgr->addMeshSceneNode(space->getMesh(0),0,01,
						irr::core::vector3df(0,0,0),irr::core::vector3df(90,0,0),irr::core::vector3df(1.0f,1.2f,0.25f));						
	nodespace2->getMaterial(0) = material;
	nodespace2->getMaterial(0).getTextureMatrix(0).setTextureScaleCenter(0.20,0.20);//assign material to nodespace1
	nodespace2->getMaterial(0).getTextureMatrix(0).setTextureTranslate(0.5,0.5);
	nodespace2->setPosition(irr::core::vector3df(300,75,600));
	nodespace->setVisible(false);

	//wall paper
	/*
	pTexture = driver->getTexture("../../new3Dmodel/right_wall.png"); //Declare texture "pTexture", assign floor(pic) to it
	material.setTexture(0, pTexture);															//assign ptexture to material
	material.Lighting = true;

	//right wall
	scene::ISceneNode* nodespace3 = 0;															
	if (space) nodespace3 = smgr->addMeshSceneNode(space->getMesh(0),0,01,core::vector3df(0,0,0),core::vector3df(90,90,0),core::vector3df(1.0f,1.2f,0.25f));						
	nodespace3->getMaterial(0) = material;	
	nodespace3->getMaterial(0).getTextureMatrix(0).setTextureScaleCenter(0.2,0.2);
	nodespace3->getMaterial(0).getTextureMatrix(0).setTextureTranslate(0.5,0.5);
	nodespace3->setPosition(core::vector3df(600,75,300));
	nodespace->setVisible(false);

	//wall paper
	pTexture = driver->getTexture("../../new3Dmodel/left_wall.png"); //Declare texture "pTexture", assign floor(pic) to it
	material.setTexture(0, pTexture);															//assign ptexture to material
	material.Lighting = true;

	//left wall
	scene::ISceneNode* nodespace4 = 0;															
	if (space) nodespace4 = smgr->addMeshSceneNode(space->getMesh(0),0,01,core::vector3df(0,0,0),core::vector3df(90,-90,0),core::vector3df(1.0f,1.2f,0.25f));						
	nodespace4->getMaterial(0) = material;	//assign material to nodespace1
	nodespace4->getMaterial(0).getTextureMatrix(0).setTextureScaleCenter(0.2,0.2);
	nodespace4->getMaterial(0).getTextureMatrix(0).setTextureTranslate(0.5,0.5);
	nodespace4->setPosition(core::vector3df(0,75,300));
	nodespace->
	*/

	device->getCursorControl()->setVisible(false);	

	/*---add billboard---*/
	irr::scene::IBillboardSceneNode * bill = smgr->addBillboardSceneNode();
	bill->setMaterialType(irr::video::EMT_TRANSPARENT_ADD_COLOR );
	bill->setMaterialTexture(0, driver->getTexture("../../Media/background/billboard/particle.bmp"));
	bill->setMaterialFlag(irr::video::EMF_LIGHTING, false);
	bill->setSize(irr::core::dimension2d<irr::f32>(15.0f, 15.0f));
	bill->setPosition(irr::core::vector3df(0,-5,0));

	/*---create light---*/
	
	irr::scene::ISceneNode* light_1 = smgr->addLightSceneNode(0, irr::core::vector3df(0,150,0),
	irr::video::SColorf(1.0f, 1.0f, 1.0f, 1.0f), 1500.0f);
	light_1->setPosition(irr::core::vector3df(150,150,150));

	//material.setTexture(0, driver->getTexture("../../texture/green.jpg"));
												//Set up cusor input for the device
	
	printf("Initialize environment background finished\n");

	/* Create Vehicle */
	

	HotDrone* hotdrone1 = new HotDrone(1);
	//HotDec* hotdec1 = new HotDec(100);

	

	hotdrone1->construct_3dmodel(smgr);
	//hotdec1->construct_3dmodel(smgr);


	hotdrone1->arm();
	//hotdec1->arm();
	//hotdrone1->deploy(1, 1);

	float time_past = 0;
	clock_t start_t, cur_t;
	start_t = cur_t = clock();
	
	int lastFPS = -1;
	while (device->run()) 
	{

		/*Algirithm run here */
		if (time_past <= 6) {
			//hotdec1->moveTo(2, 2);
			//hotdrone1->set_manual_throttle(0.5);
			
			//hotdrone1->moveTo(0,2);
			hotdrone1->riseTo(5);
		} else {
			//hotdrone1->setPitch(5);
			//hotdrone1->set_manual_throttle(-0.5);
			//hotdrone1->setYaw(10);
			hotdrone1->moveTo(2,2);
		}

		/*--End of Algorithm--*/



		/* Paint scene */

		driver->beginScene(true, true, irr::video::SColor(255,113,113,133));

		smgr->drawAll(); // draw the 3d scene
		device->getGUIEnvironment()->drawAll(); // draw the gui environment (the logo)
			
		driver->endScene();

		//printf("Scene painted...\n");

		int fps = driver->getFPS();

		if (lastFPS != fps)
		{
				irr::core::stringw tmp(L"HotDec Simulation - Irrlicht Engine [");
				tmp += driver->getName();
				tmp += L"] fps: ";
				tmp += fps;
				tmp += " Position: x=";
				//tmp += (HotDec_body->getAbsolutePosition().X+100)/100;
				tmp += "m, y=";
				//tmp += (HotDec_body->getAbsolutePosition().Z+100)/100;
				tmp += "m; Rotation=";
				//tmp += (out[0][2]/2/PI*360);
				tmp += "degree; ";
				device->setWindowCaption(tmp.c_str());
				lastFPS = fps;
		}	

		//Sleep(ENVIRON_PAINT_SCENE_PERIOD);
		cur_t = clock();
		time_past = float(cur_t - start_t)*TIME_SCALE;
	}

	

	device->drop();
	
	delete hotdrone1;
	//delete hotdec1;

	return 0;
} //end of main

	








































	














