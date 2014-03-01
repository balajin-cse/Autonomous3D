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

#ifndef _HOTDRONE_3DMODEL_H_
#define _HOTDRONE_3DMODEL_H_


#include <irrlicht.h>

class HotDrone_3dModel {
public:


private:

	irr::scene::ISceneNode* hotdrone_node;
	irr::scene::IAnimatedMeshSceneNode* prop1_node;	  //front-left
	irr::scene::IAnimatedMeshSceneNode* prop2_node;	  //front-right
	irr::scene::IAnimatedMeshSceneNode* prop3_node;	  //rear-left
	irr::scene::IAnimatedMeshSceneNode* prop4_node;	  //rear-right



};


#endif