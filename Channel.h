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

#ifndef _CHANNEL_H_
#define _CHANNEL_H_

#include "Vehicle.h"

#define CHANNEL_TYPE_WIFI 1
#define CHANNEL_TYPE_BLUETOOTH 2
#define CHANNEL_TYPE_ZIGBEE 3


template <class T>
class Channel {
public:
	Channel(int type, T alice, T bob);
	
	~Channel();
	
	int send_msg(const char* msg, T target);

	int eavsdrop_channel(T Eva);
	int spoof(const char* msg, T target);

	

private:
	int type; // channel type
	int distance; // distance between vehicle A and vehicle B
	int drop_rate; // message drop rate, directly related to vehicle distances
	int max_distance; // maximum distance of channel being able to communicate with each other
	T Alice; // channel initiator
	T Bob; // channel acceptor
	T Eva; // eavsdropper


};

#endif