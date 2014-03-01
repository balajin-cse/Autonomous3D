#include "Channel.h"

template <class T>
Channel<T>::Channel(int type, T alice, T bob)
{
	this->Alice = alice;
	this->Bob = bob;
	this->type = type;

	this->distance = 0; // stub: euclidean distance
	this->max_distance = 100; //stub: max distance based on channel type

}


template <class T>
int Channel<T>::send_msg(const char* msg, T target)
{
	// calculate A and B's distance
	// based on new distance, calculate drop rate
	// based on drop rate, determine if receiver should receive the msg;
	// 

	reutnr -1;// stub
}


