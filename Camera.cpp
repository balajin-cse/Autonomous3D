#include <stdio.h>
#include "Camera.h"


Camera::Camera()
{

}

Camera::~Camera()
{


}

void 
	Camera::update(float x, float y, vehicle_local_pos_t &local_pos)
{
	local_pos.x = x; // stub 
	local_pos.y = y; // stub
}


void 
	Camera::update(float x, float y, float theta, vehicle_local_pos_t &local_pos, 
												hotdec_att_t &att)
{
	local_pos.x = x; // stub 
	local_pos.y = y; // stub
	att.theta = theta;
	printf("theta is :%0.3f\n", att.theta*3.14159/180);
}