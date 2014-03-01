#ifndef _HOTDEC_KF_LSH_H_
#define _HOTDEC_KF_LSH_H_

#include "HotDec_Structs.h"
#include "Vehicle_Structs.h"

//input u(1~6)			={Fx,Fy,T,X,Y,theta}
//output east(1~6)		={Xdot,X,Ydot,Y,thetadot,theta}
void KF_lsh(float *u, float *dot_hat);

class HotDec_Estimator {
public:
	HotDec_Estimator();
	~HotDec_Estimator();

	void update(float *u, float *dot_hat);
	void update(const vehicle_local_pos_t local_pos, const hotdec_att_t att, const float* global_force, 
								 hotdec_plant_state_t &estimated_state);

private:
	float _x_hat[6];
	float _x_hat_next[6];

};

#endif