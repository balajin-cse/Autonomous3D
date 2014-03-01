#ifndef _HOTDEC_PLANT_H_
#define _HOTDEC_PLANT_H_

#include <time.h>

#include "HotDec_Structs.h"

#define HOTDEC_HC_M 1.829	  //Mass
#define HOTDEC_HC_J 0.021	  //Moment of Inertia
#define HOTDEC_HC_BT 0.13
#define HOTDEC_HC_BR 0.05


//Input u[1-3]: global forces Fx,Fy and Torque
//Output y[1-3] globale coordinates X,Y and Thelta


class HotDec_Plant {
public:
	HotDec_Plant();
	~HotDec_Plant();
	float get_plant_theta() { return _X.theta;}
	float get_plant_x() {return _X.x;}
	float get_plant_y() {return _X.y;}
	hotdec_plant_state_t get_plant_state() {return _X;}
	void update(const float *u);

private:
	clock_t _last_t;
	float _time_past; //in seconds

	void reset_plant_state(hotdec_plant_state_t &X);
	void zero_plant_state(hotdec_plant_state_t &X);


	float _hc_m;
	float _hc_j;
	float _hc_bt;
	float _hc_br;

	hotdec_plant_state_t _X;
	hotdec_plant_state_t _X_i;
	hotdec_plant_state_t _X_old;

	
};


#endif
