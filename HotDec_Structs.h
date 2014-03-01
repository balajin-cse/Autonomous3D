#ifndef _HOTDEC_STRUCTS_H_
#define _HOTDEC_STRUCTS_H_

/*
 *
 */
struct hotdec_plant_state_t {
	float vx;
	float x;
	float vy;
	float y;
	float theta_rate;
	float theta;

	void init() {vx=x=vy=y=theta_rate=theta=0;}

};


struct hotdec_thrust_sp_t {
	float U[4];

	void init() {U[0]=U[1]=U[2]=U[3]=0;}
};


struct hotdec_state_sp_t {
	float x;
	float y;
	float theta;
	float vx;
	float vy;
	float theta_rate;

	void init() {x=y=theta=vx=vy=theta_rate=0;}


};


struct hotdec_att_t {
	float theta;
	float theta_rate;
	void init() {theta=theta_rate=0;}
};

struct hotdec_att_sp_t {
	float theta;
	void init() {theta=0;}
};





#endif