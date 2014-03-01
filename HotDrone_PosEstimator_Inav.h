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

#define HOTDRONE_POSESTIMATOR_INAV_W_ALT_BARO 0.5
#define HOTDRONE_POSESTIMATOR_INAV_W_ALT_ACC 50.0
#define HOTDRONE_POSESTIMATOR_INAV_W_ALT_SONAR 3.0
#define HOTDRONE_POSESTIMATOR_INAV_W_POS_GPS_P 1.0
#define HOTDRONE_POSESTIMATOR_INAV_W_POS_GPS_V 2.0
#define HOTDRONE_POSESTIMATOR_INAV_W_POS_ACC 10.0
#define HOTDRONE_POSESTIMATOR_INAV_W_POS_FLOW 0.0
#define HOTDRONE_POSESTIMATOR_INAV_W_ACC_BIAS 0.0
#define HOTDRONE_POSESTIMATOR_INAV_FLOW_K 1.0
#define HOTDRONE_POSESTIMATOR_INAV_SONAR_FILT 0.02
#define HOTDRONE_POSESTIMATOR_INAV_SONAR_ERR 0.5
#define HOTDRONE_POSESTIMATOR_INAV_LAND_T 3.0
#define HOTDRONE_POSESTIMATOR_INAV_LAND_DISP 0.7
#define HOTDRONE_POSESTIMATOR_INAV_LAND_THR 0.3


#ifndef _HOTDRONE_POSESTIMATOR_INAV_H_
#define _HOTDRONE_POSESTIMATOR_INAV_H_

class HotDrone_PosEstimator_Inav {
public:
	HotDrone_PosEstimator_Inav();
	~HotDrone_PosEstimator_Inav();


private:

	/* Parameters */
	float _w_alt_baro;
	float _w_alt_acc;
	float _w_alt_sonar;
	float _w_pos_gps_p;
	float _w_pos_gps_v;
	float _w_pos_acc;
	float _w_pos_flow;
	float _w_acc_bias;
	float _flow_k;
	float _sonar_filt;
	float _sonar_err;
	float _land_t;
	float _land_disp;
	float _land_thr;

};


#endif