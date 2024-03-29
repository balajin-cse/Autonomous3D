#include "HotDrone_PosEstimator_Inav.h"




HotDrone_PosEstimator_Inav::HotDrone_PosEstimator_Inav() {
	_w_alt_baro = HOTDRONE_POSESTIMATOR_INAV_W_ALT_BARO;
	_w_alt_acc = HOTDRONE_POSESTIMATOR_INAV_W_ALT_ACC;
	_w_alt_sonar = HOTDRONE_POSESTIMATOR_INAV_W_ALT_SONAR;
	_w_pos_gps_p = HOTDRONE_POSESTIMATOR_INAV_W_POS_GPS_P;
	_w_pos_gps_v = HOTDRONE_POSESTIMATOR_INAV_W_POS_GPS_V;
	_w_pos_acc = HOTDRONE_POSESTIMATOR_INAV_W_POS_ACC;
	_w_pos_flow = HOTDRONE_POSESTIMATOR_INAV_W_POS_FLOW;
	_w_acc_bias = HOTDRONE_POSESTIMATOR_INAV_W_ACC_BIAS;
	_flow_k = HOTDRONE_POSESTIMATOR_INAV_FLOW_K;
	_sonar_filt = HOTDRONE_POSESTIMATOR_INAV_SONAR_FILT;
	_sonar_err = HOTDRONE_POSESTIMATOR_INAV_SONAR_ERR;
	_land_t = HOTDRONE_POSESTIMATOR_INAV_LAND_T;
	_land_disp = HOTDRONE_POSESTIMATOR_INAV_LAND_DISP;
	_land_thr = HOTDRONE_POSESTIMATOR_INAV_LAND_THR;

}