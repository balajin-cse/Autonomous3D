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

#define HOTDRONE_ATTESTIMATOR_EKF_Q0 1e-4f
#define HOTDRONE_ATTESTIMATOR_EKF_Q1 0.08f
#define HOTDRONE_ATTESTIMATOR_EKF_Q2 0.009f
#define HOTDRONE_ATTESTIMATOR_EKF_Q3 0.005f
#define HOTDRONE_ATTESTIMATOR_EKF_Q4 0.0f
#define HOTDRONE_ATTESTIMATOR_EKF_R0 0.0008f
#define HOTDRONE_ATTESTIMATOR_EKF_R1 10000.0f
#define HOTDRONE_ATTESTIMATOR_EKF_R2 1.0f
#define HOTDRONE_ATTESTIMATOR_EKF_R3 0.0f

#define HOTDRONE_ATTESTIMATOR_EKF_ROLL_OFF 0.0f
#define HOTDRONE_ATTESTIMATOR_EKF_PITCH_OFF 0.0f
#define HOTDRONE_ATTESTIMATOR_EKF_YAW_OFF 0.0f


#ifndef _HOTDRONE_ATTESTIMATOR_EKF_H_
#define _HOTDRONE_ATTESTIMATOR_EKF_H_

class HotDrone_AttEstimator_ekf {
public:
	HotDrone_AttEstimator_ekf();
	~HotDrone_AttEstimator_ekf();

private:
	float _q0;
	float _q1;
	float _q2;
	float _q3;
	float _q4;

	float _r0;
	float _r1;
	float _r2;
	float _r3;

	float _roll_off;
	float _pitch_off;
	float _yaw_off;

};

#endif


