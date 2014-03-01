
#include <math.h>
#include "HarborAttack.h"


void HarborAttack::PathLine(float timer, float p2p_freq, float *TargetPos, float *StartPos,float *Ref){//Generate Ref input from point to point
	 

	Ref[0]=((TargetPos[0]-StartPos[0])/2)*p2p_freq*sin(p2p_freq*timer); // x_dot
	Ref[1]=-((TargetPos[0]-StartPos[0])/2)*cos(p2p_freq*timer)+(TargetPos[0]+StartPos[0])/2;// x
	Ref[2]=((TargetPos[1]-StartPos[1])/2)*p2p_freq*sin(p2p_freq*timer);// y_dot
	Ref[3]=-((TargetPos[1]-StartPos[1])/2)*cos(p2p_freq*timer)+(TargetPos[1]+StartPos[1])/2;//y
	Ref[4]=0;
	Ref[5]=StartPos[2];


}

void HarborAttack::Stay(float *StayPos,float *Ref){

	Ref[0]=0;
	Ref[1]=StayPos[0];
	Ref[2]=0;
	Ref[3]=StayPos[1];
	Ref[4]=0;
	Ref[5]=StayPos[2];

}


int HarborAttack::CheckPreReach(float *TargetPos, float *CurrentPos){//Return 1 if reached

	float dist= sqrt(pow(TargetPos[0]-CurrentPos[0],2)+pow(TargetPos[1]-CurrentPos[1],2));
	//dist_debug=dist;
	//if(dist<= 0.05) return 1;
	if(dist<= 0.15) return 1;	
	else return 0;

}

float HarborAttack::ReturnDist(float *TargetPos, float *CurrentPos){//Return 1 if reached

	float dist= sqrt(pow(TargetPos[0]-CurrentPos[0],2)+pow(TargetPos[1]-CurrentPos[1],2));
	
	//if(dist<= 0.05) return 1;
	return dist;	
}



int HarborAttack::CheckHovCollision(float *Hov1Pos, float* Hov2Pos){

	float dist = sqrt(pow(Hov1Pos[0]-Hov2Pos[0],2)+pow(Hov1Pos[1]-Hov2Pos[1],2));
	if(dist<= D_Hov_Mesh) return 1;
	else return 0;
}