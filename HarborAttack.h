#ifndef _HARBORATTACK_H_
#define _HARBORATTACK_H_

namespace HarborAttack {

	#define D_Hov_Mesh  44.83

	void PathLine(float timer, float p2p_freq, float *TargetPos, float *StartPos,float *Ref);
	void Stay(float *StayPos,float *Ref);


	int CheckPreReach(float *TargetPos, float *CurrentPos);

	float ReturnDist(float *TargetPos, float *CurrentPos);

	int CheckHovCollision(float *Hov1Pos, float* Hov2Pos);
}

#endif