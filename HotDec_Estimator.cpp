#include "HotDec_Estimator.h"
#include <math.h>

//extern float InitialPos[2][6];float InitialPos[2][6]={{0,2.0470+x_3d_offset,0,0.4167,0,0},{0,4.8000+x_3d_offset,0,0.4167,0,0}};   //[0][.] evader:left hotdec , [1] defender
//float evader_pos[16][3]={ { 2.0470+x_3d_offset,0.4167,0}, {2.0470+x_3d_offset,0.6771,0}, {2.1533+x_3d_offset,1.0082,0}, { 2.3651+x_3d_offset,1.2961,0},{ 2.6053+x_3d_offset ,1.6109,0},{ 2.9922+x_3d_offset ,1.7999,0},{3.2578+x_3d_offset,2.0107,0},{ 3.5230+x_3d_offset, 2.2184,0},{3.6073+x_3d_offset , 2.5025,0},{3.6120+x_3d_offset , 2.5472,0},{3.5281+x_3d_offset ,2.8074,0},{3.3699+x_3d_offset , 3.0580,0},{ 3.1619+x_3d_offset , 3.2747,0},{ 2.9181+x_3d_offset,  3.3726,0},{ 2.6435+x_3d_offset , 3.4499,0},{ 2.3430+x_3d_offset, 3.6057,0} };


HotDec_Estimator::HotDec_Estimator()
{
	for (int i =0; i<6; i++) {
		_x_hat[i] = 0;
		_x_hat_next[i] = 0;
	}
}

HotDec_Estimator::~HotDec_Estimator()
{


}

void
	HotDec_Estimator::update(const vehicle_local_pos_t local_pos, const hotdec_att_t att, const float* global_force, 
								 hotdec_plant_state_t &estimated_state)
{
	float tmp1[6]={0,0,0,0,0,0};
	float tmp2[6]={0,0,0,0,0,0};
	float tmp3[9]={0,0,0,0,0,0,0,0,0};
	float tmp4[9]={0,0,0,0,0,0,0,0,0};

//10Hz, Large HotDeC
/*
    tmp1[0]= 0.9954*x_hat[0]-0.2827*x_hat[1];
    tmp1[1]= 0.09977*x_hat[0] +  0.7475*x_hat[1];
    tmp1[2]= 0.9954*x_hat[2]-0.3152*x_hat[3];
    tmp1[3]=0.09977*x_hat[2] +  0.7326 *x_hat[3];
    tmp1[4]=0.9464*x_hat[4]  -0.7687*x_hat[5];
    tmp1[5]=0.0973*x_hat[4] + 0.5586*x_hat[5];

    //B_disc=[B G]_original * u
    tmp2[0]= 0.03563*u[0] + 0.2827*local_pos.x;
    tmp2[1]= 0.001783*u[0] + 0.2525*local_pos.x;
    tmp2[2]= 0.3563*u[1] + 0.3152*local_pos.y;
    tmp2[3]= 0.001783*u[1] + 0.2674*local_pos.y;
    tmp2[4]= 1.072*u[2] + 0.7687*att.theta;
    tmp2[5]=0.05407*u[2] +   0.4414*att.theta;

    //c*x_hat
    tmp3[0]=0.7759*x_hat[1];
    tmp3[1]=0.7642*x_hat[3];
    tmp3[2]=0.6376*x_hat[5];
    tmp3[3]=x_hat[0]  -0.284*x_hat[1];
    tmp3[4]=0.7759*x_hat[1];
    tmp3[5]=x_hat[2]-0.3167*x_hat[3];
    tmp3[6]=  0.7642*x_hat[3];
    tmp3[7]=x_hat[4] -0.8122*x_hat[5];
    tmp3[8]= 0.6376*x_hat[5];

    //D*U        
    tmp4[0]= 0.2241*local_pos.x; 
    tmp4[1]= 0.2358*local_pos.y;
    tmp4[2]= 0.3624*att.theta;
    tmp4[3]= 0.2844*local_pos.x;
    tmp4[4]= 0.2241*local_pos.x;
    tmp4[5]= 0.3167*local_pos.y;
    tmp4[6]= 0.2358*local_pos.y;
    tmp4[7]= 0.8122*att.theta;
    tmp4[8]= 0.3624*att.theta;   
*/
// small HoTDec (Foam)
/*
    tmp1[0]= 0.9929*x_hat[0]-0.4175*x_hat[1];
   tmp1[1]= 0.09964*x_hat[0] +  0.6891*x_hat[1];
    tmp1[2]= 0.9929*x_hat[2]-0.4648*x_hat[3];
    tmp1[3]=0.09964*x_hat[2] +  0.6708 *x_hat[3];
    tmp1[4]=0.7881*x_hat[4]  -1.806*x_hat[5];
    tmp1[5]=0.0889*x_hat[4] + 0.2347*x_hat[5];

    //B_disc=[B G]_original * u
    tmp2[0]= 0.05475*u[0] + 0.4175*local_pos.x;
    tmp2[1]= 0.002741*u[0] + 0.3109*local_pos.x;
    tmp2[2]= 0.05475*u[1] + 0.4648*local_pos.y;
    tmp2[3]= 0.002741*u[1] + 0.3292*local_pos.y;
    tmp2[4]= 4.237*u[2] + 1.806*att.theta;
    tmp2[5]=0.2203*u[2] +   0.7653*att.theta;

    //c*x_hat
    tmp3[0]=0.731*x_hat[1];
    tmp3[1]=0.7174*x_hat[3];
    tmp3[2]=0.4386*x_hat[5];
    tmp3[3]=x_hat[0]  -0.4205*x_hat[1];
    tmp3[4]=0.731*x_hat[1];
    tmp3[5]=x_hat[2]-0.4682*x_hat[3];
    tmp3[6]=  0.7174*x_hat[3];
    tmp3[7]=x_hat[4] -2.292*x_hat[5];
    tmp3[8]= 0.4386*x_hat[5];
    //D*U        
    tmp4[0]= 0.269*local_pos.x;
    tmp4[1]= 0.2826*local_pos.y;
    tmp4[2]= 0.5614*att.theta;
    tmp4[3]= 0.4205*local_pos.x;
    tmp4[4]= 0.269*local_pos.x;
    tmp4[5]= 0.4682*local_pos.y;
    tmp4[6]= 0.2826*local_pos.y;
    tmp4[7]= 2.292*att.theta;
    tmp4[8]= 0.5614*u[5];
*/ 
/// small HotDeC (RP)
    tmp1[0]= 0.9941*_x_hat[0]-0.355*_x_hat[1];
    tmp1[1]= 0.0997*_x_hat[0] +  0.715*_x_hat[1];
    tmp1[2]= 0.9949*_x_hat[2]-0.3955*_x_hat[3];
    tmp1[3]=0.0997*_x_hat[2] +  0.6982 *_x_hat[3];
    tmp1[4]=0.8365*_x_hat[4]  -1.597*_x_hat[5];
    tmp1[5]=0.09158*_x_hat[4] + 0.3033*_x_hat[5];

    //B_disc=[B G]_original * u
    tmp2[0]= 0.04574*global_force[0] + 0.355*local_pos.x;
    tmp2[1]= 0.002298*global_force[0] + 0.285*local_pos.x;
    tmp2[2]= 0.04574*global_force[1] + 0.3955*local_pos.y;
    tmp2[3]= 0.002298*global_force[1] + 0.3018*local_pos.y;
    tmp2[4]= 3.271*global_force[2] + 1.597*att.theta;
    tmp2[5]=0.1684*global_force[2] +   0.6967*att.theta;

    //c*_x_hat
    tmp3[0]=0.7506*_x_hat[1];
    tmp3[1]=0.7378*_x_hat[3];
    tmp3[2]=0.4781*_x_hat[5];
    tmp3[3]=_x_hat[0]  -0.3571*_x_hat[1];
    tmp3[4]=0.7506*_x_hat[1];
    tmp3[5]=_x_hat[2]-0.3978*_x_hat[3];
    tmp3[6]=  0.7378*_x_hat[3];
    tmp3[7]=_x_hat[4] -1.909*_x_hat[5];
    tmp3[8]= 0.4781*_x_hat[5];
    //D*U        
    tmp4[0]= 0.2494*local_pos.x;
    tmp4[1]= 0.2622*local_pos.y;
    tmp4[2]= 0.5219*att.theta;
    tmp4[3]= 0.3571*local_pos.x;
    tmp4[4]= 0.2494*local_pos.x;
    tmp4[5]= 0.3978*local_pos.y;
    tmp4[6]= 0.2622*local_pos.y;
    tmp4[7]= 1.909*att.theta;
    tmp4[8]= 0.5219*att.theta;

	///////////////////
	int i=0;
	for(i=0;i<6;i++)
	{_x_hat_next[i]=tmp1[i]+tmp2[i];}

	estimated_state.vx=tmp3[3]+tmp4[3];  //x dot
	estimated_state.x=tmp3[4]+tmp4[4];  //x 
	estimated_state.vy=tmp3[5]+tmp4[5];   //y_dot
	estimated_state.y=tmp3[6]+tmp4[6];   //y
	estimated_state.theta_rate=tmp3[7]+tmp4[7];   //theta_dot
	estimated_state.theta=tmp3[8]+tmp4[8];   //theta

	//printf("dot_hat = %f,  %f, %f, %f, %f, %f \n", dot_hat[0],dot_hat[1],dot_hat[2],dot_hat[3],dot_hat[4],dot_hat[5] );

	_x_hat[0]=_x_hat_next[0];
	_x_hat[1]=_x_hat_next[1];
	_x_hat[2]=_x_hat_next[2];
	_x_hat[3]=_x_hat_next[3];
	_x_hat[4]=_x_hat_next[4];
	_x_hat[5]=_x_hat_next[5];     
	//printf("_x_hat   = %f , %f, %f, %f, %f, %f \n", _x_hat[0],_x_hat[1],_x_hat[2],_x_hat[3],_x_hat[4],_x_hat[5] );
	


}







