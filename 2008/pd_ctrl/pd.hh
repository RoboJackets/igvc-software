#ifndef PD_HH
#define PD_HH

#include "misc.hh"
//#include "arduino_comm.h"

#define TRUE 1
#define FALSE 0

typedef struct {
	public:
		float Pgain;
		float Dgain;
	
		struct timeval t1,t2;

		float preverr;
		
		float current_vel;
		float target_vel;

		bool isRight;
} PDstatus;

//float getError(PDstatus * pd_state){
//	return(pd_state->target_vel - pd_state->current_vel);
//}

float newPD(PDstatus * pd_state){
	gettimeofday(&pd_state->t2, NULL);

	float err = pd_state->target_vel - pd_state->current_vel;

	double dt = calcdt(pd_state->t1, pd_state->t2);

	if(dt == 0){
		return(pd_state->current_vel);
	}

	float P = pd_state->Pgain * err;

	float D = pd_state->Dgain * (err - pd_state->preverr) / (dt);
	pd_state->preverr = err;

	pd_state->current_vel += (P + D);//update signal
	return(pd_state->current_vel);
}

float getLinVel_L(PDstatus * pd_state){
	return(pd_state->current_vel);//replace with code to find real vel
}

float getLinVel_R(PDstatus * pd_state){
	return(pd_state->current_vel);//replace with code to find real vel
}


void PDupdateLin(PDstatus * pd_state){
	//float error, newsignal;

	if(pd_state->isRight){
		getLinVel_R(pd_state);
	}
	else{
		getLinVel_L(pd_state);
	}

	newPD(pd_state);
	gettimeofday(&pd_state->t1, NULL);
}

int updateMotors(PDstatus * pd_left, PDstatus * pd_right){
	return(1);
}


#endif
