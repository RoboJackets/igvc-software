#ifndef PD_HH
#define PD_HH

#include "misc.hh"
#include "arduino_comm.h"
#include "arduino_readnum.h"
#include <cmath>

#define TRUE 1
#define FALSE 0

#define COUNTER_SCALER (64)
#define F_CPU (16000000)
#define COUNTER_RATE (F_CPU/COUNTER_SCALER)
#define RAD_ENCODERTICK ( (2*M_PI)/512 )

typedef struct {
	public:
		float Pgain;
		float Dgain;
	
		struct timeval t1,t2;

		float preverr;
		
		float current_vel;
		float target_vel;

		bool isRight;

		int arduino_fd;
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

float getVel(int arduino_fd){
	unsigned short int dt;
	short int dp;
	float velocity;

	//char buffer[6] = {0};
	char v[2] = "v";
	char t[2] = "t";
	char p[2] = "p";

	writeFully(arduino_fd, v, 1);
//	dt = readUint16(arduino_fd, t, buffer);
//	dp = readSint16(arduino_fd, p, buffer);
	dt = readUint16(arduino_fd, t);
	dp = readSint16(arduino_fd, p);

	velocity = ( (float)dp) / ( (float)dt) * RAD_ENCODERTICK * COUNTER_RATE;

	return(velocity);

	//printf("dp: %d\tdt: %d\tvel: %d\n", dp, dt, velocity);
}
/*
float getLinVel_L(PDstatus * pd_state){

getVel(PDstatus);
	return(pd_state->current_vel);//replace with code to find real vel
}

float getLinVel_R(PDstatus * pd_state){
	return(pd_state->current_vel);//replace with code to find real vel
}
*/

void PDupdateLin(PDstatus * pd_state){
	//float error, newsignal;
/*
	if(pd_state->isRight){
		getLinVel_R(pd_state);
	}
	else{
		getLinVel_L(pd_state);
	}
*/

	pd_state->current_vel = getVel(pd_state->arduino_fd);

	newPD(pd_state);
	gettimeofday(&pd_state->t1, NULL);
}

int updateMotors(PDstatus * pd_left, PDstatus * pd_right){
	return(1);
}


#endif
