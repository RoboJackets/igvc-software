#include <cstdio>
//#include <cmath>
#include <unistd.h>

#include "pd.hh"
#include "misc.hh"
#include "reckon.hh"

#define P_GAIN_LIN_L .01
#define D_GAIN_LIN_L .001

#define P_GAIN_LIN_R .01
#define D_GAIN_LIN_R .001

#define INITIAL_R_VEL 0
//#define TARGET_R_VEL .3

#define INITIAL_L_VEL 0
#define TARGET_L_VEL 3


#define INITIAL_X 0
#define INITIAL_Y 0
#define INITIAL_HEADING (M_PI_2)

#define TARGET_X 10
#define TARGET_Y 20

int main(void){
	float ratio, v_left, v_right;

	struct timeval postime1, postime2, deadreckontimer1;

	ddrn * reckon = new(ddrn);
	PDstatus * pd_right = new(PDstatus);
	PDstatus * pd_left = new(PDstatus);


	reckon->x_0 = INITIAL_X;
	reckon->x = reckon->x_0;

	reckon->y_0 = INITIAL_Y;
	reckon->y = reckon->y_0;

	reckon->th_0 = INITIAL_HEADING;
	reckon->th = reckon->th_0;

	reckon->x_t = TARGET_X;
	reckon->y_t = TARGET_Y;

	reckon->t = 0;



	ratio = findSpeedRatio(reckon);



	pd_right->Pgain = P_GAIN_LIN_R;
	pd_right->Dgain = D_GAIN_LIN_R;
	pd_right->preverr = 0;
	pd_right->current_vel = INITIAL_R_VEL;
	pd_right->target_vel = TARGET_L_VEL * ratio;
	pd_right->isRight = TRUE;

	pd_left->Pgain = P_GAIN_LIN_L;
	pd_left->Dgain = D_GAIN_LIN_L;
	pd_left->preverr = 0;
	pd_left->current_vel = INITIAL_L_VEL;
	pd_left->target_vel = TARGET_L_VEL;
	pd_left->isRight = FALSE;

	gettimeofday(&reckon->t1, NULL);
	gettimeofday(&pd_right->t1, NULL);
	gettimeofday(&pd_left->t1, NULL);

	usleep(1000);
	printf("rv: %f\tlv: %f\n", pd_right->current_vel, pd_left->current_vel);

	while(1){
	//for(int i = 0; i < 1*10000; i++){
		PDupdateLin(pd_right);
		PDupdateLin(pd_left);

		updateMotors(pd_left, pd_right);

		//deadReckon(reckon, 1*ratio, 1);
		
		deadReckon(reckon, pd_right->current_vel, pd_left->current_vel);

		printf("t: %f\tdt: %f\tx: %f\ty: %f\tth: %f\trv: %f\tlv: %f\n", (float)reckon->t, (float)reckon->lastdt, (float)reckon->x, (float)reckon->y, (float)reckon->th, (float)pd_right->current_vel, (float)pd_left->current_vel);

		//printf("rv: %f\tlv: %f\n", pd_right->current_vel, pd_left->current_vel);
		//printf("%f\t%f\n",reckon->x, reckon->y);
		usleep(1000);
	}

	delete(pd_right);
	delete(pd_left);
	delete(reckon);


}
