#include "quadCoderDriver_signed.hpp"

volatile bool reset = false;

void handler(int signum)
{
	reset = true;
}


int main()
{
	signal(SIGINT, handler);
	quadCoderDriver_signed qD(ENCODER_IF_AFT_BOARD);
	qD.resetCount();
	size_t t0 = time(NULL);

	double vel_l;
	double vel_r;
	double dist_l;
	double dist_r;
	for(;;)
	{
		if(!qD.getEncoderVel(vel_l, vel_r))
		{
			std::cout << "velocity of left: " << vel_l << std::endl;
			std::cout << "velocity of right: " << vel_r <<std::endl;
		}


		if(!qD.getEncoderDist(dist_l, dist_r))
		{
			std::cout << "distance of left: " << dist_l <<std::endl;
			std::cout << "distance of right: " << dist_r<<std::endl;
		}

		if(reset)
		{
			reset = false;
			qD.resetCount();
		}

		usleep(1e5);
	}
	

	// This testing exectuable has been disabled because the definitions in quadCoderDriver_signed have changed
	// Take a look at the new object, and rewrite correctly to test different functions
	/*signal(SIGINT, handler);
	quadCoderDriver_signed qD(ENCODER_IF_AFT_RIGHT_BOARD);
	qD.resetCount();
	size_t t0 = time(NULL);
	new_encoder_single_pk_t pk;
	for(;;)*/
	//while((time(NULL) - t0) < 10)
	//{
/*
		if(!qD.getEncoderState(pk))
		{
			std::cout << pk << std::endl;
		}
*/
		/*double v;
		if(!qD.getEncoderVel(v))
		{
			std::cout << "v: " << v << std::endl;
		}

		if(reset)
		{
			reset = false;
			qD.resetCount();
		}

		usleep(1e5);
	}
	if(!qD.getEncoderState(pk))
	{
		std::cout << pk << std::endl;
	}*/

}
