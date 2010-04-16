#include "quadCoderDriver.hpp"

volatile bool reset = false;

void handler(int signum)
{
	reset = true;
}


int main()
{

	signal(SIGINT, handler);

	quadCoderDriver qD;
	qD.resetCount(ENCODER_IF_FOR_BOARD);
	size_t t0 = time(NULL);
	new_encoder_pk_t pk;
	for(;;)
	//while((time(NULL) - t0) < 10)
	{
/*
		if(!qD.getEncoderState(pk))
		{
			std::cout << pk << std::endl;
		}
*/
		double fr,fl, br,bl;
		if(!qD.getEncoderVel(fr,fl,br,bl))
		{
			std::cout << "r: " << fr << "l: " << fl << std::endl;
		}

		if(reset)
		{
			reset = false;
			qD.resetCount(ENCODER_IF_FOR_BOARD);
		}

		usleep(1e5);
	}
	//if(!qD.getEncoderState(pk))
	//{
	//	std::cout << pk << std::endl;
	//}

}
