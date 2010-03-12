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
	qD.resetCount();
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
		double r,l;
		if(!qD.getEncoderVel(r,l))
		{
			std::cout << "r: " << r << "l: " << l << std::endl;
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
	}

}
