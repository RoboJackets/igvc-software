#include "quadCoderDriver_signed.hpp"

volatile bool reset = false;

void handler(int signum)
{
	reset = true;
}


int main()
{
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
