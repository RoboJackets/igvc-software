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

	for(;;)
	{
		
		new_encoder_pk_t pk;
		if(!qD.getEncoderState(pk))
		{
			std::cout << pk << std::endl;
		}
		

		if(reset)
		{
			reset = false;
			qD.resetCount();
		}

		usleep(1e5);
	}

}
