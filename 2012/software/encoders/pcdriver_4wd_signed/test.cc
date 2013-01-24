#include "quadCoderDriver_4wd_signed.hpp"
#include <signal.h>
volatile bool reset = false;

void handler(int signum)
{
	reset = true;
}


int main()
{

	signal(SIGINT, handler);

	quadCoderDriver_4wd_signed qD;
	qD.resetCount();
	size_t t0 = time(NULL);
	new_encoder_pk_t pk;
	for(;;)
	//while((time(NULL) - t0) < 10)
	{
		double fr,fl, br,bl;
		if(!qD.getEncoderVel(fr,fl,br,bl))
		{
			std::cout << "fr: " << fr << " fl: " << fl << " br: " << br << " bl: " << bl << std::endl;
			std::cout << std::endl;
		}

		if(reset)
		{
			reset = false;
			qD.resetCount();
		}

		usleep(1e5);
	}
	//if(!qD.getEncoderState(pk))
	//{
	//	std::cout << pk << std::endl;
	//}

}
