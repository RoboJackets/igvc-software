#include "quadCoderDriver.hpp"

int main()
{
	quadCoderDriver qD;

	for(;;)
	{
		new_encoder_pk_t pk;
		if(!qD.getEncoderState(pk))
		{
			std::cout << pk << std::endl;
		}
		usleep(1e5);
	}

}
