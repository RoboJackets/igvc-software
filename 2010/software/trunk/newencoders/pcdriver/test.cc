#include "quadCoderDriver.hpp"

int main()
{
	quadCoderDriver qD;

	for(;;)
	{
		std::cout << qD.getEncoderState() << std::endl;
		usleep(1e5);
	}

}
