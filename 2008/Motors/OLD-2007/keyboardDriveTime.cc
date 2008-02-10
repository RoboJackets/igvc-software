#include <iostream>
#include "libplayerc++/playerc++.h"

long long currentTimeMillis() {
   long long t;
   struct timeval tv;

   gettimeofday(&tv, (struct timezone*)NULL);

   t = tv.tv_sec;
   t = (t *1000) + (tv.tv_usec/1000);

   return t;
}

int main(int argc, char *argv[])
{
	using namespace PlayerCc;

	PlayerClient	robot("localhost");
	printf("connected to robot\n");
	Position2dProxy	pp(&robot,0);
	printf("connected to motors\n");

	for(;;)
	{
		double dLeftVelocity, dRightVelocity;
		long long gotime,sttime;
		std::cout << std::endl << "Left Velocity: ";
		std::cin >> dLeftVelocity;
		std::cout << "Right Velocity: ";
		std::cin >> dRightVelocity;
		std::cout << "Time in Millis: ";
		std::cin >> gotime;
		sttime=currentTimeMillis();
		pp.SetSpeed(dLeftVelocity, dRightVelocity);
		
		printf("%lld\n%lld\n%lld\n",sttime,gotime,(currentTimeMillis()-sttime));
		while((currentTimeMillis()-sttime)<gotime){usleep(1);}
		printf("ended:%lld\n%lld\n%lld\n",sttime,currentTimeMillis(),(currentTimeMillis()-sttime));
		pp.SetSpeed(0, 0);
		
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
		pp.SetSpeed(0, 0);
	}
}
