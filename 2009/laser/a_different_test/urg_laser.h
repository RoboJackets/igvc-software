#ifndef URG_LASER_H
#define URG_LASER_H

#include <string.h>
#include <sys/termios.h>
#include <cstdio>

class UrgLaser
{
	FILE* dev;
	int maxrange;
	int minangle;
	int maxangle;
	int maxstep;

	speed_t bitrate(int aBR);
	int setupSerial(int aFd,int aSpeed);
	FILE* openSerial(char * aDevName,int aSpeed);

	int startSensor(FILE* aPort);
	int urgMakeScan(FILE* aPort, int aStart, int aEnd, int aSkip);
	int stopSensor(FILE* aPort);

public:
	int data[769]; // variable containing scan data

	UrgLaser();
	~UrgLaser();
	int makeScan(int aStart,int aEnd, int aSkip)
	{
		return urgMakeScan(dev, aStart, aEnd, aSkip);
	};
	void initialize(char* aDevName="/dev/ttyACM0",int aSpeed=19200);
	void closeSerial();

	int getMaxRange()
	{
		return maxrange;
	};
	int getMinAngle()
	{
		return minangle;
	};
	int getMaxAngle()
	{
		return maxangle;
	};
	int getMaxStep()
	{
		return maxstep;
	};

};


#endif
