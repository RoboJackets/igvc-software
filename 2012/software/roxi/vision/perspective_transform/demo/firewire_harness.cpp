#include "GuppyCam.h"

GuppyCam camera_firewire;
int connectToCamera()
{
	if (!camera_firewire.connect())
	{
		printf("Camera connect failure \n");
		return 1;
	}
	else
	{
		camera_firewire.loadSettings();
		printf("Camera settings loaded \n");
	}
	return 0;
}


