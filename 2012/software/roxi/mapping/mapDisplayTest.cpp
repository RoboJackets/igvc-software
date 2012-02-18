#include <math.h>
#include <stdio.h>
#include "mapDisplay.hpp"


int main()
{
	CvSize size;
	size.height=400;
	size.width=400;
	CvImage image=CvImage(size,1,2);
	mapDisplay display=mapDisplay(50,50,&image);
	display.addPointToDraw(50,50);
	display.addPointToDraw(100,100);
	display.draw(&image);
	return 0;
}
