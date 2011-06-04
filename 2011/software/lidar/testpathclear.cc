//g++ lidartestgui.cc -g3 -O0 -lcv -lhighgui -lNAV200

#include <iostream>
#include <limits>

#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <boost/thread.hpp>

#include "NAV200.hpp"
#include "lidarProc.hpp"
#include "finiteDiff.hpp"

//#include "plConfig.h"
#include "plplot/plplot.h"

int main()
{
	NAV200 lidar;
	for(;;)
	{
		if(lidar.read())
		{
			// Get the valid raw points in polar
			float goodt[NAV200::Num_Points];
			float goodr[NAV200::Num_Points];
			size_t numgoodpts = lidar.getValidData(goodt,goodr);

			std::cout << "Path in front clear: ";
			bool clear = lidarProc::isPathClear(M_PI/2.0, 1, .1, goodt, goodr, numgoodpts);
			std::cout << clear << "\n";
		}		
	}
}
