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

#define TEST_OP 1

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

			#if TEST_OP == 0
			std::cout << "Path in front clear: ";
			bool clear = lidarProc::isPathClear(M_PI/2.0, 1, .1, goodt, goodr, numgoodpts); 
			std::cout << clear << "\n";
			
			#elif TEST_OP == 1
			float* cost;
			cost = lidarProc::getSectorCost(1, goodt, goodr, numgoodpts);
			for (int i = 0; i < 20; i++)
			{
				std::cout << "From << " << i*9 << " to " << i*9+9 << " degrees, cost is " << cost[i] << "\n";
			}
			#endif
		}		
	}
}
