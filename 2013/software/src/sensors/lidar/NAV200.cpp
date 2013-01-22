#include "NAV200.h"

#include <iostream>
#include <string>

namespace IGVC {
namespace Sensors {

NAV200::NAV200()
{
    std::cout << "Lidar lives!" << std::endl;

    serialPort("/dev/HemiGPS", 9600),
	//iothread(boost::bind( &HemisphereA100GPS::threadRun, this)),
}

void NAV200::threadRun()
{
	/*while(serialPort.isConnected()) {
		std::string line = serialPort.readln();
		GPSState state;
		if(parseLine(line, state)) {
			gettimeofday(&state.laptoptime, NULL);

			boost::mutex::scoped_lock lock(queueLocker);
			stateQueue.push_back(state);
			if(stateQueue.size() > maxBufferLength) {
				stateQueue.pop_front();
			}
			fireEvent(&GPSListener::onNewStateAvailable, (void*)&state);
		}
	}*/
}

NAV200::~NAV200()
{
    //dtor
}

} /* namespace Sensors */
} /* namespace IGVC */
