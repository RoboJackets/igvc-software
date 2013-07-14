/*
 * HemisphereA100GPS.cpp
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#include "HemisphereA100GPS.h"
#include "nmea.hpp"
#include <string>

namespace IGVC {
namespace Sensors {

HemisphereA100GPS::HemisphereA100GPS():
    DefaultAccuracy(.0001, .0001, 3, 0.01),
    serialPort("/dev/ttyGPS", 4800),
	LonNewSerialLine(this),
	stateQueue()
{
    serialPort.onNewLine += &LonNewSerialLine;
	maxBufferLength = 10;
	serialPort.startEvents();
}

void HemisphereA100GPS::onNewSerialLine(string line) {
    GPSData state;
    if(parseLine(line, state)) {
        state.Accuracy(DefaultAccuracy);
        // TODO set time
//        gettimeofday(&state.laptoptime, NULL);

        boost::mutex::scoped_lock lock(queueLocker);
        stateQueue.push_back(state);
        if(stateQueue.size() > maxBufferLength) {
            stateQueue.pop_front();
        }
        onNewData(state);
    }
}

//void HemisphereA100GPS::threadRun() {
//	while(serialPort.isConnected()) {
//		std::string line = serialPort.readln();
//		GPSData state;
//		if(parseLine(line, state)) {
//			gettimeofday(&state.laptoptime, NULL);
//
//			boost::mutex::scoped_lock lock(queueLocker);
//			stateQueue.push_back(state);
//			if(stateQueue.size() > maxBufferLength) {
//				stateQueue.pop_front();
//			}
//			onNewData(state);
//		}
//	}
//}

bool HemisphereA100GPS::parseLine(std::string line, GPSData &state) {
	return nmea::decodeGPGGA(line, state) ||
		   nmea::decodeGPRMC(line, state);
}

GPSData HemisphereA100GPS::GetState() {
	boost::mutex::scoped_lock lock(queueLocker);
	GPSData state = stateQueue.back();
	stateQueue.remove(state);
	return state;
}

GPSData HemisphereA100GPS::GetStateAtTime(timeval time) {
	boost::mutex::scoped_lock lock(queueLocker);
	std::list<GPSData>::iterator iter = stateQueue.begin();
	double acceptableError = 0.1;
	while(iter != stateQueue.end()) {
		GPSData s = (*iter);
		/*time_t secDelta = difftime(time.tv_sec, s.laptoptime.tv_sec);
		suseconds_t usecDelta = time.tv_usec - s.laptoptime.tv_usec;
		double delta = double(secDelta) + 1e-6*double(usecDelta);
		if(delta <= acceptableError) {
//			iter = stateQueue.erase(iter);
			return s;
		} else {
			iter++;
		}*/
	}
	GPSData empty;
	return empty;
}

bool HemisphereA100GPS::StateIsAvailable() {
	return !stateQueue.empty();
}

HemisphereA100GPS::~HemisphereA100GPS() {
    serialPort.stopEvents();
	serialPort.close();
}

} /* namespace Sensors */
} /* namespace IGVC */
