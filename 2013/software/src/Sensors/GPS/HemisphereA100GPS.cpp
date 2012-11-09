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

HemisphereA100GPS::HemisphereA100GPS() {
	serialPort("/dev/HemiGPS",9600);
	iothread = boost::thread(boost::bind(&HemisphereA100GPS::threadRun(), this));
	maxBufferLength = 10;
	stateQueue();
}

void HemisphereA100GPS::threadRun() {
	while(serialPort.isConnected()) {
		char* line = serialPort.readln();
		GPSState state;
		if(parseLine(line, state)) {
			gettimeofday(&state.laptoptime, NULL);

			boost::mutex::scoped_lock lock(queueLocker);
			stateQueue.push_back(state);
			if(stateQueue.size() > maxBufferLength) {
				stateQueue.pop_front();
			}
		}
	}
}

bool HemisphereA100GPS::parseLine(std::string line, GPSState &state) {
	return nmea::decodeGPGGA(line, state) ||
		   nmea::decodeGPRMC(line, state);
}

GPSState HemisphereA100GPS::peekLatestState() {
	boost::mutex::scoped_lock lock(queueLocker);
	return stateQueue.back();
}

GPSState HemisphereA100GPS::popLatestState() {
	boost::mutex::scoped_lock lock(queueLocker);
	GPSState state = stateQueue.back();
	stateQueue.remove(state);
	return state;
}

GPSState HemisphereA100GPS::peekStateWithTime(timeval time, double acceptableError) {
	boost::mutex::scoped_lock lock(queueLocker);
	std::list<GPSState>::iterator iter = stateQueue.begin();
	while(iter != stateQueue.end()) {
		GPSState s = (*iter);
		time_t secDelta = difftime(time.tv_sec, s.laptoptime.tv_sec);
		suseconds_t usecDelta = time.tv_usec - s.laptoptime.tv_usec;
		double delta = double(secDelta) + 1e-6*double(usecDelta);
		if(delta <= acceptableError) {
			return s;
		}
	}
	return 0;
}

GPSState HemisphereA100GPS::popStateWithTime(timeval time, double acceptableError) {
	boost::mutex::scoped_lock lock(queueLocker);
	std::list<GPSState>::iterator iter = stateQueue.begin();
	while(iter != stateQueue.end()) {
		GPSState s = (*iter);
		time_t secDelta = difftime(time.tv_sec, s.laptoptime.tv_sec);
		suseconds_t usecDelta = time.tv_usec - s.laptoptime.tv_usec;
		double delta = double(secDelta) + 1e-6*double(usecDelta);
		if(delta <= acceptableError) {
			iter = stateQueue.erase(iter);
			return s;
		} else {
			iter++;
		}
	}
	return 0;
}

HemisphereA100GPS::~HemisphereA100GPS() {
	serialPort.close();
}

} /* namespace Sensors */
} /* namespace IGVC */
