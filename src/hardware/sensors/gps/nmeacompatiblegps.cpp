/*
 * HemisphereA100GPS.cpp
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#include "nmeacompatiblegps.h"
#include "nmea.hpp"
#include <string>

NMEACompatibleGPS::NMEACompatibleGPS(string devicePath, uint baudRate)
    :serialPort(devicePath, baudRate),
    //serialPort("/dev/ttyGPS", 19200/*For HemisphereA100 4800*/),
	LonNewSerialLine(this),
	stateQueue()
{
    serialPort.onNewLine += &LonNewSerialLine;
	maxBufferLength = 10;
	serialPort.startEvents();
    std::cout << "GPS inited" << std::endl;
}

void NMEACompatibleGPS::onNewSerialLine(string line) {
    GPSData state;
    if(parseLine(line, state)) {
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

bool NMEACompatibleGPS::parseLine(std::string line, GPSData &state) {
	return nmea::decodeGPGGA(line, state) ||
		   nmea::decodeGPRMC(line, state);
}

GPSData NMEACompatibleGPS::GetState() {
	boost::mutex::scoped_lock lock(queueLocker);
	GPSData state = stateQueue.back();
	stateQueue.remove(state);
	return state;
}

GPSData NMEACompatibleGPS::GetStateAtTime(timeval time) {
	boost::mutex::scoped_lock lock(queueLocker);
	std::list<GPSData>::iterator iter = stateQueue.begin();
    //double acceptableError = 0.1;
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

bool NMEACompatibleGPS::StateIsAvailable() {
	return !stateQueue.empty();
}

bool NMEACompatibleGPS::isOpen() {
    return serialPort.isConnected();
}

NMEACompatibleGPS::~NMEACompatibleGPS() {
    serialPort.stopEvents();
	serialPort.close();
}
