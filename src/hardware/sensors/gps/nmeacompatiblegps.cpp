/*
 * HemisphereA100GPS.cpp
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#include "nmeacompatiblegps.h"
#include "nmea.hpp"
#include <string>
#include <common/logger/logger.h>

using namespace std;

NMEACompatibleGPS::NMEACompatibleGPS(string devicePath, uint baudRate)
    :serialPort(devicePath, baudRate),
	stateQueue()
{
    if(serialPort.isConnected())
    {
        Logger::Log(LogLevel::Info, "GPS Initialized");
        connect(&serialPort, SIGNAL(onNewLine(std::string)), this, SLOT(onNewSerialLine(std::string)));
        maxBufferLength = 10;
        serialPort.startEvents();
    }
    else
    {
        Logger::Log(LogLevel::Error, "GPS failed to initialize");
    }
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
