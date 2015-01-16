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
    _moduleName = "GPS";
    if(serialPort.isWorking())
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
    try {
        nmea::decodeGPGGA(line, state);
        boost::mutex::scoped_lock lock(queueLocker);
        stateQueue.push_back(state);
        if(stateQueue.size() > maxBufferLength) {
            stateQueue.pop_front();
        }
        onNewData(state);
    } catch(const nmea::bad_nmea_sentence &e) {
        Logger::Log(LogLevel::Warning, e.what());
    }
}

const GPSData &NMEACompatibleGPS::GetState() {
	boost::mutex::scoped_lock lock(queueLocker);
	GPSData state = stateQueue.back();
	stateQueue.remove(state);
	return state;
}

bool NMEACompatibleGPS::StateIsAvailable() {
	return !stateQueue.empty();
}

bool NMEACompatibleGPS::isWorking() {
    return serialPort.isWorking();
}

NMEACompatibleGPS::~NMEACompatibleGPS() {
    serialPort.stopEvents();
	serialPort.close();
}
