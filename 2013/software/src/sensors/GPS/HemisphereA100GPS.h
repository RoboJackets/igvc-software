/*
 * HemisphereA100GPS.h
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#ifndef HEMISPHEREA100GPS_H_
#define HEMISPHEREA100GPS_H_

#include "GPS.hpp"

#include <boost/thread.hpp>
#include "ASIOSerialPort.h"

#include <list>

namespace IGVC {
namespace Sensors {

class HemisphereA100GPS: public IGVC::Sensors::GPS {
public:
	HemisphereA100GPS();
	GPSState GetState();
	GPSState GetStateAtTime(timeval time);
	GPSState peekLatestState();
	GPSState popLatestState();
	GPSState peekStateWithTime(timeval time, double acceptableError);
	GPSState popStateWithTime(timeval time, double acceptableError);
	bool StateIsAvailable();
	~HemisphereA100GPS();

private:

	ASIOSerialPort serialPort; // Serial port for GPS communication
	boost::thread iothread; // thread to poll the GPS device
	boost::mutex queueLocker; // mutex for thread-safing the buffer


	size_t maxBufferLength; // maximum number of states to be stored in the buffer

	std::list<GPSState> stateQueue; // buffer of the latest maxBufferLength states

	void threadRun(); // the method that runs on iothread

	bool parseLine(std::string line, GPSState &state); // parses a line from the GPS device
};

} /* namespace Sensors */
} /* namespace IGVC */
#endif /* HEMISPHEREA100GPS_H_ */
