/*
 * HemisphereA100GPS.h
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#ifndef HEMISPHEREA100GPS_H_
#define HEMISPHEREA100GPS_H_

#include "GPS.hpp"

//#include <boost/thread.hpp>
#include "serial/ASIOSerialPort.h"

#include <list>

namespace IGVC {
namespace Sensors {

class HemisphereA100GPS: public IGVC::Sensors::GPS {
public:
	HemisphereA100GPS();
	GPSState GetState();
	GPSState GetStateAtTime(timeval time);
	bool StateIsAvailable();
	~HemisphereA100GPS();

private:

	ASIOSerialPort serialPort; // Serial port for GPS communication

	void onNewSerialLine(string line);
	LISTENER(HemisphereA100GPS, onNewSerialLine, string);

	boost::mutex queueLocker; // mutex for thread-safing the buffer

	size_t maxBufferLength; // maximum number of states to be stored in the buffer

	std::list<GPSState> stateQueue; // buffer of the latest maxBufferLength states. This is a std::list instead of a std::queue because of the need to iterate to retrieve a state at a given timestamp

	bool parseLine(std::string line, GPSState &state); // parses a line from the GPS device
};

} /* namespace Sensors */
} /* namespace IGVC */
#endif /* HEMISPHEREA100GPS_H_ */
