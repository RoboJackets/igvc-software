/*
 * GPS.hpp
 *
 *  Created on: Nov 5, 2012
 *      Author: Matthew Barulic
 */

#ifndef GPS_HPP_
#define GPS_HPP_

namespace IGVC {
namespace Sensors {

/*
 * An enumeration of GPS quality states.
 */
enum GPS_QUALITY {
	GPS_QUALITY_NOFIX=0,
	GPS_QUALITY_NON_DIFF=1,
	GPS_QUALITY_WAAS=2,
	GPS_QUALITY_ESTIMATED=6,
	GPS_QUALITY_UNKNOWN=7
};

/*
 * A struct that represents a data packet from the GPS device.
 */
struct GPSState
{
	int num_sat;
	GPS_QUALITY qual;
	timeval laptoptime;
	double lat;
	double lon;
	double courseoverground;
	double speedoverground;
};

/*
 * Interface for GPS devices.
 */
class GPS {
public:
	virtual ~GPS() { }

	/*
	 * Returns the most recent state acquired from the GPS device.
	 */
	virtual GPSState GetState();

	/*
	 * Returns the GPSState with the given timestamp.
	 * Throws an error if no such GPSState exists in the buffer.
	 */
	virtual GPSState GetStateAtTime(timeval time);

};

} //Sensors
} //IGVC


#endif /* GPS_HPP_ */
