/*! \file GPS.hpp
 *  \date Created: Nov 5, 2012
 *  \author Matthew Barulic
 */

#ifndef GPS_HPP_
#define GPS_HPP_

#include <time.h>
#include <common/datastructures/GPSData.hpp>
#include <common/module.hpp>

/*!
 * \brief Interface for GPS devices.
 * \headerfile GPS.hpp <hardware/sensors/gps/GPS.hpp>
 */
class GPS : public Module
{
    Q_OBJECT

signals:
    void onNewData(const GPSData &);

public:
    GPS() { qRegisterMetaType<GPSData>("GPSData"); }

    virtual ~GPS() { }

    /*!
     * \brief Returns the most recent state acquired from the GPS device.
	 */
    virtual GPSData GetState() = 0;

    /*!
     * \brief Return true if there is at least one state in the buffer.
	 */
	virtual bool StateIsAvailable() = 0;
};


#endif /* GPS_HPP_ */
