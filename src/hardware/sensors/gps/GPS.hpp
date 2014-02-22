/*! \file GPS.hpp
 *  \date Created: Nov 5, 2012
 *  \author Matthew Barulic
 */

#ifndef GPS_HPP_
#define GPS_HPP_

#include <time.h>
#include <common/datastructures/GPSData.hpp>
#include <QObject>

/*!
 * \brief Interface for GPS devices.
 * \headerfile GPS.hpp <hardware/sensors/gps/GPS.hpp>
 */
class GPS : public QObject
{
    Q_OBJECT

signals:
    void onNewData(GPSData);

public:
    GPS() { qRegisterMetaType<GPSData>("GPSData"); }

    virtual ~GPS() { }

    /*!
     * \brief Returns the most recent state acquired from the GPS device.
	 */
	virtual GPSData GetState() = 0;

    /*!
     * \brief Returns the GPSState with the given timestamp.
	 */
	virtual GPSData GetStateAtTime(timeval time) = 0;

    /*!
     * \brief Return true if there is at least one state in the buffer.
	 */
	virtual bool StateIsAvailable() = 0;

    /*!
     * \brief Return true if the device is connected and communicating.
     */
    virtual bool isOpen() = 0;
};


#endif /* GPS_HPP_ */
