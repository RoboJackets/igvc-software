#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <common/utils/timing.h>

class SensorData
{
    public:
        SensorData() : MeasurementTime(seconds_since_IGVCpoch()) { }
        SensorData(double time_ms) : MeasurementTime(time_ms) { }
        double time()
	{
		return MeasurementTime;
	}
        virtual ~SensorData() { }
    private:
        double MeasurementTime; //Units should be milliseconds
};

#endif // SENSORDATA_H
