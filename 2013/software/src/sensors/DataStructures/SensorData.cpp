#include "SensorData.h"
#include "../timing.h"


SensorData::SensorData() : MeasurementTime(seconds_since_IGVCpoch())
{

}

SensorData::SensorData(double time) : MeasurementTime(time)
{

}


double SensorData::time()
{
    return MeasurementTime;
}

SensorData::~SensorData()
{
    //dtor
}
