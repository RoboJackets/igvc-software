#include "SensorData.h"
#include "timing.h"


SensorData::SensorData() : MeasurementTime(milliseconds_since_epoch())
{

}

SensorData::SensorData(long long time) : MeasurementTime(time)
{
}


long long SensorData::time()
{
    return MeasurementTime;
}

SensorData::~SensorData()
{
    //dtor
}
