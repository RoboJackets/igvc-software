#ifndef DATAPOINT_H
#define DATAPOINT_H

#include <SensorData.h>

template <class aType>
class DataPoint : public SensorData
{
	public:
        DataPoint(aType val);
        aType value();
        virtual ~DataPoint();

    private:
        aType Value;

};

template <class aType>
DataPoint<aType>::DataPoint(aType val) : SensorData(), Value(val)
{
}

template<class aType>
aType DataPoint<aType>::value()
{
    return Value;
}

template<class aType>
DataPoint<aType>::~DataPoint()
{
}

#endif
