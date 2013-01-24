#ifndef SENSORDATA_H
#define SENSORDATA_H

class SensorData
{
    public:
        SensorData();
        SensorData(long long);
        long long time();
        virtual ~SensorData();

    private:
        long long MeasurementTime; //Units should be milliseconds
};

#endif // SENSORDATA_H
