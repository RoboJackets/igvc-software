#ifndef SENSORDATA_H
#define SENSORDATA_H

class SensorData
{
    public:
        SensorData();
        SensorData(double);
        double time();
        virtual ~SensorData();
    private:
        double MeasurementTime; //Units should be milliseconds
};

#endif // SENSORDATA_H
