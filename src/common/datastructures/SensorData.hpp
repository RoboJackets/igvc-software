#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <common/utils/timing.h>
#include <boost/cstdint.hpp>
class SensorData
{
    public:
        SensorData() : MicroSeconds(seconds_since_IGVCpoch() * 1000000) { }
        SensorData(double time_s) : MicroSeconds(time_s * 1000000) { }
        SensorData(unsigned long long int time_us) : MicroSeconds(time_us) { }
        void setTime() {
            MicroSeconds  = seconds_since_IGVCpoch() * 1000000;
        }
        void setTimeSeconds(double time_s) {
            MicroSeconds = (unsigned long) (time_s * 1000000);
        }
        void setTimeMicroSeconds(unsigned long long time_us) {
            MicroSeconds = time_us;
        }
        double getTimeSeconds() {
            return MicroSeconds / (double) 1000000;
        }
        unsigned long long int getTimeMicroSeconds() {
            return MicroSeconds;
        }

        virtual ~SensorData() { }
    private:
        unsigned long long int MicroSeconds;
};

#endif // SENSORDATA_H
