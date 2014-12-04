#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <common/utils/timing.h>
#include <boost/cstdint.hpp>
class SensorData
{
    private:
        long long int MicroSeconds;
        static const int U_SEC_PER_SEC = 1000000;
    public:
        SensorData() : MicroSeconds(micro_seconds_since_IGVCpoch()) { }
        SensorData(double time_s) : MicroSeconds(time_s * U_SEC_PER_SEC) { }
        SensorData(long long time_us) : MicroSeconds(time_us) { }
        void setTime() {
            MicroSeconds = micro_seconds_since_IGVCpoch();
        }
        void setTimeSeconds(double time_s) {
            MicroSeconds = (long long int) (time_s * U_SEC_PER_SEC);
        }
        void setTimeMicroSeconds(long long int time_us) {
            MicroSeconds = time_us;
        }
        double getTimeSeconds() {
            return MicroSeconds / (double) U_SEC_PER_SEC;
        }
        long long int getTimeMicroSeconds() {
            return MicroSeconds;
        }

        virtual ~SensorData() { }
};

#endif // SENSORDATA_H
