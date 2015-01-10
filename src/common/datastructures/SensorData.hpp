#ifndef SENSORDATA_H
#define SENSORDATA_H

#include <boost/cstdint.hpp>
#include <chrono>

class SensorData
{
    private:
        long long int MicroSeconds;
        static const int U_SEC_PER_SEC = 1000000;
        unsigned long long int microseconds_since_IGVCpoch() {
            return std::chrono::high_resolution_clock::now().time_since_epoch() /
            std::chrono::microseconds(1);
        }

    public:
        SensorData() : MicroSeconds(microseconds_since_IGVCpoch()) { }
        SensorData(double time_s) : MicroSeconds(time_s * U_SEC_PER_SEC) { }
        SensorData(long long time_us) : MicroSeconds(time_us) { }
        void setTime() {
            MicroSeconds = microseconds_since_IGVCpoch();
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
