#include "kalmanpositiontracker.h"
#include <common/utils/timing.h>
#include <common/utils/GPSUtils.h>
#include <common/config/configmanager.h>
#include <common/logger/logger.h>

KalmanPositionTracker::KalmanPositionTracker(std::shared_ptr<IMU> imu, std::shared_ptr<GPS> gps)
{
    _moduleName = "PositionTracker";

    if(imu.get())
        connect(imu.get(), SIGNAL(onNewData(IMUData)), this, SLOT(onIMUData(IMUData)));
    if(gps.get())
        connect(gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onGPSData(GPSData)));
}

KalmanPositionTracker::~KalmanPositionTracker()
{
}

bool KalmanPositionTracker::isWorking() {

    return (seconds_since_IGVCpoch() - _lastIMUTime) < 1.0 && (seconds_since_IGVCpoch() - _lastGPSTime) < 1.0;
}

void KalmanPositionTracker::Reset() {
    _numOriginPointsRecorded = 0;
    _GPSDataBuffer.fill(GPSData());
}

void KalmanPositionTracker::onIMUData(IMUData data) {
    _lastIMUTime = data.time();
    _currentInternalEstimate.Heading = data.Yaw;
    _currentInternalEstimate.Heading.Variance = 0.0;
    emitCurrentEstimate();
}

void KalmanPositionTracker::onGPSData(GPSData data) {
    if(data.HDOP() > ConfigManager::Instance().getValue("KalmanPositionTracker", "MinAllowedHDOP", 20)) {
        // HDOP is too high (measurement is worthlessly inaccurate), so we'll just ignore it.
        return;
    }
    if(data.Quality() == GPS_QUALITY_INVALID) {
        // Ignore all GPS measurements with invalid data (ie. no satelites)
        return;
    }
    int numOriginPointsNeeded = ConfigManager::Instance().getValue("KalmanPositionTracker", "OriginPointsNeeded", 300);
    if(_numOriginPointsRecorded < numOriginPointsNeeded) {
        // Find our starting origin by averaging the first n points.
        // NOTE: this assumes the robot is still during this process
        _origin.Lat(_origin.Lat() + data.Lat());
        _origin.Long(_origin.Long() + data.Long());
        _numOriginPointsRecorded++;
        onOriginPercentage((int)( ( (double)_numOriginPointsRecorded/(double)numOriginPointsNeeded) * 100 ));
        if(_numOriginPointsRecorded == numOriginPointsNeeded) {
            _origin.Lat(_origin.Lat() / (double)numOriginPointsNeeded);
            _origin.Long(_origin.Long() / (double)numOriginPointsNeeded);
            Logger::Log(LogLevel::Info, "[KalmanPositionTracker] Origin found: LAT " + std::to_string(_origin.Lat()) + "\tLONG " + std::to_string(_origin.Long()));
        }

        return;
    }
    predict();
    _lastGPSTime = data.time();
    _currentInternalEstimate.Latitude = ( _currentInternalEstimate.Latitude*data.LatVar() + data.Lat()*_currentInternalEstimate.Latitude.Variance ) / (_currentInternalEstimate.Latitude.Variance+data.LatVar());
    _currentInternalEstimate.Longitude = ( _currentInternalEstimate.Longitude*data.LongVar() + data.Long()*_currentInternalEstimate.Longitude.Variance ) / (_currentInternalEstimate.Longitude.Variance+data.LongVar());
    _currentInternalEstimate.Latitude.Variance = ( 1.0 / (1.0/_currentInternalEstimate.Latitude.Variance + 1.0 / data.LatVar()) );
    _currentInternalEstimate.Longitude.Variance = ( 1.0 / (1.0/_currentInternalEstimate.Longitude.Variance + 1.0 / data.LongVar()) );
    // Push to buffer
    for(unsigned int i = 0; i < _GPSDataBuffer.size()-1; i++)
        _GPSDataBuffer[i] = _GPSDataBuffer[i+1];
    _GPSDataBuffer[_GPSDataBuffer.size()-1] = data;
    emitCurrentEstimate();
}

void KalmanPositionTracker::predict() {
    double latVel = 0.0;
    double lonVel = 0.0;
    for(unsigned int i = 1; i < _GPSDataBuffer.size(); i++) {
        GPSData &a = _GPSDataBuffer[i-1];
        GPSData &b = _GPSDataBuffer[i];
        std::cout << b.time() << "\t" << a.time() << std::endl;
        latVel += ( b.Lat() - a.Lat() ) / (b.time() - a.time());
        lonVel += ( b.Long() - a.Long()) / (b.time() - a.time());
    }

//    std::cout << latVel << "\t" << lonVel << std::endl;

    latVel /= (double)_GPSDataBuffer.size();
    lonVel /= (double)_GPSDataBuffer.size();

    double time = seconds_since_IGVCpoch();
    double deltaTime = time - _lastGPSTime;

    _currentInternalEstimate.Latitude = _currentInternalEstimate.Latitude + (latVel * deltaTime);
    _currentInternalEstimate.Longitude = _currentInternalEstimate.Longitude + (lonVel * deltaTime);

    _currentInternalEstimate.Latitude.Variance += 0.0; // TODO Some variance
    _currentInternalEstimate.Longitude.Variance += 0.0; // TODO Some variance
}

void KalmanPositionTracker::emitCurrentEstimate() {

    _currentEstimate.Heading = _currentInternalEstimate.Heading;

    if(_numOriginPointsRecorded >= ConfigManager::Instance().getValue("KalmanPositionTracker", "OriginPointsNeeded", 300)) {
        GPSUtils::coordsToMetricXY(_origin.Lat(), _origin.Long(), _currentInternalEstimate.Latitude, _currentInternalEstimate.Longitude, _currentEstimate.X, _currentEstimate.Y);
    } else {
        _currentEstimate.X = 0.0;
        _currentEstimate.Y = 0.0;
    }

//    std::cout << _currentInternalEstimate.Latitude << "\t" << _currentInternalEstimate.Longitude << std::endl;
//    std::cout << _currentEstimate.X << "\t" << _currentEstimate.Y << std::endl;

    emit onNewPosition(_currentEstimate);
}
