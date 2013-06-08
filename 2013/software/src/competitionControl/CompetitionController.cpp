#include "CompetitionController.h"

using namespace IGVC::Sensors;
using namespace std;

namespace IGVC
{
namespace Control
{

CompetitionController::CompetitionController(IGVC::Sensors::GPS* gps, IGVC::Sensors::Lidar* lidar, StereoSource* stereoCam, Ardupilot* imu, WaypointReader* waypointReader)
    : GPS_BUFFER_SIZE(5),
      LOnNewGPSData(this),
      LOnNewIMUData(this)
{
    _gps = gps;
    _gps->onNewData += &LOnNewGPSData;
    _waypointReader = waypointReader;
    _lidar = lidar;
    _stereoCam = stereoCam;
    _imu = imu;
    _imu->onNewRawData += &LOnNewIMUData;
    _mode = ObstacleAvoidance;
    _currentHeading = 0;
}

bool CompetitionController::isRunning()
{
    return true;
}

void CompetitionController::OnNewGPSData(GPSData data)
{
    if(_gpsBuffer.size() >= GPS_BUFFER_SIZE)
    {
        _gpsBuffer.push_back(data);
        GPSData last = _gpsBuffer.back();
        _currentAvgGPS.Lat(_currentAvgGPS.Lat() - ( last.Lat() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Long(_currentAvgGPS.Long() - ( last.Long() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Heading(_currentAvgGPS.Heading() - ( last.Heading() / GPS_BUFFER_SIZE ));
        _currentAvgGPS.Speed(_currentAvgGPS.Speed() - ( last.Speed() / GPS_BUFFER_SIZE ));
        _gpsBuffer.erase(_gpsBuffer.begin());
    }
    else
    {
        _gpsBuffer.push_back(data);
    }
    _currentAvgGPS.Lat(_currentAvgGPS.Lat() + ( data.Lat() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Long(_currentAvgGPS.Long() + ( data.Long() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Heading(_currentAvgGPS.Heading() + ( data.Heading() / GPS_BUFFER_SIZE ));
    _currentAvgGPS.Speed(_currentAvgGPS.Speed() + ( data.Speed() / GPS_BUFFER_SIZE ));
}

void CompetitionController::OnNewIMUData(IMURawData data)
{
    _currentHeading = data.heading;
}

double CompetitionController::headingFromAToB(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return atan2(dy, dx);
}

CompetitionController::~CompetitionController()
{
    if(_gps)
        _gps->onNewData -= &LOnNewGPSData;
    if(_imu)
        _imu->onNewRawData -= &LOnNewIMUData;
}

}
}
