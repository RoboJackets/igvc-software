#include "CompetitionController.h"

using namespace IGVC::Sensors;
using namespace std;

namespace IGVC
{
namespace Control
{

CompetitionController::CompetitionController(IGVC::Sensors::GPS* gps, Event<pcl::PointCloud<PointXYZ> > mapSource, Ardupilot* imu, WaypointReader* waypointReader)
    : GPS_BUFFER_SIZE(5),
      LOnNewGPSData(this),
      LOnNewIMUData(this),
      LOnNewMapFrame(this)
{
    _gps = gps;
    _gps->onNewData += &LOnNewGPSData;
    _waypointReader = waypointReader;
    _imu = imu;
    _imu->onNewRawData += &LOnNewIMUData;
    _currentHeading = 0;

    MaxW = 0.8;
    DeltaT = 2.5;
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

void CompetitionController::OnNewMapFrame(pcl::PointCloud<PointXYZ> mapFrame)
{
    vector< pair<double, double> > available_actions;

    double v = 0.4;
    for(double w = -MaxW; w <= MaxW; w += 0.5)
    {


        if(true/*Point is occupiable*/)
        {
            available_actions.push_back(pair<double,double>(v, w));
        }
    }

    using namespace pcl;

    pair<double, double> minPair;
    double minDist = -1;

    for(vector< pair<double, double> >::iterator iter = available_actions.begin(); iter != available_actions.end(); iter++)
    {
        pair<double, double> action = (*iter);
        pair<double, double> waypointCoords;
        waypointCoords.first = GPSdX(_currentAvgGPS, _waypointReader->Current());
        waypointCoords.second = GPSdY(_currentAvgGPS, _waypointReader->Current());



        //if(minDist == -1 || )
    }

}

double CompetitionController::headingFromAToB(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return atan2(dy, dx);
}

double CompetitionController::distBetween(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return sqrt(dx*dx + dy*dy);
}

double CompetitionController::GPSdX(GPSData A, GPSData B)
{
    return cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
}

double CompetitionController::GPSdY(GPSData A, GPSData B)
{
    return B.Lat() - A.Lat();
}

pair<double, double> CompetitionController::result(double W, double V)
{
    Eigen::Vector3d endLocation;
    if(W != 0)
    {
        double R = V / W;
        double ICCx = cos(- M_PI/2.0) * R;
        double ICCy = sin(- M_PI/2.0) * R;
        using namespace Eigen;
        Matrix3d T;
        double wdt = W*DeltaT;
        T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
        Vector3d a(-ICCx, -ICCy, 0);
        Vector3d b(ICCx, ICCy, wdt);
        endLocation = T * a + b;
    }
    else
    {
        endLocation[0] = 0;
        endLocation[1] = V * DeltaT;
    }
    return pair<double, double> (endLocation[0], endLocation[1]);
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
