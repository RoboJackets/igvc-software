#include "CompetitionController.h"

using namespace IGVC::Sensors;

CompetitionController::CompetitionController(GPS* gps, WaypointReader* waypointReader)
    : LOnNewGPSData(this)
{
    _gps = gps;
    _gps->onNewData += &LOnNewGPSData;
    _waypointReader = waypointReader;
}

void CompetitionController::OnNewGPSData(GPSData data)
{
    if(_gpsBuffer.size() >= 5)
    {
        _gpsBuffer.push_back(data);
        _gpsBuffer.erase(_gpsBuffer.begin());


    }
    else
    {
        _gpsBuffer.push_back(data);
    }
}

CompetitionController::~CompetitionController()
{
    //dtor
}
