#include "controller.h"
#include <common/utils/GPSUtils.h>


Controller::Controller(GPSWaypointSource *source, GPS *gps) : _gps(gps)
{
    _source = source;
    currentWaypoint = _source->getNext();
    if(_gps != nullptr)
        connect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

Controller::~Controller()
{
    if(_gps != nullptr)
        disconnect(_gps, SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void Controller::onNewGPS(GPSData data)
{
    std::cout << "In"<<std::endl;
    if(GPSUtils::coordsToMeter(currentWaypoint.Lat(), currentWaypoint.Long(), data.Lat(), data.Long()) < 1.0) //need to be within 1 meter radius of the waypoint
    {
        std::cout << "here" <<std::endl;
        currentWaypoint = _source->getNext();
        onNewWaypoint(currentWaypoint);
    }
}

GPSData Controller::getCurrentWaypoint()
{
    return currentWaypoint;
}
