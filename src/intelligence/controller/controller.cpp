#include "controller.h"
#include <common/utils/GPSUtils.h>


Controller::Controller(std::shared_ptr<GPSWaypointSource> source, std::shared_ptr<GPS> gps) : _gps(gps)
{
    _source = source;
    currentWaypoint = _source->getNext();
    connect(_source.get(), SIGNAL(newFileLoaded()), this, SLOT(onNewWaypointFileLoaded()));
    if(_gps.get() != nullptr)
        connect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

Controller::~Controller()
{
    if(_gps.get() != nullptr)
        disconnect(_gps.get(), SIGNAL(onNewData(GPSData)), this, SLOT(onNewGPS(GPSData)));
}

void Controller::onNewGPS(GPSData data)
{
    // need to be within 1 meter radius of the waypoint (see rules)
    if(GPSUtils::coordsToMeter(currentWaypoint.Lat(), currentWaypoint.Long(), data.Lat(), data.Long()) < 1.0)
    {
        currentWaypoint = _source->getNext();
        onNewWaypoint(currentWaypoint);
    }
}

GPSData Controller::getCurrentWaypoint()
{
    return currentWaypoint;
}

void Controller::onNewWaypointFileLoaded()
{
    currentWaypoint = _source->getNext();
    onNewWaypoint(currentWaypoint);
}
