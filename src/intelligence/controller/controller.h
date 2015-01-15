#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <hardware/sensors/gps/GPS.hpp>
#include <common/utils/GPSWaypointSource.h>
#include <QObject>
#include <common/module.hpp>
#include <memory>

class Controller : public Module
{
Q_OBJECT

public:
    Controller(std::shared_ptr<GPSWaypointSource> source, std::shared_ptr<GPS> gps);
    ~Controller();
    GPSData getCurrentWaypoint();
    bool isWorking()
    {
        return _source.get() && _gps.get() && _source->isWorking() && _gps->isWorking();
    }

signals:
    void onNewWaypoint(GPSData);

private slots:
    void onNewGPS(GPSData data);
    void onNewWaypointFileLoaded();

private:
    std::shared_ptr<GPS> _gps;
    std::shared_ptr<GPSWaypointSource> _source;
    GPSData currentWaypoint;

    void advanceWaypoint();

};

#endif // CONTROLLER_H
