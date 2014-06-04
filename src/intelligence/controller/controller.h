#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <hardware/sensors/gps/GPS.hpp>
#include <common/utils/GPSWaypointSource.h>
#include <QObject>


class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(std::shared_ptr<GPSWaypointSource> source, std::shared_ptr<GPS> gps);
    ~Controller();
    GPSData getCurrentWaypoint();
    bool isWorking()
    {
        return _source->isOpen() && _gps->isOpen();
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
