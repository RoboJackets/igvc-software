#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <hardware/sensors/gps/GPS.hpp>
#include <common/utils/GPSWaypointSource.h>
#include <QObject>


class Controller : public QObject
{
    Q_OBJECT
public:
    Controller(GPSWaypointSource *source, GPS *gps);
    ~Controller();
    GPSData getCurrentWaypoint();

signals:
    void onNewWaypoint(GPSData);

private slots:
    void onNewGPS(GPSData data);

private:
    GPS *_gps;
    GPSWaypointSource *_source;
    GPSData currentWaypoint;

};

#endif // CONTROLLER_H
