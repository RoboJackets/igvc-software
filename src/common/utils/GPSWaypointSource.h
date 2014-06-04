#ifndef GPSWAYPOINTSOURCE_H
#define GPSWAYPOINTSOURCE_H

#include <queue>
#include "gpsfilereader.h"
#include <QObject>

class GPSWaypointSource : public QObject
{
    Q_OBJECT
public:
    GPSWaypointSource(std::string file);
    void openFile(std::string file);
    GPSData getNext();

    bool isOpen()
    {
        return _isOpen;
    }

signals:
    void newFileLoaded();

private:
    std::queue<GPSData> _data;
    bool _isOpen;
};


#endif // GPSWAYPOINTSOURCE_H
