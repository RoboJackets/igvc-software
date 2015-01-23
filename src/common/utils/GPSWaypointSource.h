#ifndef GPSWAYPOINTSOURCE_H
#define GPSWAYPOINTSOURCE_H

#include <queue>
#include "gpsfilereader.h"
#include <common/module.hpp>

class GPSWaypointSource : public Module
{
    Q_OBJECT
public:
    GPSWaypointSource(std::string file);
    void openFile(std::string file);
    GPSData getNext();

    bool isWorking()
    {
        return _isOpen;
    }

signals:
    void newFileLoaded();

private:
    std::vector<GPSData> _data;
    bool _isOpen;
};


#endif // GPSWAYPOINTSOURCE_H
