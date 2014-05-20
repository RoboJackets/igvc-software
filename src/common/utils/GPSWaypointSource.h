#ifndef GPSWAYPOINTSOURCE_H
#define GPSWAYPOINTSOURCE_H

#include <queue>
#include "gpsfilereader.h"

class GPSWaypointSource
{
public:
    GPSWaypointSource(std::string file);
    void openFile(std::string file);
    GPSData getNext();

    bool isOpen()
    {
        return _isOpen;
    }

private:
    std::queue<GPSData> _data;
    bool _isOpen;
};


#endif // GPSWAYPOINTSOURCE_H
