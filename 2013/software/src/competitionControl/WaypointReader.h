#ifndef WAYPOINTREADER_H
#define WAYPOINTREADER_H

#include <vector>
#include <string>
#include <sensors/DataStructures/GPSData.h>

class WaypointReader
{
    public:
        WaypointReader();
        bool LoadWaypoints(std::string path);
        GPSData Current();
        bool Next();
        virtual ~WaypointReader();
    protected:
    private:
        std::vector<GPSData> _waypoints;
        unsigned int _index;
};

#endif // WAYPOINTREADER_H
