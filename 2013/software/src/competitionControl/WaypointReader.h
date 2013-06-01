#ifndef WAYPOINTREADER_H
#define WAYPOINTREADER_H

#include <vector>
#include <string>

struct Waypoint
{
    double Lattitude;
    double Longitude;
};

class WaypointReader
{
    public:
        WaypointReader();
        bool LoadWaypoints(std::string path);
        Waypoint Current();
        bool Next();
        virtual ~WaypointReader();
    protected:
    private:
        std::vector<Waypoint> _waypoints;
        int _index;
};

#endif // WAYPOINTREADER_H
