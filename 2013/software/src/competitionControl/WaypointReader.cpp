#include "WaypointReader.h"
#include <fstream>
#include <iostream>

WaypointReader::WaypointReader()
{
}

bool WaypointReader::LoadWaypoints(std::string path)
{
    using namespace std;
    ifstream input(path.c_str());
    if(input.is_open())
    {
        double lat, lon;
        char ignored;
        while(input.good())
        {
            lat = 0, lon = 0;
            ignored = 0;
            input >> lat;
            input >> ignored;
            input >> lon;
            if(lat != 0 && lon != 0)
            {
                Waypoint wp;
                wp.Lattitude = lat;
                wp.Longitude = lon;
                _waypoints.push_back(wp);
            }
        }
        input.close();
        _index = -1;
        return true;
    }
    else
    {
        return false;
    }
}

Waypoint WaypointReader::Current()
{
    return _waypoints[_index];
}

bool WaypointReader::Next()
{
    if(_index + 1 < _waypoints.size())
    {
        _index++;
        return true;
    }
    return false;
}

WaypointReader::~WaypointReader()
{
    _waypoints.clear();
}
