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
        //input.unsetf(std::ios_base::skipws);
        while(input.good())
        {
            double lat, lon;
            char ignored;
            input >> lat;
            input >> ignored;
            input >> lon;
            Waypoint wp;
            wp.Lattitude = lat;
            wp.Longitude = lon;
            _waypoints.push_back(wp);
        }
        std::cout << _waypoints.size() << " waypoints loaded." << std::endl;
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
