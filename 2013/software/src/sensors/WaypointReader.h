#ifndef WAYPOINTREADER
#define WAYPOINTREADER

#include <fstream>
#include <iostream>
#include <vector>

using namespace std;

class WaypointReader
{
    public:
        WaypointReader(string fileName1);
        ~WaypointReader();
        void parseFile();
        vector<double> getNextWaypoint();
        void removeWaypoint();

    private:
        string fileName;
        ifstream file;
        double waypoints[10][2];
        int count;
};



#endif // WAYPOINTREADER_H_INCLUDED
