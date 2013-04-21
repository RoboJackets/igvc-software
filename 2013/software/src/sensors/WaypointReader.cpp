#include "WaypointReader.h"
#include <iostream>
#include <string.h>
#include <fstream>
#include <sstream>
#include <cstdlib>
#include <vector>

using namespace std;

WaypointReader::WaypointReader(string fileName1)
{
    fileName = fileName1;
    file.open(fileName.c_str());
    cout.precision(17);
    count = 0;
}

WaypointReader::~WaypointReader()
{
    if(file.is_open())
        file.close();
}

void WaypointReader::parseFile()
{
    if(file.is_open())
    {
        string line;
        int count = 0;

        while(file.good())
        {
            getline(file, line);

            istringstream iss(line);

            string latS;
            iss>>latS;
            double lat = atof(latS.c_str());

            string lonS;
            iss>>lonS;
            double lon = atof(lonS.c_str());

            waypoints[count][0] = lat;
            waypoints[count++][1] = lon;
        }
    }
    else
    {
        cout<<"Unable to open file"<<endl;
    }

    file.close();
    cout<<"Lat: "<<waypoints[0][0]<<"  "<<"Lon: "<<waypoints[0][1]<<endl;
    cout<<"Lat: "<<waypoints[1][0]<<"  "<<"Lon: "<<waypoints[1][1]<<endl;
    cout<<"Lat: "<<waypoints[2][0]<<"  "<<"Lon: "<<waypoints[2][1]<<endl;
}

vector<double> WaypointReader::getNextWaypoint()
{
    vector<double> temp (2);
    temp[0] = waypoints[count][0];
    temp[1] = waypoints[count][1];
    count++;
    return temp;
}

void WaypointReader::removeWaypoint()
{

}
