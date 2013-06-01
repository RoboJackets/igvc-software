#include <iostream>
#include <competitionControl/WaypointReader.h>

using namespace std;

int main()
{
    WaypointReader reader;
    if(reader.LoadWaypoints("/home/matt/Waypoints.txt"))
    {
        while(reader.Next())
        {
            Waypoint wp = reader.Current();
            cout << wp.Lattitude << ", " << wp.Longitude << endl;
        }
    }
    else
    {
        cerr << "Couldn't open your file, dude!" << endl;
    }
    return 0;
}
