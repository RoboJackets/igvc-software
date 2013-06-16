#include <iostream>
#include <competitionControl/WaypointReader.h>

using namespace std;

int main()
{
    WaypointReader reader;
    if(reader.LoadWaypoints("/home/robojackets/practiceCourse.txt"))
    {
        while(reader.Next())
        {
            GPSData wp = reader.Current();
            cout << wp.Lat() << ", " << wp.Long() << endl;
        }
    }
    else
    {
        cerr << "Couldn't open your file, dude!" << endl;
    }
    return 0;
}
