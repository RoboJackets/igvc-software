
#include <sensors/DataStructures/GPSData.h>
#include <competitionControl/WaypointReader.h>
#include <iostream>
#include <cmath>

using namespace std;

double headingFromAToB(GPSData A, GPSData B)
{
    double dy = B.Lat() - A.Lat();
    double dx = cos(M_PI/180.0*A.Lat())*(B.Long() - A.Long());
    return atan2(-dx, dy)/M_PI * 180.0;
}

int main()
{
    WaypointReader reader;
    reader.LoadWaypoints("/home/robojackets/practiceCourse.txt");
    reader.Next();
    GPSData A = reader.Current();
    reader.Next();
    GPSData B = reader.Current();
    B.Lat(A.Lat());
    cout << A.Long() << " " << B.Long() << endl;
    cout << headingFromAToB(A, B) << endl;

    return 0;
}
