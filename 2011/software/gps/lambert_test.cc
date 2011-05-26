#include "gps_common.hpp"
#include <iostream>

int main()
{
	GPSState point1;
	GPSState point2;
	point1.lat = 39.1779888889;
	point1.lon = 95.6945472222;
	point2.lat = 39.17635;
	point2.lon = 95.6897388889;

	std::cout << "The distance is " << lambert_distance(point1,point2) << "meters.\n";
	return 0;
}
