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
	double mdist = lambert_distance(point1,point2);
	double adist = 7128;
	std::cout << "The measured distance is " << mdist << " meters.\n";
	std::cout << "The actual distance is " << adist << " meters.\n";
	return 0;
}
