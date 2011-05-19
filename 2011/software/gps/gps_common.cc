#include "gps_common.hpp"

#include <cmath>

//distance in meters
double lambert_distance(const GPSState& a, const GPSState& b)
{
	static const double eq_rad = 6378.1e3;
	static const double r = 298.257223563;

	const double psia = atan2( (r - 1.0) * tan(a.lat) , r );
	const double psib = atan2( (r - 1.0) * tan(b.lat) , r );

	const double P = (psia + psib) / 2.0;
	const double Q = (psia - psib) / 2.0;

	const double dlon = b.lon - a.lon;
	const double sigma = acos( sin(a.lat)*sin(b.lat) + cos(a.lat)*cos(b.lat) * cos(dlon) );

	const double sP = sin(P);
	const double cP = cos(P);

	const double sQ = sin(Q);
	const double cQ = cos(Q);

	const double ssgimahalf = sin(sigma/2.0);
	const double csgimahalf = cos(sigma/2.0);

	const double X = (sigma - sin(sigma)) * (sP*sP*cQ*cQ) / (csgimahalf*csgimahalf);
	const double Y = (sigma + sin(sigma)) * (cP*cP*sQ*sQ) / (ssgimahalf*ssgimahalf);

	const double d = eq_rad * (sigma - (X+Y) / (2.0*r));

	return d;
}
