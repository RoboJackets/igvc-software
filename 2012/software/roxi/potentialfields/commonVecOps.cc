#include "commonVecOps.hpp"

/* Converts the x and y components to a vector with magnitude and angle. Angle given by 0-360 with 0 at North */
void xyToVec(double x, double y, double& mag, double& ang)
{
	mag = sqrt(x*x + y*y);
	ang = rad2deg(atan2(y,x));
	ang = vec2bear(ang);
	return;
}

/* Converts a vector with magnitude and angle to x and y components. Angle given by 0-360 with 0 at North */
void VecToxy(double mag, double ang, double& x, double& y)
{
	x = mag * sin(deg2rad(ang));
	y = mag * cos(deg2rad(ang));
	return;
}

/* Adds an array of vectors together */
void AddVecs(double* xvals, double* yvals, int numVecs, double& xnet, double& ynet)
{
	xnet = ynet = 0;
	for (int i = 0; i < numVecs; i++)
	{
		xnet += xvals[i];
		ynet += yvals[i];
	}
	return;
}

/* Converts radians to degrees */
double rad2deg(double radians)
{
	return (radians*180)/M_PI;
}

/* Converts degrees to radians */
double deg2rad(double degrees)
{
	return (degrees*M_PI)/180;
}

/* Converts angle ccw from due East to angle cw from due North */
double vec2bear(double ang)
{
	double bearing = (-ang+90);
	while(bearing < 0)
	{
		bearing += 360;
	}
	while(bearing > 360)
	{
		bearing -= 360;
	}
	return bearing;
}

/* Converts angle ccw from due East to angle cw from due North */
double bear2vec(double bearing)
{
	double ang = (-bearing-90);
	ang = fmodf(ang,360);
	if (ang > 180)
		ang = ang - 180;
	return ang;
}

/* Returns distance between two coordinates */
double Distance2D(double x1, double y1, double x2, double y2)
{
	double dx = x2 - x1;
	double dy = y2 - y1;
	return sqrt(dx*dx + dy*dy);
}

/* Rotates bearing input angle0 by dangle */
double RotateBearing(double angle0, double dangle)
{
	double angle = angle0 + dangle;
	while(angle >= 360)
	{
		angle-=360;
	}	
	while(angle < 0)
	{
		angle+=360;
	}
	return angle;
}

/* Clamps angle to within the clamp angle */
double clampVector(double ang, double gps_clamp_angle)
{
	ang=fmodf(ang,360);
	if (ang > 180 && ang < 360 - gps_clamp_angle)
		ang = 360 - gps_clamp_angle;
	else if (ang < 180 && ang > gps_clamp_angle)
		ang = gps_clamp_angle;
}
