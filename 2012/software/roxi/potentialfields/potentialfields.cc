#include "potentialfields.hpp"
#include "gps_points.hpp"
#include "math.h"

/* Creates new potentialfields object and initializes fields */
potentialfields::potentialfields()
{
	// Initialize the goal GPS points
	for (int i = 0; i < num_GPS_Goals; i++)
	{
		GPS_Goals.push_back(GPS_point(goalWaypointLat[i], goalWaypointLon[i]));
	}
	return;
}

/* Destructor for potentialfields */
potentialfields::~potentialfields()
{
	return;
}

/* Adds the robot's current GPS waypoint to the class */
void potentialfields::dropWaypoint(double lat, double lon, double ang)
{
	bool isEmpty = GPS_Prev_Loc.empty();
	GPS_Prev_Loc.push_back(GPS_point(lat, lon));
	
	// If this is the first GPS point, then this needs to be added as the final goal for the robot (the course is circular) 	
	if (isEmpty)
	{
		GPS_Goals.push_back(GPS_point(lat, lon, ang));
	}
}

/* Changes value of input references vel_mag and vel_ang with the velocity and angle determined by the potential fields algorithm.
Angle given by a value between 0 and 360 with 0 at due North */
void potentialfields::getNextVector(bool* obstacles, bool* targets, int xsize, int ysize, int robotx, int roboty, double& vel_mag, double& vel_ang)
{
	removeclumps(obstacles);
	radiusfix(obstacles);

	double obstaclex, obstacley, imagetarx, imagetary, gpstarx, gpstary, gpsavoidx, gpsavoidy;

	getAvoidVec(obstacles, obstaclex, obstacley);
	getImgTargetVec(targets, imagetarx, imagetary);
	getGPSTargetVec(gpstarx, gpstary);
	getGPSAvoidVec(gpsavoidx, gpsavoidy);

	int number_vecs = 4;
	double xcomps[] = {obstaclex, imagetarx, gpstarx, gpsavoidx};
	double ycomps[] = {obstacley, imagetary, gpstary, gpsavoidy};	
	double xnet, ynet;

	AddVecs(xcomps, ycomps, number_vecs, xnet, ynet);	 
	xyToVec(xnet, ynet, vel_mag, vel_ang);	
	 
	return;
}

/* Gets rid of stray clumps of grass and other noise from the obstacle bitfield*/
void potentialfields::removeclumps(bool* obstacles)
{
	// TODO: Write this function
	return;
}

/* Adds radius of robot to all of the obstacles so that the robot doesn't try to fit into small gaps */
void potentialfields::radiusfix(bool* obstacles)
{
	for (int y = 0; y < ysize; y++)
	{
		for (int x = 0; x < xsize; x++)
		{
			if (get2Dindexvalue(obstacles, x, y) == 1)
			{
				fillinRadius(obstacles, x, y, robot_radius);
			}
		}
	}
	return;
}

/* Returns the x and y components of the obstacle avoidance vector in meters */
void potentialfields::getAvoidVec(bool* obstacles, double& xvel, double& yvel)
{
	// TODO: Write this function
	return;
}

/* Returns the x and y components of the Image target vector in meters */
void potentialfields::getImgTargetVec(bool* targets, double& xvel, double& yvel)
{
	// TODO: Write this function
	return;
}

/* Returns the x and y components of the GPS goal vector in meters */
void potentialfields::getGPSTargetVec(double& xvel, double& yvel)
{
	// TODO: Write this function
	return;
}

/* Returns the x and y components of the GPS avoidance vector in meters */
void potentialfields::getGPSAvoidVec(double& xvel, double& yvel)
{
	// TODO: Write this function
	return;
}

/* Converts a pixel x and y disrance to a real */
void potentialfields::convertToRealxy(double& xcomp, double& ycomp)
{
	xcomp *= meters_per_pixel;
	ycomp *= meters_per_pixel;
	return;
}

/* Converts a pixel vector to a real vector */
void potentialfields::convertToRealVec(double& vel, double& ang)
{
	vel *= meters_per_pixel;
	return;
}

/* Converts the x and y components to a vector with magnitude and angle. Angle given by 0-360 with 0 at North */
void potentialfields::xyToVec(double x, double y, double& mag, double& ang)
{
	mag = sqrt(x*x + y*y);
	ang = rad2deg(atan2(y,x));
	ang = vec2bear(ang);
	return;
}

/* Converts a vector with magnitude and angle to x and y components. Angle given by 0-360 with 0 at North */
void potentialfields::VecToxy(double mag, double ang, double& x, double& y)
{
	x = mag * cos(deg2rad(ang));
	y = mag * cos(rad2deg(ang));
	return;
}

/* Adds an array of vectors together */
void potentialfields::AddVecs(double* xvals, double* yvals, int numVecs, double& xnet, double& ynet)
{
	xnet = ynet = 0;
	for (int i = 0; i < numVecs; i++)
	{
		xnet += xvals[i];
		ynet += yvals[i];
	}
	return;
}

void potentialfields::attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& yvel)
{
	// TODO: Write this function
	return;
}

void potentialfields::repulsivePixels(int x0, int y0, int xt, int yt, double& x_vel, double& yvel)
{
	// TODO: Write this function
	return;
}
 
/* Returns the value at the index array[x][y] as if it were a 2D array */
bool potentialfields::get2Dindexvalue(bool* array, int x, int y)
{
	return array[y*ysize+x];
}

/* Sets the value at the index array[x][y] to val as if it were a 2D array */
void potentialfields::set2Dindexvalue(bool* array, int x, int y, bool val)
{
	array[y*ysize+x] = val;
}

/* Fills in 1's within the radius of the third input around the point (x,y) */
void potentialfields::fillinRadius(bool* obstacle, int x, int y, int radius)
{
	int* xinds = NULL;
	int* yinds = NULL;
	int numinds;	

	getIndexesInRadius(x, y, radius, xinds, yinds, numinds);
	for(int i = 0; i < numinds; i++)
	{
		set2Dindexvalue(obstacle, xinds[i], yinds[i], 1);
	}

	delete [] xinds;
	delete [] yinds;
	return;	
}

void potentialfields::getIndexesInRadius(int x0, int y0, int radius, int* xinds, int* yinds, int& numinds)
{
	// TODO: Write this function
	return;
}

/* Converts radians to degrees */
double potentialfields::rad2deg(double radians)
{
	return (radians*180)/M_PI;
}

/* Converts degrees to radians */
double potentialfields::deg2rad(double degrees)
{
	return (degrees*M_PI)/180;
}

/* Converts angle ccw from due East to angle cw from due North */
double potentialfields::vec2bear(double ang)
{
	double bearing = (-ang+90);
	while(ang < 0)
	{
		ang += 360;
	}
	while(ang > 360)
	{
		ang -= 360;
	}
	return ang;
}
