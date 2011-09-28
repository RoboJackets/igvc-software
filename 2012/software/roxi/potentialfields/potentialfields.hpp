#ifndef _POTENTIAL_FIELDS_H_
#define _POTENTIAL_FIELDS_H_

#include "gps_points.hpp"
#include <vector>

/* This class encapsulates GPS data */
class GPS_point
{
public:
	GPS_point(double i_lat, double i_lon, double i_ang=0)
	{
		lat = i_lat;
		lon = i_lon;
		
	}
private:
	double lat;
	double lon;
	double ang;
};

/* This class provides an implementation for the Potential Fields Algorithm. It also manages
GPS goal waypoints, and keeps track of the current GPS position */
class potentialfields
{
public: 
	// Constructor
	potentialfields();
	// Destructor
	~potentialfields();
	// Public methods
	void dropWaypoint(double lat, double lon, double ang);
	void getNextVector(bool* obstacles, bool* targets, int xsize, int ysize, int robotx, int roboty, double& vel_mag, double& vel_ang);
		

private:	
	// Private Members
	std::vector <GPS_point> GPS_Goals;	// GPS waypoints of the goals
	std::vector <GPS_point> GPS_Prev_Loc;	// GPS waypoints of previous locations
	int currentGoal;			// Current GPS goal waypoint
	int xsize;				// Size of bitmaps (x)
	int ysize;				// Size of bitmaps (y)

	// Constants
	const static double meters_per_pixel = 1;	// Allows conversion from image to real distances
	const static int robot_radius = 0;		// Radius of the robot in pixels of the input boolean array
	const static double obstacle_weight = 1;	// Weight given to avoiding obstacles
	const static double image_goal_weight = 1;	// Weight given to get to image goals (flags)
	const static double gps_goal_weight = 1;	// Weight given to get to GPS goal
	const static double gps_avoid_weight = 1;	// Weight given to avoid old GPS points

	// Private Methods
	void removeclumps(bool* obstacles);
	void radiusfix(bool* obstacles);
	void getAvoidVec(bool* obstacles, double& xvel, double& yvel);
	void getImgTargetVec(bool* targets, double& xvel, double& yvel);
	void getGPSTargetVec(double& xvel, double& yvel);
	void getGPSAvoidVec(double& xvel, double& yvel);
	void convertToRealxy(double& xcomp, double& ycomp);
	void convertToRealVec(double& vel, double& ang);
	void xyToVec(double x, double y, double& mag, double& ang);
	void VecToxy(double mag, double ang, double& x, double& y);
	void AddVecs(double* xvals, double* yvals, int numVecs, double& xnet, double& ynet);
	bool get2Dindexvalue(bool* array, int x, int y);
	void set2Dindexvalue(bool* array, int x, int y, bool val);
	void fillinRadius(bool* obstacles, int x, int y, int radius);
	double rad2deg(double rads);
	double deg2rad(double degs);
	double vec2bear(double ang);
	void attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& yvel);
	void repulsivePixels(int x0, int y0, int xt, int yt, double& x_vel, double& yvel);
	void getIndexesInRadius(int x0, int y0, int radius, int* xinds, int* yinds, int& numinds);
};

#endif
