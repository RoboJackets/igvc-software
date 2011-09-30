#ifndef _POTENTIAL_FIELDS_H_
#define _POTENTIAL_FIELDS_H_

#include "gps_points.hpp"
#include <vector>
#include "gps_common.hpp"

/* This struct encapsulates GPS data */
struct GPS_point
{
	GPS_point(double i_lat, double i_lon, double i_ang=0)
	{
		lat = i_lat;
		lon = i_lon;
	}
	double lat;
	double lon;
	double ang;
};

/* enum for deciding what to do in doSomethingforIndexesInRadius */
enum RAD_OPTION {FILL0, FILL1, OBSTACLES, ATTRACTORS};

/* structure for encapsolating return data fo doSomethingforIndexesInRadius */
struct ReturnData
{
	ReturnData()
	{
		x_vel = 0;
		y_vel = 0;
	}
	double x_vel;
	double y_vel;
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
	int robotlocx;			// Current x position of robot
	int robotlocy;			// Currnet y position of robot

	// Constants
	const static double meters_per_pixel = 1;	// Allows conversion from image to real distances
	const static int robot_radius = 0;		// Radius of the robot in pixels of the input boolean array
	const static double obstacle_weight = 1;	// Weight given to avoiding obstacles
	const static double image_goal_weight = 1;	// Weight given to get to image goals (flags)
	const static double gps_goal_weight = 1;	// Weight given to get to GPS goal
	const static double gps_avoid_weight = 1;	// Weight given to avoid old GPS points
	const static int obstacle_avoid_radius = 1; // Radius around the robot in which the robot considers those obstacles 
	const static int target_reach_radius = 1;	// Radius arond the robot in which the robot considers image goals
	const static double gps_goal_radius = 1;	// Radius of the gps goal
	const static double gps_max_distance = 1;	// Radius at which the attraction to the goal becomes a constant

	// Private Methods
	void removeclumps(bool* obstacles);
	void radiusfix(bool* obstacles);
	void getAvoidVec(bool* obstacles, double& xvel, double& yvel);
	void getImgTargetVec(bool* targets, double& xvel, double& yvel);
	void getGPSTargetVec(double& xvel, double& yvel);
	void getGPSAvoidVec(double& xvel, double& yvel);
	void convertToRealxy(double& xcomp, double& ycomp);
	void convertToRealVec(double& vel, double& ang);
	bool get2Dindexvalue(bool* array, int x, int y);
	void set2Dindexvalue(bool* array, int x, int y, bool val);
	void fillinRadius(bool* obstacles, int x, int y, int radius);
	void attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& y_vel);
	void repulsivePixels(int x0, int y0, int xt, int yt, int radius, double& x_vel, double& y_vel);
	void doSomethingforIndexesInRadius(int x0, int y0, int radius, bool* bitmap, RAD_OPTION OPTION, ReturnData data);
	double distBtwGPSPoints(const GPS_point& a, const GPS_point& b);
	void medianThreshFileter(bool* array, int thresh_size);
};

#endif
