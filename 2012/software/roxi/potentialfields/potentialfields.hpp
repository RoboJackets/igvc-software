#ifndef _POTENTIAL_FIELDS_H_
#define _POTENTIAL_FIELDS_H_

// Potential Fields Algorithm Implementation Written by Kenny Marino

// Angles in bearings 0-360. Please provide it angles in bearings!

// Says whether or not testing mode is on. In testing mode, access to private methods is made public.
// Should be commented out unless currently testing the functions
//#define TESTINGMODE

#include "gps_points.hpp"
#include <vector>
#include "gps_common.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Point2D.h"
#include <math.h>

/* This struct encapsulates GPS data */
// Note all angles should be in bearings with 0 at due north
struct GPS_point
{
	GPS_point(double i_lat, double i_lon, double i_ang=0)
	{
		lat = i_lat;
		lon = i_lon;
		ang = i_ang;
	}
	double lat;
	double lon;
	double ang;
};

/* enum for deciding what to do in doSomethingforIndexesInRadius */
enum RAD_OPTION {FILL0, FILL1, OBSTACLES, ATTRACTORS};

/* enums to decide  how to convert an image */
enum IMAGETYPE {FEATURE_HIGH, FEATURE_LOW};
enum FEATURETYPE {OBSTACLE, ATTRACTOR};

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
	void getNextVector(IplImage* obstacles, IplImage* targets, CvPoint robotBaseAt, CvPoint robotLookingAt, Point2D<int>& goal);
		
#ifndef TESTINGMODE
	// If testing mode is on, all member functions and members are public so they can be tested directly by the testing function
	// Otherwise, the members should all be private
	private:
#endif
	// Private Members
	std::vector <GPS_point> GPS_Goals;	// GPS waypoints of the goals
	std::vector <GPS_point> GPS_Prev_Loc;	// GPS waypoints of previous locations
	int currentGoal;			// Current GPS goal waypoint
	int xsize;				// Size of bitmaps (x)
	int ysize;				// Size of bitmaps (y)
	int robotlocx;			// Current x position of robot
	int robotlocy;			// Currnet y position of robot
	double curlat;			// Current latitude of the robot
	double curlon;			// Current longitude of the robot
	double curang;		// Current angle 		
	double imgAngle;		// Angle (in bearings) of the current image

	// Constants
	const static double meters_per_pixel = 2;	// Allows conversion from image to real distances
	const static int robot_radius = 2;		// Radius of the robot in pixels of the input boolean array
	const static double obstacle_weight = 1;	// Weight given to avoiding obstacles
	const static double image_goal_weight = 1;	// Weight given to get to image goals (flags)
	const static double gps_goal_weight = 1;	// Weight given to get to GPS goal
	const static double gps_avoid_weight = 1;	// Weight given to avoid old GPS points
	const static int obstacle_avoid_radius = 10000; // Radius around the robot in which the robot considers those obstacles 
	const static int target_reach_radius = 10000;	// Radius arond the robot in which the robot considers image goals
	const static double gps_goal_radius = 1;	// Radius of the gps goal
	const static double gps_max_distance = 1;	// Radius at which the attraction to the goal becomes a constant
	const static double obstacle_bitmap_thresh = 100;	// Threshold value for converting obstacle images to bitmaps
	const static double attractor_bitmap_thresh = 100;	// Threshold value for converting attractor images to bitmaps

	// Private Methods
	void removeclumps(bool* obstacles);
	void radiusfix(bool* obstacles);
	void getAvoidVec(bool* obstacles, double angle_of_map, double& xvel, double& yvel);
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
	void doSomethingforIndexesInRadius(int x0, int y0, int radius, bool* bitmap, RAD_OPTION OPTION, ReturnData* data);
	double distBtwGPSPoints(const GPS_point& a, const GPS_point& b);
	double angleBtwGPSPoints(const GPS_point& a, const GPS_point& b);
	void medianThreshFileter(bool* array, int thresh_size);
	void updateCurLocation();
	void printbitmap(bool* bitmap);
	void IPl2Bitmap(IplImage* img, IMAGETYPE imgType, FEATURETYPE featType, bool* bitmap, int& xsize, int& ysize);
	void setOutputs(double vel_mag, double vel_ang, Point2D<int>& goal);double GetMapAngle(CvPoint robotBaseAt, CvPoint robotLookingAt);
	double getMapAngle(CvPoint robotBaseAt, CvPoint robotLookingAt);
};

#endif
