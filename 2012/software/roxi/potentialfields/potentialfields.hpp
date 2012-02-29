#ifndef _POTENTIAL_FIELDS_H_
#define _POTENTIAL_FIELDS_H_

/**
 * Potential Fields Algorithm Implementation
 * Note: Angles in bearings 0-360. Please provide it angles in bearings! 
 * @author Kenny Marino
*/ 

/************* #defines *******************/
#define RUN_MODE GPS
//#define RUN_MODE ROBOT_POS
//#define TESTINGMODE	// Says whether or not testing mode is on. In testing mode, access to private methods is made public.
			// Should be commented out unless currently testing the functions
/******************************************/

/************* #includes ******************/
#include "gps_points.hpp"
#include <vector>
#include "gps_common.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "Point2D.h"
#include <math.h>
#include <queue>
/******************************************/

/************* Structures *****************/
// This struct encapsulates GPS data
// Note all angles should be in bearings
// with 0 at due north
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

// This struct encapsulates robot position data
// Note all angles should be in bearings
// with 0 at due north
struct Pos_point
{
	Pos_point(double i_x, double i_y, double i_ang=0)
	{
		x = i_x;
		y = i_y;
		ang = i_ang;
	}
	double x;
	double y;
	double ang;
};

// Structure for encapsolating return data fo doSomethingforIndexesInRadius
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

// class for a node in potential fields with A*
class PFieldNode
{
public:
	double field_strength;
	double dist_from_goal_m;
	double angle_to_goal;
	double x_dist_from_goal_m;
	double y_dist_from_goal_m;
	double robot_angle;
	double g_score;
	double h_score;
	double f_score;
	CvPoint robotBaseAt;
	CvPoint robotLookingAt;	
	PFieldNode* prev;
	PFieldNode* next_c;
	PFieldNode* next_l;
	PFieldNode* next_r;		

	bool operator<(const PFieldNode &other) const {
    		return (this->f_score < other.f_score);
  	}

	PFieldNode()
	{
		prev = NULL;
		next_c = NULL;
		next_l = NULL;
		next_r = NULL;
	}
};


/******************************************/

/************* Enums **********************/
// enum for deciding what to do in doSomethingforIndexesInRadius 
enum RAD_OPTION {FILL0, FILL1, OBSTACLES, ATTRACTORS};

// enums to decide how to convert an image 
enum IMAGETYPE {FEATURE_HIGH, FEATURE_LOW};
enum FEATURETYPE {OBSTACLE, ATTRACTOR};

// enum to decide mode of getNextVector
enum NEXT_MODE {INSTANT, ASTAR};  
/******************************************/

/************* Main Class *****************/
// This class provides an implementation for the Potential Fields Algorithm. It also manages
// GPS goal waypoints, and keeps track of the current GPS position 
class potentialfields
{
public: 
	/************* Public Methods *************/	
	#if RUN_MODE == GPS
	potentialfields();					// Constructor
	void dropWaypoint(double lat, double lon, double ang);	// Adds new GPS points
	#elif RUN_MODE == ROBOT_POS
	potentialfields(GPS_Point center_point);		// Constructor with argument for the GPS location of point (0,0)
	void dropWaypoint(double x, double y, double ang);	// Adds new Robot position points
	#endif
	~potentialfields();					// Destructor
	void getVectorMotor(IplImage* obstacles_ipl, IplImage* targets_ipl, CvPoint robotBaseAt, CvPoint robotLookingAt, Point2D<int>& goal);
	void getCompleteVector(IplImage* obstacles_ipl, IplImage* targets_ipl, CvPoint robotBaseAt, CvPoint robotLookingAt, Point2D<int>& goal);
	// Returns a vector of where the robot should go next, instantanous and with the A* search
	
	/******************************************/
	
#ifndef TESTINGMODE
// If testing mode is on, all member functions and members are public so they can be tested directly by the testing function
// Otherwise, the members should all be private
private:
#endif
	/************* Private Members ************/
	#if RUN_MODE == GPS	
	std::vector <GPS_point> GPS_Goals;	// GPS waypoints of the goals
	std::vector <GPS_point> GPS_Prev_Loc;	// GPS waypoints of previous locations
	int currentGoal;			// Current GPS goal waypoint
	double curlat;				// Current latitude of the robot
	double curlon;				// Current longitude of the robot
	#elif RUN_MODE == ROBOT_POS
	std::vector <Pos_point> Pos_Goals;	// Real world positions of the goals
	std::vector <Pos_point> Pos_Prev_Loc;	// Real world positions of previous locations
	int currentGoalpos;			// Current position goal 
	double curx;				// Current real-world x of the robot
	double cury;				// Current real-world y of the robot
	#endif
	int xsize;				// Size of bitmaps (x)
	int ysize;				// Size of bitmaps (y)
	int robotmaplocx;			// Current x position of robot in map
	int robotmaplocy;			// Currnet y position of robot in map
	double curang;				// Current angle 		
	double imgAngle;			// Angle (in bearings) of the current image
	double meters_per_pixel;		// Allows conversion from image to real distances
	/******************************************/

	/************* Constants ******************/
	const static int robot_radius = 2;				// Radius of the robot in pixels of the input boolean array
	const static double obstacle_weight = 3.32e-5;			// Weight given to avoiding obstacles
	const static double image_goal_weight = 1;			// Weight given to get to image goals (flags)
	const static double gps_goal_weight = 510;			// Weight given to get to GPS goal
	const static double gps_avoid_weight = 1;			// Weight given to avoid old GPS points
	const static int obstacle_avoid_radius = 10000;		 	// Radius around the robot in which the robot considers those obstacles 
	const static int target_reach_radius = 1000;			// Radius arond the robot in which the robot considers image goals
	const static double gps_goal_radius = 1;			// Radius of the gps goal
	const static double gps_max_distance = 1;			;// Radius at which the attraction to the goal becomes a constant
	const static double obstacle_bitmap_thresh = 100;		// Threshold value for converting obstacle images to bitmaps
	const static double attractor_bitmap_thresh = 100;		// Threshold value for converting attractor images to bitmaps
	const static double stepsize_m = .1;				// Step size in meters
	const static double guessed_min_potential = 10;			// Potential used in calculation of heuristic
	const static double meters_per_pixel_const = 0.02;		// Meters per pixel constant
									
	/******************************************/

	/************* Private Methods ************/
	// Map pre-processing functions
	bool* IPl2Bitmap(IplImage* img, IMAGETYPE imgType, FEATURETYPE featType, int& imgx, int& imgy);
	void removeclumps(bool* obstacles);
	void radiusfix(bool* obstacles);
	void medianThreshFilter(bool* array, int thresh_size);

	// Big picture methods
	void getNextVector(NEXT_MODE mode, IplImage* obstacles, IplImage* targets, CvPoint robotBaseAt, CvPoint robotLookingAt, double& out_mag, double& out_ang, double dist_to_goal, double angle_from_goal);

	// A* search methods
	void calculateHScore(PFieldNode* node);
	void expandNode(PFieldNode* node, IplImage* obstacles_ipl, IplImage* targets_ipl);
	bool checkForSolution(PFieldNode* node);
	void deleteTree(PFieldNode* node);
								
	// Component vector functions for obstacles and goals
	void getAvoidVec(bool* obstacles, double angle_of_map, double& xvel, double& yvel);
	void getImgTargetVec(bool* targets, double& xvel, double& yvel);
	void getGPSTargetVec(double& xvel, double& yvel);
	void getGPSTargetVec(double& xvel, double& yvel, double distance, double theta);
	void getGPSAvoidVec(double& xvel, double& yvel);

	// Helper functions for bitmap obstacles and goals
	void attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& y_vel);
	void repulsivePixels(int x0, int y0, int xt, int yt, int radius, double& x_vel, double& y_vel);
	void convertToRealxy(double& xcomp, double& ycomp);
	void convertToRealVec(double& vel, double& ang);

	// Bitmap helpers
	void fillinRadius(bool* obstacles, int x, int y, int radius);
	void doSomethingforIndexesInRadius(int x0, int y0, int radius, bool* bitmap, RAD_OPTION OPTION, ReturnData* data);	
	bool get2Dindexvalue(bool* array, int x, int y);
	void set2Dindexvalue(bool* array, int x, int y, bool val);
		
	// Real world-localization functions
	void updateCurLocation();
	double GetMapAngle(CvPoint robotBaseAt, CvPoint robotLookingAt);
	void setOutputs(double vel_mag, double vel_ang, Point2D<int>& goal);

	// Position related functions
	double getDistCur2Goal();
	double getAngleCur2Goal();
	double distBtwGPSPoints(const GPS_point& a, const GPS_point& b);
	double angleBtwGPSPoints(const GPS_point& a, const GPS_point& b);
	
	// Various helper functions
	void printbitmap(bool* bitmap);
	void loadXML();
};
/******************************************/

#endif
