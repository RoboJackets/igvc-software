#include "potentialfields.hpp"
#include "gps_points.hpp"
#include "commonVecOps.hpp"

#include "image_buffers.h"

#include <stdio.h>
#include <string.h>

#include <iostream>
#include <fstream>
#include "XmlConfiguration.h"

using namespace std;

/*************** Public Functions ***********************************************************************************************/
// Creates new potentialfields object and initializes fields 
#if RUN_MODE == GPS
potentialfields::potentialfields()
{
	// Initialize the goal GPS points
	for (int i = 0; i < num_GPS_Goals; i++)
	{
		GPS_Goals.push_back(GPS_point(goalWaypointLat[i], goalWaypointLon[i]));
	}
	currentGoal = 0;
	loadXML();
	return;
}
#elif RUN_MODE == ROBOT_POS
potentialfields::potentialfields(GPS_point center_point)
{
	for (int i=0; i M num_GPS_Goals; i++)
	
		GPS_point goalpoint = GPS_point(goalWaypointLat[i], goalWaypointLon[i]);
		double distance = distBtwGPSPoints(center_point, goal_point);
		double angle = angleBtwGPSPoints(center_point, goal_point);
		double x = distance*sin(deg2rad(angle));
		double y = distance*cos(deg2rad(angle));
		Pos_Goals.push_back(Pos_point(x,y));
	}
	currentGoal = 0;
	loadXML();
	return;
}
#endif

// Destructor for potentialfields
potentialfields::~potentialfields()
{
	return;
}

// Adds the robot's current GPS waypoint to the class
#if RUN_MODE == GPS
void potentialfields::dropWaypoint(double lat, double lon, double ang)
{
	//cout << "lat: " << lat << endl << "lon: " << lon << endl << "ang: " << ang << endl;
	bool isEmpty = GPS_Prev_Loc.empty();
	GPS_Prev_Loc.push_back(GPS_point(lat, lon, ang));
	
	// If this is the first GPS point, then this needs to be added as the final goal for the robot (the course is circular) 	
	if (isEmpty)
	{
		GPS_Goals.push_back(GPS_point(lat, lon, ang));
	}

	// If the current GPS location is within the right radius of the goal GPS location, index up the goal GPS point
	// If the goal is out of bounds, just start back at 0
	if (distBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1], GPS_Goals[currentGoal]) < gps_goal_radius)
	{
		currentGoal = (currentGoal+1) % GPS_Goals.size();
	}

	// Write to GPS points file
	ofstream gpsfile;
	gpsfile.open("gps.txt", ios::app);
	gpsfile << "lat: " << lat << "\tlon: " << lon << "\tangle: " << ang << endl;
}
#elif RUN_MODE == ROBOT_POS
void potentialfields::dropWaypoint(double x, double y, double ang)
{
	bool isEmpty = Pos_Prev_Loc.empty()
	Pos_Prev_Loc.push_back(Pos_point(x,y,ang));
	
	// If this is the first real-world position, then this needs to be added as the final goal for the robot (the course is circular)	
	if (isEmpty)
	{
		Pos_Goals.push_back(Pos_point(x,y,ang));
	}
	
	// If the current real-world location is within the right radius of the goal location, index up the next goal point
	// If the goal is out of bounds, just start back at 0
	double x1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].x;
	double y1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].y;
	double x2 = Pos_Goals[currentGoal].x;
	double y2 = Pos_Goals[currentGoal].y;
	if (Distance2D(x1, y1, x2, y2) < gps_goal_radius)
	{
		currentGoal = (currentGoal+1) % Pos_Goals.size();
	} 
}
#endif

// Public function for running potential fields
void potentialfields::getVectorMotor(IplImage* obstacles_ipl, IplImage* targets_ipl, CvPoint robotBaseAt, CvPoint robotLookingAt, Point2D<int>& goal)
{
	double vel_mag, vel_ang;
	getNextVector(INSTANT, obstacles_ipl, targets_ipl, robotBaseAt, robotLookingAt, vel_mag, vel_ang, 0, 0);

	cout << "vel_mag " << vel_mag << "\nvel_ang " << vel_ang << endl;

	// Finally, round the values, put them in range and set them to the output point
	setOutputs(vel_mag, vel_ang, goal);
	
	cout << "goal.x " << goal.x << "\ngoal.y " << goal.y << endl; 
}

void potentialfields::getCompleteVector(IplImage* obstacles_ipl, IplImage* targets_ipl, CvPoint robotBaseAt, CvPoint robotLookingAt, Point2D<int>& goal)
{
	std::priority_queue<PFieldNodeShell> open_set;	// The set of tentative nodes to be evaluated
	std::vector<indexNode> visited_set;		// Indexes of nodes already visited

	// Create the first node
	PFieldNode* root = new PFieldNode();
	updateCurLocation();
	root->robot_angle = curang;
	double dist = getDistCur2Goal();
	double angle = getAngleCur2Goal();
	root->dist_from_goal_m = dist;
	root->angle_to_goal = angle;
	root->x_dist_from_goal_m = dist*sin(deg2rad(angle));
	root->y_dist_from_goal_m = dist*cos(deg2rad(angle));
	root->x_ind_from_goal = floor(root->x_dist_from_goal_m/stepsize_m + 0.5);
	root->y_ind_from_goal = floor(root->y_dist_from_goal_m/stepsize_m + 0.5);
	root->g_score = 0;
	calculateHScore(root);
	root->f_score = root->g_score + root->h_score;
	root->robotBaseAt = robotBaseAt;
	root->robotLookingAt = robotLookingAt;
	getNextVector(ASTAR, obstacles_ipl, targets_ipl, root->robotBaseAt, root->robotLookingAt, root->field_strength, root->field_direction, dist, angle);
	root->prev = NULL;
	open_set.push(PFieldNodeShell(root));
	visited_set.push_back(indexNode(root->x_ind_from_goal, root->y_ind_from_goal));
	
	//cout << "root node: " << root << endl;

	bool solutionFound = false;
	PFieldNode* solNode;
	// Loops until a solution is found
	while(!solutionFound)
	{
		PFieldNodeShell curNodeTemp = open_set.top();
		solNode = curNodeTemp.node;
		//cout << "cur node: " << solNode << endl;
		//cout << "prev: " << solNode->prev << endl;
		open_set.pop();
		if (checkForSolution(solNode) == true)
		{
			solutionFound = true;
		}
		else
		{
			expandNode(solNode, obstacles_ipl, targets_ipl);
			if (notVisited(visited_set, solNode->next_c))
			{
				open_set.push(PFieldNodeShell(solNode->next_c));
				visited_set.push_back(indexNode(solNode->next_c));
			}
			if (notVisited(visited_set, solNode->next_l))
			{
				open_set.push(PFieldNodeShell(solNode->next_l));
				visited_set.push_back(indexNode(solNode->next_l));
			}
			if (notVisited(visited_set, solNode->next_r))
			{
				open_set.push(PFieldNodeShell(solNode->next_r));
				visited_set.push_back(indexNode(solNode->next_r));
			}			
			//if (solNode->potential < 0)
			//	addPotential(open_set, 1000);
		}
	} 

	//cout << "Found a solution" << endl;

	// Find  topmost node (after initial position)
	bool topFound = false;
	PFieldNode* lastNode = NULL;
	PFieldNode* curNode = solNode;
	//cout << "cur " <<  curNode << endl;
	//cout << "prev " << curNode->prev << endl;
	while(!topFound)
	{
		//cout << "cur " <<  curNode << endl;
		//cout << "prev " << curNode->prev << endl;
		// If it's the top node
		if (curNode->prev == NULL)
		{
			//cout << "I free now?" << endl;
			// If it's the root node
			if (lastNode == NULL)
			{	
				// This shouldn't happen
				//cout << "Error: PField should have reset goals" << endl;
				goal.x = 0;
				goal.y = 0;
			}
			// If it's the center node
			else if (lastNode == curNode->next_c)
			{	
				//cout << "strength: " << curNode->field_strength << endl;
				//cout << "angle: " << lastNode->robot_angle << endl;
				// Use the solution from the first call to getNextVector
				setOutputs(curNode->field_strength, lastNode->robot_angle, goal);
			}
			// Else it's not the center
			else
			{
				// Use the angle, but set the magnitude to be way slower
				setOutputs(slow_speed, lastNode->robot_angle, goal);
			}
			topFound = true;
		}
		//cout << "What about here" << endl;
		lastNode = curNode;
		curNode = curNode->prev;
	}
	//cout << "I out" << endl;
	deleteTree(root);

	/*for (int i = 0; i < visited_set.size(); i++)
	{
		cout << i << "\nx_ind: " << visited_set[i].x_ind << endl;
		cout << "y_ind: " << visited_set[i].y_ind << endl;
	}*/
}

/********************************************************************************************************************************/

/*************** Big Picture Methods ********************************************************************************************/

// Changes value of input references vel_mag and vel_ang with the velocity and angle determined by the potential fields algorithm.
// Angle given by a value between 0 and 360 with 0 at due North
void potentialfields::getNextVector(NEXT_MODE mode, IplImage* obstacles_ipl, IplImage* targets_ipl, CvPoint robotBaseAt, CvPoint robotLookingAt, double& out_mag, double& out_ang, double dist_from_goal_m, double angl_to_goal)
{
	/*cout << "Mode: " << mode << endl;
	cout << "obstacles: " << obstacles_ipl << endl;
	cout << "targets: " << targets_ipl << endl;
	cout << "base at x: " << robotBaseAt.x << endl;
	cout << "base at y: " << robotBaseAt.y << endl;
	cout << "looking at x: " << robotLookingAt.x << endl;
	cout << "looking at y: " << robotLookingAt.y << endl;
	cout << "dist_from_goal: " << dist_from_goal_m << endl;
	cout << "ang to goal: " << angl_to_goal << endl;*/

	/*IplImage* showImage = cvCloneImage(obstacles_ipl);
	cvCircle(showImage, robotBaseAt, 10, CV_RGB(255,0,0), 1);
	cv::imshow("mainWin", showImage);
	cvWaitKey(1);	*/

	// Transform the input image to a bitmap of obstacles
	int imgx, imgy;
	bool* obstacles = IPl2Bitmap(obstacles_ipl, FEATURE_LOW, OBSTACLE, imgx, imgy);
	
	
	IplImage * pfThresh=cvCloneImage(obstacles_ipl);
	cvThreshold(obstacles_ipl,pfThresh,obstacle_bitmap_thresh,255,CV_THRESH_BINARY_INV);
	ImageBufferManager::getInstance().setpfThresh(pfThresh);
	//printbitmap(obstacles);
	
	xsize = imgx;
	ysize = imgy;
	//cout << "xsize: " << xsize << endl;
	//cout << "ysize: " << ysize << endl;
	//printbitmap(obstacles);	

	// Transform the input image to a bitmap of targets
	int tempx, tempy;
	bool* targets;
	if (targets_ipl == NULL)
	{
		targets = NULL;
		tempx = xsize;
		tempy = ysize;
	}
	else
	{
		targets = IPl2Bitmap(targets_ipl, FEATURE_LOW, ATTRACTOR, tempx, tempy);
	}
	if (xsize != tempx || ysize != tempy)
	{
		//cout << "Input images not the same size" << endl;
		// TODO: Should probably actually do something about this. Don't care right now.
	}

	if (mode == INSTANT)
	{
		// Update the current GPS location
		updateCurLocation();	
	}	

	// Set the robot's current location on the bitmap
	robotmaplocx = robotBaseAt.x;
	robotmaplocy = robotBaseAt.y;

	// Figure out what direction the map is facing relative toworld coordinates
	double map_ang_from_world = GetMapAngle(robotBaseAt, robotLookingAt);	

	// Alter the bitmap to remove stray clumps of non-obstacles
	removeclumps(obstacles);

	// Increase the size of obstacles so robot doesn't try to squeese through
	radiusfix(obstacles);

	double obstaclex, obstacley, imagetarx, imagetary, gpstarx, gpstary, gpsavoidx, gpsavoidy;

	// Get the vector contribution from the obstacles on the bitmap
	getAvoidVec(obstacles, map_ang_from_world, obstaclex, obstacley);
	/*cout << "obstacle x " << obstaclex << endl;
	cout << "obstacle y " << obstacley << endl;*/
	
	// Get the vector contribution from the goals on the bitmap
	getImgTargetVec(targets, imagetarx, imagetary);

	// Get the vector contribution from the GPS goal(s)
	if (mode == INSTANT)
	{
		getGPSTargetVec(gpstarx, gpstary);
	}
	else
	{
		getGPSTargetVec(gpstarx, gpstary, dist_from_goal_m, deg2rad(angl_to_goal));
	}
	/*out << "gps x " << gpstarx << endl;
	cout << "gps y " << gpstary << endl;
	cout << "dist " << dist_from_goal_m << endl;
	cout << "angle " << angl_to_goal << endl;*/

	// Get the vector contribution from the GPS past goal(s)
	getGPSAvoidVec(gpsavoidx, gpsavoidy);

	int number_vecs = 4;
	double xcomps[] = {obstaclex, imagetarx, gpstarx, gpsavoidx};
	double ycomps[] = {obstacley, imagetary, gpstary, gpsavoidy};	
	double xnet, ynet;

	cout << "obstaclex: " << obstaclex << endl << "obstacley: " << obstacley << endl;
	cout << "gpstarx: " << gpstarx << endl << "gpstary: " << gpstary << endl;

	// Find the resulting vector by adding up all of the components
	AddVecs(xcomps, ycomps, number_vecs, xnet, ynet);
	
	//cout << "xnet: " << xnet << endl << "ynet: " << ynet << endl;

	double vel_mag, vel_ang;	 
	xyToVec(xnet, ynet, vel_mag, vel_ang);	
	
	cout << "vel_mag: " << vel_mag << endl << "vel_ang: " << vel_ang << endl;

	// Rotates the velocity vector to the angle from the perspective of the robot
	vel_ang = RotateBearing(vel_ang, -curang);

	//cout << "curang: " << curang << endl;
	out_mag = vel_mag;
	out_ang = vel_ang; 
	
	delete obstacles;

	return;
}

/********************************************************************************************************************************/

/*************** A* Methods *****************************************************************************************************/
// Calculates the h_score at the given node
void potentialfields::calculateHScore(PFieldNode* node)
{
	// H score is the straight line number of steps to the goal times an estimated min potential
	double numsteps = node->dist_from_goal_m / stepsize_m;
	node->h_score = numsteps*guessed_min_potential;
}

// Creates three child nodes and calculates necessary paramater for those nodes
void potentialfields::expandNode(PFieldNode* node, IplImage* obstacles_ipl, IplImage* targets_ipl)
{
	//cout << "node: " << node << endl;
	double out_mag, out_ang, dist, ang;
	
	out_mag = node->field_strength;
	out_ang = node->field_direction;
	PFieldNode* center = new PFieldNode();
	PFieldNode* left = new PFieldNode();
	PFieldNode* right = new PFieldNode();

	double x_traveled, y_traveled, x_from_goal, y_from_goal;
	int x_pixels, y_pixels; 
	CvPoint robotBaseAt, robotLookingAt;	

	// Set fields for center node
	center->robot_angle = out_ang;
	x_traveled = stepsize_m * sin(deg2rad(out_ang));
	y_traveled = -(stepsize_m * cos(deg2rad(out_ang)));
	x_from_goal = node->x_dist_from_goal_m - x_traveled;
	y_from_goal = node->y_dist_from_goal_m + y_traveled;	
	dist = sqrt(x_from_goal*x_from_goal + y_from_goal*y_from_goal);
	center->dist_from_goal_m = dist;
	center->angle_to_goal = atan2(y_from_goal, x_from_goal);
	center->x_dist_from_goal_m = x_from_goal;
	center->y_dist_from_goal_m = y_from_goal;
	center->x_ind_from_goal = floor(center->x_dist_from_goal_m/stepsize_m + 0.5);
	center->y_ind_from_goal = floor(center->y_dist_from_goal_m/stepsize_m + 0.5);
	x_pixels = node->robotBaseAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotBaseAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotBaseAt.x = x_pixels;
	robotBaseAt.y = y_pixels;
	center->robotBaseAt = robotBaseAt;
	x_pixels = node->robotLookingAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotLookingAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotLookingAt.x = x_pixels;
	robotLookingAt.y = y_pixels;
	center->robotLookingAt = robotLookingAt;
	// Get potential
	dist = sqrt(center->x_dist_from_goal_m*center->x_dist_from_goal_m+center->y_dist_from_goal_m*center->y_dist_from_goal_m);
	ang = vec2bear(rad2deg(atan2(center->y_dist_from_goal_m, center->x_dist_from_goal_m)));
	getNextVector(ASTAR, obstacles_ipl, targets_ipl, center->robotBaseAt, center->robotLookingAt, center->field_strength, center->field_direction, dist, ang);
	center->g_score = node->g_score + center->field_strength*stepsize_m;
	calculateHScore(center);
	center->f_score = center->g_score + center->h_score;
	//cout << "Center potential: " << center->field_strength << endl;
	//cout << "Center g_score: " << center->g_score << endl;
	center->prev = node;

	// Set fields for left node
	left->robot_angle = RotateBearing(out_ang,-90);
	x_traveled = stepsize_m * sin(deg2rad(left->robot_angle));
	y_traveled = -(stepsize_m * cos(deg2rad(left->robot_angle)));
	x_from_goal = node->x_dist_from_goal_m - x_traveled;
	y_from_goal = node->y_dist_from_goal_m + y_traveled;	
	dist = sqrt(x_from_goal*x_from_goal + y_from_goal*y_from_goal);
	left->dist_from_goal_m = dist;
	left->angle_to_goal = atan2(y_from_goal, x_from_goal);
	left->x_dist_from_goal_m = x_from_goal;
	left->y_dist_from_goal_m = y_from_goal;	
	left->x_ind_from_goal = floor(left->x_dist_from_goal_m/stepsize_m + 0.5);
	left->y_ind_from_goal = floor(left->y_dist_from_goal_m/stepsize_m + 0.5);
	x_pixels = node->robotBaseAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotBaseAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotBaseAt.x = x_pixels;
	robotBaseAt.y = y_pixels;
	left->robotBaseAt = robotBaseAt;
	x_pixels = node->robotLookingAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotLookingAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotLookingAt.x = x_pixels;
	robotLookingAt.y = y_pixels;
	left->robotLookingAt = robotLookingAt;
	// Get potential
	dist = sqrt(left->x_dist_from_goal_m*left->x_dist_from_goal_m+left->y_dist_from_goal_m*left->y_dist_from_goal_m);
	ang = vec2bear(rad2deg(atan2(left->y_dist_from_goal_m, left->x_dist_from_goal_m)));
	getNextVector(ASTAR, obstacles_ipl, targets_ipl, left->robotBaseAt, left->robotLookingAt, left->field_strength, left->field_direction, dist, ang);
	left->g_score = node->g_score + left->field_strength*stepsize_m;
	calculateHScore(left);
	left->f_score = left->g_score + left->h_score;
	//cout << "Left potential: " << left->field_strength << endl;
	//cout << "Left g_score: " << left->g_score << endl;
	left->prev = node;

	// Set fields for right node
	right->robot_angle = RotateBearing(out_ang, 90);
	x_traveled = stepsize_m * sin(deg2rad(right->robot_angle));
	y_traveled = -(stepsize_m * cos(deg2rad(right->robot_angle)));
	x_from_goal = node->x_dist_from_goal_m - x_traveled;
	y_from_goal = node->y_dist_from_goal_m + y_traveled;	
	dist = sqrt(x_from_goal*x_from_goal + y_from_goal*y_from_goal);
	right->dist_from_goal_m = dist;
	right->angle_to_goal = atan2(y_from_goal, x_from_goal);
	right->x_dist_from_goal_m = x_from_goal;
	right->y_dist_from_goal_m = y_from_goal;	
	right->x_ind_from_goal = floor(right->x_dist_from_goal_m/stepsize_m + 0.5);
	right->y_ind_from_goal = floor(right->y_dist_from_goal_m/stepsize_m + 0.5);	
	x_pixels = node->robotBaseAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotBaseAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotBaseAt.x = x_pixels;
	robotBaseAt.y = y_pixels;
	right->robotBaseAt = robotBaseAt;
	x_pixels = node->robotLookingAt.x + floor((x_traveled / meters_per_pixel_const)+.5);
	y_pixels = node->robotLookingAt.y + floor((y_traveled / meters_per_pixel_const)+.5);
	robotLookingAt.x = x_pixels;
	robotLookingAt.y = y_pixels;
	right->robotLookingAt = robotLookingAt;
	// Get potential
	dist = sqrt(right->x_dist_from_goal_m*right->x_dist_from_goal_m+right->y_dist_from_goal_m*right->y_dist_from_goal_m);
	ang = vec2bear(rad2deg(atan2(right->y_dist_from_goal_m, right->x_dist_from_goal_m)));
	getNextVector(ASTAR, obstacles_ipl, targets_ipl, right->robotBaseAt, right->robotLookingAt, right->field_strength, right->field_direction, dist, ang);
	right->g_score = node->g_score + right->field_strength*stepsize_m;
	calculateHScore(right);
	right->f_score = right->g_score + right->h_score;
	//cout << "Right potential: " << right->field_strength << endl;
	//cout << "Right g_score: " << right->g_score << endl;
	right->prev = node;

	IplImage* showImage = cvCloneImage(obstacles_ipl);
	cvCircle(showImage, node->robotBaseAt, 10, CV_RGB(255,0,0), 10);
	cvCircle(showImage, left->robotBaseAt, 3, CV_RGB(0,0,255), 1); 
	cvCircle(showImage, right->robotBaseAt, 3, CV_RGB(0,0,255), 1); 
	cvCircle(showImage, center->robotBaseAt, 3, CV_RGB(0,0,255), 1); 
	CvPoint goal_at;
	//cout << "goal x in meters: " << node->x_dist_from_goal_m << endl;
	//cout << "goal y in meters: " << node->y_dist_from_goal_m << endl;
	goal_at.x = floor((node->x_dist_from_goal_m)/meters_per_pixel_const + .5) + node->robotBaseAt.x;
	goal_at.y = -floor((node->y_dist_from_goal_m)/meters_per_pixel_const + .5) + node->robotBaseAt.y;
	cvCircle(showImage, goal_at, 5, CV_RGB(0,255,0), 5);
	//cout << "goal x: " << goal_at.x << "\ngoal y: " << goal_at.y << endl;
	cv::imshow("mainWin", showImage);
	cout << "field strength " << node->field_strength << endl;
	cout << "g score " << node->g_score << endl;
	cout << "h score " << node->h_score << endl;
	cout << "f score " << node->f_score << endl;
	cvWaitKey(1);

	// Adds new nodes to root node
	node->next_c = center;
	node->next_l = left;
	node->next_r = right;

	assert(node == node->next_c->prev);
	assert(node == node->next_l->prev);
	assert(node == node->next_r->prev);
}

// Checks to see if the input node is close enough to the solution
bool potentialfields::checkForSolution(PFieldNode* node)
{
	if (node->dist_from_goal_m < gps_goal_radius)
		return true;
	else
		return false;
}

// Returns true if the node has not yet been visited
bool potentialfields::notVisited(std::vector<indexNode>& visited_set, PFieldNode* node)
{
	for (uint i = 0; i < visited_set.size(); i++)
	{
		indexNode curNode = visited_set[i];
		if (curNode.x_ind == node->x_ind_from_goal && curNode.y_ind == node->y_ind_from_goal)
			return false;
	}
	return true;
}

// Adds a number to all of the potentials in the open set
/*void potentialfields::addPotential(std::priority_queue<PFieldNodeShell>& open_set, double potential)
{	
	cout << "Add potential !!!!!!!" << endl;
	std::vector<PFieldNodeShell> temp;
	while(!open_set.empty())
	{
		PFieldNodeShell curNodeTemp = open_set.top();
		temp.push_back(PFieldNodeShell(curNodeTemp.node));
		open_set.pop();
	}
	for (uint i = 0; i < temp.size(); i++)
	{	
		PFieldNode* curNodeTemp = temp[i].node;
		curNodeTemp->potential += potential;
		//cout << "New potential: " << curNodeTemp->potential << endl;
		open_set.push(PFieldNodeShell(curNodeTemp));
	}		
	
	//open_set.push(PFieldNodeShell(root));
	//PFieldNodeShell curNodeTemp = open_set.top();
	//solNode = curNodeTemp.node;
	//open_set.pop();
}*/

// Deletes entire PFieldNode tree recursively
void potentialfields::deleteTree(PFieldNode* node)
{
	// Check and delete right tree
	if (node->next_l != NULL)
		deleteTree(node->next_l);
	if (node->next_c != NULL)
		deleteTree(node->next_c);
	if (node->next_r != NULL)
		deleteTree(node->next_r);
	delete node;
}

/********************************************************************************************************************************/

/*************** Map pre-processing functions ***********************************************************************************/
// Converts an IPlImage into a bitmap based on the thresholds in the header file 
bool* potentialfields::IPl2Bitmap(IplImage* img, IMAGETYPE imgType, FEATURETYPE featType, int& width, int& height)
{
	double thresh;
	// Sets which threshold we'll be using
	if (featType == OBSTACLE)
		thresh = obstacle_bitmap_thresh;
	else
		thresh = attractor_bitmap_thresh;

	height = img->height;
	width = img->width;
	int step = img->widthStep;
	int channels = img->nChannels;
	uchar* data = (uchar *)img->imageData;
	int index = 0;
	bool* bitmap = new bool[height*width];
	for(int hindex = 0; hindex < height; hindex++)
	{
		for(int windex = 0; windex < width; windex++)
		{
			// Finds average intensity over all the channels. If there's just one channel, this isn't strictly necessary
			double avgval = 0;
			for (int i = 0; i < channels; i++)
			{
				avgval += (double)data[hindex*step+windex*channels+i];
			}
			avgval /= channels;

			// If it's a feature high type, it is 1 if the average value is greater than the threshold. If it's
			// a feature low type, it is a 1 if the average value is less than the threshold. 
			bool isFeature;
			if (imgType == FEATURE_HIGH)
				isFeature = avgval > thresh;
			else
				isFeature = avgval < thresh;

			if(isFeature)
			{
				bitmap[index] = 1;
			}
			else
			{
				bitmap[index] = 0;
			}
			index++;
		}
	}
	//printbitmap(bitmap);
	return bitmap;
}

// Gets rid of stray clumps of grass and other noise from the obstacle bitfield
void potentialfields::removeclumps(bool* obstacles)
{
	obstacles = obstacles;
	// TODO: Write this function
	return;
}

// Adds radius of robot to all of the obstacles so that the robot doesn't try to fit into small gaps 
void potentialfields::radiusfix(bool* obstacles)
{
	std::vector<int> xinds;
	std::vector<int> yinds;
	for (int y = 0; y < ysize; y++)
	{
		for (int x = 0; x < xsize; x++)
		{
			// For each pixel in the bitmap, if it is an obstacle pixel, fill in all of the pixels around it
			// within the radius of the robot
			if (get2Dindexvalue(obstacles, x, y) == 1)
			{
				xinds.push_back(x);
				yinds.push_back(y);
			}
		}
	}

	for(int i=0; i < (int) xinds.size(); i++)
	{
		fillinRadius(obstacles, xinds[i], yinds[i], robot_radius);
	}

	return;
}

// Implementation of a median threshold filter as a possible solution to the clumps of grass problem
void potentialfields::medianThreshFilter(bool* array, int thresh_size)
{
	// Makes a copy of the array and then 0s out the old array
	bool arraycopy[xsize*ysize];
	memcpy(arraycopy, array, xsize*ysize);
	for(int i = 0; i < xsize*ysize; i++)
		array[i] = 0;
	 
	

	// Finds the square root of the perfect square at least twice the size of the threshold
	int n = ceil(sqrt(thresh_size*2));

	for(int xoff = 0; xoff < n; xoff++)
	{
		for(int yoff = 0; yoff < n; xoff++)
		{
			for(int x = xoff; (x+n) <= xsize; x+=n)
			{
				for(int y = yoff; (y+n) <= ysize; y+=n)
				{
					int sum = 0;
					for(int i = 0; i<n; i++)
					{
						for(int j = 0; j<n; j++)
						{
							sum += int(get2Dindexvalue(arraycopy, x+i, y+j));
						}
					}

					if(sum/(n*n) >= 0.5)
					{
						for(int i = 0; i<n; i++)
						{
							for(int j = 0; j<n; j++)
							{
								set2Dindexvalue(array, x+i, y+j, get2Dindexvalue(arraycopy,x+i,y+j));
							}
						}
					}
				} 
			}
		}
	}
}
/********************************************************************************************************************************/

/*************** Component vector functions for obstacles and goals *************************************************************/
// Returns the x and y components of the obstacle avoidance vector in meters 
void potentialfields::getAvoidVec(bool* obstacles, double angle_of_map, double& xvel, double& yvel)
{
	ReturnData data;

	// Return a sum of all of the x and y components from all of the obstacle pixels within the
	// predefined radius
	//cout << "robotmaplocx: " << robotmaplocx << " robotlocy: " << robotlocy << " obstacle_rad: " << obstacle_avoid_radius << endl;
	doSomethingforIndexesInRadius(robotmaplocx, robotmaplocy, obstacle_avoid_radius, obstacles, OBSTACLES, &data);

	//cout << "data.x_vel: " << data.x_vel << endl << "data.y_vel: " << data.y_vel << endl;

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * obstacle_weight * meters_per_pixel;
	yvel = data.y_vel * obstacle_weight * meters_per_pixel;

	// Rotate the vector based on the current bearing (assume that this is the orientation of the image)
	double mag, ang;
	xyToVec(xvel, yvel, mag, ang);
	ang = RotateBearing(ang, angle_of_map);
	void VecToxy(double mag, double ang, double& x, double& y);
	
	return;
}

// Returns the x and y components of the Image target vector in meters 
void potentialfields::getImgTargetVec(bool* targets, double& xvel, double& yvel)
{
	if (targets == 0)
	{
		xvel = 0;
		yvel = 0;
		return;
	}
	ReturnData data;

	// Return a sum of all of the x and y components from all of the goal pixels within the
	// predefined radius
	doSomethingforIndexesInRadius(robotmaplocx, robotmaplocy, target_reach_radius, targets, ATTRACTORS, &data);

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * image_goal_weight * meters_per_pixel;
	yvel = data.y_vel * image_goal_weight * meters_per_pixel;
	return;
}

// Returns the x and y components of the GPS goal vector in meters
void potentialfields::getGPSTargetVec(double& xvel, double& yvel)
{
	double distance = getDistCur2Goal();
	double theta = deg2rad(getAngleCur2Goal());
	getGPSTargetVec(xvel, yvel, distance, theta);
}

// Returns the x and y components of the GPS goal vector in meters
void potentialfields::getGPSTargetVec(double& xvel, double& yvel, double distance, double theta)
{
	// TODO: Figure out the distance and angle from the current GPS coordinate to the goal GPS coordinate
	if (distance > (gps_max_distance + gps_goal_radius))
	{	
		// If the robot is far away from the GPS goal, gets a constant vector towards the GPS location
		xvel = gps_goal_weight * gps_max_distance * sin(theta);
		yvel = gps_goal_weight * gps_max_distance * cos(theta);
	}
	else
	{
		// If the robot is close to the goal, vector towards the GPS location smaller and smaller
		xvel = gps_goal_weight*(distance - gps_goal_radius)*sin(theta);
		yvel = gps_goal_weight*(distance - gps_goal_radius)*cos(theta);
	}
	return;
}

// Returns the x and y components of the GPS avoidance vector in meters 
void potentialfields::getGPSAvoidVec(double& xvel, double& yvel)
{
	xvel = xvel;
	yvel = yvel;
	// TODO: Write this function
	// Write only if it becomes useful
	return;
}

/********************************************************************************************************************************/

/*************** Helper functions for bitmap obstacles and goals ****************************************************************/
// Finds the potential field vector for a single attractive pixel
void potentialfields::attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& y_vel)
{
	// Finds the distance between the pixels and the angle to go to the second pixel
	double d = Distance2D(x0, y0, xt, yt);
	double theta = atan2((yt-y0), (xt-x0));

	// Creates vectors pointing towards the goal pixel, proportional to the distance from the target
	x_vel = d*cos(theta);
	y_vel = d*sin(theta);

	//cout << "x/y: " << sqrt((x_vel/y_vel)*(x_vel/y_vel));

	// Reverses direction of y_vel because the image indexes for height start at the top, so all the
	// y values have to be reversed
	y_vel = -y_vel;
	return;
}

// Finds the potential field vector for a single repulsive pixel
void potentialfields::repulsivePixels(int x0, int y0, int xt, int yt, int radius, double& x_vel, double& y_vel)
{
	// Finds the distance between the pixels and the angle to go to the second pixel
	double d = Distance2D(x0, y0, xt, yt);
	double theta = atan2((yt-y0), (xt-x0));

	// Creates vectors pointing away from the obstacle pixel, proportional to the distance from the target
	//x_vel = -(radius-d)*cos(theta);
	//y_vel = -(radius-d)*sin(theta);
	if (d == 0)
	{
		x_vel = 0;
		y_vel = 0;
	}
	else
	{
		x_vel = -1/(d*d)*cos(theta);
		y_vel = -1/(d*d)*sin(theta);
	}
	//cout << "x/y: " << sqrt((x_vel/y_vel)*(x_vel/y_vel));

	// Reverses direction of y_vel because the image indexes for height start at the top, so all the
	// y values have to be reversed
	y_vel = -y_vel;
	return;
}

// Converts a pixel x and y disrance to a real 
void potentialfields::convertToRealxy(double& xcomp, double& ycomp)
{
	xcomp *= meters_per_pixel;
	ycomp *= meters_per_pixel;
	return;
}

// Converts a pixel vector to a real vector 
void potentialfields::convertToRealVec(double& vel, double& ang)
{
	ang = ang;
	vel *= meters_per_pixel;
	return;
}

/********************************************************************************************************************************/

/*************** Bitmap helpers *************************************************************************************************/
// Fills in 1's within the radius of the third input around the point (x,y) 
void potentialfields::fillinRadius(bool* obstacle, int x, int y, int radius)
{
	doSomethingforIndexesInRadius(x, y, radius, obstacle, FILL1, NULL);
	return;	
}
 
// Performs some action given by RAD_OPTION on every pixel within a certain radius of a given pixel
void potentialfields::doSomethingforIndexesInRadius(int x0, int y0, int radius, bool* bitmap, RAD_OPTION OPTION, ReturnData* data)
{
	//cout << "x0: " << x0 << "\ny0: " << y0 << "\nradius: " << radius << endl;
	// Starts at the lowest y up to the highest y possible within the radius
	for (int y = y0 - radius; y <= y0 + radius; y++)
	{
		//cout << "y: " << y << endl;
		// Checks to make sure the indexes are within the bounds of the bitmask
		if (y < 0 || y >= ysize)
			continue; 

		// Figures out the maximum x distance for the current y
		int xdistmax = floor(sqrt(radius*radius - (y0-y)*(y0-y))+.5);
		//cout << "xdistmax at y = " << y << " is " << xdistmax << endl;

		// Starts at the lowest x possible for the current y and goes to the highest x possible for the current y
		for(int x = x0 - xdistmax; x <= x0 + xdistmax; x++)
		{
			// Checks to make sure the indexes are within the bounds of the bitmask
			if (x < 0 || x >= xsize)
				continue;
			
			//cout << "I'm at (" << x << "," << y << ")\n";

			// Does different operations depending on the value of OPTION
			switch(OPTION)
			{
				// Fills all of the pixels in the radius with 0
				case FILL0:
					set2Dindexvalue(bitmap, x, y, 0);
					break;
				
				// Fills all of the pixels in the radius with 1
				case FILL1:
					set2Dindexvalue(bitmap, x, y, 1);
					break;
	
				// Finds the pixel repulsions and sums them together
				case OBSTACLES:
					//printbitmap(bitmap);
					if (get2Dindexvalue(bitmap, x, y) == 1)
					{
						//cout << "Here1\n";
						double curx_vel, cury_vel;
						repulsivePixels(x0, y0, x, y, radius, curx_vel, cury_vel);
						data->x_vel += curx_vel;
						data->y_vel += cury_vel;
						/*cout << "x0: " << x0 << endl;	
						cout << "y0: " << y0 << endl;
						cout << "x: " << x << endl;	
						cout << "y: " << y << endl;
						cout << "xvel: " << data->x_vel << endl;
						cout << "yvel: " << data->y_vel << endl;*/
					}
					break;

				// Finds the pixel attractions and sums them together
				case ATTRACTORS:
					if (get2Dindexvalue(bitmap, x, y) == 1)
					{
						//cout << "Here2\n";
						double curx_vel, cury_vel;
						attractorPixels(x0, y0, x, y, curx_vel, cury_vel);
						data->x_vel += curx_vel;
						data->y_vel += cury_vel;
						/*cout << "x0: " << x0 << endl;	
						cout << "y0: " << y0 << endl;
						cout << "x: " << x << endl;	
						cout << "y: " << y << endl;
						cout << "xvel: " << data->x_vel << " " ;*/
					}
					break;

				// Default (shouldn't happen)
				default:
					break;
			}
		}	
	}
	return;
}

// Returns the value at the index array[x][y] as if it were a 2D array
bool potentialfields::get2Dindexvalue(bool* array, int x, int y)
{
	/*cout << endl << "xsize is "  << xsize << endl;
	cout << endl << "(" << x << "," << y << ") is at index " << y*xsize+x << " in this array" << endl;*/
	return array[y*xsize+x];
}

// Sets the value at the index array[x][y] to val as if it were a 2D array
void potentialfields::set2Dindexvalue(bool* array, int x, int y, bool val)
{
	array[y*xsize+x] = val;
}
/********************************************************************************************************************************/

/*************** Real world-localization functions ******************************************************************************/
// Updates the current latitude, longitude and image angle based on latest GPS data
void potentialfields::updateCurLocation()
{
	// Finds the index of the latest GPS_point
	int curEl = GPS_Prev_Loc.size() - 1;
	
	// Updates curlat, curlon, and curAngle based on latest data	
	curlat = GPS_Prev_Loc[curEl].lat;
	curlon = GPS_Prev_Loc[curEl].lon;
	curang = GPS_Prev_Loc[curEl].ang;	

	return;
}

// Calculates the angle that the map is pointed rrelative to the world frame
double potentialfields::GetMapAngle(CvPoint robotBaseAt, CvPoint robotLookingAt)
{
	double angle = atan2((robotLookingAt.y-robotBaseAt.y), (robotLookingAt.x - robotBaseAt.x));
	angle = rad2deg(angle);
	angle += curang;
	angle = fmodf(90-angle, 360);
	return angle;
}

// Puts the calculated vector into the range of the output and round it to the nearest integer 
void potentialfields::setOutputs(double vel_mag, double vel_ang, Point2D<int>& goal)
{
	double x_real, y_real;
	
	vel_ang=(fmod((vel_ang+180),360)-180);        //put angle into -180 to 180
	vel_ang=vel_ang<-90?-90:vel_ang;	//if ang<-90 set to -90
	vel_ang=vel_ang> 90? 90:vel_ang;	//if ang> 90 set to  90
	
	VecToxy(vel_mag, vel_ang, x_real, y_real);
	
	goal.x = floor(x_real + 0.5);
	goal.y = floor(y_real + 0.5);	
}
/********************************************************************************************************************************/

/*************** Position related functions *************************************************************************************/
// Finds the distance in meters between two GPS points 
double potentialfields::distBtwGPSPoints(const GPS_point& a, const GPS_point& b)
{
	GPSState gpsa, gpsb;
	gpsa.lon = a.lon;
	gpsa.lat = a.lat;
	gpsb.lon = b.lon;
	gpsb.lat = b.lat;
	return lambert_distance(gpsa, gpsb);
}

// Finds the angle between two GPS points in degrees
double potentialfields::angleBtwGPSPoints(const GPS_point& a, const GPS_point& b)
{
	GPSState gpsa, gpsb;
	gpsa.lon = a.lon;
	gpsa.lat = a.lat;
	gpsb.lon = b.lon;
	gpsb.lat = b.lat;
	return lambert_bearing(gpsa, gpsb);
}

// Gets distance between the robot's current real world location and the goal
#if RUN_MODE == GPS
double potentialfields::getDistCur2Goal()
{
	return distBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1], GPS_Goals[currentGoal]);
}
#elif RUN_MODE == ROBOT_POS
double potentialfields::getDistCur2Goal()
{
	double x1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].x;
	double y1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].y;
	double x2 = Pos_Goals[currentGoal].x;	
	double y2 = Pos_Goals[currentGoal].y;	
	return Distance2D(x1, y1, x2, y2);
}
#endif

// Gets angle between the robot's current real world location and the goal
#if RUN_MODE == GPS
double potentialfields::getAngleCur2Goal()
{
	return angleBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1], GPS_Goals[currentGoal]);
}
#elif RUN_MODE == ROBOT_POS
double potentialfields::getAngleCur2Goal()
{
	double x1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].x;
	double y1 = Pos_Prev_Loc[Pos_Prev_Loc.size()-1].y;
	double x2 = Pos_Goals[currentGoal].x;	
	double y2 = Pos_Goals[currentGoal].y;	
	double theta = atan2((y2-y1),(x2-x1));
	double ang = vec2bear(rad2deg(theta));		
}
#endif


/********************************************************************************************************************************/

/*************** Various Helper functions ***************************************************************************************/
// Prints bitmap with 1's and 0's 
void potentialfields::printbitmap(bool* bitmap)
{
	for(int row=0; row<ysize; row++)
	{
		for(int col=0; col<xsize; col++)
		{
			cout << bitmap[row*xsize+col] << " ";
		}
		cout << endl;
	}
}

// Loads variables from the XML file
void potentialfields::loadXML()
{
	//XmlConfiguration cfg("../Config.xml");
	//meters_per_pixel = cfg.getFloat("meters_per_pixel");
	meters_per_pixel = meters_per_pixel_const;
}
/********************************************************************************************************************************/

