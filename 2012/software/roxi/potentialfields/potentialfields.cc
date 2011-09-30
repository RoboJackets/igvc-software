#include "potentialfields.hpp"
#include "gps_points.hpp"
#include "commonVecOps.hpp"
#include <stdio.h>
#include <string.h>

/* Creates new potentialfields object and initializes fields */
potentialfields::potentialfields()
{
	// Initialize the goal GPS points
	for (int i = 0; i < num_GPS_Goals; i++)
	{
		GPS_Goals.push_back(GPS_point(goalWaypointLat[i], goalWaypointLon[i]));
	}
	currentGoal = 0;
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
	
	// TODO: If the current GPS location is within the right radius of the goal GPS location, index up the goal GPS point
	// If the goal is out of bounds, just start back at 0
}

/* Changes value of input references vel_mag and vel_ang with the velocity and angle determined by the potential fields algorithm.
Angle given by a value between 0 and 360 with 0 at due North */
void potentialfields::getNextVector(bool* obstacles, bool* targets, int xsize, int ysize, int robotx, int roboty, double& vel_mag, double& vel_ang)
{
	// Set the robot's current location on the bitmap
	robotlocx = robotx;
	robotlocy = roboty;

	// Alter the bitmap to remove stray clumps of non-obstacles
	removeclumps(obstacles);

	// Increase the size of obstacles so robot doesn't try to squeese through
	radiusfix(obstacles);

	double obstaclex, obstacley, imagetarx, imagetary, gpstarx, gpstary, gpsavoidx, gpsavoidy;

	// Get the vector contribution from the obstacles on the bitmap
	getAvoidVec(obstacles, obstaclex, obstacley);

	// Get the vector contribution from the goals on the bitmap
	getImgTargetVec(targets, imagetarx, imagetary);

	// Get the vector contribution from the GPS goal(s)
	getGPSTargetVec(gpstarx, gpstary);

	// Get the vector contribution from the GPS past goal(s)
	getGPSAvoidVec(gpsavoidx, gpsavoidy);

	int number_vecs = 4;
	double xcomps[] = {obstaclex, imagetarx, gpstarx, gpsavoidx};
	double ycomps[] = {obstacley, imagetary, gpstary, gpsavoidy};	
	double xnet, ynet;

	// Find the resulting vector by adding up all of the components
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
			// For each pixel in the bitmap, if it is an obstacle pixel, fill in all of the pixels around it
			// within the radius of the robot
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
	ReturnData data;

	// Return a sum of all of the x and y components from all of the obstacle pixels within the
	// predefined radius
	doSomethingforIndexesInRadius(robotlocx, robotlocy, obstacle_avoid_radius, obstacles, OBSTACLES, data);

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * obstacle_weight * meters_per_pixel;
	yvel = data.y_vel * obstacle_weight * meters_per_pixel;
	return;
}

/* Returns the x and y components of the Image target vector in meters */
void potentialfields::getImgTargetVec(bool* targets, double& xvel, double& yvel)
{
	ReturnData data;

	// Return a sum of all of the x and y components from all of the goal pixels within the
	// predefined radius
	doSomethingforIndexesInRadius(robotlocx, robotlocy, target_reach_radius, targets, ATTRACTORS, data);

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * image_goal_weight * meters_per_pixel;
	yvel = data.y_vel * image_goal_weight * meters_per_pixel;
	return;
}

/* Returns the x and y components of the GPS goal vector in meters */
void potentialfields::getGPSTargetVec(double& xvel, double& yvel)
{
	double distance = 0;
	double theta = 0;
	// TODO: Figure out the distance and angle from the current GPS coordinate to the goal GPS coordinate
	if (distance > (gps_max_distance + gps_goal_radius))
	{	
		// If the robot is far away from the GPS goal, gets a constant vector towards the GPS location
		xvel = gps_goal_weight * gps_max_distance * cos(theta);
		yvel = gps_goal_weight * gps_max_distance * sin(theta);
	}
	else
	{
		// If the robot is close to the goal, vector towards the GPS location smaller and smaller
		xvel = gps_goal_weight*(distance - gps_goal_radius)*cos(theta);
		yvel = gps_goal_weight*(distance - gps_goal_radius)*sin(theta);
	}
	return;
}

/* Returns the x and y components of the GPS avoidance vector in meters */
void potentialfields::getGPSAvoidVec(double& xvel, double& yvel)
{
	// TODO: Write this function
	// Write only if it becomes useful
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

void potentialfields::attractorPixels(int x0, int y0, int xt, int yt, double& x_vel, double& y_vel)
{
	// Finds the distance between the pixels and the angle to go to the second pixel
	double d = Distance2D(x0, y0, xt, yt);
	double theta = atan2((y0-yt), (x0-xt));

	// Creates vectors pointing towards the goal pixel, proportional to the distance from the target
	x_vel = d*cos(theta);
	y_vel = d*sin(theta);
	return;
}

void potentialfields::repulsivePixels(int x0, int y0, int xt, int yt, int radius, double& x_vel, double& y_vel)
{
	// Finds the distance between the pixels and the angle to go to the second pixel
	double d = Distance2D(x0, y0, xt, yt);
	double theta = atan2((y0-yt), (x0-xt));

	// Creates vectors pointing away from the obstacle pixel, proportional to the distance from the target
	x_vel = -(radius-d)*cos(theta);
	y_vel = -(radius-d)*sin(theta);
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
	doSomethingforIndexesInRadius(x, y, radius, obstacle, FILL1, ReturnData());
	return;	
}

void potentialfields::doSomethingforIndexesInRadius(int x0, int y0, int radius, bool* bitmap, RAD_OPTION OPTION, ReturnData data)
{
	// Starts at the lowest y up to the highest y possible within the radius
	for (int y = y0 - radius; y <= y0 + radius; y++)
	{
		// Checks to make sure the indexes are within the bounds of the bitmask
		if (y < 0 || y >= ysize)
			continue; 

		// Figures out the maximum x distance for the current y
		int xdistmax = floor(sqrt(radius*radius - y*y)+.5);

		// Starts at the lowest x possible for the current y and goes to the highest x possible for the current y
		for(int x = x0 - xdistmax; x <= x0 + xdistmax; x++)
		{
			// Checks to make sure the indexes are within the bounds of the bitmask
			if (x < 0 || x >= xsize)
				continue;

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
					if (get2Dindexvalue(bitmap, x, y) == 1)
					{
						double curx_vel, cury_vel;
						repulsivePixels(x0, y0, x, y, radius, curx_vel, cury_vel);
						data.x_vel += curx_vel;
						data.y_vel += cury_vel;
					}
					break;

				// Finds the pixel attractions and sums them together
				case ATTRACTORS:
					if (get2Dindexvalue(bitmap, x, y) == 1)
					{
						double curx_vel, cury_vel;
						attractorPixels(x0, y0, x, y, curx_vel, cury_vel);
						data.x_vel += curx_vel;
						data.y_vel += cury_vel;
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

/* Finds the distance in meters between two GPS points */
double potentialfields::distBtwGPSPoints(const GPS_point& a, const GPS_point& b)
{
	GPSState gpsa, gpsb;
	gpsa.lon = a.lon;
	gpsa.lat = a.lat;
	gpsb.lon = b.lon;
	gpsb.lat = b.lat;
	return lambert_distance(gpsa, gpsb);
}

void potentialfields::medianThreshFileter(bool* array, int thresh_size)
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
