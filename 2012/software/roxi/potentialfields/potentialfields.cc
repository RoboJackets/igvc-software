#include "potentialfields.hpp"
#include "gps_points.hpp"
#include "commonVecOps.hpp"
#include <stdio.h>
#include <string.h>
#include <iostream>

using std::cout;
using std::endl;

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
	cout << "lat: " << lat << endl << "lon: " << lon << endl << "ang: " << ang << endl;
	bool isEmpty = GPS_Prev_Loc.empty();
	GPS_Prev_Loc.push_back(GPS_point(lat, lon, ang));
	
	// If this is the first GPS point, then this needs to be added as the final goal for the robot (the course is circular) 	
	if (isEmpty)
	{
		GPS_Goals.push_back(GPS_point(lat, lon, ang));
	}

	// TODO: If the current GPS location is within the right radius of the goal GPS location, index up the goal GPS point
	// If the goal is out of bounds, just start back at 0
	if (distBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1],GPS_Goals[currentGoal]) < gps_goal_radius)
	{
		currentGoal = (currentGoal+1) % GPS_Goals.size();
	}
}

/* Changes value of input references vel_mag and vel_ang with the velocity and angle determined by the potential fields algorithm.
Angle given by a value between 0 and 360 with 0 at due North */
void potentialfields::getNextVector(bool* obstacles, bool* targets, int xsize, int ysize, int robotx, int roboty, double& vel_mag, double& vel_ang)
{
	// Update the current GPS location
	updateCurLocation();	
	
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

	// Rotates the velocity vector to the angle from the perspective of the robot
	vel_ang = RotateBearing(vel_ang, -curang);

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

	for(int i=0; i<xinds.size(); i++)
	{
		fillinRadius(obstacles, xinds[i], yinds[i], robot_radius);
	}

	return;
}

/* Returns the x and y components of the obstacle avoidance vector in meters */
void potentialfields::getAvoidVec(bool* obstacles, double& xvel, double& yvel)
{
	ReturnData data;

	// Return a sum of all of the x and y components from all of the obstacle pixels within the
	// predefined radius
	//cout << "robotlocx: " << robotlocx << " robotlocy: " << robotlocy << " obstacle_rad: " << obstacle_avoid_radius << endl;
	doSomethingforIndexesInRadius(robotlocx, robotlocy, obstacle_avoid_radius, obstacles, OBSTACLES, &data);

	//cout << "data.x_vel: " << data.x_vel << endl << "data.y_vel: " << data.y_vel << endl;

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * obstacle_weight * meters_per_pixel;
	yvel = data.y_vel * obstacle_weight * meters_per_pixel;

	// Rotate the vector based on the current bearing (assume that this is the orientation of the image)
	double mag, ang;
	xyToVec(xvel, yvel, mag, ang);
	ang = RotateBearing(ang, imgAngle);
	void VecToxy(double mag, double ang, double& x, double& y);
	
	return;
}

/* Returns the x and y components of the Image target vector in meters */
void potentialfields::getImgTargetVec(bool* targets, double& xvel, double& yvel)
{
	ReturnData data;

	// Return a sum of all of the x and y components from all of the goal pixels within the
	// predefined radius
	doSomethingforIndexesInRadius(robotlocx, robotlocy, target_reach_radius, targets, ATTRACTORS, &data);

	// Multiply each by the constant multiplier and convert it to m/s
	xvel = data.x_vel * image_goal_weight * meters_per_pixel;
	yvel = data.y_vel * image_goal_weight * meters_per_pixel;
	return;
}

/* Returns the x and y components of the GPS goal vector in meters */
void potentialfields::getGPSTargetVec(double& xvel, double& yvel)
{
	double distance = distBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1], GPS_Goals[currentGoal]);
	double theta = angleBtwGPSPoints(GPS_Prev_Loc[GPS_Prev_Loc.size()-1], GPS_Goals[currentGoal]);
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

void potentialfields::repulsivePixels(int x0, int y0, int xt, int yt, int radius, double& x_vel, double& y_vel)
{
	// Finds the distance between the pixels and the angle to go to the second pixel
	double d = Distance2D(x0, y0, xt, yt);
	double theta = atan2((yt-y0), (xt-x0));

	// Creates vectors pointing away from the obstacle pixel, proportional to the distance from the target
	x_vel = -(radius-d)*cos(theta);
	y_vel = -(radius-d)*sin(theta);

	//cout << "x/y: " << sqrt((x_vel/y_vel)*(x_vel/y_vel));

	// Reverses direction of y_vel because the image indexes for height start at the top, so all the
	// y values have to be reversed
	y_vel = -y_vel;
	return;
}
 
/* Returns the value at the index array[x][y] as if it were a 2D array */
bool potentialfields::get2Dindexvalue(bool* array, int x, int y)
{
	/*cout << endl << "xsize is "  << xsize << endl;
	cout << endl << "(" << x << "," << y << ") is at index " << y*xsize+x << " in this array" << endl;*/
	return array[y*xsize+x];
}

/* Sets the value at the index array[x][y] to val as if it were a 2D array */
void potentialfields::set2Dindexvalue(bool* array, int x, int y, bool val)
{
	array[y*xsize+x] = val;
}

/* Fills in 1's within the radius of the third input around the point (x,y) */
void potentialfields::fillinRadius(bool* obstacle, int x, int y, int radius)
{
	doSomethingforIndexesInRadius(x, y, radius, obstacle, FILL1, NULL);
	return;	
}

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

double angleBtwGPSPoints(const GPS_point& a, const GPS_point& b)
{
	GPSState gpsa, gpsb;
	gpsa.lon = a.lon;
	gpsa.lat = a.lat;
	gpsb.lon = b.lon;
	gpsb.lat = b.lat;
	return lambert_bearing(gpsa, gpsb);;
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

/* Updates the current latitude, longitude and image angle based on latest GPS data */
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

/* Prints bitmap with 1's and 0's */
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

/* Converts an IPlImage into a bitmap based on the thresholds in the header file */
void potentialfields::IPl2Bitmap(IplImage* img, IMAGETYPE imgType, FEATURETYPE featType, bool* bitmap, int& xsize, int& ysiiiize)
{
	double thresh;
	// Sets which threshold we'll be using
	if (featType == OBSTACLE)
		thresh = obstacle_bitmap_thresh;
	else
		thresh = attractor_bitmap_thresh;

	int height = img->height;
	int width = img->width;
	int step = img->widthStep;
	int channels = img->nChannels;
	uchar* data = (uchar *)img->imageData;
	int index = 0;
	bitmap = new bool[height*width];
	for(int hindex = 0; hindex < height; hindex++)
	{
		for(int windex = 0; windex < width; windex++)
		{
			// Finds average intensity over all the channels. If there's just one channel, this isn't strictly necessary
			double avgval;
			for (int i = 0; i < channels; i++)
			{
				avgval += (double)data[hindex*step+windex*channels+i];
			}
			avgval /= channels;

			// If it's a feature high type, it is 1 if the average value is greater than the threshold. If it's
			// a feature low type, it is a 1 if the average value is greater than the threshold. 
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
}
