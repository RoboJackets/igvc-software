#include "vision_navigation.h"

#include "vision.h"
#include "vision_color.h"      // for pixelIsOrange, pixelIsWhite
#include "vision_barrels.h"	// for visAnnotateBarrelBounds
#include "vision_line_blobber.h"
#include "Graphics.h"
#include "Point2D.h"
#include "DriveMotion.h"
#include <math.h>		// for trig functions, M_PI
#include "hw/hw.h"		// for GetLaserRangeAtBearing

// ### FORWARD REFERENCES (C/C++ SPECIFIC) ###

double deg2rad(double degrees);

Point2D<double> navPath_start(int pathID);
Point2D<double> navPath_vector(int pathID);
Point2D<double> navPath_end(int pathID);

double navPath_angle(int pathID);

Pixel navPath_color(int pathDanger);

// ### PARAMETERS ###

// Number of paths that are assessed between the starting/ending angles
// 1 <= NAV_PATH__NUM
const int NAV_PATH__NUM = 30;

// Defines the "view/navigation cone", which is where the set of
// considered navigation paths is taken from.
const double NAV_PATH__VIEW_CONE__START_ANGLE = 0.0 + 30.0;	// >= 0.0
const double NAV_PATH__VIEW_CONE__END_ANGLE = 180.0 - 30.0;	// <= 180.0

const double NAV_PATH__VIEW_CONE__DELTA_ANGLE = NAV_PATH__VIEW_CONE__END_ANGLE - NAV_PATH__VIEW_CONE__START_ANGLE;
const double NAV_PATH__VIEW_CONE__SPACING = NAV_PATH__VIEW_CONE__DELTA_ANGLE / (NAV_PATH__NUM-1);

// XXX: Controls how many pixels *near* a path-pixel are searched for dangerous pixels
// 0 <= NAV_PATH__PATH_SEARCH_GIRTH
const int NAV_PATH__PATH_SEARCH_GIRTH = 5;

// (do not change without reason!)
const int NAV_PATH__CENTER_PATH_ID = (int) round((90.0 - NAV_PATH__VIEW_CONE__START_ANGLE) / NAV_PATH__VIEW_CONE__SPACING);

// Proportional to the lengths of the paths (in image space)
const double NAV_PATH__VIEW_DISTANCE_MULTIPLIER = 1.00;		// > 0.0

// Amount of danger posed by a single barrel-pixel
const int DANGER_PER_BARREL_PIXEL = 1;
// Amount of danger posed by a single line-pixel
const int DANGER_PER_LINE_PIXEL = 4;

// Path danger values higher than this will be clipped to this value
const int MAX_PATH_DANGER = 50;					// >= 0

const int NAV_PATH__DANGER_SMOOTHING__RADIUS = 5;		// >= 0

// ---------------------------------------------------------

const bool PRINT_DANGER_VALUES = false;

const Pixel MIN_PATH_DANGER_COLOR = Pixel(255, 255, 0);		// yellow
const Pixel MAX_PATH_DANGER_COLOR = Pixel(0, 0, 0);		// black

const Pixel DANGEROUS_PIXEL_COLOR = Pixel(0, 0, 255);		// blue

const Pixel WHITE_BARREL_STRIPE_PIXEL_ANNOTATION_COLOR = 
	Pixel(255 * 3/4, 255 * 3/4, 255 * 3/4);		// light gray


// ### ALGORITHM ###

// OUTPUT: Visual representation of the navigation parameters.
Buffer2D<Pixel> visNavigationParams;

// OUTPUT: Trajectory picked by the vision-navigation algorithm
MotorOutput autonomousModeMotorOutput;

// public
void visPlotNavigationParams(void) {
	// Initialize the navigation-param view with the raw camera image
	// (so that we can annotate on top of it)
	Graphics g(&visNavigationParams);
	visNavigationParams.copyFrom(visRaw);
	
	// Annotate the colors of the pixels in the raw camera image
	// (since the danger map that navigation relys on is generated
	//  from the set of identified pixel colors)
	{
		// Annotate standard colors
		visAnnotatePixelColors(visNavigationParams);
		
		// Annotate white specially:
		// 1. white barrel stripes -> light gray (custom)
		// 2. white lines -> pure white (the default)
		for (int i=0, n=visNavigationParams.numElements(); i<n; i++) {
			if (pixelIsWhite[i] && pixelIsWithinBarrel[i]) {
				visNavigationParams[i] = WHITE_BARREL_STRIPE_PIXEL_ANNOTATION_COLOR;
			}
		}
	}
	
	// Annotate the navigation output image
	// with the bounds of identified barrels
	// TODO: actually make use of barrel bound information
	//       when computing the danger map
	//       (in particular, use it to distinguish white barrel stripes
	//        from white lines on the field)
	visAnnotateBarrelBounds(visNavigationParams, false);
	
	/* Compute and draw all navigation paths we are considering (and do other actions) */
	static int pathDanger[NAV_PATH__NUM];
	for (int pathID=0; pathID<NAV_PATH__NUM; pathID++) {
		// Calculate path parameters
		Point2D<double> pathStart = navPath_start(pathID);
		Point2D<double> pathEnd = navPath_end(pathID);
		
		// Calculate the set of points in the path
		QVector< Point2D<int> > pathPoints;
		Graphics::calculatePointsInLine(
			(int) pathStart.x, (int) pathStart.y,
			(int) pathEnd.x, (int) pathEnd.y,
			&pathPoints);
		
		// Calculate the danger value for the path
		// along with the danger contributions of all
		// the pixels in the path
		int curPathDanger = 0;
		uchar pixelDanger[pathPoints.count()];
		for (uint j=0, n2=pathPoints.count(); j<n2; j++) {
			Point2D<int> curPathPoint = pathPoints[j];
			
			// Calculate the danger contribution of the current
			// path-pixel to the path danger
			int curPixelDanger = 0;
			for (int delta=-NAV_PATH__PATH_SEARCH_GIRTH; delta<=NAV_PATH__PATH_SEARCH_GIRTH; delta++) {
				// (XXX: Consider *nearby* pixels as different fragments of the path-pixel)
				Point2D<int> curPoint = pathPoints[j];
				curPoint.x += delta;
				
				if (g.contains(curPoint.x, curPoint.y)) {
					if (pixelIsWhite.get(curPoint.x, curPoint.y)) {
						if (pixelIsWithinBarrel.get(curPoint.x, curPoint.y)) {
							// Barrel: White stripe
							curPixelDanger += DANGER_PER_BARREL_PIXEL;
						} else {
							// Line: White
							curPixelDanger += DANGER_PER_LINE_PIXEL;
						}
					} else if (pixelIsOrange.get(curPoint.x, curPoint.y)) {
						// Barrel: Orange stripe
						curPixelDanger += DANGER_PER_BARREL_PIXEL;
					} else {
						// Nothing special: Probably grass
						curPixelDanger += 0;
					}
				} else {
					// Offscreen: Probably grass
					curPixelDanger += 0;
				}
			}
			
			// (XXX: Compensate for considering *nearby* pixels)
			curPixelDanger /= ((NAV_PATH__PATH_SEARCH_GIRTH*2) + 1);
			
			pixelDanger[j] = curPixelDanger;
			curPathDanger += curPixelDanger;
		}
		if (curPathDanger > MAX_PATH_DANGER) {
			// Clip high danger values to be no higher than MAX_PATH_DANGER
			curPathDanger = MAX_PATH_DANGER;
		}
		pathDanger[pathID] = curPathDanger;
		
		if (PRINT_DANGER_VALUES) {
			printf("PATH #%d: %d\n", (int) pathID, (int) curPathDanger);
		}
		
		// Draw the body of the path using a color that is determined from the path's danger value
		g.setColor(navPath_color(curPathDanger));
		g.drawLine(
			(int) pathStart.x, (int) pathStart.y,
			(int) pathEnd.x, (int) pathEnd.y);
		
		// Hilight the "dangerous" pixels in the path
		// (that contributed to the path's total danger value)
		for (uint j=0, n2=pathPoints.count(); j<n2; j++) {
			uchar curPixelDanger = pixelDanger[j];
			if (curPixelDanger != 0) {
				Point2D<int> curPoint = pathPoints[j];
				
				g.setColor(DANGEROUS_PIXEL_COLOR);
				
				// Color the pixel either thickly/thinly,
				// depending on how dangerous it is
				// (NOTE: This no longer works when using neighboring
				//        pixels to calculate danger values for path-pixels)
				if (curPixelDanger == DANGER_PER_LINE_PIXEL) {	// maximum danger
					g.drawRect_rational(
						curPoint.x, curPoint.y,
						2, 2);
				} else {
					g.drawPixel(curPoint.x, curPoint.y);
				}
			}
		}
		
		// Get the range to any obstacle that intersects the nav-path
		#define MAX_EXPECTED_RANGE 4.25
		double range = GetLaserRangeAtBearing(-deg2rad(90 - navPath_angle(pathID)));
		if (range != -1) {
			if (range > MAX_EXPECTED_RANGE) range = MAX_EXPECTED_RANGE;
			
			Point2D<double> obstaclePos = 
				navPath_start(pathID) + 
				(navPath_end(pathID)-navPath_start(pathID)) * range/MAX_EXPECTED_RANGE;
			
			g.setColor(Pixel(0, 255, 0));	// green
			g.fillRect_rational(
				(int) obstaclePos.x-2, (int) obstaclePos.y-2,
				5, 5);
		}
	}
	if (PRINT_DANGER_VALUES) {
		printf("PATH: done\n");
	}
	
	/* 
	 * Apply smoothing to the path danger values so that paths
	 * that are *near* dangerous paths are also considered to
	 * be dangerous
	 */
	{
		static int smoothedPathDangers[NAV_PATH__NUM];
		
		// Copy first edge	
		for (int curPath_id = 0; curPath_id < NAV_PATH__DANGER_SMOOTHING__RADIUS; curPath_id++) {
			smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
		}
		
		// Smooth interior
		for (int curPath_id = NAV_PATH__DANGER_SMOOTHING__RADIUS;
		     curPath_id < (NAV_PATH__NUM - NAV_PATH__DANGER_SMOOTHING__RADIUS + 1);
		     curPath_id++)
		{
			int sumOfNearbyDangers = 0;
			for (int delta = -NAV_PATH__DANGER_SMOOTHING__RADIUS;
			     delta < NAV_PATH__DANGER_SMOOTHING__RADIUS;
			     delta++)
			{
				sumOfNearbyDangers += pathDanger[curPath_id + delta];
			}
			int avgOfNearbyDangers = sumOfNearbyDangers / (NAV_PATH__DANGER_SMOOTHING__RADIUS*2 + 1);
			
			smoothedPathDangers[curPath_id] = avgOfNearbyDangers;
		}
		
		// Copy second edge
		for (int curPath_id = (NAV_PATH__NUM - NAV_PATH__DANGER_SMOOTHING__RADIUS);
		     curPath_id < NAV_PATH__NUM;
		     curPath_id++)
		{
			smoothedPathDangers[curPath_id] = pathDanger[curPath_id];
		}
		
		// Transfer smoothed dangers back to primary danger buffer
		for (int curPath_id=0; curPath_id<NAV_PATH__NUM; curPath_id++) {
			pathDanger[curPath_id] = smoothedPathDangers[curPath_id];
		}
	}
	
	/* Pick a path with a low danger-value to follow */
	{
		/*
		 * Find the path with the lowest danger value.
		 * If there are multiple such paths, pick the that is closest to the center
		 * (i.e., pick the path that points the most straight upward)
		 */
		int bestPath_id = 0;
		{
			int bestPath_danger = pathDanger[bestPath_id];
			int bestPath_distanceFromCenter = abs(NAV_PATH__CENTER_PATH_ID - bestPath_id);
			for (int curPath_id=1; curPath_id<NAV_PATH__NUM; curPath_id++) {
				int curPath_danger = pathDanger[curPath_id];
				int curPath_distanceFromCenter = abs(NAV_PATH__CENTER_PATH_ID - curPath_id);
				
				if (curPath_danger < bestPath_danger) {
					bestPath_danger = curPath_danger;
					
					bestPath_id = curPath_id;
					bestPath_distanceFromCenter = curPath_distanceFromCenter;
				} else if (curPath_danger == bestPath_danger) {
					if (curPath_distanceFromCenter < bestPath_distanceFromCenter) {
						bestPath_id = curPath_id;
						bestPath_distanceFromCenter = curPath_distanceFromCenter;
					}
				}
			}
		}
		
		/* Hilight the path that was picked */
		{
			g.setColor(navPath_color(pathDanger[bestPath_id]));
			
			// Redraw the path, but much more thickly (in order to hilight it)
			Point2D<double> bestPath_start = navPath_start(bestPath_id);
			Point2D<double> bestPath_end = navPath_end(bestPath_id);
			for (int deltaX = -1; deltaX <= 1; deltaX++) {
				g.drawLine(
					((int) bestPath_start.x) + deltaX, (int) bestPath_start.y,
					((int) bestPath_end.x) + deltaX, (int) bestPath_end.y);
			}
		}
		
		/*
		 * Direct the motors (while in autonomous mode)
		 * to drive in the direction of the best path
		 * with a thrust inversely proportional to the
		 * sharpness of the turn required to follow
		 * the best path
		 *
		 * NOTE: The interpretation of the constructed
		 * DriveMotion value is affected by the algorithm
		 * used to convert it into the final MotorOutput.
		 */
		DriveMotion nav_driveMotion;
		MotorOutput nav_motorOutput;
		{
			/*
			// Swivel is proportional to the angle of the best path.
			int nav_swivel = 127 * (90 - ((int) navPath_angle(bestPath_id)))/90;
			*/
			
			// TODO: document this swivel strategy calculation
			// WARNING: Assumes that the navigation view cone is symmetric about the vertical axis.
			int nav_swivel = 127 * (NAV_PATH__CENTER_PATH_ID - bestPath_id)/NAV_PATH__CENTER_PATH_ID;
			if (nav_swivel < -128)
				nav_swivel = -128;
			if (nav_swivel > 127)
				nav_swivel = 127;
			
			/*
			// TODO: make thrust inversely proportional to the danger value of the best path
			// TODO: if this value is kept a constant, make it a calibratable parameter
			int nav_thrust = 127;
			*/
			
			// TODO: document thrust strategy
			int nav_thrust = 127 * (MAX_PATH_DANGER - pathDanger[bestPath_id])/MAX_PATH_DANGER;
			
			nav_driveMotion = DriveMotion(nav_thrust, nav_swivel);
			nav_motorOutput = nav_driveMotion.toMotorOutput();
			
			autonomousModeMotorOutput = nav_motorOutput;
		}
		
		/*
		 * Display the output DriveMotion and MotorOutput
		 */
		{
			const int DRIVE_BAR_THICKNESS = 5;
			
			int halfWidth = visRaw.width/2;
			int halfHeight = visRaw.height/2;
			
			int thrustExtent = nav_driveMotion.thrust * halfHeight/128;
			int swivelExtent = nav_driveMotion.swivel * halfWidth/128;
			
			int leftSpeedExtent = nav_motorOutput.leftSpeed * halfHeight/128;
			int rightSpeedExtent = nav_motorOutput.rightSpeed * halfHeight/128;
			
			/* Draw the swivel bar along the top and bottom of the view */
			g.setColor(Pixel(0, 0, 200));	// dark blue
			g.fillRect_rational(
				(swivelExtent > 0) ? halfWidth : (halfWidth + swivelExtent),
				0,
				abs(swivelExtent),
				DRIVE_BAR_THICKNESS);
			g.fillRect_rational(
				(swivelExtent > 0) ? halfWidth : (halfWidth + swivelExtent),
				visRaw.height - DRIVE_BAR_THICKNESS,
				abs(swivelExtent),
				DRIVE_BAR_THICKNESS);
			
			/* Draw the thrust bar along the left and right of the view */
			g.setColor(Pixel(200, 0, 0));	// dark red
			g.fillRect_rational(
				0,
				(thrustExtent < 0) ? halfHeight : (halfHeight - thrustExtent),
				DRIVE_BAR_THICKNESS,
				abs(thrustExtent));
			g.fillRect_rational(
				visRaw.width - DRIVE_BAR_THICKNESS,
				(thrustExtent < 0) ? halfHeight : (halfHeight - thrustExtent),
				DRIVE_BAR_THICKNESS,
				abs(thrustExtent));
			
			/* Draw the left/right wheel speed bars along the left and right of the view */
			g.setColor(Pixel(255-30, 255-30, 255-30));	// light grey
			g.fillRect_rational(
				DRIVE_BAR_THICKNESS,
				(leftSpeedExtent < 0) ? halfHeight : (halfHeight - leftSpeedExtent),
				DRIVE_BAR_THICKNESS,
				abs(leftSpeedExtent));
			g.fillRect_rational(
				visRaw.width - (DRIVE_BAR_THICKNESS * 2),
				(rightSpeedExtent < 0) ? halfHeight : (halfHeight - rightSpeedExtent),
				DRIVE_BAR_THICKNESS,
				abs(rightSpeedExtent));
		}
	}
	
	//DEBUG: lines from blobber drawn in red
	g.setColor(Pixel(200, 0, 0));	// dark red
	for(int i=0;i<numwhitelines;i++){
		g.drawLine(whitelines[i]);
	}
}

// Converts a measurement in degrees to radians.
double deg2rad(double degrees) {
	return degrees * M_PI / 180.0;
}

Point2D<double> navPath_start(int pathID) {
	return Point2D<double>(
		visRaw.width / 2,
		visRaw.height - 1);
}

Point2D<double> navPath_vector(int pathID) {
	double deg = navPath_angle(pathID);
	double rad = deg2rad(deg);
	
	double radius = 0.75 * ((double) visRaw.width) / 2;
	return
		Point2D<double>(
			 radius*cos(rad),
			-radius*sin(rad))	// computer y-axis is inverse of math y-axis
		* NAV_PATH__VIEW_DISTANCE_MULTIPLIER;	
}

Point2D<double> navPath_end(int pathID) {
	return navPath_start(pathID) + navPath_vector(pathID);
}

// result is in degrees
double navPath_angle(int pathID) {
	return 
		NAV_PATH__VIEW_CONE__START_ANGLE +
		(pathID * NAV_PATH__VIEW_CONE__SPACING);
}

Pixel navPath_color(int pathDanger) {
	return Pixel::fadeBetween(
		MIN_PATH_DANGER_COLOR, MAX_PATH_DANGER_COLOR,
		pathDanger, MAX_PATH_DANGER);
}
