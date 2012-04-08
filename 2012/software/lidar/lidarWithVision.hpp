#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvtypes.h>
#include <opencv/cvinternal.h>
#include <vector>

#define ROBOT_POS_X_IN_CAMERA_FRAME 0.5
#define ROBOT_POS_Y_IN_CAMERA_FRAME 0.5
typedef struct point{
	int x;
	int y;
}Point;


typedef struct shape{
	int numPoints;
	float avgX;
	float avgY;
	int totalX;
	int totalY;
	int maxX;
	int minX;
	int maxY;
	int minY;
	std::vector<Point> points;
}Shape;


/*
take in cartesian points from the lidar, create ipl image of lidar points, erase shadows from camera image.
*/
IplImage correctVisionWithLidar(float lidarX[], float lidarY[], IplImage* visionImage);
void buildShape(int currX,int currY, bool* checked, bool* bitMap,Shape* currShape);
void addPointToShape(Point point, Shape* addTo);
void eraseShapeOnBitmap(Shape shape,bool* eraseFrom);
void eraseShadow(Shape shape, IplImage* img);
