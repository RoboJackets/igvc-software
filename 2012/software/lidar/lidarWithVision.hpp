#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvtypes.h>
#include <opencv/cvinternal.h>
#include <vector>

typedef struct shape{
	std::vector <Point> points;
	int numPoints;
	float avgX;
	float avgY;
	int totalX;
	int totalY;
	int maxX;
	int minX;
	int maxY;
	int minY;
}Shape;

typedef struct point{
	int x;
	int y;
}Point;

/*
take in cartesian points from the lidar, create ipl image of lidar points, erase shadows from camera image.
*/
IplImage correctVisionWithLidar(float lidarX[], float lidarY[], IplImage* visionImage);
void buildShape(int currX,int currY, bool* checked, bool* bitMap,std::vector<Point>* currShape);
void addPointToShape(Point point, Shape* addTo);
void eraseShapeOnBitmap(std::vector<Point> shape,bool** eraseFrom);
void eraseShadow(Shape shape, IplImage img);
