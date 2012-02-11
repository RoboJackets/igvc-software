
#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include "gps.hpp"
#include "gps_common.hpp"

#define X_COORD_ON_SCREEN 500
#define Y_COORD_ON_SCREEN 500
#define WIDTH 50
#define HEIGHT 50

class mapDisplay
{
private:
IplImage lastImage;
std::vector <cvPoint> GPSPoints;
std::vector <cvPoint> additionalPoints;
GPSState referencePoint;
double meters_per_pixel;
public:
mapDisplay(GPSState reference, cvImage startImage);
void draw();
double addPointToDraw(GPSState newPoint);
double addGPSPointToDraw(GPSState newGPSPoint);
double addPointToDraw(double lat, double lon);
double addGPSPointToDraw(double lat, double lon);
};
