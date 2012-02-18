
#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvtypes.h>
#include <opencv/cvinternal.h>
#include "gps.hpp"
#include "gps_common.hpp"

#define X_COORD_ON_SCREEN 500
#define Y_COORD_ON_SCREEN 500
#define WIDTH 50
#define HEIGHT 50

class mapDisplay
{
private:
IplImage* lastImage;
std::vector <CvPoint> GPSPoints;
std::vector <CvPoint> additionalPoints;
GPSState referencePoint;
double meters_per_pixel;
public:
mapDisplay(GPSState reference, CvImage* startImage);
void draw(CvImage* newImage);
void addPointToDraw(GPSState newPoint);
void addGPSPointToDraw(GPSState newGPSPoint);
void addPointToDraw(double lat, double lon);
void addGPSPointToDraw(double lat, double lon);
};
