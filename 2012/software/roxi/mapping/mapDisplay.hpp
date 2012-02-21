
#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvtypes.h>
#include <opencv/cvinternal.h>
#include "gps.hpp"
#include "gps_common.hpp"
//#include "XmlConfiguration.h"

#define X_COORD_ON_SCREEN 500
#define Y_COORD_ON_SCREEN 500
#define WIDTH 50
#define HEIGHT 50
#define meters_per_pixel 0.02

class mapDisplay
{
private:
IplImage* lastImage;
std::vector <CvPoint> GPSPoints;
std::vector <CvPoint> additionalPoints;
GPSState referencePoint;

public:
mapDisplay(GPSState, CvImage* );
mapDisplay(int lat, int lon, CvImage*);
void draw(CvImage* );
void addPointToDraw(GPSState );
void addGPSPointToDraw(GPSState );
void addPointToDraw(double , double );
void addGPSPointToDraw(double , double );
};
