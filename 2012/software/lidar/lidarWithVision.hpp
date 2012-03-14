#include <math.h>
#include <sys/time.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cvaux.h>
#include <opencv/cvtypes.h>
#include <opencv/cvinternal.h>
#include <vector>

/*
take in cartesian points from the lidar, create ipl image of lidar points, erase shadows from camera image.
*/
IplImage correctVisionWithLidar(float lidarX[], float lidarY[], IplImage* visionImage);
