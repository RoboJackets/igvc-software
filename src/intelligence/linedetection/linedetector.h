#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <common/logger/logger.h>
#include <sstream>
#include <common/events/Event.hpp>
#include <hardware/sensors/DataStructures/ImageData.hpp>

/*!\def ABS(a)
*\brief a Macro that returns the absolute value of a number
*/
#define ABS(a) (((a) < 0)? -1*(a):(a))
/*!\def ABSDIFF(a,b)
*\brief a Macro that returns the positive difference between a and b
*/
#define ABSDIFF(a, b) (ABS((a)-(b)))
///\def DELAY
///\brief the Delay to use when displaying images. 0 for infinity
#define DELAY 1
///\def GAUSSIZE
///\brief The size of the Gaussian blur. The bigger the greater the blur
///       Must be odd!
#define GAUSSSIZE 7


using namespace std;
using namespace cv;

class LineDetector
{
public:
    LineDetector(Event<ImageData> &evtSrc);
    void onImageEvent(ImageData imgd);
    LISTENER(LineDetector, onImageEvent, ImageData)
private:
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    float getAvg(void);
    void displayImage();
    void blackAndWhite(float totalAvg);
    int display_dst(int delay);
    ///\brief the VideoCapture of the image/video
    VideoCapture cap;
    void detectObstacle(int i, int j);

    void Erosion(int, void*);
    void Dilation(int, void*);

    const char* window_name;
    const char* original_window_name;

    //For the erosion/dilation stuff
    ///\var int erosion_elem
    ///\brief contains the number corresponding to the element used for erosion
    ///       2 is what we are currently using (an ellipse)
    int erosion_elem;
    ///\var int erosion_size
    ///\brief specifies the size of the area to be eroded.
    int erosion_size;
    int dilation_elem;
    int dilation_size;

    const int max_elem;
    const int max_kernel_size;

    ///\var Mat src
    ///\brief contains the original, unprocessed image

    ///\var Mat dst
    ///\brief contains the new, processed image that isolates the lines
    Mat src, dst;
};
#endif // LINEDETECTOR_H
