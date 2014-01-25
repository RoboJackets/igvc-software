#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

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
    LineDetector(std::string imgFile);
    void applyAlgorithm();
    bool loadImage(std::string imgFile);
private:
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    float getAvg(void);
    void displayImage();
    void blackAndWhite(float totalAvg);
    ///\brief The location of the image/video file to load
    std::string imgFile;
    int display_dst(int delay);
    ///\brief the VideoCapture of the image/video
    VideoCapture cap;
    void detectObstacle(int i, int j);
};

#endif // LINEDETECTOR_H
