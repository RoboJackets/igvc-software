#ifndef _VISION_H_
#define _VISION_H_

#include <cv.h>
#include <highgui.h>


/*
 * This file contains the robot's primary vision processing code main function.
 *   by: Chris McClanahan
 *
 */

class Vision
{

public:

    Vision();
    ~Vision();
    void init();



    /* flag for performing image perspective transform */
    int DO_TRANSFORM ;


 



    // HSV thresholds
    int satThreshold;
    int hueThreshold;



    // splits raw image into RGB channel images
    void GetRGBChannels();
    // splits raw image into HSV channel images
    void GetHSVChannels();
    // binary thresholds an image
    void ThresholdImage(IplImage *src, IplImage *dst, int thresh);
    // analyze visCvThresh and generate visCvPath
    void visGenPath(IplImage* img);

    // helpers for visGenPath
    int checkPixel(IplImage* img, int x, int y);
    void scanFillLeft(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
    void scanFillRight(IplImage* img, int middleX, int y, int goodFirst, int end, int blackout);
    int findBestX(IplImage* img, int height, int center);

    // helpers for path finding
    void robotWidthScan(IplImage* img, int& goalx, int& goaly);
    void visPlanPath(IplImage* img, int& goalx, int& goaly);

    // loads the thresholds for vision processing from the xml config file
    void LoadVisionXMLSettings();

    // trackbar value determines image view to display
    void ConvertAllImageViews(int trackbarVal);

    // manually does color analysis that the HSV colorspace doesn't
    void preProcessColors(IplImage* img);

    // for image display names
    CvFont font;

    // finds min/max values in a 0-255 greyscale image and normalizes using those
    void Normalize(IplImage* img);

    // width of robot in pixels
    int ROBOT_WIDTH;

    // for the adaptive algorithms
    void Adapt();
    void visAdaptiveProcessing();

    int adapt_maxDiff;
    int adapt_boxPad;
    int DO_ADAPTIVE;

};


#endif // _VISION_H_

