#ifndef LINEDETECTOR_H
#define LINEDETECTOR_H
#include <iostream>
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"

#define ABS(a) (((a) < 0)? -1*(a):(a))
#define ABSDIFF(a, b) (ABS((a)-(b)))
#define AVGDIFF(a, b ,c) ((ABSDIFF(a,b)+ABSDIFF(b,c)+ABSDIFF(a,c))/3)
#define LOWTHRESHOLDCANNY 40
#define RATIOCANNY 1.05
#define DELAY 1
#define COLORRATIO 1.4
#define COLORRATIOMAX 1.47
#define GAUSSSIZE 19
#define WHITETHRESHOLDBLOCK 5

typedef void (*fpt) (int, void*);

using namespace std;
using namespace cv;

class LineDetector
{
public:
    LineDetector(std::string imgFile);
    void applyAlgorithm();
    bool loadImage(std::string imgFile);
    //Mat src;
   // Mat dst;
private:
    int getBlockAvg(int rowl, int rowu, int coll, int colu);
    int checkNbors(int rowStartNum, int colStartNum, int rowSize, int colSize, int threshold);
    void blackoutSection(int rowl, int rowu, int coll, int colu);
    float getAvg(void);
    void filter(int numBlocks, int whiteThreshold);
    void filter2(int numBlocks, int whiteThreshold);
    void displayImage();
    void blackAndWhite(float totalAvg);
    std::string imgFile;
    int display_dst(int delay);
    VideoCapture cap;
    void detectObstacle(int i, int j);
};

#endif // LINEDETECTOR_H
