// compile with:
// g++ `pkg-config --cflags opencv` -o tuningParameters tuningParameters.cpp `pkg-config --libs opencv` -std=c++11

#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include <iostream>
#include <vector>
#include <iomanip>

using namespace std;
using namespace cv;

std::string window = "test canny";
int cannyThresh;
int cannyThresh_max = 100;
int lengthThresh;
int lengthThresh_max = 125;
int lineGap;
int lineGap_max = 50;
Mat src_img;
Mat src_gray;
Mat working;
Mat dst_img;

void linedetect(int, void*) {
    working = Mat::ones(src_img.size(), src_img.type());
    GaussianBlur(src_img, working, Size(0, 0), 2.0);

    Canny(working, working, cannyThresh, cannyThresh * 3, 3);

    vector<Vec4i> lines;
    HoughLinesP(working, lines, 1.0, CV_PI/180, 80, lengthThresh, lineGap);

    cvtColor(working, dst_img, CV_GRAY2BGR);
    for (size_t i = 0; i < lines.size(); ++i) {
        line(dst_img, Point(lines[i][0], lines[i][1]), Point(lines[i][2], lines[i][3]), Scalar(0, 0, 255), 3, 8);
    }

    imshow(window, dst_img);
}

int main(int argc, char* argv[]) {
    if(argc != 2) {cerr << "ERROR, 1 argument expected, the file path of an image" << endl; return -1;}
    src_img = imread(argv[1]);
    if(!src_img.data) {cerr << "ERROR, unable to load image: " << argv[1] << endl; return -1;}

    resize(src_img, src_img, Size(525, 525));

    cvtColor(src_img, src_gray, CV_BGR2GRAY);

    namedWindow(window, CV_WINDOW_AUTOSIZE );

    createTrackbar("Low treshold - Canny", window, &cannyThresh, cannyThresh_max, linedetect);
    createTrackbar("length threshold", window, &lengthThresh, lengthThresh_max, linedetect);
    createTrackbar("line gap", window, &lineGap, lineGap_max, linedetect);

    dst_img = src_gray;
    working = Mat(src_gray.size(), 0.0);

    imshow(window, dst_img);
    waitKey();
}
