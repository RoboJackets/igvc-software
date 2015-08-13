//
// Created by thaeds on 8/10/15.
//

// Compile with:
// g++ `pkg-config --cflags opencv` -o test_line_detector test_line_detector.cpp `pkg-config --libs opencv` -std=c++11

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#define KERNAL_COUNT 4

using namespace std;
using namespace cv;

void DetectLines(int lineThickness, int lineLength, int lineAnchor, int lineContinue);
void WhitenessFilter(Mat& hsv_image, Mat& result);
void MultiplyByComplements(Mat* images, Mat* complements, Mat* results);
void initLineDetection();

// 4 sliders
    // line thickness in pixels
const int lineThickness_slider_max = 255;
int lineThickness;
    // line threshold to be a start
const int lineAnchor_slider_max = 255;
int lineAnchor;
    // line threshold to continue
const int lineContinue_slider_max = 255;
int lineContinue;
    // line length minimum to consider
const int lineLength_slider_max = 255;
int lineLength;

const char* windowName = "LineDetection";
char* filename = nullptr;

Mat src_img;
Mat dst_img;
Mat kernels[KERNAL_COUNT];
Mat kernelComplements[KERNAL_COUNT];
Mat working;
Mat kernelResults[KERNAL_COUNT];
Mat complementResults[KERNAL_COUNT];

void SliderCallback(int, void*) {
    //cerr << "LineThickness: " << lineThickness << " LineLength: " << lineLength << " LineAnchor: " << lineAnchor << " LineContinue: " << lineContinue << endl;
    //cerr << "Calling DetectLines" << endl;
    DetectLines(lineThickness, lineLength, lineAnchor, lineContinue);
    //cerr << "DetectLines returned" << endl;
    imshow(windowName, dst_img);
}

int main(int argc, char* argv[]) {
    if(argc != 2) {cerr << "ERROR, 1 argument expected, the file path of an image" << endl; return -1;}
    src_img = imread(argv[1]);
    if(!src_img.data) {cerr << "ERROR, unable to load image: " << argv[1] << endl; return -1;}
    filename = argv[1];

    initLineDetection();

    // Create the window
    namedWindow(windowName, 1);

    // Add the sliders
    createTrackbar("Line Thickness (pixels)", windowName, &lineThickness, lineThickness_slider_max, SliderCallback);
    createTrackbar("Line Length Minimum (pixels)", windowName, &lineLength, lineLength_slider_max, SliderCallback);
    createTrackbar("Line Anchor Threshold (pixel value)", windowName, &lineAnchor, lineAnchor_slider_max, SliderCallback);
    createTrackbar("Line Continue Threshold (pixel value)", windowName, &lineContinue, lineContinue_slider_max, SliderCallback);

    // Callback to show original image
    //cerr << "Calling callback" << endl;
    SliderCallback(0, nullptr);

    // Exit on keypress
    while(waitKey(0) != 27);
    return 0;
}

void initLineDetection() {
    //cerr << "DetectLines::Initing" << endl;
    lineThickness = 20;
    lineAnchor = 1;
    lineContinue = 20;
    lineLength = 1;
    float karray[2][9][9] = {
            {
                    {-1,   -1,   -1,   -1,   -1,   -1,   -1,   -1, -1},
                    {-1,   -1,   -1,   -1,   -1,   -1,   -1, -1, -1},
                    {-1,   -1,   -1,   -1,   -1,   -1, -1, -1, -1},
                    {1,    1,    1,    1,    1, 1, 1, 1, 1},
                    {1,    1,    1,    1, 1, 1, 1, 1, 1},
                    {1,    1,    1, 1, 1, 1, 1, 1, 1},
                    {0,    0, 0, 0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0, 0, 0, 0},
                    {0, 0, 0, 0, 0, 0, 0, 0, 0}
            },
//                {
//                        {-1,   -1,   -1,   -1,   -1,   -1,   -1,   -1, -1},
//                        {-1,   -1,   -1,   -1,   -1,   -1,   -1, 0,  1},
//                        {-1,   -1,   -1,   -1,   -1,   0,  1,  1,  1},
//                        {-1,   -1,   -1,   0,    1, 1, 1, 1, 1},
//                        {-1,   0,    1,    1, 1, 1, 1, .5, 0},
//                        {1,    1,    1, 1, 1, .5, 0, 0, 0},
//                        {1,    1, 1, .5, 0, 0, 0, 0, 0},
//                        {1, .5, 0, 0, 0, 0, 0, 0, 0},
//                        {0, 0, 0, 0, 0, 0, 0, 0, 0}
//                },
            {
                    {-.89, -.89, -.89, -.89, -.89, -.89, -.89, 1,  1},
                    {-.89, -.89, -.89, -.89, -.89, -.89, 1,  1,  1},
                    {-.89, -.89, -.89, -.89, -.89, 1,  1,  1,  0},
                    {-.89, -.89, -.89, -.89, 1, 1, 1, 0, 0},
                    {-.89, -.89, -.89, 1, 1, 1, 0, 0, 0},
                    {-.89, -.89, 1, 1, 1, 0, 0, 0, 0},
                    {-.89, 1, 1, 1, 0, 0, 0, 0, 0},
                    {1, 1, 1, 0, 0, 0, 0, 0, 0},
                    {1, 1, 0, 0, 0, 0, 0, 0, 0}
            }
    };

    Mat kernel1(9, 9, CV_32FC1, karray[0]);
    kernel1 /= 27;
    //        Mat kernel2(9, 9, CV_32FC1, karray[1]);
    //        kernel2 /= 25;
    Mat kernel3(9, 9, CV_32FC1, karray[1]);
    kernel3 /= 25;

    //        Mat kernel4 = kernel2.t();
    Mat kernel5 = kernel1.t();

    //        Mat kernel6;
    Mat kernel7;
    //        Mat kernel8;

    //        flip(kernel4, kernel6, 0);
    flip(kernel3, kernel7, 0);
    //        flip(kernel2, kernel8, 0);

    kernels[0] = kernel1.clone();
    //                kernel2,
    kernels[1] = kernel3.clone();
    //                kernel4,
    kernels[2] = kernel5.clone();
    //                kernel6,
    kernels[3] = kernel7.clone();
    //                kernel8

    for (int i = 0; i < KERNAL_COUNT; i++) {
        Mat kernelComplement;
        flip(kernels[i], kernelComplement, -1);
        kernelComplements[i] = kernelComplement.clone();
    }
}

void DetectLines(int lineThickness, int lineLength, int lineAnchor, int lineContinue) {
    src_img = imread(filename);
    dst_img = Mat::zeros(src_img.size(), src_img.type());

    // Resize the image such that the lines are approximately 3 pixels wide
    //cerr << "DetectLines::Reducing Image" << endl;
    lineThickness = max(1, lineThickness); // 0 thickness doesn't make sense
    resize(src_img, working, Size(3*src_img.cols/lineThickness, 3*src_img.rows/lineThickness), 0, 0, INTER_LANCZOS4);

    // Convert the image into HSV space to make processing white lines easier
    //cerr << "DetectLines::Converting to HSV" << endl;
    cvtColor(working, working, CV_BGR2HSV);

    // Calculate each pixel's "whiteness" defined as value*(255-saturation);
    //cerr << "DetectLines::Filtering Whiteness" << endl;
    WhitenessFilter(working, working);

    // Pass directional kernels over image
    //cerr << "DetectLines::Filtering kernels" << endl;
    for(size_t i = 0; i < KERNAL_COUNT; i++)
        filter2D(working, kernelResults[i], -1, kernels[i]);

    // Pass directional kernel complements over image (same edge, rotated 180 degrees, 3 pixels between edge and complement)
    //cerr << "DetectLines::Filtering complements" << endl;
    for(size_t i = 0; i < KERNAL_COUNT; i++)
        filter2D(working, complementResults[i], -1, kernelComplements[i]);

    // Multiply the results of kernel filter by its complement
    //cerr << "DetectLines::Multiplying Results" << endl;
    MultiplyByComplements(kernelResults, complementResults, kernelResults);

    // LineDrawing implementation reduces detection to single pixel wide lines of a minimum width
//    Mat* detectedLines = new Mat(LineDrawing(geometricResults, lineAnchor, lineContinue, lineLength));

    working = Mat::zeros(kernelResults[0].size(), kernelResults[0].type());
    //cerr << "DetectLines::Thresholding Results" << endl;
    for(int i = 0; i < KERNAL_COUNT; i++) {
        threshold(kernelResults[i], kernelResults[i], ((float)lineContinue*lineContinue)/255, 255, CV_THRESH_BINARY);
        bitwise_or(working, kernelResults[i], working);
    }
    resize(working, dst_img, src_img.size(), src_img.type());
    //cerr << "lineContinue: " << lineContinue << endl;
}

void WhitenessFilter(Mat& hsv_image, Mat& fin_img) {
    Mat result = 255 * Mat::ones(hsv_image.size(), CV_16UC1);
    Mat tmp;
    hsv_image.convertTo(tmp, CV_16UC3, 1.0);
    Mat channel[3];
    split(tmp, channel);
    result = result - channel[1];
    result = result.mul(channel[2]);
    result.convertTo(fin_img, CV_8UC1, 1.0/255);
}

void MultiplyByComplements(Mat* images, Mat* complements, Mat* results) {
    for(size_t i = 0; i < KERNAL_COUNT; i++) {
        Mat image;
        Mat complement;
        Mat result = Mat::zeros(images[0].size(), CV_16UC1);
        images[i].convertTo(image, CV_16UC1, 1.0);
        complements[i].convertTo(complement, CV_16UC1, 1.0);
        result = image.mul(complement);
        result.convertTo(results[i], CV_8UC1, 1.0/255); // TODO figure this const out
    }
}


