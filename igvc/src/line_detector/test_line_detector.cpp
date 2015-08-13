//
// Created by thaeds on 8/10/15.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

#define KERNAL_COUNT 4

using namespace std;
using namespace cv;

void DetectLines(int lineThickness, int lineLength, int lineAnchor, int lineContinue);
void WhitenessFilter(Mat& hsv_image, Mat& result);
void PassFilters(Mat& image, Mat* results, Mat* kernals);
void MultiplyByComplements(Mat* images, Mat* complements, Mat* results);

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

void SliderCallback(int, void*) {
    cerr << "Calling DetectLines" << endl;
    DetectLines(lineThickness, lineLength, lineAnchor, lineContinue);
    cerr << "DetectLines returned" << endl;
    imshow(windowName, dst_img);
}

int main(int argc, char* argv[]) {
    if(argc != 2) {cerr << "ERROR, 1 argument expected, the file path of an image" << endl; return -1;}
    src_img = imread(argv[1]);
    if(!src_img.data) {cerr << "ERROR, unable to load image: " << argv[1] << endl; return -1;}

    // Initialize Variables
    filename = argv[1];
    dst_img = Mat::zeros(src_img.size(), src_img.type());
    lineThickness = 20;
    lineAnchor = 5;
    lineContinue = 1;
    lineLength = 1;

    // Create the window
    namedWindow(windowName, 0);

    // Add the sliders
    createTrackbar("Line Thickness (pixels)", windowName, &lineThickness, lineThickness_slider_max, SliderCallback);
    createTrackbar("Line Length Minimum (pixels)", windowName, &lineLength, lineLength_slider_max, SliderCallback);
    createTrackbar("Line Anchor Threshold (pixel value)", windowName, &lineAnchor, lineAnchor_slider_max, SliderCallback);
    createTrackbar("Line Continue Threshold (pixel value)", windowName, &lineContinue, lineContinue_slider_max, SliderCallback);

    // Callback to show original image
    cerr << "Calling callback" << endl;
    SliderCallback(0, nullptr);

    // Exit on keypress
    while(waitKey(0) != 'q');
    return 0;
}

void DetectLines(int lineThickness, int lineLength, int lineAnchor, int lineContinue) {
    static bool init = true;
    static Mat kernals[4];
    static Mat kernalComplements[4];
    Mat working;
    Mat kernalResults[KERNAL_COUNT];
    Mat complementResults[KERNAL_COUNT];
    src_img = imread(filename);
    dst_img = Mat::zeros(src_img.size(), src_img.type());
    if(init) {
        cerr << "DetectLines::Initing" << endl;
        float karray[2][9][9] = {
                {
                        {-1,   -1,   -1,   -1,   -1,   -1,   -1,   -1, -1},
                        {-1,   -1,   -1,   -1,   -1,   -1,   -1, -1, -1},
                        {-1,   -1,   -1,   -1,   -1,   -1, -1, -1, -1},
                        {1,    1,    1,    1,    1, 1, 1, 1, 1},
                        {1,    1,    1,    1, 1, 1, 1, 1,  1},
                        {1,    1,    1, 1, 1, 1,  1, 1, 1},
                        {0,    0, 0, 0,  0, 0, 0, 0, 0},
                        {0, 0,  0, 0, 0, 0, 0, 0, 0},
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
                        {-.89, -.89, -.89, 1, 1, 1, 0, 0,  0},
                        {-.89, -.89, 1, 1, 1, 0,  0, 0, 0},
                        {-.89, 1, 1, 1,  0, 0, 0, 0, 0},
                        {1, 1,  1, 0, 0, 0, 0, 0, 0},
                        {1, 1, 0, 0, 0, 0, 0, 0, 0}
                }
        };

            Mat kernal1(9, 9, CV_32FC1, karray[0]);
            kernal1 /= 27;
    //        Mat kernal2(9, 9, CV_32FC1, karray[1]);
    //        kernal2 /= 25;
            Mat kernal3(9, 9, CV_32FC1, karray[1]);
            kernal3 /= 25;

    //        Mat kernal4 = kernal2.t();
            Mat kernal5 = kernal1.t();

    //        Mat kernal6;
            Mat kernal7;
    //        Mat kernal8;

    //        flip(kernal4, kernal6, 0);
            flip(kernal3, kernal7, 0);
    //        flip(kernal2, kernal8, 0);

            kernals[0] = kernal1;
    //                kernal2,
            kernals[1] = kernal3;
    //                kernal4,
            kernals[2] = kernal5;
    //                kernal6,
            kernals[3] = kernal7;
    //                kernal8

            for(int i = 0; i < KERNAL_COUNT; i++) {
                Mat kernalComplement;
                flip(kernals[i], kernalComplement, -1);
                kernalComplements[i] = kernalComplement;
            }
            init = false;
        }

        // Resize the image such that the lines are approximitely 3 pixels wide
        cerr << "DetectLines::Reducing Image" << endl;
        resize(src_img, working, Size(3*src_img.cols/lineThickness, 3*src_img.rows/lineThickness), 0, 0, INTER_LANCZOS4);

        // Convert the image into HSV space to make processing white lines easier
        cerr << "DetectLines::Converting to HSV" << endl;
        cvtColor(working, working, CV_BGR2HSV);

        // Calculate each pixel's "whiteness" defined as value*(255-saturation);
        cerr << "DetectLines::Filtering Whiteness" << endl;
        WhitenessFilter(working, working);

        // Pass directional kernals over image
        cerr << "DetectLines::Filtering kernals" << endl;
        PassFilters(working, kernalResults, kernals);

        // Pass directional kernal complements over image (same edge, rotated 180 degrees, 3 pixels between edge and complement)
        cerr << "DetectLines::Filtering complements" << endl;
        PassFilters(working, complementResults, kernalComplements);

        // Multiply the results of kernal filter by its complement
        cerr << "DetectLines::Multiplying Results" << endl;
        MultiplyByComplements(kernalResults, complementResults, kernalResults);


        // LineDrawing implementation reduces detection to single pixel wide lines of a minimum width
    //    Mat* detectedLines = new Mat(LineDrawing(geometricResults, lineAnchor, lineContinue, lineLength));

        working = Mat::zeros(kernalResults[0].size(), kernalResults[0].type());
        cerr << "DetectLines::Thresholding Results" << endl;
        for(int i = 0; i < KERNAL_COUNT; i++) {
            threshold(kernalResults[i], kernalResults[i], lineContinue, 255, CV_THRESH_BINARY);
            bitwise_or(working, kernalResults[i], working);
        }
        resize(working, dst_img, src_img.size(), src_img.type());
        cerr << "DetectLines::Converting to BGR" << endl;
//        cvtColor(dst_img, dst_img, CV_GRAY2BGR);
}

void WhitenessFilter(Mat& hsv_image, Mat& fin_img) {
    Mat result = 255 * Mat::ones(hsv_image.size(), CV_16UC1);
    Mat tmp;
    hsv_image.convertTo(tmp, CV_16UC3, 1.0);
    Mat channel[3];
    split(tmp, channel);
    result -= channel[1];
    result = result.mul(channel[2]);
    result.convertTo(fin_img, CV_8UC1, 1.0/255);
}

void PassFilters(Mat& image, Mat* results, Mat* kernals) {
   for(int i = 0; i < KERNAL_COUNT; i++) {
        filter2D(image, results[i], -1, kernals[i]);
    }
}

void MultiplyByComplements(Mat* images, Mat* complements, Mat* results) {
    for(int i = 0; i < KERNAL_COUNT; i++) {
        Mat image;
        Mat complement;
        images[i].convertTo(image, CV_16UC1, 1.0);
        complements[i].convertTo(complement, CV_16UC1, 1.0);
        Mat result = Mat::zeros(images[0].size(), CV_16UC1);
        result = image.mul(complement);
        result.convertTo(results[i], CV_8UC1, 1.0/255); // TODO figure this const out
    }
}


