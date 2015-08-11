//
// Created by thaeds on 8/10/15.
//

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>

using namespace std;
using namespace cv;

Mat DetectLines(Mat& source_image, int lineThickness, int lineLength, int lineAnchor, int lineContinue);
Mat WhitenessFilter(Mat& hsv_image);
vector<Mat> PassFilters(Mat image, vector<Mat> kernals);
vector<Mat> MultiplyByComplements(vector<Mat> images, vector<Mat> complements);

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

Mat src_img;

void SliderCallback(int, void*) {
    cout << "Calling DetectLines" << endl;
    Mat detectedLines = DetectLines(src_img, lineThickness, lineLength, lineAnchor, lineContinue);
    imshow(windowName, detectedLines);
}

int main(int argc, char* argv[]) {
    if(argc != 2) {cout << "ERROR, 1 argument expected, the file path of an image" << endl; return -1;}
    src_img = imread(argv[1]);
    if(!src_img.data) {cout << "ERROR, unable to load image: " << argv[1] << endl; return -1;}

    // Initialize Variables
    lineThickness = 0;
    lineAnchor = 0;
    lineContinue = 0;
    lineLength = 0;

    // Create the window
    namedWindow(windowName, 1);

    // Add the sliders
    createTrackbar("Line Thickness (pixels)", windowName, &lineThickness, lineThickness_slider_max, SliderCallback);
    createTrackbar("Line Length Minimum (pixels)", windowName, &lineLength, lineLength_slider_max, SliderCallback);
    createTrackbar("Line Anchor Threshold (pixel value)", windowName, &lineAnchor, lineAnchor_slider_max, SliderCallback);
    createTrackbar("Line Continue Threshold (pixel value)", windowName, &lineContinue, lineContinue_slider_max, SliderCallback);

    // Callback to show original image
    cout << "Calling callback" << endl;
    SliderCallback(0, nullptr);

    // Exit on keypress
    waitKey(0);
    return 0;
}

Mat DetectLines(Mat& source_image, int lineThickness, int lineLength, int lineAnchor, int lineContinue) {
    bool init = true;
    static vector<Mat> kernals;
    static vector<Mat> kernalComplements;
    if(init) {
        cout << "DetectLines::Initing" << endl;
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
        Mat kernal3(9, 9, CV_32FC1, karray[2]);
        kernal3 /= 25;

//        Mat kernal4 = kernal2.t();
        Mat kernal5 = kernal1.t();

//        Mat kernal6;
        Mat kernal7;
//        Mat kernal8;

//        flip(kernal4, kernal6, 0);
        flip(kernal3, kernal7, 0);
//        flip(kernal2, kernal8, 0);

        kernals = {
                kernal1,
//                kernal2,
                kernal3,
//                kernal4,
                kernal5,
//                kernal6,
                kernal7,
//                kernal8
        };
        for(int i = 0; i < kernals.size(); i++) {
            Mat kernalComplement;
            flip(kernals[i], kernalComplement, -1);
            kernalComplements.push_back(kernalComplement);
        }
        init = false;
    }

    // Resize the image such that the lines are approximitely 3 pixels wide
    cout << "DetectLines::Reducing Image" << endl;
    Mat reduced_image;
    resize(source_image, reduced_image, Size(3*source_image.cols/lineThickness, 3*source_image.rows/lineThickness), 0, 0, INTER_LANCZOS4);

    // Convert the image into HSV space to make processing white lines easier
    cout << "DetectLines::Converting to HSV" << endl;
    Mat hsv_image;
    cvtColor(reduced_image, hsv_image, CV_BGR2HSV);

    // Calculate each pixel's "whiteness" defined as value*(255-saturation);
    cout << "DetectLines::Filtering Whiteness" << endl;
    Mat whiteness_image = WhitenessFilter(hsv_image);

    // Pass directional kernals over image
    cout << "DetectLines::Filtering kernals" << endl;
    vector<Mat> filterKernals = PassFilters(whiteness_image, kernals);

    // Pass directional kernal complements over image (same edge, rotated 180 degrees, 3 pixels between edge and complement)
    cout << "DetectLines::Filtering complements" << endl;
    vector<Mat> filterComplements = PassFilters(whiteness_image, kernalComplements);

    // Multiply the results of kernal filter by its complement
    cout << "DetectLines::Multiplying Results" << endl;
    vector<Mat> geometricResults = MultiplyByComplements(filterKernals, filterComplements);

    // LineDrawing implementation reduces detection to single pixel wide lines of a minimum width
//    Mat* detectedLines = new Mat(LineDrawing(geometricResults, lineAnchor, lineContinue, lineLength));

    Mat detectedLines;
    cout << "DetectLines::Thresholding Results" << endl;
    threshold(geometricResults, detectedLines, lineContinue, 255, CV_THRESH_BINARY);
    cout << "DetectLines::Converting to BGR" << endl;
    cvtColor(detectedLines, detectedLines, CV_GRAY2BGR);

    cout << "DetectLines::Returning" << endl;
    return detectedLines;
}

Mat WhitenessFilter(Mat& hsv_image) {
    Mat channel[3];
    split(hsv_image, channel);
    Mat result = 255 * Mat::ones(hsv_image.size(), CV_16UC1);
    result -= channel[1];
    result = result.mul(channel[2]);
    result.convertTo(result, CV_8UC1, 1.0/255);

    return result;
}

vector<Mat> PassFilters(Mat image, vector<Mat> kernals) {
    vector<Mat> results;
    for(int i = 0; i < kernals.size(); i++) {
        Mat result;
        filter2D(image, result, -1, kernals[i]);
        results.push_back(result);
    }

    return results;
}

vector<Mat> MultiplyByComplements(vector<Mat> images, vector<Mat> complements) {
    vector<Mat> results;
    for(int i = 0; i < images.size(); i++) {
        Mat result = Mat::zeros(images[0].size(), CV_16UC1);
        result = images[i].mul(complements[i]);
        result.convertTo(result, CV_8UC1, 1.0/255);
        results.push_back(result);
    }

    return results;
}


