#include "linedetector.h"
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <queue>
#include <chrono>
#include <ctime>

using namespace std;
using namespace cv;
using namespace pcl;


cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

constexpr double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    // chrono::time_point<chrono::system_clock> start, end; // instantiate time to use
    // start = chrono::system_clock::now();

    cv_ptr = cv_bridge::toCvCopy(msg, "");
    src_img = cv_ptr->image;
    dst_img = Mat::zeros(src_img.size(), src_img.type());
    fin_img = Mat::zeros(src_img.size(), src_img.type());

    // What separates lines from other objects?
    // 1. Lines are whiter than their surroundings
    // 2. Lines are on the ground
    // 3. A section of a line has two other line sections adjascent to it
    // 4. We know roughly how wide a line is going to be

    DetectLines(lineThickness);

    EnforceLength(working, lineLengthThreshold);

    resize(working, fin_img, Size(src_img.cols, src_img.rows), 0, 0, INTER_LANCZOS4);

    cloud = toPointCloud(fin_img);
    _line_cloud.publish(cloud);

    cvtColor(fin_img, fin_img, CV_GRAY2BGR);
    cv_ptr->image = fin_img;
    _filt_img.publish(cv_ptr->toImageMsg());

    // end = chrono::system_clock::now();
    // chrono::duration<double> elapsedTime = end - start;

    // cerr<<elapsedTime.count();
}

typedef struct Node {
    shared_ptr<struct Node> prev;
    Point point;
    int length;

    Node(shared_ptr<struct Node> pr, Point po, int l)
    {
        prev = pr;
        point = po;
        length = l;
    }
} Node;

LineDetector::LineDetector(ros::NodeHandle &handle)
      : _it(handle)
      , tf_listener(handle)
{
     cout<<"Running"<<endl;
    _src_img = _it.subscribe("/stereo/left/image_raw", 1, &LineDetector::img_callback, this);
	  _filt_img = _it.advertise("/filt_img", 1);
    _line_cloud = handle.advertise<PCLCloud>("/line_cloud", 100);
    initLineDetection();
}


PointCloud<PointXYZ>::Ptr LineDetector::toPointCloud(Mat src){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera_left", ros::Time(0), transform);
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    auto origin_z = transform.getOrigin().getZ();
    auto origin_y = transform.getOrigin().getY();
    auto HFOV = radians(66.0);
    auto VFOV = radians(47.6);
    pitch = -roll; // Because conventions are different and I'm in the middle of comp, and give me a break.
    for(int r = src.rows/2; r < src.rows; r++)
    {
        uchar *row = src.ptr<uchar>(r);
        for(int c = 0; c < src.cols; c++)
        {
            if(row[c] > 0)
            {
                auto pitch_offset = ((float)(r-src.rows/2) / src.rows) * VFOV;
                auto y = origin_z /tan(pitch + pitch_offset) + origin_y;

                auto theta = ((float)(c-src.cols/2) / src.cols) * HFOV;
                auto x = y * tan(theta);

                cloud->points.push_back(PointXYZ(x, y, 0));
            }
        }
    }
    cloud->header.frame_id = "base_footprint";
    return cloud;
}

void LineDetector::initLineDetection() {
    cerr << "DetectLines::Initing" << endl;
    lineThickness = 20;
    lineLengthThreshold = 30;

    float karray[3][9][9] = {
            {
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},    
                    {1,     1,     1,     1,     1,     1,     1,     1,     1},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0}
            },     
            {
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,    -1,    -1},    
                    {-1,   -1,    -1,    -1,    -1,    -1,    -1,     0,     1},    
                    {-1,   -1,    -1,    -1,    -1,     0,     1,     1,     1},    
                    {-1,   -1,    -1,     0,     1,     1,     1,     1,     1},    
                    {-1,    0,     1,     1,     1,     1,     1,    .5,     0},    
                    {1,     1,     1,     1,     1,    .5,     0,     0,     0},    
                    {1,     1,     1,    .5,     0,     0,     0,     0,     0},    
                    {1,    .5,     0,     0,     0,     0,     0,     0,     0},    
                    {0,     0,     0,     0,     0,     0,     0,     0,     0}
            },
            {
                    {-.89,-.89, -.89,  -.89,  -.89,  -.89,  -.89,     1,     1},    
                    {-.89,-.89, -.89,  -.89,  -.89,  -.89,     1,     1,     1},    
                    {-.89,-.89, -.89,  -.89,  -.89,     1,     1,     1,     0},    
                    {-.89,-.89, -.89,  -.89,     1,     1,     1,     0,     0},    
                    {-.89,-.89, -.89,     1,     1,     1,     0,     0,     0},    
                    {-.89,-.89,    1,     1,     1,     0,     0,     0,     0},    
                    {-.89,  1,     1,     1,     0,     0,     0,     0,     0},    
                    {1,     1,     1,     0,     0,     0,     0,     0,     0},    
                    {1,     1,     0,     0,     0,     0,     0,     0,     0}
            }
    };

    Mat kernel1(9, 9, CV_32FC1, karray[0]);
    kernel1 /= 27;
    Mat kernel2(9, 9, CV_32FC1, karray[1]);
    kernel2 /= 25;
    Mat kernel3(9, 9, CV_32FC1, karray[2]);
    kernel3 /= 25;

    Mat kernel4 = kernel2.t();
    Mat kernel5 = kernel1.t();

    Mat kernel6;
    Mat kernel7;
    Mat kernel8;

    flip(kernel4, kernel6, 0);
    flip(kernel3, kernel7, 0);
    flip(kernel2, kernel8, 0);

    kernels[0] = kernel1.clone();
    kernels[1] = kernel2.clone();
    kernels[2] = kernel3.clone();
    kernels[3] = kernel4.clone();
    kernels[4] = kernel5.clone();
    kernels[5] = kernel6.clone();
    kernels[6] = kernel7.clone();
    kernels[7] = kernel8.clone();

    for (int i = 0; i < KERNAL_COUNT; i++) {
        Mat kernelComplement;
        flip(kernels[i], kernelComplement, -1);
        kernelComplements[i] = kernelComplement.clone();
    }
}


void LineDetector::DetectLines(int lineThickness) {

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

    working = Mat::zeros(kernelResults[0].size(), kernelResults[0].type());
    //cerr << "DetectLines::Thresholding Results" << endl;
    for(int i = 0; i < KERNAL_COUNT; i++) {
        // threshold(kernelResults[i], kernelResults[i], ((float)lineContinue*lineContinue)/255, 255, CV_THRESH_BINARY);
        adaptiveThreshold(kernelResults[i], kernelResults[i], 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 9, -1.85);
        bitwise_or(working, kernelResults[i], working);
    }
}

void LineDetector::WhitenessFilter(Mat& hsv_image, Mat& fin_img) {
    Mat result = 255 * Mat::ones(hsv_image.size(), CV_16UC1);
    Mat tmp;
    hsv_image.convertTo(tmp, CV_16UC3, 1.0);
    Mat channel[3];
    split(tmp, channel);
    result = result - channel[1];
    result = result.mul(channel[2]);
    result.convertTo(fin_img, CV_8UC1, 1.0/255);
}

void LineDetector::MultiplyByComplements(Mat* images, Mat* complements, Mat* results) {
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


void LineDetector::EnforceLength(Mat& img, int length) {
    // Thresholding the line length using the area of the contours
    vector<vector<Point>> contours;
    vector<vector<Point>> contoursThreshold;
    vector<Vec4i> hierarchy;

    Mat copy = img.clone();
    findContours(copy, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);

    for (unsigned int i = 0; i < contours.size(); ++i) {
        if ((int) contours[i].size() < length) {
            contoursThreshold.push_back(contours[i]);
        }
    }

    Scalar color(0, 0, 0);
    drawContours(img, contoursThreshold, -1, color, 3);
}
