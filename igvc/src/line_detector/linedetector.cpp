#include "linedetector.h"
#include <sstream>
#include <vector>
#include <math.h>
#include <iostream>
#include <cstdlib>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <queue>
#include <algorithm>
#include <vector>
#include <functional>
#include <algorithm>
#include <memory>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
Ptr<BackgroundSubtractor> bg;
cv::Mat fore;
cv::Mat back;
int refresh;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

constexpr double radians(double degrees)
{
    return degrees / 180.0 * M_PI;
}

void LineDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cerr << "CALLBACK CALLED" << endl;
    cv_ptr = cv_bridge::toCvCopy(msg, "");

	// UNCOMMENT THESE LINES TO RUN OTHER METHOD
	//
//	if (primaryMethod()) { // If primary method returns true, stop
//	    return;
//    } else { // if not, use other method
//		otherMethod(msg, cv_ptr);
//		return;
//	}
	//

	// What separates lines from other objects?
    // 1. Lines are whiter than their surroundings
    // 2. Lines are on the ground
    // 3. A section of a line has two other line sections adjascent to it
    // 4. We know roughly how wide a line is going to be

    const int linewidthpixels = 11;
    const int edgethreshold = 15;

    Mat grnd = (cv_ptr->image).clone();
    Mat dview_dst = (cv_ptr->image).clone();
    Mat nature_dst = (cv_ptr->image).clone();
    Mat squish_dst = (cv_ptr->image).clone();

    const double cannyLowerBound = 60;
    const double cannyUpperBound = 100;
    const int cannyApertureSize = 3;
    const bool cannyL2gradient = false;

//// Nature Detector
//    cvtColor(grnd, nature_dst, CV_BGR2GRAY);
//    Canny(nature_dst, nature_dst, cannyLowerBound, cannyUpperBound, cannyApertureSize, cannyL2gradient);
////    blur(nature_dst, nature_dst, Size(5, 5));
//    Mat binnature_dst;
//    threshold(nature_dst, binnature_dst, 0, 255, CV_THRESH_BINARY | CV_THRESH_OTSU);
//    dilate(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 4);
////    erode(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 30);
////    dilate(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 3);
////    erode(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 4);
////    dilate(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 5);
////    erode(binnature_dst, binnature_dst, Mat(), Point(-1, -1), 6);
//    cvtColor(binnature_dst, binnature_dst, CV_GRAY2BGR);
//    cvtColor(nature_dst, nature_dst, CV_GRAY2BGR);
//
//    cv_ptr->image = nature_dst;
//    _filt_img.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = binnature_dst;
//    _filt_img1.publish(cv_ptr->toImageMsg());
//// Ground Detector
//    cvtColor(color1_dst, ground_dst, CV_BGR2GRAY);
//    rectangle(ground_dst, Point(0,0), Point(ground_dst.cols, ground_dst.
//2), Scalar(0, 0, 0), CV_FILLED);
//    rectangle(ground_dst, Point(0, ground_dst.rows/2), Point(ground_dst.cols, ground_dst.rows), Scalar(255, 255, 255), CV_FILLED);

// Ground Slicer
//    grnd = grnd(Rect(0, grnd.rows/3, grnd.cols, 2*grnd.rows/3));
//    grnd = returnWhite(grnd);

// Squish
    resize(grnd, squish_dst, Size(3*grnd.cols/linewidthpixels, 3*grnd.rows/linewidthpixels), 0, 0, INTER_LANCZOS4);

//// Down View
//    transformPoints(squish_dst, grnd);

    float karray[3][9][9] = {
            {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            }, {
                    {-1, -1, -1, -1, -1, -1, -1, -1, -1},
                    {-1, -1, -1, -1, -1, -1, -1,  0,  1},
                    {-1, -1, -1, -1, -1,  0,  1,  1,  1},
                    {-1, -1, -1,  0,  1,  1,  1,  1,  1},
                    {-1,  0,  1,  1,  1,  1,  1, .5,  0},
                    { 1,  1,  1,  1,  1, .5,  0,  0,  0},
                    { 1,  1,  1, .5,  0,  0,  0,  0,  0},
                    { 1, .5,  0,  0,  0,  0,  0,  0,  0},
                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
            },  {
                    {-.89,-.89,-.89,-.89,-.89,-.89,-.89,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,-.89,   1,   1,   1},
                    {-.89,-.89,-.89,-.89,-.89,   1,   1,   1,   0},
                    {-.89,-.89,-.89,-.89,   1,   1,   1,   0,   0},
                    {-.89,-.89,-.89,   1,   1,   1,   0,   0,   0},
                    {-.89,-.89,   1,   1,   1,   0,   0,   0,   0},
                    {-.89,   1,   1,   1,   0,   0,   0,   0,   0},
                    {   1,   1,   1,   0,   0,   0,   0,   0,   0},
                    {   1,   1,   0,   0,   0,   0,   0,   0,   0}
            }
    };

    vector<Mat> kernals;
    Mat kernal1(9, 9, CV_32FC1, karray[0]);
    kernal1 /= 27;
    Mat kernal2(9, 9, CV_32FC1, karray[1]);
    kernal2 /= 25;
    Mat kernal3(9, 9, CV_32FC1, karray[2]);
    kernal3 /= 25;

    Mat kernal4 = kernal2.t();
    Mat kernal5 = kernal1.t();

    Mat kernal6;
    Mat kernal7;
    Mat kernal8;

    flip(kernal4, kernal6, 0);
    flip(kernal3, kernal7, 0);
    flip(kernal2, kernal8, 0);

    kernals.push_back(kernal1);
    kernals.push_back(kernal2);
    kernals.push_back(kernal3);
    kernals.push_back(kernal4);
    kernals.push_back(kernal5);
    kernals.push_back(kernal6);
    kernals.push_back(kernal7);
    kernals.push_back(kernal8);

    Mat testimage = squish_dst;
//    Mat testimage = imread("/home/thaeds/Robojackets/rosigvc/catkin_ws/build/testimage.jpg");

    vector<Mat> results = getGeometricMean(testimage, kernals);

    Mat fadedimage = testimage/2;
    Mat fin_img = results[0].clone();
//
//    cv_ptr->image = results[7];// + fadedimage;
//    _filt_img.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[0];// + fadedimage;
//    _filt_img1.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[1];// + fadedimage;
//    _filt_img2.publish(cv_ptr->toImageMsg());

    subtractOrthog(results);
    fin_img = results[0].clone();

//    cout << "results[1].type(): " << results[1].type() << endl;
//    cout << "fit_img.type(): " << fin_img.type() << endl;

    RemoveNonMax(results);

//    cout << "Removed NonMax" << endl;

    int threshval = 3;
    Mat bgr[3];
    for(int i = 0; i < results.size(); i++) {
        Mat bgr[3];
        split(results[i], bgr);
        threshold(bgr[0], results[i], threshval, 255, CV_THRESH_BINARY);
        cvtColor(results[i], results[i], CV_GRAY2BGR);
    }


//    cout << "results[1].type(): " << results[1].type() << endl;
//    cout << "fit_img.type(): " << fin_img.type() << endl;
    EnforceContinuity(results, fin_img);

//    cerr << "Continuity has been enforced" << endl;
//    drawWhite(fin_img);
//    cout << "results[1].type(): " << results[1].type() << endl;
//    cout << "fit_img.type(): " << fin_img.type() << endl;
//    cv_ptr->image = results[7];// + fadedimage;
//    _filt_img3.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[0];// + fadedimage;
//    _filt_img4.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[1];// + fadedimage;
//    _filt_img5.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[6];
//    _filt_img6.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[7];
//    _filt_img7.publish(cv_ptr->toImageMsg());

//    cout << "about to iterate over results" << endl;
//    for(Mat& r : results)
//        cvtColor(r, r, CV_GRAY2BGR);
//    cout << "last print" << endl;
//    cout << "results[1].type(): " << results[1].type() << endl;
//    cout << "fit_img.type(): " << fin_img.type() << endl;
//    cv_ptr->image = results[0] + fadedimage;
//    _filt_img.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[1] + fadedimage;
//    _filt_img1.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[2] + fadedimage;
//    _filt_img2.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[3] + fadedimage;
//    _filt_img3.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[4] + fadedimage;
//    _filt_img4.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[5] + fadedimage;
//    _filt_img5.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[6] + fadedimage;
//    _filt_img6.publish(cv_ptr->toImageMsg());
//    cv_ptr->image = results[7] + fadedimage;
//    _filt_img7.publish(cv_ptr->toImageMsg());
//
//    for(Mat r : results)
//        fin_img = max(fin_img, r);

    int rows = dview_dst.rows;
    int cols = dview_dst.cols;
//    transformPoints(fin_img, transformDst);
//    cerr << "transformDst rows: " << transformDst.rows << " cols: " << transformDst.cols << endl;
    resize(fin_img, fin_img, Size(dview_dst.cols, dview_dst.rows), 0, 0, INTER_LANCZOS4);
    cloud = toPointCloud(fin_img);
    _line_cloud.publish(cloud);

//    cerr << "fi_img rows: " << fin_img.rows << " cols: " << fin_img.cols << endl;
//    cv_ptr->image = grnd;
//    _filt_img1.publish(cv_ptr->toImageMsg());



//    cerr << "just published to line_cloud" << endl;
}

void LineDetector::RemoveNonMax(vector<Mat>& images) {
    Mat maxRes = images[0].clone();
    for(Mat& r : images) {
        maxRes = max(maxRes, r);
    }
    for(Mat& r : images) {
        r = r - (r != maxRes);
    }
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

void LineDetector::EnforceContinuity(vector<Mat>& directions, Mat& out) {
    cerr << "Enforcing Continuity" << endl;
    int iterations = 300;
    int minpathlength = 20;
    Mat gradient = Mat::zeros(directions[0].rows, directions[0].cols, CV_8UC1);
    Mat fin_img = Mat::zeros(directions[0].rows, directions[0].cols, CV_8UC3);
    for(int i = 0; i < directions.size(); i++) {
        Mat bgr[3];
        split(directions[i], bgr);
//        cerr << "Oring gradient and directions[" << i << "]" << endl;
//        cerr << "gradient r: " << gradient.rows << " c: " <<gradient.cols<<" type: " << gradient.type() <<endl;
//        cerr << "directions r: " << directions[i].rows << " c: " <<directions[i].cols<< " type: " << directions[i].type() <<endl;
        bitwise_and(bgr[0], (0b00000001 << i), bgr[0]);
        bitwise_or(gradient, bgr[0], gradient);
    }
//    cout << "gradient complete: " << endl << gradient << endl;
    auto getneighbors = [](shared_ptr<Node> n, Mat& gradient) {
        auto similarbits = [](unsigned char a, unsigned char b) {
            return ((a == b) | (a == b << 1) | (a == b >> 1) | (129 == (a | b)));
        };
        vector <shared_ptr<Node> > neighbors;
        if(!n) return neighbors;
        Point p = n->point;
        const unsigned char g = gradient.at<uchar>(p);
        bool forward = true;
        int px = n->prev ? (n->point.x > n->prev->point.x ? 1 : -1) : n->length;
        int py = n->prev ? (n->point.y > n->prev->point.y ? 1 : -1) : n->length;

        if (g & 0b00010000) {           // forward -> y new < y old
            if (py > 0) forward = false;
            px = 0;
            py = -2;
        } else if (g & 0b00000001) {    // forward -> x new > x old
            if(px < 0) forward = false;
            px = 2;
            py = 0;
        } else if (g & 0b00000010) {    // forward -> x new > x old
            if(px < 0) forward = false;
            px = 2;
            py = -1;
        } else if (g & 0b00000100) {    // forward -> x new > x old
            if(px < 0 || py > 0) forward = false;
            px = 2;
            py = -2;
        } else if (g & 0b00001000) {    // forward -> y new < y old
            if(py > 0) forward = false;
            px = 1;
            py = -2;
        } else if (g & 0b00100000) {    // forward -> y new < y old
            if(py > 0) forward = false;
            px = -1;
            py = -2;
        } else if (g & 0b01000000) {    // forward -> y new < y old
            if(px > 0 || py > 0) forward = false;
            px = -2;                   //            x new < x old
            py = -2;
        } else if (g & 0b10000000) {    // forward -> x new < x old
            if(px > 0) forward = false;
            px = -2;
            py = -1;
        }
        if(!forward) {
            px *= -1;
            py *= -1;
        }
        if (g) {
            for (int dx = -1; dx < 2; dx++) {
                for (int dy = -1; dy < 2; dy++) {
                    Point newp(p.x + px + dx, p.y + py + dy);
                    if (newp.x < gradient.cols &&
                        newp.x >= 0 &&
                        newp.y < gradient.rows &&
                        newp.y >= 0)
                        if (similarbits(gradient.at<uchar>(newp), g)) {
//                            cerr << "similarbits n->length: " << n->length << endl;
                            neighbors.push_back(make_shared<Node>(n, newp, (n->length > 0) ? (n->length + 1) : (n->length - 1)));
                        }

                }
            }
        }
//        cerr << endl;
//        cerr << "returning current: " << p << " and neighbors: ";
//        for(Node*& neighbor : neighbors)
//            cerr << neighbor->point << ", ";
//        cerr << endl;
//        cerr << "gradient: " << (int) gradient.at<uchar>(p) << "dir: x=" << px << " y=" << py << endl;
        gradient.at<uchar>(p) = 0;
//        cout << "g after del: " << (int) gradient.at<uchar>(p) << endl;

        return neighbors;
    };

    Mat randlineinds;
//    cerr << "Findina Non Zeros" << endl;
    findNonZero(gradient, randlineinds);
//    cerr << "Random Shuffling" << endl;
    randShuffle(randlineinds);
//    cerr << "randlineinds.total()" << randlineinds.total() << endl;

    for(int i = 0; i < iterations && i < randlineinds.total(); i++) {
//        cerr << "Iteration: " << i << " of graph search" << endl;
        Mat gradientcopy = gradient.clone();
        vector<shared_ptr<Node> > frontier;
        shared_ptr<Node> start = make_shared<Node>(nullptr, randlineinds.at<Point>(i), 1);
        shared_ptr<Node> trats = make_shared<Node>(nullptr, randlineinds.at<Point>(i), -1);
        frontier.push_back(trats);
        frontier.push_back(start);

        shared_ptr<Node> current = start;
        shared_ptr<Node> bestpathforward = start;
        shared_ptr<Node> bestpathbackward = trats;

        bool forward = false;
        while(!frontier.empty()) {
            current = frontier.back();
            frontier.pop_back();
//            cerr << "current length: " << current->length << endl;
            if (current->length > 0) {
                if (current->length > bestpathforward->length)
                    bestpathforward = current;
            } else {
                if (current->length < bestpathbackward->length)
                    bestpathbackward = current;
            }

            vector<shared_ptr<Node>> neighbors = getneighbors(current, gradientcopy);
            if(neighbors.empty()) continue;
            for(shared_ptr<Node>& n : neighbors)
                frontier.push_back(n);
        }
//        cerr << endl << endl;
//        cerr << "Current node: point: " << current->point << " prev: " << current->prev << "len: " << current->length << endl;
//        cerr << "gradient: " << (int) gradientcopy.at<uchar>(current->point) << endl;
//        cerr << "forward: " << forward << endl;
//        cerr << "BestF node: point: " << bestpathforward->point << " prev: " << bestpathforward->prev << "len: " << bestpathforward->length << endl;
//        cerr << "BestB node: point: " << bestpathbackward->point << " prev: " << bestpathbackward->prev << "len: " << bestpathbackward->length << endl;
//        cerr << "frontier: " ;
//        for (auto f : frontier)
//            cerr << f << ", ";
//        cerr << endl;
//        char c;
//        cin >> c;

//        cerr <<  " bpb->point == trats->pount" << (bool) (bestpathbackward->point == trats->point) << endl;
//        cerr <<  " bpf->len" << bestpathforward->length << endl;
//        cerr <<  " bpb->len" << bestpathbackward->length << " point: " << bestpathbackward->point << endl;
//        char c;
//        cin >> c;
        if(bestpathforward->length - bestpathbackward->length > minpathlength) {
//            cerr << "adding a line: ";
            current = bestpathforward;
            while (current) {
                fin_img.at<Vec3b>(current->point) = Vec3b(255, 255, 255);
                current = current->prev;
            }
            current = bestpathbackward;
            while (current) {
                fin_img.at<Vec3b>(current->point) = Vec3b(255, 255, 255);
                current = current->prev;
            }
        }
        cv_ptr->image = fin_img;
        cerr << "Publishing to filt_img" << endl;
        _filt_img.publish(cv_ptr->toImageMsg());
        Mat channels[3];
        split(fin_img, channels);
        out = channels[0];
    }

//    cerr << "Graph search complete, setting out and returning" << endl;
//    dilate(fin_img, out, Mat(), Point(-1, -1), 1);
//    vector<Mat> neighborvotes(8);
//    vector<Mat> kernal(8);
//
//    float karray[3][9][9] = {
//            {
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 1,  1,  1,  1,  1,  1,  1,  1,  1},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
//            }, {
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  1,  1},
//                    { 0,  0,  0,  0,  0,  1,  1,  0,  0},
//                    { 0,  0,  0,  1,  1,  0,  0,  0,  1},
//                    { 0,  1,  1,  0,  0,  0,  1,  1,  0},
//                    { 1,  0,  0,  0,  1,  1,  0,  0,  0},
//                    { 0,  0,  1,  1,  0,  0,  0,  0,  0},
//                    { 1,  1,  0,  0,  0,  0,  0,  0,  0},
//                    { 0,  0,  0,  0,  0,  0,  0,  0,  0}
//            },  {
//                    {  0,  0,  0,  0,  0,  0,  0,  1,  0},
//                    {  0,  0,  0,  0,  0,  0,  1,  0,  1},
//                    {  0,  0,  0,  0,  0,  1,  0,  1,  0},
//                    {  0,  0,  0,  0,  1,  0,  1,  0,  0},
//                    {  0,  0,  0,  1,  0,  1,  0,  0,  0},
//                    {  0,  0,  1,  0,  1,  0,  0,  0,  0},
//                    {  0,  1,  0,  1,  0,  0,  0,  0,  0},
//                    {  1,  0,  1,  0,  0,  0,  0,  0,  0},
//                    {  0,  1,  0,  0,  0,  0,  0,  0,  0}
//        }
//   };
//// float karray[3][13][13] = {
////   {
////           { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////           { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////           { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////           { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1,.7,.3, 0, 0, 0, 0, 0, 0, 0,.3,.7, 1},
////            { 1, 1, 1, 1,.7,.3, 1,.3,.7, 1, 1, 1, 1},
////            { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1},
////            { 1, 1, 1, 1,.7,.3, 1,.3,.7, 1, 1, 1, 1},
////            { 1,.7,.3, 0, 0, 0, 0, 0, 0, 0,.3,.7, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////    }, {
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,.9, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,.7, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0,.5, 1, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0,.3, 1, 1, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 1,.1, 1, 1, 1, 1, 1},
////            { 0,.1,.3,.5,.7,.9, 1,.9,.7,.5,.3,.1, 0},
////            { 1, 1, 1, 1, 1,.1, 1, 0, 0, 0, 0, 0, 0},
////            { 1, 1, 1, 1,.3, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1, 1, 1,.5, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1, 1,.7, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1,.9, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////    }, {
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0, 0,.6, 1, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0,.2, 1, 1, 1, 1},
////            { 0, 0, 0, 0, 0, 0, 0, 0,.8, 1, 1,.6, 0},
////            { 0, 0, 0, 0, 0, 0, 0,.4, 1,.8,.2, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 1, 1,.4, 0, 0, 0, 0},
////            { 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0},
////            { 0, 0, 0, 0,.4, 1, 1, 0, 0, 0, 0, 0, 0},
////            { 0, 0,.2,.8, 1,.4, 0, 0, 0, 0, 0, 0, 0},
////            { 0,.6, 1, 1,.8, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1, 1, 1, 1,.2, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1, 1, 1,.6, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////            { 1, 1, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
////    }};
//
//    kernal[0] = Mat(13, 13, CV_32FC1, karray[0]) / 18;
//    kernal[1] = Mat(13, 13, CV_32FC1, karray[1]) / 18;
//    kernal[2] = Mat(13, 13, CV_32FC1, karray[2]) / 16;
//
//    kernal[3] = kernal[1].t();
//    kernal[4] = kernal[0].t();
//
//    flip(kernal[3], kernal[5], 0);
//    flip(kernal[2], kernal[6], 0);
//    flip(kernal[1], kernal[7], 0);
//
//    for(int i = 0; i < directions.size(); i++) {
//        neighborvotes[i]  = applyFilter(directions[(i + kernal.size() - 1) % kernal.size()], kernal[(i + kernal.size() - 1) % kernal.size()]);
//        neighborvotes[i] += applyFilter(directions[i], kernal[i]);
//        neighborvotes[i] += applyFilter(directions[(i + 1) % kernal.size()], kernal[(i + 1) % kernal.size()]);
//    }
//
//    out = neighborvotes[0].clone();
//    for(int i = 1; i < directions.size(); i++) {
//        out = max(out, neighborvotes[i]);
//    }
//
//    return;
}

vector<Mat> LineDetector::getGeometricMean(Mat& image, vector<Mat> &kernals) {
    Mat filtered1, filtered2;
    vector<Mat> results;

    for(int i = 0; i < kernals.size(); i++) {
        Mat kernal2 = kernals[i].clone();
        Mat normmat;
        flip(kernals[i], kernal2, -1);

        filtered1 = applyFilter(image, kernals[i]);
        filtered2 = applyFilter(image, kernal2);


        filtered1.convertTo(filtered1, CV_16UC3, 1);
        filtered2.convertTo(filtered2, CV_16UC3, 1);

        results.push_back(filtered1.mul(filtered2));
        results[i].convertTo(results[i], CV_8UC3, 1.0 / 256);
    }

//    char c;
//    cin >> c;

    return results;
};

Mat LineDetector::applyFilter(Mat &image, const Mat &kernal) {
    Mat fin_img;
    vector<Mat> newchannel(3);
    vector<Mat> oldchannel(3);
    split(image, oldchannel);
    for(int i = 0; i < 3; i++) {
        filter2D(oldchannel[i], newchannel[i], -1, kernal); //, Point(-1, -1), 200);
    }

    merge(newchannel, fin_img);
    return fin_img;
}

bool operator<(const Vec3f& s1, const Vec3f& s2) {
    return s1[0] < s2[0];
}

void LineDetector::subtractOrthog(vector<Mat>& images) {
    for (int r = 0; r < images[0].rows; r++) {
        for(int c = 0; c < images[0].cols; c++) {
            Vec3b blackpixel(0, 0, 0);
            Vec3b maxdiff(0, 0, 0);
            Vec3b smax(0, 0, 0);
            for(int i = 0; i < images.size(); i++){
                Vec3b s1 = images[i].at<Vec3b>(r, c);
                Vec3b s2 = images[(i+images.size()/2) % images.size()].at<Vec3b>(r, c);
                images[i].at<Vec3b>(r, c) = s1 - s2;
            }
        }
    }
}

int threshold_value = 0;
int threshold_type = 3;
int const max_value = 255;
int const max_type = 4;
int const max_BINARY_value = 255;

Mat src_gray;

string window_name = "Threshold Demo";

string trackbar_type = "Type: \n 0: Binary \n 1: Binary Inverted \n 2: Truncate \n 3: To Zero \n 4: To Zero Inverted";
string trackbar_value = "Value";

void Threshold_Test(int, void*) {
    Mat dst;
    Mat bgr[3];

    split(src_gray, bgr);
    hconcat(bgr[1], bgr[2], dst);
    hconcat(bgr[0], dst, dst);
    threshold( dst, dst, threshold_value, max_BINARY_value, threshold_type );

    imshow(window_name, dst);
}

void LineDetector::drawWhite(Mat &image) {

    /// Convert the image to Gray
    src_gray = image.clone();
//    cvtColor( image, src_gray, CV_BGR2GRAY );

    /// Spread pixel values over space
//    equalizeHist(src_gray, src_gray);

    /// Create a window to display results
    namedWindow( window_name, CV_WINDOW_AUTOSIZE );

    /// Create Trackbar to choose type of Threshold
    createTrackbar( trackbar_type,
                    window_name, &threshold_type,
                    max_type, Threshold_Test );

    createTrackbar( trackbar_value,
                    window_name, &threshold_value,
                    max_value, Threshold_Test );

    /// Call the function to initialize
    Threshold_Test( 0, 0 );

    /// Wait until user finishes program
    while(true)
    {
        int c;
        c = waitKey( 20 );
        if( (char)c == 27 )
        { break; }
    }
}

void LineDetector::Erosion(Mat* dst) {
  int erosion_type = MORPH_ELLIPSE;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  // Apply the erosion operation
  erode( *dst, *dst, element );
}

LineDetector::LineDetector(ros::NodeHandle &handle)
    : max_elem(2),
      max_kernel_size(2),
      gaussian_size(7),
      _it(handle),
      tf_listener(handle)
{
    erosion_elem = 2;
    erosion_size = 1;
    dilation_elem = 2;
    dilation_size = 2;
    bg = new BackgroundSubtractorMOG2();
	refresh = 0;

    _src_img = _it.subscribe("/left/image_rect_color", 1, &LineDetector::img_callback, this);
//    _src_img = _it.subscribe("/left/image_raw", 1, &LineDetector::img_callback, this);
	_filt_img = _it.advertise("/filt_img", 1);
//    _filt_img1 = _it.advertise("/filt_img1", 1);
//    _filt_img2 = _it.advertise("/filt_img2", 1);
//    _filt_img3 = _it.advertise("/filt_img3", 1);
//    _filt_img4 = _it.advertise("/filt_img4", 1);
//    _filt_img5 = _it.advertise("/filt_img5", 1);
//    _filt_img6 = _it.advertise("/filt_img6", 1);
//    _filt_img7 = _it.advertise("/filt_img7", 1);
//    _filt_img8 = _it.advertise("/filt_img8", 1);
    _line_cloud = handle.advertise<PCLCloud>("/line_cloud", 100);

}

bool LineDetector::otherMethod(const sensor_msgs::ImageConstPtr& msg, cv_bridge::CvImagePtr cv_ptr) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
	
    //cv::resize(cv_ptr->image, cv_ptr->image, cv::Size(562, 384));
    
	Mat imgmat = cv_ptr->image;
    int rows = imgmat.rows;
    int cols = imgmat.cols;
	
	int threshh = 150;
	int rb = rows - (rows / 2);
	int sum = 0;
	blackoutSection(imgmat, 0, rb, 0, cols);
	int rb2 = rows - (rows / 10);
    for (int i = rb; i< rows; i++){
        for(int j = 0; j< cols; j++) {
            sum += imgmat.at<Vec3b>(i, j)[0];
        }
    }
	threshh = (sum / ((rows-rb) * cols)) + 55;
	int thresh = getAvg(imgmat);

	cv::Mat test3;
	inRange(imgmat, Scalar(150, 150, 150), Scalar(255, 255, 255), test3);
	cv::Mat test4;
	cvtColor(test3, test4, CV_GRAY2BGR);
	//imgmat = imgmat & test4;
	inRange(imgmat, Scalar(0, 0, 0), Scalar(255, 255, 150), test3);
	cvtColor(test3, test4, CV_GRAY2BGR);
	//imgmat = imgmat & test4;
	inRange(imgmat, Scalar(0, 0, 0), Scalar(255, 150, 255), test3);
	cvtColor(test3, test4, CV_GRAY2BGR);
	//imgmat = imgmat & test4;


    
	
	cvtColor(imgmat, imgmat, CV_BGR2GRAY);
	cvtColor(imgmat, imgmat, CV_GRAY2BGR);
    for (int i = 0; i< rows; i++){
        for(int j=0; j< cols; j++) {
			if (imgmat.at<Vec3b>(i, j)[0] <= thresh) {
				imgmat.at<Vec3b>(i, j)[0] = 0;
				imgmat.at<Vec3b>(i, j)[1] = 0;
				imgmat.at<Vec3b>(i, j)[2] = 0;
			}
        }
    }
    GaussianBlur(imgmat, imgmat, Size(17, 17), 6, 6);
	cvtColor(imgmat, imgmat, CV_BGR2GRAY);

	cv::Mat test;
	double otsuVal = cv::threshold(imgmat, test, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	double highThresh = otsuVal;
	double lowThresh = otsuVal * 0.5;
	Canny(imgmat, imgmat, lowThresh, highThresh);
	vector<Vec4i> lines;
	HoughLinesP(imgmat, lines, 1, (M_PI/180), 40, 30, 10);
	cvtColor(imgmat, imgmat, CV_GRAY2BGR);
	
	//blackoutSection(imgmat, 0, rows, 0, cols);
    for( size_t i = 0; i < lines.size(); i++ )
    {
        line( imgmat, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }
	blackoutSection(imgmat, rb2, rows, 0, cols);
    rb = rb + 10;
	blackoutSection(imgmat, 0, rb, 0, cols);

	//cvtColor(imgmat, imgmat, CV_BGR2GRAY);
	/*cv::Mat skel(imgmat.size(), CV_8UC1, cv::Scalar(0));
	cv::Mat temp(imgmat.size(), CV_8UC1);
	cv::Mat element = cv::getStructuringElement(cv::MORPH_CROSS, cv::Size(3, 3));

	bool done;
	do
	{
	  cv::morphologyEx(imgmat, temp, cv::MORPH_OPEN, element);
	  cv::bitwise_not(temp, temp);
	  cv::bitwise_and(imgmat, temp, temp);
	  cv::bitwise_or(skel, temp, skel);
	  cv::erode(imgmat, imgmat, element);
	 
	  double max;
	  cv::minMaxLoc(imgmat, 0, &max);
	  done = (max == 0);
	} while (!done);*/


	
    cv_ptr->image = imgmat;
	cv::Mat transformDst(rows, cols, CV_8UC3);
    transformPoints(imgmat, transformDst);
//	cv_ptr->image = transformDst;
	//cvtColor(cv_ptr->image, cv_ptr->image, CV_BGR2GRAY);
	//cvtColor(cv_ptr->image, cv_ptr->image, CV_GRAY2BGR);
    _filt_img.publish(cv_ptr->toImageMsg());
    cloud = toPointCloud(transformDst);

    cout <<"Sending new matrix"<<endl;

    _line_cloud.publish(cloud);
}

bool LineDetector::primaryMethod() {

    dst = &cv_ptr->image;
    cv::resize(*dst, *dst, cv::Size(1024, 768));
    /** Total Average of the pixels in the screen. Used to account for brightness variability. */
    float totalAvg = getAvg();

    /** Blurs the picture just a little */
    GaussianBlur(*dst, *dst, Size(gaussian_size,gaussian_size),2,0);
    /** Separates the pixels into black(not lines) and white (lines) */
    blackAndWhite(totalAvg);

	cvtColor(*dst, *dst, CV_BGR2GRAY);

	cv::Mat test;
	double otsuVal = cv::threshold(*dst, test, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
	double highThresh = otsuVal;
	double lowThresh = otsuVal * 0.5;
	Canny(*dst, *dst, lowThresh, highThresh);
	vector<Vec4i> lines;
	HoughLinesP(*dst, lines, 1, (M_PI/180), 40, 30, 10);
	cvtColor(*dst, *dst, CV_GRAY2BGR);
    if (lines.size() > 200) {
        return false; // If there are too many lines detected, use other method
    }
    int rows = dst->rows;
    int cols = dst->cols;
	//blackoutSection(*dst, 0, rows, 0, cols);
	for( size_t i = 0; i < lines.size(); i++ )
    {
        line( *dst, Point(lines[i][0], lines[i][1]),
            Point(lines[i][2], lines[i][3]), Scalar(0,0,255), 3, 8 );
    }
	int rb2 = rows - (rows / 10);
	blackoutSection(*dst, rb2, rows, 0, cols);
    cv::Mat transformDst(dst->rows, dst->cols, CV_8UC3);
    transformPoints(*dst, transformDst);
    _filt_img.publish(cv_ptr->toImageMsg());
    cloud = toPointCloud(transformDst);
		

    cout <<"Sending new matrix"<<endl;

    _line_cloud.publish(cloud);

    return true;

	// TODO REPLACE WITH ROS COMMUNICATION
    //onNewCloud(cloud, offset);

}

/** Dilation enhances the white lines */
void LineDetector::Dilation(Mat* dst)
{
  int dilation_type = MORPH_ELLIPSE;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );
  // Apply the dilation operation
  dilate( *dst, *dst, element );

}

/**
 *  @brief LineDetector::blackAndWhite converts the image into
 *         black (not lines) and white (lines)
 *  @param totalAvg The average brightness of the picture
 */
void LineDetector::blackAndWhite(float totalAvg){
    Vec3b p;
    int rows = dst->rows;
    int cols = dst->cols;

    //Turn the top quarter of the screen and bottom sixth of the screen black
    //We can disregard these areas - may extend the bottom of the screen slightly later on
	blackoutSection(0, rows*4/9, 0, cols);
//    for (int i = rows*5/6; i< rows; i++){
//        for(int j=0; j< cols; j++){
//             dst->at<Vec3b>(i,j)[0] = 0;
//             dst->at<Vec3b>(i,j)[1] = 0;
//             dst->at<Vec3b>(i,j)[2] = 0;
//        }
//    }

    //Loops through relevant parts of the image and scans for white lines
    //Also tries to detect obstacles
    int tempAvg;
    float redUp = 1;
    float redDown = 1;
    float greenUp = 1;
    float greenDown = 1;
    float blueUp = 3.5;
    float blueDown = 3;
    int diff = 5;
    for (int i = rows*4/9; i< rows; i++){
        for(int j=0; j< cols; j++){
            tempAvg = totalAvg*(1.1 - i*.1/768);
            p = dst->at<Vec3b>(i, j); //Current pixel

            //If there is a significant amount of red in the pixel, it's most likely an orange cone
            //Get rid of the obstacle
            if (/*p[2] > totalAvg*2 && */p[2] > 253){
                detectObstacle(i, j, dst);
            }

            //Filters out the white and makes it pure white
            if((p[0]>tempAvg*blueDown) && (p[0] < tempAvg*blueUp) || (p[0] < 20 && p[1] < 20 && p[2] < 20) && (p[1] < tempAvg*greenUp) && (p[2]>tempAvg*redDown)
                    && (p[2]<tempAvg*redUp) && (p[1]>tempAvg*greenDown) && (abs(p[1] - p[2]) <tempAvg/diff)) {
                dst->at<Vec3b>(i,j)[0] = 255;
                dst->at<Vec3b>(i,j)[1] = 255;
                dst->at<Vec3b>(i,j)[2] = 255;

            }

            else { //Otherwise, set pixel to black
                dst->at<Vec3b>(i,j)[0] = 0;
                dst->at<Vec3b>(i,j)[1] = 0;
                dst->at<Vec3b>(i,j)[2] = 0;//all 0's
            }
        }
    }
}

/**
 *  \brief LineDetector::detectObstacle detects orange and bright white obstacles
 *  \param col the column of the left of the obstacle
 */
void LineDetector::detectObstacle(int row, int col, cv::Mat* dst){
    Vec3b p = dst->at<Vec3b>(row,col);
    int row2 = row;
    int col2 = col;

    //While the pixel is still orange, turn it black
    //Then on to the next one, by row
    while (p[2] > 100 /*&& (p[1] < 150 || p[1] > 250)*/){
        dst->at<Vec3b>(row2, col)[0] = 0;
        dst->at<Vec3b>(row2, col)[1] = 0;
        dst->at<Vec3b>(row2, col)[2] = 0;
        p = dst->at<Vec3b>(++row2, col);
    }
    p = dst->at<Vec3b>(row,col);

    //While the pixel is still orange, turn it black
    //Then on to the next one, by column
    while (p[2] > 100 /*&& (p[1] < 150 || p[1] > 250)*/){
        dst->at<Vec3b>(row, col2)[0] = 0;
        dst->at<Vec3b>(row, col2)[1] = 0;
        dst->at<Vec3b>(row, col2)[2] = 0;
        p = dst->at<Vec3b>(row, ++col2);
    }

    //Turn everything in that block we just found black
    for(int i = row+1; i<row2;i++){
        for (int j = col+1; j<col2; j++){
            dst->at<Vec3b>(i,j)[0] = 0;
            dst->at<Vec3b>(i,j)[1] = 0;
            dst->at<Vec3b>(i,j)[2] = 0;
        }
    }
}

/**
 *  \brief LineDetector::getAvg gets the average of the relevant pixels
 *  \return the average as a floating point number
 */
float LineDetector::getAvg(){
    Mat region = (*dst)(Range(dst->rows/6, 5*dst->rows/6), Range(dst->cols/6, 5*dst->cols/6));
    Scalar sumScalar = cv::sum(region);
    float avg = sumScalar[0] + sumScalar[1] + sumScalar[2];
    avg /= dst->rows * dst->cols * dst->channels();
    return avg;
}

float LineDetector::getAvg(Mat &imgmat){
    Mat region = imgmat(Range(imgmat.rows/6, 5*imgmat.rows/6), Range(imgmat.cols/6, 5*imgmat.cols/6));
    Scalar sumScalar = cv::sum(region);
    float avg = sumScalar[0] + sumScalar[1] + sumScalar[2];
    avg /= imgmat.rows * imgmat.cols * imgmat.channels();
    return avg;
}

/**
 *  \brief LineDetector::blackoutSection turns a section of the image black
 *  \param rowl the lower row bound
 *  \param rowu the upper row bound
 *  \param coll the left column bound
 *  \param colu the right column bound
 */
void LineDetector::blackoutSection(int rowl, int rowu, int coll, int colu){

    for (int i=rowl;i<=rowu;i++){
        for (int j = coll; j<=colu; j++){
            dst->at<Vec3b>(i,j)[0] = 0;
            dst->at<Vec3b>(i,j)[1] = 0;
            dst->at<Vec3b>(i,j)[2] = 0;
        }
    }
}

void LineDetector::blackoutSection(Mat &imgmat, int rowl, int rowu, int coll, int colu) {
	for (int i = rowl; i < rowu; i++) {
		for (int j = coll; j < colu; j++) {
			imgmat.at<Vec3b>(i, j)[0] = 0;
			imgmat.at<Vec3b>(i, j)[1] = 0;
			imgmat.at<Vec3b>(i, j)[2] = 0;
		}
	}
}

void LineDetector::transformPoints(Mat &src, Mat &dst){
	//pcam is where the coordinates are in actual space (in meters right now)
	//pcam = (cv::Mat_<float>(4,2) << offset-12,72, offset, 72, offset, 60,offset -12, 60);
	// pcam = (cv::Mat_<float>(4,2) << 4,81, -8, 81, -8, 93,4, 93);
	int squareSize = 100;
	Mat pcam = (cv::Mat_<float>(4,2) << dst.cols/2 - (squareSize/2),dst.rows-squareSize, dst.cols/2+(squareSize/2), dst.rows-squareSize, dst.cols/2-(squareSize/2), dst.rows - squareSize*2, dst.cols/2+(squareSize/2), dst.rows - squareSize*2);
	//p is where they show up as pixels on the camera
	//p = (cv::Mat_<float>(4,2) << 427, 642, 515, 642, 512, 589, 432, 588);
	// p= (cv::Mat_<float>(4,2) << 440, 674, 356, 679, 364, 631, 439, 627);
	float ratioRows = (float) src.rows/768;
	float ratioCols = (float) src.cols/1024;
	Mat p= (cv::Mat_<float>(4,2) << 344*ratioCols, 646*ratioRows, 668*ratioCols, 636*ratioRows, 415*ratioCols, 496*ratioRows, 619*ratioCols, 488*ratioRows);
	//pcam = pcam*3+450; //This is just so we can see it on the screen
	//Getting the transform
	Mat transformMat = cv::getPerspectiveTransform(p, pcam);
	//Apply the transform to dst and store result in dst
	cv::warpPerspective(src, dst, transformMat, dst.size());
}

PointCloud<PointXYZ>::Ptr LineDetector::toPointCloud(Mat src){
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera", ros::Time(0), transform);
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

void LineDetector::backgroundSubtractor() {
//	bg.set("nmixtures", 3);
//	bg.set("bShadowDetection", true);
//	bg.set("nShadowDetection", 0);
//	bg.set("fTau", 0.5);
//	if (refresh > 2) {
//    	bg = new BackgroundSubtractorMOG2();
//		refresh = 0;
//	} else {
//		refresh += 1;
//	}
//    vector<std::vector<cv::Point>> contours;
//    
//    bg->operator()(test4, fore);
//	cvtColor(imgmat, imgmat, CV_BGR2GRAY);
//	imgmat = imgmat & fore;
//    bg->getBackgroundImage(back);

//    cv::erode(fore,fore,cv::Mat());
//    cv::dilate(fore,fore,cv::Mat());

//    cv::findContours(fore,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
//    cv::drawContours(imgmat,contours,-1,cv::Scalar(0,0,255),2);
}

void LineDetector::houghLines() {
	//vector<Vec2f> lines;
	/*for( size_t i = 0; i < lines.size(); i++ )
    {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( imgmat, pt1, pt2, Scalar(0,0,255), 3, 8 );
    }*/
}

void LineDetector::horzVertEdgeDet() {
	 /*Mat temp1;
	 Mat temp2;
	 Mat kernal = (Mat_<double>(7, 1) << 1, 1, 1, 0, -1, -1, -1);
	 kernal = kernal/7;
	 filter2D(cv_ptr->image, temp1, -1, kernal);
	 kernal = (Mat_<double>(1, 7) << 1, 1, 1, 0, -1, -1, -1);
	 kernal = kernal/7;
	 filter2D(cv_ptr->image, temp2, -1, kernal);
	 imgmat = temp1 + temp2;*/
}

void LineDetector::blobFinder() {
	/*vector< tuple<int, int> > visited;
	vector< vector< tuple<int, int> > > leftblobs;
	vector< vector< tuple<int, int> > > rightblobs;
    //cv::resize(imgmat, imgmat, cv::Size(281, 192));
    rows = imgmat.rows;
    cols = imgmat.cols;
	int rl = rows - (rows / 3);
	int blobLength = 50;
	for (int i = rl; i < rows; i+=4){
        grey = imgmat.at<Vec3b>(i, 0)[0];
        if (grey != 0) {
			if (!helperContains(tuple<int, int>(i, 0), &visited)) {
				vector< tuple<int, int> > blob;
            	helperSearch(&visited, &blob, i, 0, &imgmat, thresh);
				if (blob.size() > blobLength) {
					leftblobs.push_back(blob);
				}
        	}
        }
		grey = imgmat.at<Vec3b>(i, 4)[0];
        if (grey != 0) {
			if (!helperContains(tuple<int, int>(i, 4), &visited)) {
				vector< tuple<int, int> > blob;
            	helperSearch(&visited, &blob, i, 4, &imgmat, thresh);
				if (blob.size() > blobLength) {
					leftblobs.push_back(blob);
				}
        	}
        }
		int rx = cols - 1;
        grey = imgmat.at<Vec3b>(i, rx)[0];
        if (grey != 0) {
			if (!helperContains(tuple<int, int>(i, rx), &visited)) {
				vector< tuple<int, int> > blob;
            	helperSearch(&visited, &blob, i, rx, &imgmat, thresh);
				if (blob.size() > blobLength) {
					rightblobs.push_back(blob);
				}
        	}
        }
		rx -= 4;
        grey = imgmat.at<Vec3b>(i, rx)[0];
        if (grey != 0) {
			if (!helperContains(tuple<int, int>(i, rx), &visited)) {
				vector< tuple<int, int> > blob;
            	helperSearch(&visited, &blob, i, rx, &imgmat, thresh);
				if (blob.size() > blobLength) {
					rightblobs.push_back(blob);
				}
        	}
        }
		if (i >= rows-15 && i < rows - 6) {
			int mcols = cols / 3;
			for(int j=0; j < mcols; j+=4) {
		        grey = imgmat.at<Vec3b>(i, j)[0];
		        if (grey != 0) {
					if (!helperContains(tuple<int, int>(i, j), &visited)) {
						vector< tuple<int, int> > blob;
		            	helperSearch(&visited, &blob, i, j, &imgmat, thresh);
						if (blob.size() > blobLength) {
							leftblobs.push_back(blob);
						}
		        	}
		        }
		    }
			int ncols = 2*mcols;
			for(int j=ncols; j < cols; j+=4) {
		        grey = imgmat.at<Vec3b>(i, j)[0];
		        if (grey != 0) {
					if (!helperContains(tuple<int, int>(i, j), &visited)) {
						vector< tuple<int, int> > blob;
		            	helperSearch(&visited, &blob, i, j, &imgmat, thresh);
						if (blob.size() > blobLength) {
							rightblobs.push_back(blob);
						}
		        	}
		        }
		    }
		}
    }
	vector< tuple<int, int> > line1;
	vector< tuple<int, int> > line2;
	Mat test(rows, cols, CV_8UC3);
	for (int i = 0; i< rows; i++){
	    for(int j=0; j< cols; j++) {
		    test.at<Vec3b>(i, j)[0] = 0;
		    test.at<Vec3b>(i, j)[1] = 0;
		    test.at<Vec3b>(i, j)[2] = 0;
	    }
	}
	
	int x;
	int y;

	if (leftblobs.size() > 0) {
		line1 = leftblobs.at(0);
		
		for (int i = 1; i < leftblobs.size(); i++) {
			if (leftblobs.at(i).size() > line1.size()) {
				line1 = leftblobs.at(i);
			}
		}

		for (int i = 0; i < line1.size(); i++) {
			x = get<0>(line1.at(i));
			y = get<1>(line1.at(i));
			if (x >= 0 && x < rows && y >= 0 && y < cols) {
				test.at<Vec3b>(x, y)[0] = 255;
				test.at<Vec3b>(x, y)[1] = 255;
				test.at<Vec3b>(x, y)[2] = 255;
			}
		}
	}
	
	if (rightblobs.size() > 0) {
		line2 = rightblobs.at(0);

		for (int i = 1; i < rightblobs.size(); i++) {
			if (rightblobs.at(i).size() > line2.size()) {
				line2 = rightblobs.at(i);
			}
		}

		for (int i = 0; i < line2.size(); i++) {
			x = get<0>(line2.at(i));
			y = get<1>(line2.at(i));
			if (x >= 0 && x < rows && y >= 0 && y < cols) {
				test.at<Vec3b>(x, y)[0] = 255;
				test.at<Vec3b>(x, y)[1] = 255;
				test.at<Vec3b>(x, y)[2] = 255;
			}
		}
	}*/
}



void LineDetector::helperSearch(void *totvis, void *input, int i, int j, cv::Mat *imgmat, int thresh) {
    vector< tuple<int, int> > *totalvisited = (vector< tuple<int, int> >*)totvis;
    vector< tuple<int, int> > *visited = (vector< tuple<int, int> >*)input;
    if (!helperContains(tuple<int, int>(i, j), visited) && !helperContains(tuple<int, int>(i, j), totalvisited)) {
		visited->push_back( tuple<int, int>(i, j) );
		totalvisited->push_back( tuple<int, int>(i, j) );
	} else {
		return;
	}
	if (visited->size() > 500) {
		return;
	}
    int grey = 0;
    for (int k=0; k<5; k+=4) {
        if (i-2+k < 0 || i-2+k >= imgmat->rows || j < 0 || j >= imgmat->cols) {
            continue;
        }
        grey = imgmat->at<Vec3b>(i-2+k, j)[0];
        if (adjWhite(i-2+k, j, imgmat, thresh)) {
            helperSearch(totalvisited, visited, i-2+k, j, imgmat, thresh);
        }
	}
	for (int k=0; k<3; k+=2) {
        if (i-2+k < 0 || i-2+k >= imgmat->rows || j+2 < 0 || j+2 >= imgmat->cols) {
            continue;
        }
        if (adjWhite(i-2+k, j+2, imgmat, thresh)) {
            helperSearch(totalvisited, visited, i-2+k, j+2, imgmat, thresh);
        }
	}
	/*for (int k=0; k<3; k+=2) {
        if (i-2+k < 0 || i-2+k > imgmat->rows || j-2 < 0 || j-2 > imgmat->cols) {
            continue;
        }
        grey = imgmat->at<Vec3b>(i-2+k, j-2)[0];
        if (grey >= 150) {
            helperSearch(totalvisited, visited, i-2+k, j-2, imgmat, thresh);
        }
	}*/
}

bool LineDetector::adjWhite(int i, int j, cv::Mat *img, int thresh) {
	int grey;
	int scal = 1;
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j+scal;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	for (int k = -1; k < 2; k++) {
		int n = i+(k*scal);
		int m = j-scal;
        if (n < 0 || n >= img->rows || m < 0 || m >= img->cols) {
            continue;
        }
        grey = img->at<Vec3b>(n, m)[0];
        if (grey >= thresh) {
            return true;
        }
	}
	return false;
}

bool LineDetector::helperContains(tuple<int, int> pos, void *vis) {
    vector< tuple<int, int> > *visited = (vector< tuple<int, int> >*)vis;
    for (int i=0; i<visited->size(); i++) {
        if (visited->at(i) == pos) {
            return true;
        }
	}
    return false;
}

//
////void LineDetector::FindLines(CvImagePtr cv_ptr) {
//
//
////}
//
bool LineDetector::isWorking() {
    return true;
}
