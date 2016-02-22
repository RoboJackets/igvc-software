#include "potholedetector.h"
#include <opencv2/video/video.hpp>
#include <sensor_msgs/image_encodings.h>
#include <pcl_ros/point_cloud.h>
#include <camera_info_manager/camera_info_manager.h>
#include <math.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

const int maxRadius = 50;
const int minRadius = 10;

// Set 1 to disable
const int minWhiteThreshold = 255 * 0.60;
const int maxWhiteThreshold = 255 * 1;
const int whiteSampleRadius = 30;

const int sizeThreshold = 200;

const int lightROrange = 230;
const int lightGOrange = 180;
const int lightBOrange = 180;
const int darkROrange = 140;
const int darkGOrange = 50;
const int darkBOrange = 40;

int getDiff(int a, int b) {
    return abs(a - b);
}

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat orig = cv_ptr->image.clone();
    src = cv_ptr->image.clone();

    //Crops the image (removes sky)
    int topCrop = src.rows / 2 - 100;
    cv::Rect myROI(0, topCrop, src.cols, src.rows - topCrop);
    src = src(myROI);

    cvtColor(src, src_gray, CV_BGR2GRAY);

    //Find the mean and stddev of the grayscale image in order to do adaptive thresholding
    Mat mean;
    Mat stddev;
    meanStdDev(src_gray, mean, stddev);

    double thresh = mean.at<double>(0,0) + (stddev.at<double>(0,0) * 2);
    if(thresh > 254) {
        thresh = 254;
    }

    threshold(src_gray, src_gray, thresh, 255, THRESH_BINARY);

    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 2, 2);

    vector<Vec3f> circles;

    // TODO tune circle radii for actual potholes
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 50, 10, minRadius, maxRadius);

    Mat cloudMat = Mat::zeros(orig.rows, orig.cols, CV_32F);
    /// Put the circles into a matrix
    for( size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
        int radius = cvRound(circles[i][2]);

        // If the circle is too close to the top / bottom edges, filter
        if (center.y <= 100 || center.y >= src_gray.rows - 100) {
            continue;
        }

        // If the circle is too close to the left / right edges, filter
        if (center.x <= 100 || center.x >= src_gray.cols - 100) {
            continue;
        }

        int cropSize = (125 / (double) src_gray.rows) * center.y;

        int xStart = center.x - (cropSize * 2);
        if (xStart < 0) {
            xStart = 0;
        }
        int yStart = center.y - cropSize;
        if (yStart < 0) {
            yStart = 0;
        }
        int xWidth = cropSize * 4;
        if (xWidth + xStart > src_gray.cols) {
            xWidth = src_gray.cols - xStart;
        }
        int yHeight = cropSize * 2;
        if (yHeight + yStart > src_gray.rows) {
            yHeight = src_gray.rows - yStart;
        }

        //Get a matrix around circle center
        cv::Rect myROI(xStart, yStart, xWidth, yHeight);
        Mat roi = src_gray(myROI);

        // If the sum of all the pixels in the rectangle is less than sumThreshold
        // Then this circle is not encompassing mostly white pixels
        const int sumThreshold = whiteSampleRadius * whiteSampleRadius * (double) 3.141592 * 3;
        double sum = 0;
        for (int i = -whiteSampleRadius; i <= whiteSampleRadius; i++) {
            for (int j = -whiteSampleRadius; j <= whiteSampleRadius; j++) {
                if (i * i + j * j <= whiteSampleRadius * whiteSampleRadius) {
                    Vec3b currentPixel = src.at<Vec3b>(j + center.y, i + center.x);
                    sum += currentPixel[0] + currentPixel[1] + currentPixel[2];
                }
            }
        }
        Point sampleCenter(center.x, center.y);

        if (sum < sumThreshold * minWhiteThreshold || sum > sumThreshold * maxWhiteThreshold) {
            continue;
        }

        //circle(cloudMat, center, radius, Scalar(255), 1, 8, 0);
        rectangle(src, myROI, Scalar(255), 2, 8, 0);
        circle(src, sampleCenter, whiteSampleRadius, Scalar(255), 1, 8, 0);
        circle(src, center, radius, Scalar(255), 2, 8, 0);

        Point offset(center.x - (cropSize * 2), center.y - cropSize);
        vector<vector<Point>> contours;
        findContours(roi, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, Point((center.x - (cropSize * 2)), center.y - cropSize));

        // Filter false positives - small contours
        /*
        for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it) {
            vector<Point> curCont = *it;
            if (curCont.size() <= sizeThreshold) {
                contours.erase(it);
                --it;
            }
        } */

        // Filter false positives - small contours and cone stripes
        for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it) {
            if ((*it).size() > sizeThreshold) {
                // Find center in x frame
                int minY = src_gray.rows;
                int minX = src_gray.cols;
                int maxY = 0;
                int maxX = 0;
                for (Point p : *it) {
                    int x = p.x;
                    if (x > maxX) {
                        maxX = x;
                    }
                    if (x < minX) {
                        minX = x;
                    }
                }
                int centerX = (minX + maxX) / 2;
                int realMinY = src_gray.rows;
                int realMaxY;

                // Find centers of top and bottom edges
                for (Point p : *it) {
                    int x = p.x;
                    int y = p.y;
                    if (x == centerX && y > maxY) {
                        maxY = y;
                    }
                    if (x == centerX && y < minY) {
                        minY = y;
                    }

                    if (y > realMaxY) {
                        realMaxY = y;
                    }
                    if (y < realMinY) {
                        realMinY = y;
                    }
                }

                int realCenterY = (realMinY + realMaxY) / 2;

                // Check if contour is within bounds of image
                if (minY - 35 >= 0) {
                    // Average rgb values above and below the contour
                    int blueAbove = 0;
                    int greenAbove = 0;
                    int redAbove = 0;
                    int blueBelow = 0;
                    int greenBelow = 0;
                    int redBelow = 0;
                    Vec3b currentPixel;
                    for (int j = 5; j < 36; j++) {
                        currentPixel = src.at<Vec3b>(minY - j, centerX);
                        blueAbove += currentPixel[0];
                        greenAbove += currentPixel[1];
                        redAbove += currentPixel[2];
                        currentPixel[0] = 0;
                        currentPixel[1] = 0;
                        currentPixel[2] = 0;
                        src.at<Vec3b>(minY - j, centerX) = currentPixel;
                        currentPixel = src.at<Vec3b>(maxY + j, centerX);
                        blueBelow += currentPixel[0];
                        greenBelow += currentPixel[1];
                        redBelow += currentPixel[2];
                        currentPixel[0] = 0;
                        currentPixel[1] = 0;
                        currentPixel[2] = 0;
                        src.at<Vec3b>(maxY + j, centerX) = currentPixel;
                    }
                    blueAbove /= 30;
                    greenAbove /= 30;
                    redAbove /= 30;
                    blueBelow /= 30;
                    greenBelow /= 30;
                    redBelow /= 30;

                    if (redAbove > greenAbove + 50 && redAbove > blueAbove + 50 && redBelow > greenBelow + 50 && redBelow > blueBelow + 50) {
                        contours.erase(it);
                        --it;
                    } else {
                        // Delete if the contour itself is orange
                        Vec3b intensity = src.at<Vec3b>((minY + maxY) / 2, centerX);
                        uchar blue = intensity.val[0];
                        uchar green = intensity.val[1];
                        uchar red = intensity.val[2];
                        if ((getDiff(red, darkROrange) < 50 && getDiff(green, darkGOrange) < 50
                                && getDiff(blue, darkBOrange) < 50)
                                || (getDiff(red, lightROrange) < 50 && getDiff(green, lightGOrange) < 50
                                && getDiff(blue, lightBOrange) < 50)) {
                            contours.erase(it);
                            --it;
                        } else {
                            cerr << to_string(maxX - minX) + " " + to_string(maxY - minY) << endl;
                            int xDist = maxX - minX;
                            int yDist = realMaxY - realMinY;
                            if (xDist < yDist * 1.9 || xDist > yDist * 3.1) {
                                contours.erase(it);
                                --it;
                            }
                        }
                    }

                    // // Filter out contours with orange above and below
                    // // If above is light or dark orange and below is light or dark
                    // if (((getDiff(redAbove, darkROrange) < 50
                    //         && getDiff(greenAbove, darkGOrange) < 50
                    //         && getDiff(blueAbove, darkBOrange) < 50)
                    //         || (getDiff(redAbove, lightROrange) < 50
                    //         && getDiff(greenAbove, lightGOrange) < 50
                    //         && getDiff(blueAbove, lightBOrange) < 50))
                    //     && ((getDiff(redBelow, darkROrange) < 50
                    //         && getDiff(greenBelow, darkGOrange) < 50
                    //         && getDiff(blueBelow, darkBOrange) < 50)
                    //         || (getDiff(redBelow, lightROrange) < 50
                    //         && getDiff(greenBelow, lightGOrange) < 50
                    //         && getDiff(blueBelow, lightBOrange) < 50))) {
                    //     // Delete contour
                    //     contours.erase(it);
                    //     --it;
                    // } else {
                    //     cerr << "" + to_string(blueAbove) + ", " + to_string(greenAbove) + ", " + to_string(redAbove) + "     " + to_string(blueBelow) + ", " + to_string(greenBelow) + ", " + to_string(redBelow)  << endl;
                    // }
                } else {
                    contours.erase(it);
                    --it;
                }
            } else {
                contours.erase(it);
                --it;
            }
        }

        drawContours(src, contours, -1, Scalar(20, 236, 27), 3, 8);
    }

    cvtColor(src_gray, src_gray, CV_GRAY2BGR);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image    = src;

    cv_ptr->image = src;
    _pothole_filt_img.publish(cv_ptr->toImageMsg());
    _pothole_thres.publish(out_msg.toImageMsg());
    cloud = toPointCloud(cloudMat);
    _pothole_cloud.publish(cloud);
}

PotholeDetector::PotholeDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribe("/stereo/right/image_raw", 1, &PotholeDetector::img_callback, this);
    _pothole_filt_img = _it.advertise("/pothole_filt_img", 1);
    _pothole_thres = _it.advertise("/pothole_thres", 1);
    _pothole_cloud = handle.advertise<PCLCloud>("/pothole_cloud", 100);
}

// FIXME does not take into account distance from groud to camera
PointCloud<PointXYZ>::Ptr PotholeDetector::toPointCloud(Mat src) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    for(int r = 0; r < src.rows; r++) {
        float *row = src.ptr<float>(r);
        for(int c = 0; c < src.cols; c++) {
            if(row[c] > 0) {
                cloud->points.push_back(PointXYZ(r, c, 0));
            }
        }
    }
	cloud->header.frame_id = "base_footprint";
	return cloud;
}
