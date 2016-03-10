#include "potholedetector.h"

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

// Radii for hough circle detector
const int maxRadius = 50;
const int minRadius = 10;

// Radius for averaging pixels in a circle around the center
const int whiteSampleRadius = 30;
// Size for filtering out contours that are too small
const int contourSizeThreshold = 200;

int getDiff(int a, int b) {
    return abs(a - b);
}

double toRadians(double degrees) {
    return degrees / 180.0 * M_PI;
}

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg) {
    cv_ptr = cv_bridge::toCvCopy(msg, "");
    Mat orig = cv_ptr->image.clone();
    src = cv_ptr->image.clone();

    // Crops the image (removes sky)
    int topCrop = src.rows / 2 - 100;
    Rect roiNoSky(0, topCrop, src.cols, src.rows - topCrop);
    src = src(roiNoSky);

    // Determine the average of blue, green, and red pixels in the entire picture in order to do adaptive thresholding.
    // Therefore, we can account for sunny / cloudy days.
    double blueImageAverage = 0;
    double greenImageAverage = 0;
    double redImageAverage = 0;
    for (int i = 0; i <= src.cols; i++) {
        for (int j = 0; j <= src.rows; j++) {
            Vec3b currentPixel = src.at<Vec3b>(j, i);
            blueImageAverage += currentPixel[0];
            greenImageAverage += currentPixel[1];
            redImageAverage += currentPixel[2];
        }
    }
    blueImageAverage /= (src.rows * src.cols);
    greenImageAverage /= (src.rows * src.cols);
    redImageAverage /= (src.rows * src.cols);

    // Converts the image into grayscale, storing it in src_gray
    cvtColor(src, src_gray, CV_BGR2GRAY);

    // Find the mean and stddev of the grayscale image in order to do adaptive thresholding
    Mat mean;
    Mat stddev;
    meanStdDev(src_gray, mean, stddev);
    double thresh = mean.at<double>(0,0) + (stddev.at<double>(0,0) * 2);
    if(thresh > 254) {
        thresh = 254;
    }

    // Threshold and adaptive blur of the grayscale image
    threshold(src_gray, src_gray, thresh, 255, THRESH_BINARY);
    GaussianBlur(src_gray, src_gray, Size(gaussian_size, gaussian_size), 2, 2);

    // Detect circles on the image
    vector<Vec3f> circles;
    HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, src_gray.rows / 8, 50, 10, minRadius, maxRadius);

    // All the contours that will be published to the pointcloud
    vector<vector<Point>> allContours;

    // Traverse through all the circles, and do more detection for each one
    for(size_t i = 0; i < circles.size(); i++ ) {
        Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

        // If the circle is too close to the top / bottom edges, filter
        if (center.y <= 100 || center.y >= src_gray.rows - 100) {
            continue;
        }

        // If the circle is too close to the left / right edges, filter
        if (center.x <= 100 || center.x >= src_gray.cols - 100) {
            continue;
        }

        // Determine the crop size for the rectangle around the circle
        int cropSize = (125 / (double) src_gray.rows) * center.y;

        // Set up the rectangle cordinates
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

        // Create the rectangle and an src_gray with only that region of interest
        Rect roiAroundCircle(xStart, yStart, xWidth, yHeight);
        Mat src_gray_roiAroundCircle = src_gray(roiAroundCircle);

        // Average up all the pixels in a circle around the center of the pothole.
        double bluePotholeAverage = 0;
        double greenPotholeAverage = 0;
        double redPotholeAverage = 0;
        for (int i = -whiteSampleRadius; i <= whiteSampleRadius; i++) {
            for (int j = -whiteSampleRadius; j <= whiteSampleRadius; j++) {
                // Equation to only check points in a circle...
                if (i * i + j * j <= whiteSampleRadius * whiteSampleRadius) {
                    Vec3b currentPixel = src.at<Vec3b>(j + center.y, i + center.x);
                    bluePotholeAverage += currentPixel[0];
                    greenPotholeAverage += currentPixel[1];
                    redPotholeAverage += currentPixel[2];
                }
            }
        }
        bluePotholeAverage /= (M_PI * whiteSampleRadius * whiteSampleRadius);
        greenPotholeAverage /= (M_PI * whiteSampleRadius * whiteSampleRadius);
        redPotholeAverage /= (M_PI * whiteSampleRadius * whiteSampleRadius);

        // Our adaptive thresholding comes into play here.
        // Filter out the circle if the average color of the image is not an offset less than the average color of the center circle sample.
        if (bluePotholeAverage - blueImageAverage < 75 || greenPotholeAverage - greenImageAverage < 40 || redPotholeAverage - redImageAverage < 20) {
            continue;
        }

        // Find contours within this small region of interest around the circle
        // Use an offset to align with the entire src image (which has a cropped sky)
        vector<vector<Point>> contours;
        Point offset(xStart, yStart);
        findContours(src_gray_roiAroundCircle, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, offset);

        // Here is the start of our code to filter out false positives....... :/
        // Traverse through each single contour first
        for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it) {
            // Filter out contours that are too small
            if ((*it).size() > contourSizeThreshold) {
                int minY = src_gray.rows;
                int minX = src_gray.cols;
                int maxY = 0;
                int maxX = 0;

                // For each contour, find the min and max X coordinates by traversing through all the points.
                for (Point p : *it) {
                    int x = p.x;
                    if (x > maxX) {
                        maxX = x;
                    }
                    if (x < minX) {
                        minX = x;
                    }
                }

                // Determine the center x
                int centerX = (minX + maxX) / 2;

                // Find top and bottom points that correspond to the centerX (so get the y values)
                for (Point p : *it) {
                    int x = p.x;
                    int y = p.y;
                    if (x == centerX && y > maxY) {
                        maxY = y;
                    }
                    if (x == centerX && y < minY) {
                        minY = y;
                    }
                }

                // Average rgb values in a line above and below the contour
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

                    currentPixel = src.at<Vec3b>(maxY + j, centerX);
                    blueBelow += currentPixel[0];
                    greenBelow += currentPixel[1];
                    redBelow += currentPixel[2];
                }
                blueAbove /= 30;
                greenAbove /= 30;
                redAbove /= 30;
                blueBelow /= 30;
                greenBelow /= 30;
                redBelow /= 30;

                // We now have the average color of the line above and below the contour.
                // Use these averages to determine if the contour is a white strip on the traffic barrel.
                // Aka if there is orange above and below the contour, filter it!
                if (redAbove > greenAbove + 50 && redAbove > blueAbove + 50
                        && redBelow > greenBelow + 50 && redBelow > blueBelow + 50) {
                    contours.erase(it);
                    --it;
                }
            } else {
                contours.erase(it);
                --it;
            }
        }

        // Push all of the remaining good points to the main vector of contours
        for (vector<Point> goodContour : contours) {
            allContours.push_back(goodContour);
        }
    }

    // Convert the contours to a pointcloud
    cloud = toPointCloud(allContours, orig.rows, orig.cols);

    cv_bridge::CvImage out_msg;
    out_msg.header   = msg->header;
    out_msg.encoding = msg->encoding;
    out_msg.image    = src;

    cv_ptr->image = src;
    _pothole_filt_img.publish(cv_ptr->toImageMsg());
    _pothole_thres.publish(out_msg.toImageMsg());
    _pothole_cloud.publish(cloud);
}

PotholeDetector::PotholeDetector(ros::NodeHandle &handle)
    : gaussian_size(7),
      _it(handle),
      tf_listener(handle) {
    _src_img = _it.subscribe("/left/image_rect_color", 1, &PotholeDetector::img_callback, this);
    _pothole_filt_img = _it.advertise("/pothole_filt_img", 1);
    _pothole_thres = _it.advertise("/pothole_thres", 1);
    _pothole_cloud = handle.advertise<PCLCloud>("/pothole_cloud", 100);
}

PointCloud<PointXYZ>::Ptr PotholeDetector::toPointCloud(vector<vector<Point>> &contours, int height, int width) {
    PointCloud<PointXYZ>::Ptr cloud(new PointCloud<PointXYZ>);
    tf::StampedTransform transform;
    tf_listener.lookupTransform("/base_footprint", "/camera_left", ros::Time(0), transform);
    double roll, pitch, yaw;
    tf::Matrix3x3(transform.getRotation()).getRPY(roll, pitch, yaw);
    auto origin_z = transform.getOrigin().getZ();
    auto origin_y = transform.getOrigin().getY();
    auto HFOV = toRadians(66.0);
    auto VFOV = toRadians(47.6);
    pitch = -roll;

    for (vector<Point> contour : contours) {
        for (Point p : contour) {
            int xP = p.x;
            int yP = p.y + (height / 2 - 100);

            auto pitch_offset = ((float) (yP - height / 2) / height) * VFOV;
            auto y = origin_z /tan(pitch + pitch_offset) + origin_y;

            auto theta = ((float) (xP - width / 2) / width) * HFOV;
            auto x = y * tan(theta);

            cloud->points.push_back(PointXYZ(x, y, 0));
        }
    }

	cloud->header.frame_id = "base_footprint";
	return cloud;
}
