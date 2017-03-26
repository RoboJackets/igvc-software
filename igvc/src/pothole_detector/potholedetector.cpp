#include "potholedetector.h"
#include <igvc/CVUtils.hpp>

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

void PotholeDetector::info_img_callback(const sensor_msgs::ImageConstPtr& msg,
                                        const sensor_msgs::CameraInfoConstPtr& cam_info)
{
  cam.fromCameraInfo(cam_info);
  img_callback(msg);
}

void PotholeDetector::img_callback(const sensor_msgs::ImageConstPtr& msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg, "");

  cv::Mat orig = cv_ptr->image.clone();
  src = cv_ptr->image.clone();

  // Crops the image (removes sky)
  int topCrop = 2 * src.rows / 3;
  cv::Rect roiNoSky(0, topCrop, src.cols, src.rows - topCrop);
  src = src(roiNoSky);

  // Determine the average of blue, green, and red pixels in the entire picture in order to do adaptive thresholding.
  // Therefore, we can account for sunny / cloudy days.
  double blueImageAverage = 0;
  double greenImageAverage = 0;
  double redImageAverage = 0;
  for (int i = 0; i <= src.cols; i++)
  {
    for (int j = 0; j <= src.rows; j++)
    {
      cv::Vec3b currentPixel = src.at<cv::Vec3b>(j, i);
      blueImageAverage += currentPixel[0];
      greenImageAverage += currentPixel[1];
      redImageAverage += currentPixel[2];
    }
  }
  blueImageAverage /= (src.rows * src.cols);
  greenImageAverage /= (src.rows * src.cols);
  redImageAverage /= (src.rows * src.cols);

  // Converts the image into grayscale, storing it in src_gray
  cv::cvtColor(src, src_gray, CV_BGR2GRAY);
  // Find the mean and stddev of the grayscale image in order to do adaptive thresholding
  cv::Mat mean;
  cv::Mat stddev;
  meanStdDev(src_gray, mean, stddev);
  double thresh = mean.at<double>(0, 0) + (stddev.at<double>(0, 0) * 1.5);
  if (thresh > 254)
  {
    thresh = 254;
  }
  // Threshold and adaptive blur of the grayscale image
  threshold(src_gray, src_gray, thresh, 255, cv::THRESH_BINARY);
  GaussianBlur(src_gray, src_gray, cv::Size(gaussian_size, gaussian_size), 2, 2);

  // Detect circles on the image
  std::vector<cv::Vec3f> circles;
  // cvHoughCircles(CvArr* image, void* circle_storage, int method, double dp, double min_dist, double param1=100,
  // double param2=100, int min_radius=0, int max_radius=0 )
  HoughCircles(src_gray, circles, CV_HOUGH_GRADIENT, 1, orig.rows / 8, 50, 10, minRadius, maxRadius);

  // All the contours that will be published to the pointcloud
  std::vector<std::vector<cv::Point>> allContours;
  // Traverse through all the circles, and do more detection for each one
  for (size_t i = 0; i < circles.size(); i++)
  {
    cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));

    // If the circle is too close to the top / bottom edges, filter
    if (center.y <= 100 || center.y >= src_gray.rows - 100)
    {
      // Note: bounds must be greater than a gap of 36, else invalid memory reference
      continue;
    }

    // If the circle is too close to the left / right edges, filter
    if (center.x <= 100 || center.x >= src_gray.cols - 100)
    {
      continue;
    }

    // Determine the crop size for the rectangle around the circle
    int cropSize = (125 / (double)src_gray.rows) * center.y;

    // Set up the rectangle cordinates
    int xStart = center.x - (cropSize * 2);
    if (xStart < 0)
    {
      xStart = 0;
    }
    int yStart = center.y - cropSize;
    if (yStart < 0)
    {
      yStart = 0;
    }
    int xWidth = cropSize * 4;
    if (xWidth + xStart > src_gray.cols)
    {
      xWidth = src_gray.cols - xStart;
    }
    int yHeight = cropSize * 2;
    if (yHeight + yStart > src_gray.rows)
    {
      yHeight = src_gray.rows - yStart;
    }

    circle(src_gray, center, circles[i][2], 255, 5);

    // Create the rectangle and an src_gray with only that region of interest
    cv::Rect roiAroundCircle(xStart, yStart, xWidth, yHeight);
    cv::Mat src_gray_roiAroundCircle = src_gray(roiAroundCircle);

    // Average up all the pixels in a circle around the center of the pothole.
    double bluePotholeAverage = 0;
    double greenPotholeAverage = 0;
    double redPotholeAverage = 0;
    for (int i = -whiteSampleRadius; i <= whiteSampleRadius; i++)
    {
      for (int j = -whiteSampleRadius; j <= whiteSampleRadius; j++)
      {
        // Equation to only check points in a circle...
        if (i * i + j * j <= whiteSampleRadius * whiteSampleRadius)
        {
          cv::Vec3b currentPixel = src.at<cv::Vec3b>(j + center.y, i + center.x);
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
    // Filter out the circle if the average color of the image is not an offset less than the average color of the
    // center circle sample.
    if (bluePotholeAverage - blueImageAverage < blueAdaptiveThreshold ||
        greenPotholeAverage - greenImageAverage < greenAdaptiveThreshold ||
        redPotholeAverage - redImageAverage < redAdaptiveThreshold)
    {
      continue;
    }

    // Find contours within this small region of interest around the circle
    // Use an offset to align with the entire src image (which has a cropped sky)
    std::vector<std::vector<cv::Point>> contours;
    cv::Point offset(xStart, yStart);
    findContours(src_gray_roiAroundCircle, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE, offset);

    // Here is the start of our code to filter out false positives....... :/
    // Traverse through each single contour first
    for (std::vector<std::vector<cv::Point>>::iterator it = contours.begin(); it != contours.end(); ++it)
    {
      // Filter out contours that are too small
      if ((*it).size() > (unsigned int)(contourSizeThreshold))
      {
        int minY = src_gray.rows;
        int minX = src_gray.cols;
        int maxY = 0;
        int maxX = 0;

        // For each contour, find the min and max X coordinates by traversing through all the points.
        for (cv::Point p : *it)
        {
          int x = p.x;
          if (x > maxX)
          {
            maxX = x;
          }
          if (x < minX)
          {
            minX = x;
          }
        }

        // Determine the center x
        int centerX = (minX + maxX) / 2;
        // Find top and bottom points that correspond to the centerX (so get the y values)
        for (cv::Point p : *it)
        {
          int x = p.x;
          int y = p.y;
          if (x == centerX && y > maxY)
          {
            maxY = y;
          }
          if (x == centerX && y < minY)
          {
            minY = y;
          }
        }
        int centerY = (minY + maxY) / 2;

        // Average rgb values in a line above and below the contour
        int blueAbove = 0;
        int greenAbove = 0;
        int redAbove = 0;
        int blueBelow = 0;
        int greenBelow = 0;
        int redBelow = 0;
        cv::Vec3b currentPixel;
        for (int j = 5; j < 36; j++)
        {
          currentPixel = src.at<cv::Vec3b>(minY - j, centerX);
          blueAbove += currentPixel[0];
          greenAbove += currentPixel[1];
          redAbove += currentPixel[2];

          currentPixel = src.at<cv::Vec3b>(maxY + j, centerX);
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
        if (redAbove > greenAbove + 50 && redAbove > blueAbove + 50 && redBelow > greenBelow + 50 &&
            redBelow > blueBelow + 50)
        {
          contours.erase(it);
          --it;
        }

        // Average rgb values in a line left and right of the contour
        int blueLeft = 0;
        int greenLeft = 0;
        int redLeft = 0;
        int blueRight = 0;
        int greenRight = 0;
        int redRight = 0;
        cv::Vec3b currentPixelLR;
        for (int j = 5; j < 36; j++)
        {
          currentPixelLR = src.at<cv::Vec3b>(centerY, centerX - j);
          blueLeft += currentPixelLR[0];
          greenLeft += currentPixelLR[1];
          redLeft += currentPixelLR[2];

          currentPixelLR = src.at<cv::Vec3b>(centerY, centerY + j);
          blueRight += currentPixelLR[0];
          greenRight += currentPixelLR[1];
          redRight += currentPixelLR[2];
        }
        blueLeft /= 30;
        greenLeft /= 30;
        redLeft /= 30;
        blueRight /= 30;
        greenRight /= 30;
        redRight /= 30;

        // We now have the average color of the line to the left and the right of the contour.
        // Use these averages to determine if the contour is a line rather than a pothole.
        // Aka if there is green to the left and the right of the contour, filter it!
        if (greenLeft > redLeft + 50 && greenLeft > blueLeft + 50 && greenRight > redRight + 50 &&
            greenRight > blueRight + 50)
        {
          contours.erase(it);
          --it;
        }
      }
      else
      {
        contours.erase(it);
        --it;
      }
    }

    // Push all of the remaining good points to the main vector of contours
    for (std::vector<cv::Point> goodContour : contours)
    {
      allContours.push_back(goodContour);
    }
  }

  // Re-fill sky area of image with black
  cv::Mat black = cv::Mat::zeros(cv::Size(src_gray.cols, topCrop), src_gray.type());
  cv::vconcat(black, src_gray, src_gray);

  // Shift contour coordinates down to account for black sky space
  for (std::vector<cv::Point>& cont : allContours)
  {
    for (cv::Point& p : cont)
    {
      p.y += topCrop;
    }
  }

  // Convert the contours to a pointcloud
  cloud = toPointCloud(tf_listener, allContours, cam, topic);

  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;
  out_msg.encoding = msg->encoding;
  out_msg.image = src_gray;

  cv::cvtColor(src_gray, src_gray, CV_GRAY2BGR);
  cv_ptr->image = src_gray;
  _pothole_filt_img.publish(cv_ptr->toImageMsg());
  _pothole_thres.publish(out_msg.toImageMsg());
  _pothole_cloud.publish(cloud);
}

PotholeDetector::PotholeDetector(ros::NodeHandle& handle, const std::string& topic)
  : gaussian_size(7), _it(handle), tf_listener(handle), topic(topic)
{
  _src_img = _it.subscribeCamera(topic + "/image_raw", 1, &PotholeDetector::info_img_callback, this);
  _pothole_filt_img = _it.advertise(topic + "/pothole_filt_img", 1);
  _pothole_thres = _it.advertise(topic + "/pothole_thres", 1);
  _pothole_cloud = handle.advertise<PCLCloud>(topic + "/pothole_cloud", 100);

  // Import tuning parameters from yaml file (file specified in launch file)
  handle.getParam(ros::this_node::getName() + "/config/pothole/minRadius", minRadius);
  handle.getParam(ros::this_node::getName() + "/config/pothole/maxRadius", maxRadius);
  handle.getParam(ros::this_node::getName() + "/config/pothole/whiteSampleRadius", whiteSampleRadius);
  handle.getParam(ros::this_node::getName() + "/config/pothole/contourSizeThreshold", contourSizeThreshold);
  handle.getParam(ros::this_node::getName() + "/config/pothole/blueAdaptiveThreshold", blueAdaptiveThreshold);
  handle.getParam(ros::this_node::getName() + "/config/pothole/greenAdaptiveThreshold", greenAdaptiveThreshold);
  handle.getParam(ros::this_node::getName() + "/config/pothole/redAdaptiveThreshold", redAdaptiveThreshold);
}
