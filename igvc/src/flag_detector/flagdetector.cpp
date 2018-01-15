#include "flagdetector.h"
#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <pcl_ros/point_cloud.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/image_encodings.h>
#include <cstdint>
#include <opencv2/video/video.hpp>

using namespace std;
using namespace cv;
using namespace pcl;

cv_bridge::CvImagePtr cv_ptr;
typedef pcl::PointCloud<pcl::PointXYZ> PCLCloud;

// Offsets for red and blue flag detections
const int redDiff = 35;
const int blueDiff = 70;
// Bounds for contours
const unsigned int minLength = 30;
const unsigned int maxLength = 120;

double toRadians(double degrees)
{
  return degrees / 180.0 * M_PI;
}

void FlagDetector::img_callback(const sensor_msgs::ImageConstPtr &msg)
{
  cv_ptr = cv_bridge::toCvCopy(msg, "");
  Mat orig = cv_ptr->image.clone();
  src = cv_ptr->image.clone();

  // Crops the image (removes sky)
  int topCrop = src.rows / 2 - 100;
  Rect roiNoSky(0, topCrop, src.cols, src.rows - topCrop);
  src = src(roiNoSky);

  Mat src_binary = Mat::zeros(src.rows, src.cols, CV_8UC1);
  // Determine which points corresponds to red / blue flags.
  for (int i = 0; i < src.cols; i++)
  {
    for (int j = 0; j < src.rows; j++)
    {
      Vec3b currentPixel = src.at<Vec3b>(j, i);
      if ((currentPixel[0] - blueDiff > currentPixel[1] && currentPixel[0] - blueDiff > currentPixel[2]) ||
          (currentPixel[2] - redDiff > currentPixel[0] && currentPixel[2] - redDiff > currentPixel[1]))
      {
        // cerr << to_string(currentPixel[2]) + " " + to_string(currentPixel[1]) + " " + to_string(currentPixel[0]) <<
        // endl;
        src_binary.at<uchar>(j, i) = 1;
        /*currentPixel[0] = 0;
        currentPixel[1] = 0;
        currentPixel[2] = 0;
        src.at<Vec3b>(j, i) = currentPixel;*/
      }
    }
  }
  // Identify contours in binary image
  vector<vector<Point>> contours;
  findContours(src_binary, contours, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  for (vector<vector<Point>>::iterator it = contours.begin(); it != contours.end(); ++it)
  {
    int minY = src.rows;
    int minX = src.cols;
    int maxX = 0;
    int maxY = 0;
    // Finds center pixel of contour
    for (Point p : *it)
    {
      int x = p.x;
      int y = p.y;
      if (x > maxX)
      {
        maxX = x;
      }
      if (x < minX)
      {
        minX = x;
      }
      if (y > maxY)
      {
        maxY = y;
      }
      if (y < minY)
      {
        minY = y;
      }
    }
    int centerX = (minX + maxX) / 2;
    int centerY = (minY + maxY) / 2;
    // cerr << to_string(centerX) + " " + to_string(centerY) << endl;
    Vec3b tempPix = src.at<Vec3b>(centerY, centerX);
    if ((*it).size() < minLength + (double)(*it)[0].y / 460 * 270 ||
        ((*it).size() > maxLength + (double)(*it)[0].y / 460 * 250 &&
         !((tempPix[0] - blueDiff > tempPix[1] && tempPix[0] - blueDiff > tempPix[2]))) ||
        src_binary.at<uchar>(centerY, centerX) == 0 || maxY - minY < 30)
    {
      contours.erase(it);
      --it;
    }
    else
    {
      cerr << "Contour size: " + to_string((*it).size()) << endl;
      cerr << "Max length @ " + to_string((double)(*it)[0].y) + " is " +
                  to_string(maxLength + (double)(*it)[0].y / 460 * 250);
    }
  }
  drawContours(src, contours, -1, Scalar(20, 236, 27), 2, 8);

  // Convert the contours to a pointcloud
  cloud = toPointCloud(contours, orig.rows, orig.cols);

  cv_bridge::CvImage out_msg;
  out_msg.header = msg->header;
  out_msg.encoding = msg->encoding;
  out_msg.image = src;

  cv_ptr->image = src;
  _flag_filt_img.publish(cv_ptr->toImageMsg());
  _flag_thres.publish(out_msg.toImageMsg());
  _flag_cloud.publish(cloud);
}

FlagDetector::FlagDetector(ros::NodeHandle &handle) : gaussian_size(7), _it(handle), tf_listener(handle)
{
  _src_img = _it.subscribe("/stereo/right/image_raw", 1, &FlagDetector::img_callback, this);
  _flag_filt_img = _it.advertise("/flag_filt_img", 1);
  _flag_thres = _it.advertise("/flag_thres", 1);
  _flag_cloud = handle.advertise<PCLCloud>("/flag_cloud", 100);
}

PointCloud<PointXYZ>::Ptr FlagDetector::toPointCloud(vector<vector<Point>> &contours, int height, int width)
{
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

  for (vector<Point> contour : contours)
  {
    for (Point p : contour)
    {
      int xP = p.x;
      int yP = p.y + (height / 2 - 100);

      auto pitch_offset = ((float)(yP - height / 2) / height) * VFOV;
      auto y = origin_z / tan(pitch + pitch_offset) + origin_y;

      auto theta = ((float)(xP - width / 2) / width) * HFOV;
      auto x = y * tan(theta);

      cloud->points.push_back(PointXYZ(x, y, 0));
    }
  }

  cloud->header.frame_id = "base_footprint";
  return cloud;
}
