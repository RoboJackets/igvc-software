#include <parameter_assertions/assertions.h>
#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/mat.hpp>
#include <igvc_msgs/BackCircle.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>

tf::TransformListener listener;

bool backCircleCallback(igvc_msgs::BackCircle::Request &req, igvc_msgs::BackCircle::Response &res)
{
  ros::NodeHandle pNh("~");

  double width, length, grid_size, thickness, offset;
  std::vector<double> xVec, yVec, zVec;

  assertions::getParam(pNh, std::string("width"), width);
  assertions::getParam(pNh, std::string("length"), length);
  assertions::getParam(pNh, std::string("grid_size"), grid_size);
  assertions::getParam(pNh, std::string("thickness"), thickness);
  assertions::getParam(pNh, std::string("offset"), offset);
  double meters = (length + width) * 2;

  using namespace cv;

  int img_size = static_cast<int>(std::round(meters / grid_size));
  Mat img(img_size, img_size, CV_8U, Scalar::all(0));
  ellipse(img, Point(img_size / 2, img_size / 2), Size(width / grid_size, length / grid_size), 0, 90, 270, Scalar(255),
          static_cast<int>(std::round(thickness / grid_size)));

  pcl::PointCloud<pcl::PointXYZ> back_circle_pointcloud_base_footprint;
  back_circle_pointcloud_base_footprint.header.stamp = ros::Time::now().toSec();
  back_circle_pointcloud_base_footprint.header.frame_id = "/base_footprint";

  for (int i = 0; i < img.rows; i++)
  {
    for (int j = 0; j < img.cols; j++)
    {
      if (img.at<uchar>(i, j) == 255)
      {
        pcl::PointXYZ bc_point((j - img_size / 2) * grid_size + offset, (i - img_size / 2) * grid_size, 0);
        back_circle_pointcloud_base_footprint.push_back(bc_point);
      }
    }
  }

  pcl::PointCloud<pcl::PointXYZ> back_circle_pointcloud_odom;
  pcl_ros::transformPointCloud("odom", back_circle_pointcloud_base_footprint, back_circle_pointcloud_odom, listener);

  for (size_t i = 0; i < back_circle_pointcloud_odom.points.size(); ++i)
  {
    xVec.push_back(back_circle_pointcloud_odom.points[i].x);
    yVec.push_back(back_circle_pointcloud_odom.points[i].y);
    zVec.push_back(back_circle_pointcloud_odom.points[i].z);
  }

  res.x = xVec;
  res.y = yVec;
  res.z = zVec;

  ROS_INFO("Confirmation");

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "back_circle");

  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("/back_circle_service", backCircleCallback);
  ROS_INFO("Back Circle Service Ready");

  // ros::Publisher pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/back_circle_layer", 1);
  // pub.publish(back_circle_pointcloud);

  ros::spin();
  return 0;
}