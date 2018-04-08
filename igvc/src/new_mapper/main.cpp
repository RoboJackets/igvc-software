#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <pcl_ros/transforms.h>
#include <igvc_msgs>

ros::Publisher pointcloud_pub;
cv::Mat published_map; // get to work
Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic> global_map;
tf::StampedTransform lidar_trans;
tf::StampedTransform cam_trans;
tf::TransformListener *tf_listener;
std::string topics;

double resolution;
double position [2];
int length;
int width;

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  //transform pointcloud into the occupancy grid, no filtering right now
  bool offMap = false;
  //cv::Mat frame(msg->points);

  //make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  if (msg->header.frame_id == "/scan/pointcloud") {
    pcl_ros::transformPointCloud(*msg, *transformed, lidar_trans);

  } else {
    pcl_ros::transformPointCloud(*msg, *transformed, cam_trans);
  }



  pcl::PointCloud<pcl::PointXYZ>::const_iterator point;
  for (point = transformed->begin(); point < transformed->points.end(); point++)
  {
    int x = (int) std::round((point->x / resolution) + position[0]);
    int y = (int) std::round((point->y / resolution) + position[1]);

    if(x > 0 && y > 0 && x < length && y < width)
    {
      global_map(x, y) = 1.0;
    } else if(!offMap){
      ROS_WARN_STREAM("Some points out of range, won't be put on map.");
      offMap = true;
    }
  }

  //won't work until we make a message to publish
  //pointcloud_pub.publish(published_map);
  //pointcloud_pub.publish(&global_map);
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "new_mapper");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::list<ros::Subscriber> subs;

  tf_listener = new tf::TransformListener();

  double start_x;
  double start_y;

  pNh.getParam("topics", topics);
  pNh.getParam("occupancy_grid_length", length);
  pNh.getParam("occupancy_grid_width", width);
  pNh.getParam("qoccupancy_grid_resolution", resolution);
  pNh.getParam("start_X", start_x);
  pNh.getParam("start_Y", start_y);

  length = (int) std::round(length / resolution);
  width = (int) std::round(width / resolution);
  position[0] = (int) std::round(start_x / resolution);
  position[1] = (int) std::round(start_y / resolution);

  //set up tokens and get list of subscribers
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };

  for (std::string topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  //init transforms
  ros::Time time = ros::Time::now();
  if (tf_listener->waitForTransform("/odom", "/scan/pointcloud", time, ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/odom", "/scan/pointcloud", time, lidar_trans);

  }
  if (tf_listener->waitForTransform("/odom", "/usb_cam_center/line_cloud", time, ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/odom", "/usb_cam_center/line_cloud", time, cam_trans);

  }


  //TODO: Initialize the map variables, not working
  global_map = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic>(length, width);
  //published_map(length, width, CV_32FC1, const cv::Scalar(0,0,0)); // I cant instatiate this
  //https://docs.opencv.org/2.4/doc/tutorials/core/mat_the_basic_image_container/mat_the_basic_image_container.html
  //global_map(published_map.data()); // I can't instantiate this either
  //https://stackoverflow.com/questions/14783329/opencv-cvmat-and-eigenmatrix

  pointcloud_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map", 1);

  ros::spin();
}
