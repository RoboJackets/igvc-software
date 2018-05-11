#include <cv_bridge/cv_bridge.h>
#include <igvc_msgs/map.h>
#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/publisher.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <stdlib.h>
#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Odometry.h>

igvc_msgs::map msgBoi;  // >> message to be sent
cv_bridge::CvImage img_bridge;
sensor_msgs::Image imageBoi;  // >> image in the message

ros::Publisher map_pub;
ros::Publisher debug_pub;
ros::Subscriber odom_sub;
ros::Publisher debug_pcl_pub;
cv::Mat *published_map;  // matrix will be publishing
tf::StampedTransform lidar_trans;
tf::StampedTransform cam_trans;
tf::TransformListener *tf_listener;
std::string topics;
Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>> *eigenRep;

double resolution;
double orientation;
int x;  // start x location
int y;  // start y location
int length_y;
int width_x;
bool debug;
double cur_x;
double cur_y;

void odom_callback(nav_msgs::Odometry::ConstPtr &msg) {
    cur_x = msg->pose.pose.position.x;
    cur_y = msg->pose.pose.position.y;
}

void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  // transform pointcloud into the occupancy grid, no filtering right now

  bool offMap = false;
  int count = 0;

  // make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  ros::Time time = ros::Time::now();
  if (msg->header.frame_id == "/scan/pointcloud")
  {
      tf_listener->lookupTransform("/base_footprint", "/scan/pointcloud", time, lidar_trans);  //update transform to change location in map
      pcl_ros::transformPointCloud(*msg, *transformed, lidar_trans);
  }
  else
  {
      tf_listener->lookupTransform("/base_footprint", "/usb_cam_center/line_cloud", time, cam_trans);
      pcl_ros::transformPointCloud(*msg, *transformed, cam_trans);
  }

  pcl::PointCloud<pcl::PointXYZ>::const_iterator point;
  for (point = transformed->begin(); point < transformed->points.end(); point++)
  {
    int point_x = (int)std::round((point->x / resolution) + x);
    point_x = (int)std::round(point_x + cur_x);
    int point_y = (int)std::round((point->y / resolution) + y);
    point_y = (int)std::round(point_y + cur_y);
    if (point_x >= 0 && point_y >= 0 && point_x < length_y && y < width_x)
    {
      published_map->at<char>(point_x, point_y) = (unsigned char)255;
      count++;
    }
    else if (!offMap)
    {
      ROS_WARN_STREAM("Some points out of range, won't be put on map.");
      offMap = true;
    }
  }
  // make image message from img bridge

  img_bridge = cv_bridge::CvImage(msgBoi.header, sensor_msgs::image_encodings::MONO8, *published_map);
  img_bridge.toImageMsg(imageBoi);    // from cv_bridge to sensor_msgs::Image
  time = ros::Time::now();  // so times are exact same
  imageBoi.header.stamp = time;
  // set fields of map message
  msgBoi.header.stamp = time;
  msgBoi.image = imageBoi;
  msgBoi.length = length_y;
  msgBoi.width = width_x;
  msgBoi.resolution = resolution;
  msgBoi.orientation = orientation;
  map_pub.publish(msgBoi);
  if (debug)
  {
    debug_pub.publish(imageBoi);
    ROS_INFO_STREAM("\n\nTRANSORMED " << count << " POINTS\n\n");
    pcl::PointCloud<pcl::PointXYZ>::Ptr fromOcuGrid=
              pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());
      for(int i = 0; i < width_x; i++){ // init frame so edges are easily visible
          for(int j = 0; j < length_y; j++){
              if(published_map->at<char>(i, j) == (unsigned char)255) {
                  pcl::PointXYZ p(i * resolution, j*resolution, 0);
                  fromOcuGrid->points.push_back(p);
              }
          }
      }
      debug_pcl_pub.publish(fromOcuGrid);
  }
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

  // assumes all params inputted in meters
  pNh.getParam("topics", topics);
  pNh.getParam("occupancy_grid_length", length_y);
  pNh.getParam("occupancy_grid_width", width_x);
  pNh.getParam("occupancy_grid_resolution", resolution);
  pNh.getParam("start_X", start_x);
  pNh.getParam("start_Y", start_y);
  pNh.getParam("orientation", orientation);
  pNh.getParam("debug", debug);

  // convert from meters to grid
  length_y = (int)std::round(length_y / resolution);
  width_x = (int)std::round(width_x / resolution);
  x = (int)std::round(start_x / resolution);
  y = (int)std::round(start_y / resolution);
  ROS_INFO_STREAM("cv::Mat length: " << length_y << "  width: " << width_x << "  resolution: " << resolution);

  // set up tokens and get list of subscribers
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };
  odom_sub = nh.subscribe("/odometry/filtered", 1, odom_callback);

  for (std::string topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  // init transforms
  ros::Time time = ros::Time::now();
  if (tf_listener->waitForTransform("/base_footprint", "/scan/pointcloud", time, ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/base_footprint", "/scan/pointcloud", time, lidar_trans);
  }
  if (tf_listener->waitForTransform("/base_footprint", "/usb_cam_center/line_cloud", time, ros::Duration(5.0)))
  {
    tf_listener->lookupTransform("/base_footprint", "/usb_cam_center/line_cloud", time, cam_trans);
  }

  published_map = new cv::Mat(length_y, width_x, CV_8UC1);
  eigenRep = new Eigen::Map<Eigen::Matrix<unsigned char, Eigen::Dynamic, Eigen::Dynamic>>(
      (unsigned char *)published_map, width_x, length_y);

  map_pub = nh.advertise<igvc_msgs::map>("/new_map", 1);
  if (debug)
  {

    debug_pub = nh.advertise<sensor_msgs::Image>("/map_debug", 1);
    debug_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/map_debug_pcl", 1);
    for(int i = 0; i < width_x; i++){ // init frame so edges are easily visible
        for(int j = 0; j < length_y; j++){
            if(i == 0 || i == width_x - 1 || j == 0 || j == length_y - 1) {
                published_map->at<char>(i, j) = (unsigned char)255;
            }
        }
    }
  }

  ros::spin();
}
