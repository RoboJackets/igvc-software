// Subscribes to Point Cloud Data, updates the occupancy grid, then publishes the data.

#include <cv_bridge/cv_bridge.h>
#include <igvc_msgs/map.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
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
#include "tf/transform_datatypes.h"
#include <limits.h>
#include <pcl_conversions/pcl_conversions.h>

cv_bridge::CvImage img_bridge;

ros::Publisher map_pub;
ros::Publisher debug_pub;
ros::Publisher debug_pcl_pub;
ros::Subscriber odom_sub;
std::unique_ptr<cv::Mat> published_map;  // matrix will be publishing
std::map<std::string, tf::StampedTransform> transforms;
std::unique_ptr<tf::TransformListener> tf_listener;

double resolution;
double orientation;
double _roll;
double _pitch;
int start_x;  // start x location
int start_y;  // start y location
int length_y;
int width_x;
uchar occupancy_grid_threshold;
int increment_step;
bool debug;
bool update = true;

struct Position
{
    double x;
    double y;
};

Position robot_pos;

std::tuple<double, double> rotate(double x, double y)
{
  double newX = x * cos(orientation) - y * sin(orientation);
  double newY = x * sin(orientation) + y * cos(orientation);
  return (std::make_tuple(newX, newY));
}

void odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
  if (update)
  {
    robot_pos.x = msg->pose.pose.position.x;
    robot_pos.y = msg->pose.pose.position.y;
    tf::Quaternion quat;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, quat);
    double roll, pitch, yaw;
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    orientation = yaw;
    _roll = roll;
    _pitch = pitch;
    update = false;
  }
}

void setMsgValues(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp)
{
  pcl_conversions::fromPCL(pcl_stamp, image.header.stamp);
  pcl_conversions::fromPCL(pcl_stamp, message.header.stamp);
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = length_y;
  message.width = width_x;
  message.resolution = resolution;
  message.orientation = orientation;
  message.x = std::round(robot_pos.x / resolution) + start_x;
  message.y = std::round(robot_pos.y / resolution) + start_y;
  message.x_initial = start_x;
  message.y_initial = start_y;
}

void updateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed) {
  int offMapCount = 0;

  pcl::PointCloud<pcl::PointXYZ>::const_iterator point_iter;

  for (point_iter = transformed->begin(); point_iter < transformed->points.end(); point_iter++)
  {
    double x_point_raw, y_point_raw;
    std::tie(x_point_raw, y_point_raw) = rotate(point_iter->x, point_iter->y);

    int point_x = static_cast<int>(std::round(x_point_raw / resolution + robot_pos.x / resolution + start_x));
    int point_y = static_cast<int>(std::round(y_point_raw / resolution + robot_pos.y / resolution + start_y));
    if (point_x >= 0 && point_y >= 0 && point_x < length_y && start_y < width_x)
    {
      // TODO: Investigate decay, potentially use CV_16U, decrement at a certain rate.
      // TODO: Potentially change
      if (published_map->at<uchar>(point_x, point_y) <= UCHAR_MAX - (uchar)increment_step)
      {
        published_map->at<uchar>(point_x, point_y) += (uchar)increment_step;
      } else {
        published_map->at<uchar>(point_x, point_y) = UCHAR_MAX;
      }
    }
    else
    {
        offMapCount++;
    }
  }
  if (offMapCount > 0)
  {
    ROS_WARN_STREAM(offMapCount << " points were off the map");
  }
}

void checkExistsStaticTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic) {
  if (transforms.find(topic) == transforms.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    if (tf_listener->waitForTransform("/base_footprint", msg->header.frame_id, ros::Time(0), ros::Duration(3.0)))
    {
      ROS_INFO_STREAM("\n\ngetting transform for " << topic << "\n\n");
      tf::StampedTransform transform;
      tf_listener->lookupTransform("/base_footprint", msg->header.frame_id, ros::Time(0), transform);
      transforms.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
    }
    else
    {
      ROS_ERROR_STREAM("\n\nfailed to find transform using empty transform\n\n");
    }
  }
}



void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  // transform pointcloud into the occupancy grid, no filtering right now

  // make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // Check if static transform already exists for this topic.
  checkExistsStaticTransform(msg, topic);

  // Apply transformation to msg points using the transform for this topic.
  pcl_ros::transformPointCloud(*msg, *transformed, transforms.at(topic));
  updateOccupancyGrid(transformed);

  igvc_msgs::map message;  // >> message to be sent
  sensor_msgs::Image image;  // >> image in the message
  img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, *published_map);
  img_bridge.toImageMsg(image);  // from cv_bridge to sensor_msgs::Image

  setMsgValues(message, image, msg->header.stamp);
  map_pub.publish(message);
  if (debug)
  {
    ROS_INFO_STREAM("robot location " << std::round(robot_pos.x / resolution) + start_x << ", "
                                      << std::round(robot_pos.y / resolution) + start_y);
    debug_pub.publish(image);
    // ROS_INFO_STREAM("\nThe robot is located at " << x_robot << "," << y_robot << "," << orientation);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromOcuGrid =
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>());
    for (int i = 0; i < width_x; i++)
    {
      for (int j = 0; j < length_y; j++)
      {
        if (published_map->at<uchar>(i, j) >= occupancy_grid_threshold)
        {
          // Set x y coordinates as the center of the grid cell.
          pcl::PointXYZRGB p(255, published_map->at<uchar>(i, j), published_map->at<uchar>(i, j));
          p.x = (i - start_x) * resolution;
          p.y = (j - start_y) * resolution;
          fromOcuGrid->points.push_back(p);
        }
      }
    }
    fromOcuGrid->header.frame_id = "/odom";
    fromOcuGrid->header.stamp = msg->header.stamp;
    debug_pcl_pub.publish(fromOcuGrid);
  }
  update = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  std::string topics;

  std::list<ros::Subscriber> subs;
  tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  double cont_start_x;
  double cont_start_y;
  int cont_occupancy_grid_threshold;

  if (!(pNh.hasParam("topics") && pNh.hasParam("occupancy_grid_width") && pNh.hasParam("occupancy_grid_length") &&
      pNh.hasParam("occupancy_grid_resolution") && pNh.hasParam("start_X") && pNh.hasParam("start_Y") &&
      pNh.hasParam("increment_step") && pNh.hasParam("occupancy_grid_threshold") && pNh.hasParam("debug")))
  {
    ROS_ERROR_STREAM("missing parameters; exiting");
    return 0;
  }

  // assumes all params inputted in meters
  pNh.getParam("topics", topics);
  pNh.getParam("occupancy_grid_length", length_y);
  pNh.getParam("occupancy_grid_width", width_x);
  pNh.getParam("occupancy_grid_resolution", resolution);
  pNh.getParam("occupancy_grid_threshold", cont_occupancy_grid_threshold);
  pNh.getParam("start_X", cont_start_x);
  pNh.getParam("start_Y", cont_start_y);
  pNh.getParam("increment_step", increment_step);
  pNh.getParam("orientation", orientation);
  pNh.getParam("debug", debug);

  // convert from meters to grid
  length_y = static_cast<int>(std::round(length_y / resolution));
  width_x = static_cast<int>(std::round(width_x / resolution));
  start_x = static_cast<int>(std::round(cont_start_x / resolution));
  start_y = static_cast<int>(std::round(cont_start_y / resolution));
  occupancy_grid_threshold = static_cast<uchar>(cont_occupancy_grid_threshold);
  ROS_INFO_STREAM("cv::Mat length: " << length_y << "  width: " << width_x << "  resolution: " << resolution);

  // set up tokens and get list of subscribers
  std::istringstream iss(topics);
  std::vector<std::string> tokens{ std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>() };
  odom_sub = nh.subscribe("/odometry/filtered", 1, odom_callback);

  for (const std::string &topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  published_map = std::unique_ptr<cv::Mat>(new cv::Mat(length_y, width_x, CV_8UC1));

  map_pub = nh.advertise<igvc_msgs::map>("/map", 1);

  if (debug)
  {
    debug_pub = nh.advertise<sensor_msgs::Image>("/map_debug", 1);
    debug_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl", 1);
  }

  ros::spin();
}
