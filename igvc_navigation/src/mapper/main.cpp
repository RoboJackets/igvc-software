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
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>
#include "tf/transform_datatypes.h"

cv_bridge::CvImage img_bridge;

ros::Publisher map_pub;                  // Publishes map
ros::Publisher debug_pub;                // Debug version of above
ros::Publisher debug_pcl_pub;            // Publishes map as individual PCL points
std::unique_ptr<cv::Mat> published_map;  // matrix will be publishing
std::map<std::string, tf::StampedTransform> transforms;
std::unique_ptr<tf::TransformListener> tf_listener;

double resolution;
double transform_max_wait_time;
int start_x;  // start x location
int start_y;  // start y location
int length_y;
int width_x;
uchar occupancy_grid_threshold;
int increment_step;
bool debug;
RobotState state;

std::tuple<double, double> rotate(double x, double y)
{
  double newX = x * cos(state.yaw) - y * sin(state.yaw);
  double newY = x * sin(state.yaw) + y * cos(state.yaw);
  return (std::make_tuple(newX, newY));
}

/**
 * Updates <code>RobotState state</code> with the latest tf transform using the timestamp of the message passed in
 * @param msg <code>pcl::PointCloud</code> message with the timestamp used for looking up the tf transform
 */
void getOdomTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  tf::StampedTransform transform;
  ros::Time messageTimeStamp;
  pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
  if (tf_listener->waitForTransform("/odom", "/base_link", messageTimeStamp, ros::Duration(transform_max_wait_time)))
  {
    tf_listener->lookupTransform("/odom", "/base_link", messageTimeStamp, transform);
    state.setState(transform);
  }
}

// Populates igvc_msgs::map message with information from sensor_msgs::Image and the timestamp from pcl_stamp
/**
 * Populates <code>igvc_msgs::map message</code> with information from <code>sensor_msgs::Image</code> and the
 * timestamp from <code>pcl_stamp</code>
 * @param message message to be filled out
 * @param image image containing map data to be put into <code>message</code>
 * @param pcl_stamp time stamp from the pcl to be used for <code>message</code>
 */
void setMsgValues(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp)
{
  pcl_conversions::fromPCL(pcl_stamp, image.header.stamp);
  pcl_conversions::fromPCL(pcl_stamp, message.header.stamp);
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = length_y;
  message.width = width_x;
  message.resolution = resolution;
  message.orientation = state.yaw;
  message.x = std::round(state.x / resolution) + start_x;
  message.y = std::round(state.y / resolution) + start_y;
  message.x_initial = start_x;
  message.y_initial = start_y;
}

/**
 * Updates the occupancy grid using information from the <code>pcl::PointCloud transformed</code>
 * @param transformed Reference to pointcloud containing information from lidar / segmentation.
 */
void updateOccupancyGrid(const pcl::PointCloud<pcl::PointXYZ>::Ptr &transformed)
{
  int offMapCount = 0;

  pcl::PointCloud<pcl::PointXYZ>::const_iterator point_iter;

  for (point_iter = transformed->begin(); point_iter < transformed->points.end(); point_iter++)
  {
    double x_point_raw, y_point_raw;
    std::tie(x_point_raw, y_point_raw) = rotate(point_iter->x, point_iter->y);

    int point_x = static_cast<int>(std::round(x_point_raw / resolution + state.x / resolution + start_x));
    int point_y = static_cast<int>(std::round(y_point_raw / resolution + state.y / resolution + start_y));
    if (point_x >= 0 && point_y >= 0 && point_x < length_y && start_y < width_x)
    {
      // Check for overflow
      if (published_map->at<uchar>(point_x, point_y) <= UCHAR_MAX - (uchar)increment_step)
      {
        published_map->at<uchar>(point_x, point_y) += (uchar)increment_step;
      }
      else
      {
        published_map->at<uchar>(point_x, point_y) = UCHAR_MAX;
      }
    }
    else
    {
      offMapCount++;
    }
  }
  if (debug && (offMapCount > 0))
  {
    ROS_WARN_STREAM(offMapCount << " points were off the map");
  }
}

/**
 * Checks if transform from base_footprint to msg.header.frame_id exists
 * @param msg
 * @param topic
 */
void checkExistsStaticTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  if (transforms.find(topic) == transforms.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ros::Time messageTimeStamp;
    pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
    if (tf_listener->waitForTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, ros::Duration(3.0)))
    {
      ROS_INFO_STREAM("\n\ngetting transform for " << topic << "\n\n");
      tf::StampedTransform transform;
      tf_listener->lookupTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, transform);
      transforms.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
    }
    else
    {
      ROS_ERROR_STREAM("\n\nfailed to find transform using empty transform\n\n");
    }
  }
}

/**
 * Decays map by 1 universally on callback
 */
void decayMap(const ros::TimerEvent &)
{
  int nRows = published_map->rows;
  int nCols = published_map->cols;

  if (published_map->isContinuous())
  {
    nCols *= nRows;
    nRows = 1;
  }
  int i, j;
  uchar *p;
  for (i = 0; i < nRows; i++)
  {
    p = published_map->ptr<uchar>(i);
    for (j = 0; j < nCols; j++)
    {
      if (p[j] > 0)
      {
        p[j] -= (uchar)1;
      }
    }
  }
}

/**
 * Callback for updates on lidar / segmentation PCL topics. Updates the occupancy grid, then publishes it.
 * @param msg pointcloud information
 * @param topic topic which the pointcloud came from
 */
void frame_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  // transform pointcloud into the occupancy grid, no filtering right now

  // make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // Check if static transform already exists for this topic.
  checkExistsStaticTransform(msg, topic);

  // Lookup transform form Ros Localization for position
  getOdomTransform(msg);

  // Apply transformation to msg points using the transform for this topic.
  pcl_ros::transformPointCloud(*msg, *transformed, transforms.at(topic));
  updateOccupancyGrid(transformed);

  igvc_msgs::map message;    // >> message to be sent
  sensor_msgs::Image image;  // >> image in the message
  img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, *published_map);
  img_bridge.toImageMsg(image);  // from cv_bridge to sensor_msgs::Image

  setMsgValues(message, image, msg->header.stamp);
  map_pub.publish(message);
  if (debug)
  {
    debug_pub.publish(image);
    // ROS_INFO_STREAM("\nThe robot is located at " << state);
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
  double decay_period;
  int cont_occupancy_grid_threshold;

  // assumes all params inputted in meters
  igvc::getParam(pNh, "topics", topics);
  igvc::getParam(pNh, "occupancy_grid_length", length_y);
  igvc::getParam(pNh, "occupancy_grid_width", width_x);
  igvc::getParam(pNh, "occupancy_grid_resolution", resolution);
  igvc::getParam(pNh, "start_X", cont_start_x);
  igvc::getParam(pNh, "start_Y", cont_start_y);
  igvc::getParam(pNh, "debug", debug);
  igvc::getParam(pNh, "occupancy_grid_threshold", cont_occupancy_grid_threshold);
  igvc::getParam(pNh, "decay_period", decay_period);
  igvc::getParam(pNh, "transform_max_wait_time", transform_max_wait_time);
  igvc::getParam(pNh, "increment_step", increment_step);

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

  for (const std::string &topic : tokens)
  {
    ROS_INFO_STREAM("Mapper subscribing to " << topic);
    subs.push_back(nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(topic, 1, boost::bind(frame_callback, _1, topic)));
  }

  // Timer for map decay
  if (decay_period > 0)
  {
    ros::Timer timer = nh.createTimer(ros::Duration(decay_period), decayMap);
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
