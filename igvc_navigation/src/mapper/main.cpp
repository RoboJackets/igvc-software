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
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <signal.h>
#include <stdlib.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Core>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <unordered_set>
#include "octomapper.h"

using radians = double;
class Mapper
{
public:
  Mapper();

private:
  // Callbacks
  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc);
  void line_map_callback(const sensor_msgs::ImageConstPtr &segmented);
  void projected_line_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc);

  void publish(const cv::Mat &map, uint64_t stamp);
  void setMessageMetadata(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp);
  bool checkExistsStaticTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic);
  bool getOdomTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg);
  int discretize(radians angle) const;
  void get_empty_points(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &empty_pc);
  void filter_points_behind(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &filtered_pc);
  void blur(cv::Mat &blurred_map);

  cv_bridge::CvImage m_img_bridge;

  ros::Publisher m_map_pub;                                  // Publishes map
  ros::Publisher m_blurred_pub;                              // Publishes blurred map
  ros::Publisher m_debug_pub;                                // Debug version of above
  ros::Publisher m_debug_pcl_pub;                            // Publishes map as individual PCL points
  ros::Publisher m_debug_blurred_pc;                         // Publishes blurred map as individual PCL points
  ros::Publisher m_ground_pub;                               // Publishes ground points
  ros::Publisher m_nonground_pub;                            // Publishes non ground points
  ros::Publisher m_sensor_pub;                               // Publishes lidar position
  std::unique_ptr<cv::Mat> m_published_map;                  // Matrix will be publishing
  std::map<std::string, tf::StampedTransform> m_transforms;  // Map of static transforms TODO: Refactor this
  std::unique_ptr<tf::TransformListener> m_tf_listener;      // TF Listener

  bool m_use_lines;
  double m_resolution;
  double m_transform_max_wait_time;
  int m_start_x;   // start x (m)
  int m_start_y;   // start y (m)
  int m_length_x;  // length (m)
  int m_width_y;   // width (m)
  int m_kernel_size;
  int m_segmented_kernel;
  int m_segmented_sigma;
  int m_segmented_threshold;

  bool m_debug;
  double m_radius;  // Radius to filter lidar points // TODO: Refactor to a new node
  double m_lidar_miss_cast_distance;
  double m_filter_distance;
  double m_blur_std_dev;
  radians m_filter_angle;
  radians m_lidar_start_angle;
  radians m_lidar_end_angle;
  radians m_angular_resolution;
  std::string m_lidar_topic;
  std::string m_line_topic;
  std::string m_projected_line_topic;
  RobotState m_state;                      // Odom -> Base_link
  RobotState m_odom_to_lidar;              // Odom -> Lidar
  RobotState m_odom_to_camera_projection;  // Odom -> Camera Projection

  sensor_msgs::CameraInfo camera_info;

  std::unique_ptr<Octomapper> m_octomapper;
  pc_map_pair m_pc_map_pair;      // Struct storing both the octomap for the lidar and the cv::Mat map
  pc_map_pair m_camera_map_pair;  // Struct storing both the octomap for the camera projections and the cv::Mat map
};

Mapper::Mapper() : m_tf_listener{ std::unique_ptr<tf::TransformListener>(new tf::TransformListener()) }
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  igvc::getParam(pNh, "octree/resolution", m_resolution);
  igvc::getParam(pNh, "map/length", m_length_x);
  igvc::getParam(pNh, "map/width", m_width_y);
  igvc::getParam(pNh, "map/start_x", m_start_x);
  igvc::getParam(pNh, "map/start_y", m_start_y);

  igvc::getParam(pNh, "sensor_model/max_range", m_radius);
  igvc::getParam(pNh, "sensor_model/angular_resolution", m_angular_resolution);
  igvc::getParam(pNh, "sensor_model/lidar_miss_cast_distance", m_lidar_miss_cast_distance);
  igvc::getParam(pNh, "sensor_model/lidar_angle_start", m_lidar_start_angle);
  igvc::getParam(pNh, "sensor_model/lidar_angle_end", m_lidar_end_angle);

  igvc::getParam(pNh, "filter/filter_angle", m_filter_angle);
  igvc::getParam(pNh, "filter/distance", m_filter_distance);

  igvc::getParam(pNh, "blur/kernel_size", m_kernel_size);
  igvc::getParam(pNh, "blur/std_dev", m_blur_std_dev);
  igvc::getParam(pNh, "blur/segmented_kernel_size", m_segmented_kernel);
  igvc::getParam(pNh, "blur/segmented_sigma", m_segmented_sigma);

  igvc::getParam(pNh, "topics/lidar", m_lidar_topic);
  igvc::getParam(pNh, "topics/line_segmentation", m_line_topic);
  igvc::getParam(pNh, "topics/projected_line_pc", m_projected_line_topic);

  igvc::getParam(pNh, "segmented/threshold", m_segmented_threshold);

  igvc::getParam(pNh, "node/debug", m_debug);
  igvc::getParam(pNh, "node/use_lines", m_use_lines);

  m_octomapper = std::unique_ptr<Octomapper>(new Octomapper(pNh));
  m_octomapper->create_octree(m_pc_map_pair);

  ros::Subscriber pcl_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(m_lidar_topic, 1, &Mapper::pc_callback, this);
  if (m_use_lines) {
    m_octomapper->create_octree(m_camera_map_pair);
    ros::Subscriber line_map_sub = nh.subscribe<sensor_msgs::Image>(m_line_topic, 1, &Mapper::line_map_callback, this);
    ros::Subscriber projected_line_map_sub =
        nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(m_projected_line_topic, 1, &Mapper::projected_line_callback, this);
  }

  m_published_map = std::unique_ptr<cv::Mat>(new cv::Mat(m_length_x, m_width_y, CV_8UC1));

  m_map_pub = nh.advertise<igvc_msgs::map>("/map", 1);
  m_blurred_pub = nh.advertise<igvc_msgs::map>("/map/blurred", 1);

  if (m_debug)
  {
    m_debug_pub = nh.advertise<sensor_msgs::Image>("/map_debug", 1);
    m_debug_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl", 1);
    m_debug_blurred_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl/blurred", 1);
    m_ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/ground_pcl", 1);
    m_nonground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/nonground_pcl", 1);
    m_sensor_pub = nh.advertise<visualization_msgs::Marker>("/sensor_pos", 1);
  }

  ros::spin();
}

/**
 * Updates <code>RobotState state</code> with the latest tf transform using the timestamp of the message passed in
 * @param[in] msg <code>pcl::PointCloud</code> message with the timestamp used for looking up the tf transform
 */
bool Mapper::getOdomTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  ros::Time messageTimeStamp;
  pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
  static ros::Duration wait_time = ros::Duration(m_transform_max_wait_time);
  try
  {
    if (m_tf_listener->waitForTransform("/odom", "/base_link", messageTimeStamp, wait_time))
    {
      m_tf_listener->lookupTransform("/odom", "/base_link", messageTimeStamp, transform);
      m_state.setState(transform);
      m_tf_listener->lookupTransform("/odom", "/lidar", messageTimeStamp, transform2);
      m_odom_to_lidar.setState(transform2);
      m_tf_listener->lookupTransform("/odom", m_projected_line_topic, messageTimeStamp, transform2);
      m_odom_to_camera_projection.setState(transform2);
      return true;
    }
    else
    {
      ROS_DEBUG("Failed to get transform from /base_link to /odom in time, using newest transforms");
      m_tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      m_state.setState(transform);
      m_tf_listener->lookupTransform("/odom", "/lidar", ros::Time(0), transform2);
      m_odom_to_lidar.setState(transform2);
      m_tf_listener->lookupTransform("/odom", m_projected_line_topic, ros::Time(0), transform2);
      m_odom_to_camera_projection.setState(transform2);
      return true;
    }
  }
  catch (const tf::TransformException &ex)
  {
    ROS_ERROR("tf Transform error");
    ROS_ERROR("%s", ex.what());
    return false;
  }
  catch (std::runtime_error &ex)
  {
    ROS_ERROR("runtime error");
    ROS_ERROR("Runtime Exception at getOdomTransform: [%s]", ex.what());
    return false;
  }
}

// Populates igvc_msgs::map message with information from sensor_msgs::Image and the timestamp from pcl_stamp
/**
 * Populates <code>igvc_msgs::map message</code> with information from <code>sensor_msgs::Image</code> and the
 * timestamp from <code>pcl_stamp</code>
 * @param[out] message message to be filled out
 * @param[in] image image containing map data to be put into <code>message</code>
 * @param[in] pcl_stamp time stamp from the pcl to be used for <code>message</code>
 */
void Mapper::setMessageMetadata(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp)
{
  pcl_conversions::fromPCL(pcl_stamp, image.header.stamp);
  pcl_conversions::fromPCL(pcl_stamp, message.header.stamp);
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = static_cast<unsigned int>(m_length_x / m_resolution);
  message.width = static_cast<unsigned int>(m_width_y / m_resolution);
  message.resolution = static_cast<float>(m_resolution);
  message.orientation = static_cast<float>(m_state.yaw());
  message.x = static_cast<unsigned int>(std::round(m_state.x() / m_resolution) + m_start_x / m_resolution);
  message.y = static_cast<unsigned int>(std::round(m_state.y() / m_resolution) + m_start_y / m_resolution);
  message.x_initial = static_cast<unsigned int>(m_start_x / m_resolution);
  message.y_initial = static_cast<unsigned int>(m_start_y / m_resolution);
}

/**
 * Checks if transform from base_footprint to msg.header.frame_id exists
 * @param[in] msg
 * @param[in] topic Topic to check for
 */
bool Mapper::checkExistsStaticTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic)
{
  if (m_transforms.find(topic) == m_transforms.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ros::Time messageTimeStamp;
    pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
    ROS_INFO_STREAM("Getting transform for " << topic << " from " << msg->header.frame_id << " to /base_footprint \n");
    if (m_tf_listener->waitForTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, ros::Duration(3.0)))
    {
      tf::StampedTransform transform;
      m_tf_listener->lookupTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, transform);
      m_transforms.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
      ROS_INFO_STREAM("Found static transform from " << msg->header.frame_id << " to /base_footprint");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to find transform using empty transform");
      return false;
    }
  }
  return true;
}

/**
 * Publishes the given map at the given stamp
 * @param[in] map map to be published
 * @param[in] stamp pcl stamp of the timestamp to be used
 */
void Mapper::publish(const cv::Mat &map, uint64_t stamp)
{
  // Combine line and lidar maps, then blur them
  cv::Mat blurred_map = m_pc_map_pair.map->clone();
  if (m_use_lines) {
    cv::max(blurred_map, *m_camera_map_pair.map, blurred_map);
  }
  blur(blurred_map);

  igvc_msgs::map message;
  igvc_msgs::map blurred_message;
  sensor_msgs::Image image;
  sensor_msgs::Image blurred_image;
  m_img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, map);
  const cv_bridge::CvImage &blurred_img_bridge =
      cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, map);
  m_img_bridge.toImageMsg(image);
  blurred_img_bridge.toImageMsg(blurred_image);

  setMessageMetadata(message, image, stamp);
  setMessageMetadata(blurred_message, image, stamp);
  m_map_pub.publish(message);
  m_blurred_pub.publish(blurred_message);
  if (m_debug)
  {
    m_debug_pub.publish(image);
    // ROS_INFO_STREAM("\nThe robot is located at " << state);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromOcuGrid = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (int i = 0; i < m_width_y / m_resolution; i++)
    {
      for (int j = 0; j < m_length_x / m_resolution; j++)
      {
        pcl::PointXYZRGB p;
        uchar prob = map.at<uchar>(i, j);
        if (prob > 127)
        {
          p = pcl::PointXYZRGB();
          p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
          p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
          p.r = 0;
          p.g = static_cast<uint8_t>((prob - 127) * 2);
          p.b = 0;
          fromOcuGrid->points.push_back(p);
          //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
        }
        else if (prob < 127)
        {
          p = pcl::PointXYZRGB();
          p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
          p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
          p.r = 0;
          p.g = 0;
          p.b = static_cast<uint8_t>((127 - prob) * 2);
          fromOcuGrid->points.push_back(p);
          //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
        }
        else if (prob == 127)
        {
        }
        // Set x y coordinates as the center of the grid cell.
      }
    }
    fromOcuGrid->header.frame_id = "/odom";
    fromOcuGrid->header.stamp = stamp;
    //    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
    m_debug_pcl_pub.publish(fromOcuGrid);
  }
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr blurredPC = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < m_width_y / m_resolution; i++)
  {
    for (int j = 0; j < m_length_x / m_resolution; j++)
    {
      pcl::PointXYZRGB p;
      uchar prob = blurred_map.at<uchar>(i, j);
      if (prob > 127)
      {
        p = pcl::PointXYZRGB();
        p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
        p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
        p.r = 0;
        p.g = static_cast<uint8_t>((prob - 127) * 2);
        p.b = 0;
        blurredPC->points.push_back(p);
        //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
      }
      else if (prob < 127)
      {
        p = pcl::PointXYZRGB();
        p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
        p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
        p.r = 0;
        p.g = 0;
        p.b = static_cast<uint8_t>((127 - prob) * 2);
        blurredPC->points.push_back(p);
        //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
      }
      else if (prob == 127)
      {
      }
      // Set x y coordinates as the center of the grid cell.
    }
  }
  blurredPC->header.frame_id = "/odom";
  blurredPC->header.stamp = stamp;
  //    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
  m_debug_blurred_pc.publish(blurredPC);
}

/**
 * Discretizes angle to ints according to the angular resolution of the lidar
 * @param angle
 * @return
 */
inline int Mapper::discretize(radians angle) const
{
  static double coeff = 1 / m_angular_resolution;
  return static_cast<int>(std::round(coeff * angle));
}

/**
 * Returns a pcl::PointCloud of points which have not been detected by the lidar scan, according to the angular
 * resolution
 * @param pc lidar scan
 * @param empty_pc points that are found to be free
 */
void Mapper::get_empty_points(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &empty_pc)
{
  // Iterate over pointcloud, insert discretized angles into set
  std::unordered_set<int> discretized_angles{};
  for (auto i : pc)
  {
    double angle = atan2(i.y, i.x);
    discretized_angles.emplace(discretize(angle));
  }

  // For each angle, if it's not in the set (empty), put it into a pointcloud
  static double coeff = 1 / m_angular_resolution;
  // From Robot's frame. Need to rotate angle to world frame
  for (int i = static_cast<int>(m_lidar_start_angle * coeff); i < m_lidar_end_angle * coeff; i++)
  {
    if (discretized_angles.find(i) == discretized_angles.end())
    {
      double angle = i * m_angular_resolution;
      pcl::PointXYZ point{ static_cast<float>(m_lidar_miss_cast_distance * cos(angle)),
                           static_cast<float>(m_lidar_miss_cast_distance * sin(angle)), 0 };
      empty_pc.points.emplace_back(point);
    }
  }
}

/**
 * Filters points behind the robot
 * @param pc
 * @param filtered_pc
 */
void Mapper::filter_points_behind(const pcl::PointCloud<pcl::PointXYZ> &pc, pcl::PointCloud<pcl::PointXYZ> &filtered_pc)
{
  static double start_angle = -M_PI + m_filter_angle / 2;
  static double end_angle = M_PI - m_filter_angle / 2;
  static double squared_distance = m_filter_distance * m_filter_distance;
  // Iterate over pointcloud, insert discretized angles into set
  for (auto i : pc)
  {
    double angle = atan2(i.y, i.x);
    if ((-M_PI <= angle && angle < start_angle) || (end_angle < angle && angle <= M_PI))
    {
      if (i.x * i.x + i.y * i.y > squared_distance)
      {
        filtered_pc.points.emplace_back(i);
      }
    }
    else
    {
      filtered_pc.points.emplace_back(i);
    }
  }
}

/**
 * Callback for pointcloud. Filters the lidar scan, then inserts it into the octree.
 * @param[in] pc Lidar scan
 */
void Mapper::pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc)
{
  // make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // Check if static transform already exists for this topic.
  if (!checkExistsStaticTransform(pc, "/scan/pointcloud"))
  {
    ROS_ERROR("Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Lookup transform form Ros Localization for position
  if (!getOdomTransform(pc))
  {
    ROS_ERROR("Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  pcl::PointCloud<pcl::PointXYZ> empty_pc{};
  pcl::PointCloud<pcl::PointXYZ> filtered_pc{};
  get_empty_points(*pc, empty_pc);
  filter_points_behind(*pc, filtered_pc);

  filtered_pc.header = pc->header;
  m_ground_pub.publish(filtered_pc);

  // Apply transformation from lidar to base_link aka robot pose
  pcl_ros::transformPointCloud(filtered_pc, *transformed, m_transforms.at(m_lidar_topic));
  pcl_ros::transformPointCloud(*transformed, *transformed, m_state.transform);
  pcl_ros::transformPointCloud(empty_pc, empty_pc, m_transforms.at(m_lidar_topic));
  pcl_ros::transformPointCloud(empty_pc, empty_pc, m_state.transform);

  m_octomapper->insert_scan(m_odom_to_lidar.transform.getOrigin(), m_pc_map_pair, *transformed, empty_pc);

  // Get updated map from octomapper
  m_octomapper->get_updated_map(m_pc_map_pair);

  // Blur map (Test?) for pathfinding
  cv::Mat blurred = m_pc_map_pair.map->clone();
  blur(blurred);
  publish(*(m_pc_map_pair.map), pc->header.stamp);
}

/**
 * Applied a gaussian blur with a kernel size of m_kernel_size
 * @param[in/out] blurred_map The map to be blurred
 */
void Mapper::blur(cv::Mat &blurred_map)
{
  cv::Mat original = blurred_map.clone();
  //  cv::GaussianBlur(blurred_map, blurred_map, cv::Size(m_kernel_size, m_kernel_size), m_blur_std_dev,
  //  m_blur_std_dev);
  cv::blur(blurred_map, blurred_map, cv::Size(m_kernel_size, m_kernel_size));
  cv::max(original, blurred_map, blurred_map);
}

/**
 * Callback for the neural network segmented image.
 * @param camera_info
 */
void Mapper::line_map_callback(const sensor_msgs::ImageConstPtr &segmented)
{
  // Convert to OpenCV
  cv_bridge::CvImagePtr segmented_ptr = cv_bridge::toCvCopy(segmented, "mono8");

  // Gaussian blur
  cv::GaussianBlur(segmented_ptr->image, segmented_ptr->image, cv::Size(m_segmented_kernel, m_segmented_kernel),
                   m_segmented_sigma, m_segmented_sigma);
  segmented_ptr->image = segmented_ptr->image * (m_segmented_kernel * m_segmented_kernel);

  // Threshold
  cv::threshold(segmented_ptr->image, segmented_ptr->image, m_segmented_threshold, UCHAR_MAX, cv::THRESH_BINARY);

  // Project to ground (Parameters)

  // Insert into octree

  // Publish
}

/**
 * Callback for the line projetced onto lidar point cloud. Directly insert it into the line octomap
 * @param pc
 */
void Mapper::projected_line_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc)
{
  // Check if static transform already exists for this topic.
  if (!checkExistsStaticTransform(pc, m_projected_line_topic))
  {
    ROS_ERROR("Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Lookup transform form Ros Localization for position
  if (!getOdomTransform(pc))
  {
    ROS_ERROR("Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Transformed
  pcl::PointCloud<pcl::PointXYZ> transformed{};

  // Apply transformation from camera to base_link aka robot pose
  pcl_ros::transformPointCloud(*pc, transformed, m_transforms.at(m_projected_line_topic));
  pcl_ros::transformPointCloud(transformed, transformed, m_state.transform);

  // TODO: Take into account that area from camera to line is free space maybe?
  //  m_octomapper->insert_camera_projection(m_odom_to_camera_projection.transform.getOrigin(), m_camera_map_pair,
  //  transformed);
  m_octomapper->insert_camera_projection(m_camera_map_pair, transformed);

  // Get updated map from octomapper
  m_octomapper->get_updated_map(m_camera_map_pair);

  publish(*(m_pc_map_pair.map), pc->header.stamp);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  Mapper mapper;
}
