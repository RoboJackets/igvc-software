// Subscribes to Point Cloud Data, updates the occupancy grid, then publishes the data.

#include <math.h>

#include <pcl_ros/transforms.h>

#include <ros/ros.h>

#include <igvc_msgs/map.h>
#include <nav_msgs/Odometry.h>
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

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <unordered_set>
#include "mapper.h"

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

  igvc::getParam(pNh, "topics/lidar", m_lidar_topic);
  igvc::getParam(pNh, "topics/line_segmentation", m_line_topic);
  igvc::getParam(pNh, "topics/projected_line_pc", m_projected_line_topic);
  igvc::getParam(pNh, "topics/camera_info", m_camera_info_topic);

  igvc::getParam(pNh, "cameras/resize_width", m_resize_width);
  igvc::getParam(pNh, "cameras/resize_height", m_resize_height);

  igvc::getParam(pNh, "node/debug", m_debug);
  igvc::getParam(pNh, "node/use_lines", m_use_lines);

  m_octomapper = std::unique_ptr<Octomapper>(new Octomapper(pNh));
  m_octomapper->create_octree(m_pc_map_pair);

  ros::Subscriber pcl_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(m_lidar_topic, 1, &Mapper::pc_callback, this);
  ros::Subscriber line_map_sub;
  ros::Subscriber projected_line_map_sub;
  ros::Subscriber camera_info_sub;
  if (m_use_lines)
  {
    m_octomapper->create_octree(m_camera_map_pair);
    ROS_INFO_STREAM("Subscribing to " << m_line_topic << " for image and " << m_projected_line_topic
                                      << " for projected pointclouds");
    line_map_sub = nh.subscribe<sensor_msgs::Image>(m_line_topic, 1, &Mapper::line_map_callback, this);
    projected_line_map_sub =
        nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(m_projected_line_topic, 1, &Mapper::projected_line_callback, this);
    camera_info_sub =
        nh.subscribe<sensor_msgs::CameraInfo>(m_camera_info_topic, 1, &Mapper::camera_info_callback, this);
  }

  //  m_published_map = std::unique_ptr<cv::Mat>(new cv::Mat(m_length_x, m_width_y, CV_8UC1));

  m_map_pub = nh.advertise<igvc_msgs::map>("/map", 1);
  m_blurred_pub = nh.advertise<igvc_msgs::map>("/map/blurred", 1);

  if (m_debug)
  {
    m_debug_pub = nh.advertise<sensor_msgs::Image>("/map_debug", 1);
    m_debug_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl", 1);
    m_debug_blurred_pc = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl/blurred", 1);
    m_ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/ground_pcl", 1);
    m_nonground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/nonground_pcl", 1);
    m_random_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/mapper/debug", 1);
  }

  ros::spin();
}

/**
 * Updates <code>RobotState state</code> with the latest tf transform using the timestamp of the message passed in
 * @param[in] msg <code>pcl::PointCloud</code> message with the timestamp used for looking up the tf transform
 */
template <>
bool Mapper::get_odom_transform(const ros::Time message_timestamp)
// bool Mapper::getOdomTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg)
{
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  static ros::Duration wait_time = ros::Duration(m_transform_max_wait_time);
  try
  {
    if (m_tf_listener->waitForTransform("/odom", "/base_link", message_timestamp, wait_time))
    {
      m_tf_listener->lookupTransform("/odom", "/base_link", message_timestamp, transform);
      m_state.setState(transform);
      m_tf_listener->lookupTransform("/odom", "/lidar", message_timestamp, transform2);
      m_odom_to_lidar.setState(transform2);
      if (m_use_lines)
      {
        if (m_camera_frame.size() == 0)
        {
          ROS_INFO_STREAM(m_camera_frame << " has .size() == 0. Return false.");
          return false;
        }
        m_tf_listener->lookupTransform("/odom", m_camera_frame, message_timestamp, transform2);
        m_odom_to_camera_projection.setState(transform2);
      }
      return true;
    }
    else
    {
      ROS_DEBUG("Failed to get transform from /base_link to /odom in time, using newest transforms");
      m_tf_listener->lookupTransform("/odom", "/base_link", ros::Time(0), transform);
      m_state.setState(transform);
      m_tf_listener->lookupTransform("/odom", "/lidar", ros::Time(0), transform2);
      m_odom_to_lidar.setState(transform2);
      if (m_use_lines)
      {
        if (m_camera_frame.size() == 0)
        {
          return false;
        }
        m_tf_listener->lookupTransform("/odom", m_camera_frame, ros::Time(0), transform2);
        m_odom_to_camera_projection.setState(transform2);
      }
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

template <>
bool Mapper::get_odom_transform(const uint64 timestamp)
{
  ros::Time message_timestamp;
  pcl_conversions::fromPCL(timestamp, message_timestamp);
  return get_odom_transform(message_timestamp);
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

template <>
bool Mapper::checkExistsStaticTransform(const std::string &frame_id, ros::Time message_timestamp,
                                        const std::string &topic)
{
  if (m_transforms.find(topic) == m_transforms.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ROS_INFO_STREAM("Getting transform for " << topic << " from " << frame_id << " to /base_footprint \n");
    if (m_tf_listener->waitForTransform("/base_footprint", frame_id, message_timestamp, ros::Duration(3.0)))
    {
      tf::StampedTransform transform;
      m_tf_listener->lookupTransform("/base_footprint", frame_id, message_timestamp, transform);
      m_transforms.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
      ROS_INFO_STREAM("Found static transform from " << frame_id << " to /base_footprint");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to find transform using empty transform");
      return false;
    }
  }
  return true;
}

template <>
bool Mapper::checkExistsStaticTransform(const std::string &frame_id, uint64 timestamp, const std::string &topic)
{
  if (m_transforms.find(topic) == m_transforms.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ros::Time message_timestamp;
    pcl_conversions::fromPCL(timestamp, message_timestamp);
    return checkExistsStaticTransform(frame_id, message_timestamp, topic);
  }
  return true;
}

/**
 * Publishes the given map at the given stamp
 * @param[in] map map to be published
 * @param[in] stamp pcl stamp of the timestamp to be used
 */
void Mapper::publish(uint64_t stamp)
{
  // Combine line and lidar maps, then blur them
  cv::Mat blurred_map;
  if (m_pc_map_pair.map)
  {
    blurred_map = m_pc_map_pair.map->clone();
    if (m_use_lines)
    {
      cv::max(blurred_map, *m_camera_map_pair.map, blurred_map);
    }
    blur(blurred_map);
  }
  else
  {
    blurred_map = cv::Mat(m_camera_map_pair.map->size().height, m_camera_map_pair.map->size().width, CV_8UC1, 127);
    if (m_use_lines)
    {
      cv::max(blurred_map, *m_camera_map_pair.map, blurred_map);
    }
    blur(blurred_map);
  }

  igvc_msgs::map message;
  igvc_msgs::map blurred_message;
  sensor_msgs::Image image;
  sensor_msgs::Image blurred_image;
  if (m_pc_map_pair.map)
  {
    m_img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, blurred_map);
    m_img_bridge.toImageMsg(image);
    setMessageMetadata(message, image, stamp);
    m_map_pub.publish(message);
  }
  const cv_bridge::CvImage &blurred_img_bridge =
      cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, blurred_map);
  blurred_img_bridge.toImageMsg(blurred_image);

  setMessageMetadata(blurred_message, image, stamp);
  m_blurred_pub.publish(blurred_message);
  if (m_debug)
  {
    m_debug_pub.publish(image);
    if (m_pc_map_pair.map)
    {
      publish_as_pcl(m_debug_pcl_pub, *m_pc_map_pair.map, "/odom", stamp);
    }
    publish_as_pcl(m_debug_blurred_pc, blurred_map, "/odom", stamp);
  }
}

void Mapper::publish_as_pcl(const ros::Publisher &pub, const cv::Mat &mat, const std::string &frame_id, uint64_t stamp)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < m_width_y / m_resolution; i++)
  {
    for (int j = 0; j < m_length_x / m_resolution; j++)
    {
      pcl::PointXYZRGB p;
      uchar prob = mat.at<uchar>(i, j);
      if (prob > 127)
      {
        p = pcl::PointXYZRGB();
        p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
        p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
        p.r = 0;
        p.g = static_cast<uint8_t>((prob - 127) * 2);
        p.b = 0;
        pointcloud->points.push_back(p);
      }
      else if (prob < 127)
      {
        p = pcl::PointXYZRGB();
        p.x = static_cast<float>((i * m_resolution) - (m_width_y / 2.0));
        p.y = static_cast<float>((j * m_resolution) - (m_length_x / 2.0));
        p.r = 0;
        p.g = 0;
        p.b = static_cast<uint8_t>((127 - prob) * 2);
        pointcloud->points.push_back(p);
      }
      else
      {
        // Do nothing if p=0.5
      }
    }
  }
  pointcloud->header.frame_id = frame_id;
  pointcloud->header.stamp = stamp;
  //    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
  pub.publish(pointcloud);
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
  if (!checkExistsStaticTransform(pc->header.frame_id, pc->header.stamp, m_lidar_topic))
  {
    ROS_ERROR("Couldn't find static transform for pointcloud. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Lookup transform form Ros Localization for position
  if (!get_odom_transform(pc->header.stamp))
  {
    ROS_ERROR("Couldn't find odometry transform in pc_callback. Sleeping 2 seconds then trying again...");
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

  publish(pc->header.stamp);
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
  if (!m_camera_model_initialized)
  {
    return;
  }
  m_camera_frame = "optical_cam_center";
  // Check if static transform already exists for this topic.
  if (!checkExistsStaticTransform("optical_cam_center", segmented->header.stamp, m_projected_line_topic))
  {
    ROS_ERROR("Finding static transform failed. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Lookup transform form Ros Localization for position
  if (!get_odom_transform(segmented->header.stamp))
  {
    ROS_ERROR("Finding odometry transform failed. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }
  // Convert to OpenCV
  cv_bridge::CvImagePtr segmented_ptr = cv_bridge::toCvCopy(segmented, "mono8");

  cv::Mat image = segmented_ptr->image;

  // Insert into octree
  m_octomapper->insert_camera_free(m_camera_map_pair, image, m_camera_model,
                                   m_state.transform * m_transforms.at(m_projected_line_topic));

  // Get updated map from octomapper
  m_octomapper->get_updated_map(m_camera_map_pair);

  publish(pcl_conversions::toPCL(segmented->header.stamp));
}

/**
 * Callback for the line projetced onto lidar point cloud. Directly insert it into the line octomap
 * @param pc
 */
void Mapper::projected_line_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc)
{
  m_camera_frame = pc->header.frame_id;
  m_camera_frame = "optical_cam_center";

  // Lookup transform form Ros Localization for position
  if (!get_odom_transform(pc->header.stamp))
  {
    ROS_ERROR("Finding odometry transform failed. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  // Transformed
  pcl::PointCloud<pcl::PointXYZ> transformed{};

  // Apply transformation from camera to base_link aka robot pose
  pcl_ros::transformPointCloud(*pc, transformed, m_state.transform);
  transformed.header.stamp = pc->header.stamp;
  transformed.header.frame_id = "odom";
  m_random_pub.publish(transformed);

  m_octomapper->insert_camera_projection(m_camera_map_pair, transformed, true);
  //
  //  // Get updated map from octomapper
  m_octomapper->get_updated_map(m_camera_map_pair);

  publish(pc->header.stamp);
}

void Mapper::camera_info_callback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
  if (!m_camera_model_initialized)
  {
    sensor_msgs::CameraInfo changed_camera_info = *camera_info;
    changed_camera_info.D = camera_info->D;
    changed_camera_info.distortion_model = camera_info->distortion_model;
    changed_camera_info.R = camera_info->R;
    changed_camera_info.roi = camera_info->roi;
    changed_camera_info.binning_x = camera_info->binning_x;
    changed_camera_info.binning_y = camera_info->binning_y;

    //    waf = float(resize_width) / camera_info.width
    double waf = static_cast<double>(m_resize_width) / static_cast<double>(camera_info->width);
    ROS_INFO_STREAM("waf: (" << waf << ") " << m_resize_width << " / " << camera_info->width);
    //    haf = float(resize_height) / camera_info.height
    double haf = static_cast<double>(m_resize_height) / static_cast<double>(camera_info->height);
    ROS_INFO_STREAM("haf: (" << haf << ") " << m_resize_height << " / " << camera_info->height);
    //    camera_info.height = resize_height
    changed_camera_info.height = m_resize_height;
    //    camera_info.width = resize_width
    changed_camera_info.width = m_resize_width;
    //    K = camera_info.K
    //    camera_info.K = (K[0]*waf,         0.,  K[2]*waf,
    //        0.,  K[4]*haf,  K[5]*haf,
    //        0.,        0.,         1.)
    ROS_INFO("(K) Before: \n");
    for (auto x : camera_info->K)
    {
      ROS_INFO("%.2f ", x);
    }
    ROS_INFO("\n");
    changed_camera_info.K = { { camera_info->K[0] * waf, 0, camera_info->K[2] * waf, 0, camera_info->K[4] * haf,
                                camera_info->K[5] * haf, 0, 0, 1 } };
    ROS_INFO("After: \n");
    for (auto x : changed_camera_info.K)
    {
      ROS_INFO("%.2f ", x);
    }
    ROS_INFO("\n");
    //
    //    P = camera_info.P
    //    camera_info.P = (P[0]*waf,        0.,  P[2]*waf,  0.,
    //        0.,  P[5]*haf,  P[6]*haf,  0.,
    //        0.,        0.,        1.,  0.)
    ROS_INFO("(P) Before: \n");
    for (auto x : camera_info->P)
    {
      ROS_INFO("%.2f ", x);
    }
    ROS_INFO("\n");
    changed_camera_info.P = { { camera_info->P[0] * waf, 0, camera_info->P[2] * waf, 0, 0, camera_info->P[5] * haf,
                                camera_info->P[6] * haf, 0, 0, 0, 1, 0 } };
    ROS_INFO("(P) After: \n");
    for (auto x : changed_camera_info.P)
    {
      ROS_INFO("%.2f ", x);
    }
    ROS_INFO("\n");
    m_camera_model.fromCameraInfo(changed_camera_info);

    m_camera_model_initialized = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  Mapper mapper;
}
