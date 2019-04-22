// Subscribes to Point Cloud Data, updates the occupancy grid, then publishes the data.
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <unordered_set>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>

#include <pcl/point_cloud.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ros/ros.h>

#include <igvc_msgs/map.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>

#include <igvc_utils/robot_state.h>
#include <visualization_msgs/Marker.h>
#include <igvc_utils/NodeUtils.hpp>

#include <cv_bridge/cv_bridge.h>

#include "map_utils.h"
#include "ros_mapper.h"

ROSMapper::ROSMapper() : tf_listener_{ std::unique_ptr<tf::TransformListener>(new tf::TransformListener()) }
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  igvc::getParam(pNh, "map/length", length_x_);
  igvc::getParam(pNh, "map/width", width_y_);
  igvc::getParam(pNh, "map/start_x", start_x_);
  igvc::getParam(pNh, "map/start_y", start_y_);
  igvc::getParam(pNh, "octree/resolution", resolution_);

  igvc::getParam(pNh, "topics/lidar", lidar_topic_);
  igvc::getParam(pNh, "topics/line_segmentation", line_topic_);
  igvc::getParam(pNh, "topics/projected_line_pc", projected_line_topic_);
  igvc::getParam(pNh, "topics/camera_info", camera_info_topic_);
  igvc::param(pNh, "frames/camera", camera_frame_, std::string{ "optical_cam_center" });

  igvc::getParam(pNh, "cameras/resize_width", resize_width_);
  igvc::getParam(pNh, "cameras/resize_height", resize_height_);

  igvc::getParam(pNh, "node/debug", debug_);
  igvc::getParam(pNh, "node/use_lines", use_lines_);
  igvc::param(pNh, "node/transform_max_wait_time", transform_max_wait_time_, 3.0);

  mapper_ = std::make_unique<Mapper>(pNh);

  ros::Subscriber pcl_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(lidar_topic_, 1, &ROSMapper::pcCallback, this);
  ros::Subscriber line_map_sub;
  ros::Subscriber projected_line_map_sub;
  ros::Subscriber camera_info_sub;

  if (use_lines_)
  {
    ROS_INFO_STREAM("Subscribing to " << line_topic_ << " for image and " << projected_line_topic_
                                      << " for projected pointclouds");
    line_map_sub = nh.subscribe<sensor_msgs::Image>(line_topic_, 1, &ROSMapper::segmentedImageCallback, this);
    projected_line_map_sub =
        nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(projected_line_topic_, 1, &ROSMapper::projectedLineCallback, this);
    camera_info_sub =
        nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_, 1, &ROSMapper::cameraInfoCallback, this);
  }

  map_pub_ = nh.advertise<igvc_msgs::map>("/map", 1);

  if (debug_)
  {
    debug_pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl", 1);
  }

  ros::spin();
}

template <>
bool ROSMapper::getOdomTransform(const ros::Time message_timestamp)
{
  tf::StampedTransform transform;
  static ros::Duration wait_time = ros::Duration(transform_max_wait_time_);
  try
  {
    if (tf_listener_->waitForTransform("/odom", "/base_footprint", message_timestamp, wait_time))
    {
      tf_listener_->lookupTransform("/odom", "/base_footprint", message_timestamp, transform);
      state_.setState(transform);
      return true;
    }
    else
    {
      ROS_DEBUG("Failed to get transform from /base_footprint to /odom in time, using newest transforms");
      tf_listener_->lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
      state_.setState(transform);
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
bool ROSMapper::getOdomTransform(const uint64 timestamp)
{
  ros::Time message_timestamp;
  pcl_conversions::fromPCL(timestamp, message_timestamp);
  return getOdomTransform(message_timestamp);
}

void ROSMapper::setMessageMetadata(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp)
{
  pcl_conversions::fromPCL(pcl_stamp, image.header.stamp);
  pcl_conversions::fromPCL(pcl_stamp, message.header.stamp);
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = static_cast<unsigned int>(length_x_ / resolution_);
  message.width = static_cast<unsigned int>(width_y_ / resolution_);
  message.resolution = static_cast<float>(resolution_);
  message.orientation = static_cast<float>(state_.yaw());
  message.x = static_cast<unsigned int>(std::round(state_.x() / resolution_) + start_x_ / resolution_);
  message.y = static_cast<unsigned int>(std::round(state_.y() / resolution_) + start_y_ / resolution_);
  message.x_initial = static_cast<unsigned int>(start_x_ / resolution_);
  message.y_initial = static_cast<unsigned int>(start_y_ / resolution_);
}

template <>
bool ROSMapper::checkExistsStaticTransform(const std::string &frame_id, ros::Time message_timestamp,
                                           const std::string &topic)
{
  if (transforms_.find(topic) == transforms_.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ROS_INFO_STREAM("Getting transform for " << topic << " from " << frame_id << " to /base_footprint \n");
    if (tf_listener_->waitForTransform("/base_footprint", frame_id, message_timestamp,
                                       ros::Duration(transform_max_wait_time_)))
    {
      tf::StampedTransform transform;
      tf_listener_->lookupTransform("/base_footprint", frame_id, message_timestamp, transform);
      transforms_.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
      ROS_INFO_STREAM("Found static transform from " << frame_id << " to /base_footprint");
    }
    else
    {
      ROS_ERROR_STREAM("Failed to find transform for " << topic << " from " << frame_id
                                                       << " to /base_footprint using empty transform");
      return false;
    }
  }
  return true;
}

template <>
bool ROSMapper::checkExistsStaticTransform(const std::string &frame_id, uint64 timestamp, const std::string &topic)
{
  if (transforms_.find(topic) == transforms_.end())
  {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ros::Time message_timestamp;
    pcl_conversions::fromPCL(timestamp, message_timestamp);
    return checkExistsStaticTransform(frame_id, message_timestamp, topic);
  }
  return true;
}

void ROSMapper::publish(uint64_t stamp)
{
  std::optional<cv::Mat> map = mapper_->getMap();
  if (!map)
  {
    ROS_WARN_STREAM_THROTTLE(1, "Couldn't get a map");
    return;
  }
  igvc_msgs::map message;
  sensor_msgs::Image image;

  img_bridge_ = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, *map);
  img_bridge_.toImageMsg(image);

  setMessageMetadata(message, image, stamp);
  map_pub_.publish(message);

  if (debug_)
  {
    publishAsPCL(debug_pcl_pub_, *map, "/odom", stamp);
  }
}

void ROSMapper::publishAsPCL(const ros::Publisher &pub, const cv::Mat &mat, const std::string &frame_id, uint64_t stamp)
{
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
  for (int i = 0; i < width_y_ / resolution_; i++)
  {
    for (int j = 0; j < length_x_ / resolution_; j++)
    {
      pcl::PointXYZRGB p{};
      p.x = static_cast<float>((i * resolution_) - (width_y_ / 2.0));
      p.y = static_cast<float>((j * resolution_) - (length_x_ / 2.0));
      uchar prob = mat.at<uchar>(i, j);
      if (prob > 127)
      {
        p.r = 0;
        p.g = static_cast<uint8_t>((prob - 127) * 2);
        p.b = 0;
        pointcloud->points.push_back(p);
      }
      else if (prob < 127)
      {
        p.r = 0;
        p.g = 0;
        p.b = static_cast<uint8_t>((127 - prob) * 2);
        pointcloud->points.push_back(p);
      }
    }
  }
  pointcloud->header.frame_id = frame_id;
  pointcloud->header.stamp = stamp;
  pub.publish(pointcloud);
}

void ROSMapper::pcCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc)
{
  if (!checkExistsStaticTransform(pc->header.frame_id, pc->header.stamp, lidar_topic_))
  {
    ROS_ERROR("Couldn't find static transform for pointcloud. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  if (!getOdomTransform(pc->header.stamp))
  {
    ROS_ERROR("Couldn't find odometry transform in pcCallback. Sleeping 2 seconds then trying again...");
    ros::Duration(2).sleep();
    return;
  }

  mapper_->insertLidarScan(pc, state_.transform * transforms_.at(lidar_topic_));

  publish(pc->header.stamp);
}

void ROSMapper::segmentedImageCallback(const sensor_msgs::ImageConstPtr &segmented)
{
  if (!camera_model_initialized_)
  {
    return;
  }
  if (!checkExistsStaticTransform(camera_frame_, segmented->header.stamp, projected_line_topic_))
  {
    ROS_ERROR_STREAM_THROTTLE(2, "Finding static transform to " << camera_frame_ << "failed. Trying again...");
    return;
  }

  if (!getOdomTransform(segmented->header.stamp))
  {
    ROS_ERROR_STREAM_THROTTLE(2, "Finding odometry transform failed. Trying again...");
    return;
  }
  // Convert to OpenCV
  cv_bridge::CvImagePtr segmented_ptr = cv_bridge::toCvCopy(segmented, "mono8");
  cv::Mat image = segmented_ptr->image;

  mapper_->insertSegmentedImage(std::move(image), state_.transform, transforms_.at(projected_line_topic_),
                                segmented->header.stamp);

  publish(pcl_conversions::toPCL(segmented->header.stamp));
}

void ROSMapper::projectedLineCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc)
{
  if (!getOdomTransform(pc->header.stamp))
  {
    ROS_ERROR_STREAM_THROTTLE(2, "Finding odometry transform failed. Trying again...");
    return;
  }
  mapper_->insertCameraProjection(pc, state_.transform);
  publish(pc->header.stamp);
}

void ROSMapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info)
{
  if (!camera_model_initialized_)
  {
    image_geometry::PinholeCameraModel camera_model;
    sensor_msgs::CameraInfoConstPtr changed_camera_info =
        MapUtils::scaleCameraInfo(camera_info, resize_width_, resize_height_);
    camera_model.fromCameraInfo(changed_camera_info);
    mapper_->setProjectionModel(camera_model);

    camera_model_initialized_ = true;
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "mapper");
  ROSMapper mapper;
}
