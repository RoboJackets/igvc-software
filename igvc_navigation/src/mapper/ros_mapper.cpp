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

#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <parameter_assertions/assertions.h>

#include "map_utils.h"
#include "ros_mapper.h"

ROSMapper::ROSMapper() : tf_listener_{ std::unique_ptr<tf::TransformListener>(new tf::TransformListener()) }
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  assertions::getParam(pNh, "map/length", length_x_);
  assertions::getParam(pNh, "map/width", width_y_);
  assertions::getParam(pNh, "map/start_x", start_x_);
  assertions::getParam(pNh, "map/start_y", start_y_);
  assertions::getParam(pNh, "octree/resolution", resolution_);

  assertions::getParam(pNh, "topics/lidar", lidar_topic_);

  assertions::getParam(pNh, "topics/line_segmentation/left", line_topic_left_);
  assertions::getParam(pNh, "topics/line_segmentation/center", line_topic_center_);
  assertions::getParam(pNh, "topics/line_segmentation/right", line_topic_right_);

  assertions::getParam(pNh, "topics/projected_line_pc/left", projected_line_topic_left_);
  assertions::getParam(pNh, "topics/projected_line_pc/center", projected_line_topic_center_);
  assertions::getParam(pNh, "topics/projected_line_pc/right", projected_line_topic_right_);

  assertions::getParam(pNh, "topics/camera_info/left", camera_info_topic_left_);
  assertions::getParam(pNh, "topics/camera_info/center", camera_info_topic_center_);
  assertions::getParam(pNh, "topics/camera_info/right", camera_info_topic_right_);

  assertions::getParam(pNh, "frames/camera/left", camera_frame_left_);
  assertions::getParam(pNh, "frames/camera/center", camera_frame_center_);
  assertions::getParam(pNh, "frames/camera/right", camera_frame_right_);

  assertions::getParam(pNh, "node/camera/left/enable", enable_left_cam_);
  assertions::getParam(pNh, "node/camera/center/enable", enable_center_cam_);
  assertions::getParam(pNh, "node/camera/right/enable", enable_right_cam_);

  assertions::getParam(pNh, "node/camera/use_passed_in_pointcloud", use_passed_in_pointcloud_);

  assertions::getParam(pNh, "cameras/resize_width", resize_width_);
  assertions::getParam(pNh, "cameras/resize_height", resize_height_);

  assertions::getParam(pNh, "topics/camera_center", center_camera_topic_);

  assertions::getParam(pNh, "node/debug/publish/map_debug_pcl", debug_pub_map_pcl);
  assertions::getParam(pNh, "node/use_lines", use_lines_);
  assertions::param(pNh, "node/transform_max_wait_time", transform_max_wait_time_, 3.0);

  mapper_ = std::make_unique<Mapper>(pNh);

  ros::Subscriber pcl_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(lidar_topic_, 1, &ROSMapper::pcCallback, this);
  ros::Subscriber projected_line_map_sub;
  ros::Subscriber center_cam_pub;

  if (use_lines_)
  {
    if (enable_left_cam_)
    {
      line_map_subs_.emplace(std::make_pair(
          Camera::left,
          nh.subscribe<sensor_msgs::Image>(line_topic_left_, 1,
                                           boost::bind(&ROSMapper::segmentedImageCallback, this, _1, Camera::left))));
      camera_infos_.emplace(std::make_pair(
          Camera::left,
          nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_left_, 1,
                                                boost::bind(&ROSMapper::cameraInfoCallback, this, _1, Camera::left))));
      if (use_passed_in_pointcloud_)
      {
        projected_line_subs_.emplace(
            std::make_pair(Camera::left, nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
                                             projected_line_topic_left_, 1,
                                             boost::bind(&ROSMapper::projectedLineCallback, this, _1, Camera::left))));
      }
    }
    if (enable_center_cam_)
    {
      line_map_subs_.emplace(std::make_pair(
          Camera::center,
          nh.subscribe<sensor_msgs::Image>(line_topic_center_, 1,
                                           boost::bind(&ROSMapper::segmentedImageCallback, this, _1, Camera::center))));
      camera_infos_.emplace(
          std::make_pair(Camera::center, nh.subscribe<sensor_msgs::CameraInfo>(
                                             camera_info_topic_center_, 1,
                                             boost::bind(&ROSMapper::cameraInfoCallback, this, _1, Camera::center))));
      if (use_passed_in_pointcloud_)
      {
        projected_line_subs_.emplace(std::make_pair(
            Camera::center, nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
                                projected_line_topic_center_, 1,
                                boost::bind(&ROSMapper::projectedLineCallback, this, _1, Camera::center))));
      }
    }
    if (enable_right_cam_)
    {
      line_map_subs_.emplace(std::make_pair(
          Camera::right,
          nh.subscribe<sensor_msgs::Image>(line_topic_right_, 1,
                                           boost::bind(&ROSMapper::segmentedImageCallback, this, _1, Camera::right))));
      camera_infos_.emplace(std::make_pair(
          Camera::right,
          nh.subscribe<sensor_msgs::CameraInfo>(camera_info_topic_right_, 1,
                                                boost::bind(&ROSMapper::cameraInfoCallback, this, _1, Camera::right))));
      if (use_passed_in_pointcloud_)
      {
        projected_line_subs_.emplace(std::make_pair(
            Camera::right, nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>(
                               projected_line_topic_right_, 1,
                               boost::bind(&ROSMapper::projectedLineCallback, this, _1, Camera::right))));
      }
    }
    center_cam_pub = nh.subscribe(center_camera_topic_, 1, &ROSMapper::centerCamCallback, this);
  }

  back_circle_sub_ = nh.subscribe("/back_circle", 1, &ROSMapper::backCircleCallback, this);

  map_pub_ = nh.advertise<igvc_msgs::map>("/map", 1);

  if (debug_pub_map_pcl)
  {
    debug_pcl_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/mapper/debug/pcl", 1);
  }

  ROS_INFO("Mapper started!");
}

void ROSMapper::centerCamCallback(const sensor_msgs::CompressedImageConstPtr &image)
{
  auto cv_img = cv_bridge::toCvCopy(image, "bgr8")->image;
  cv::resize(cv_img, cv_img, cv::Size(resize_width_, resize_height_));
  mapper_->setCenterImage(cv_img);
}

void ROSMapper::backCircleCallback(const pcl::PointCloud<pcl::PointXYZ>::Ptr msg)
{
  // do the tf tree
  // getOdomTransform(msg->header.stamp);
  mapper_->insertBackCircle(msg, state_.transform);
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

  if (debug_pub_map_pcl)
  {
    publishMapDebugPC(debug_pcl_pub_, *map, "/odom", stamp);
  }
}

void ROSMapper::publishMapDebugPC(const ros::Publisher &pub, const cv::Mat &mat, const std::string &frame_id,
                                  uint64_t stamp)
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

void ROSMapper::segmentedImageCallback(const sensor_msgs::ImageConstPtr &segmented, Camera camera)
{
  if (camera_infos_.find(camera) != camera_infos_.end())
  {
    ROS_INFO_STREAM_THROTTLE(1, "camera info for " << static_cast<int>(camera) << " still not found...");
    return;
  }
  if (segmented->width != resize_width_ || segmented->height != resize_height_)
  {
    ROS_ERROR_STREAM_THROTTLE(5, "Segmented image dimensions ("
                                     << segmented->width << ", " << segmented->height << ") differ from params: ("
                                     << resize_width_ << ", " << resize_height_ << "). Projection will be incorrect.");
  }
  std::string frame;
  switch (camera)
  {
    case Camera::left:
      frame = camera_frame_left_;
      break;
    case Camera::center:
      frame = camera_frame_center_;
      break;
    case Camera::right:
      frame = camera_frame_right_;
      break;
  }
  std::string topic;
  switch (camera)
  {
    case Camera::left:
      topic = line_topic_left_;
      break;
    case Camera::center:
      topic = line_topic_center_;
      break;
    case Camera::right:
      topic = line_topic_right_;
      break;
  }
  if (!checkExistsStaticTransform(frame, segmented->header.stamp, topic))
  {
    ROS_ERROR_STREAM_THROTTLE(2, "Finding static transform to " << frame << "failed. Trying again...");
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

  mapper_->insertSegmentedImage(std::move(image), state_.transform, transforms_.at(topic), segmented->header.stamp,
                                camera, use_passed_in_pointcloud_);

  publish(pcl_conversions::toPCL(segmented->header.stamp));
}

void ROSMapper::projectedLineCallback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc, Camera camera)
{
  if (!getOdomTransform(pc->header.stamp))
  {
    ROS_ERROR_STREAM_THROTTLE(2, "Finding odometry transform failed. Trying again...");
    return;
  }
  mapper_->insertCameraProjection(pc, state_.transform);
  publish(pc->header.stamp);
}

void ROSMapper::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &camera_info, Camera camera)
{
  image_geometry::PinholeCameraModel camera_model;
  sensor_msgs::CameraInfoConstPtr changed_camera_info =
      MapUtils::scaleCameraInfo(camera_info, resize_width_, resize_height_);
  camera_model.fromCameraInfo(changed_camera_info);
  mapper_->setProjectionModel(camera_model, camera);

  camera_infos_.erase(camera);
}
