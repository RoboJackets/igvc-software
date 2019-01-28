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
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include "octomapper.h"
#include <visualization_msgs/Marker.h>
#include <signal.h>

cv_bridge::CvImage img_bridge;

ros::Publisher map_pub;                  // Publishes map
ros::Publisher debug_pub;                // Debug version of above
ros::Publisher debug_pcl_pub;            // Publishes map as individual PCL points
ros::Publisher ground_pub;            // Publishes map as individual PCL points
ros::Publisher nonground_pub;            // Publishes map as individual PCL points
ros::Publisher sensor_pub;            // Publishes map as individual PCL points
std::unique_ptr<cv::Mat> published_map;  // matrix will be publishing
std::map<std::string, tf::StampedTransform> transforms;
std::unique_ptr<tf::TransformListener> tf_listener;

double resolution;
double transform_max_wait_time;
int start_x;  // start x location
int start_y;  // start y location
int length_y;
int width_x;
bool debug;
double radius;
RobotState state;
RobotState state2;

std::unique_ptr<Octomapper> octomapper;
pc_map_pair pc_map_pair;

/**
 * Updates <code>RobotState state</code> with the latest tf transform using the timestamp of the message passed in
 * @param msg <code>pcl::PointCloud</code> message with the timestamp used for looking up the tf transform
 */
void getOdomTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg) {
  tf::StampedTransform transform;
  tf::StampedTransform transform2;
  ros::Time messageTimeStamp;
  pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
  if (tf_listener->waitForTransform("/odom", "/base_link", messageTimeStamp, ros::Duration(transform_max_wait_time))) {
    tf_listener->lookupTransform("/odom", "/base_link", messageTimeStamp, transform);
    state.setState(transform);
    tf_listener->lookupTransform("/odom", "/lidar", messageTimeStamp, transform2);
    state2.setState(transform2);
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
void setMsgValues(igvc_msgs::map &message, sensor_msgs::Image &image, uint64_t pcl_stamp) {
  pcl_conversions::fromPCL(pcl_stamp, image.header.stamp);
  pcl_conversions::fromPCL(pcl_stamp, message.header.stamp);
  message.header.frame_id = "/odom";
  message.image = image;
  message.length = length_y;
  message.width = width_x;
  message.resolution = resolution;
  message.orientation = state.yaw(); message.x = std::round(state.x() / resolution) + start_x;
  message.y = std::round(state.y() / resolution) + start_y;
  message.x_initial = start_x;
  message.y_initial = start_y;
}

/**
 * Checks if transform from base_footprint to msg.header.frame_id exists
 * @param msg
 * @param topic
 */
void checkExistsStaticTransform(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &msg, const std::string &topic) {
  if (transforms.find(topic) == transforms.end()) {
    // Wait for transform between frame_id (ex. /scan/pointcloud) and base_footprint.
    ros::Time messageTimeStamp;
    pcl_conversions::fromPCL(msg->header.stamp, messageTimeStamp);
    if (tf_listener->waitForTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, ros::Duration(3.0))) {
      ROS_INFO_STREAM("\n\ngetting transform for " << topic << "\n\n");
      tf::StampedTransform transform;
      tf_listener->lookupTransform("/base_footprint", msg->header.frame_id, messageTimeStamp, transform);
      transforms.insert(std::pair<std::string, tf::StampedTransform>(topic, transform));
    } else {
      ROS_ERROR_STREAM("\n\nfailed to find transform using empty transform\n\n");
    }
  }
}
void publish(const cv::Mat &map, uint64_t stamp) {
  igvc_msgs::map message;    // >> message to be sent
  sensor_msgs::Image image;  // >> image in the message
  img_bridge = cv_bridge::CvImage(message.header, sensor_msgs::image_encodings::MONO8, map);
  img_bridge.toImageMsg(image);  // from cv_bridge to sensor_msgs::Image

  setMsgValues(message, image, stamp);
  map_pub.publish(message);
  if (debug) {
    debug_pub.publish(image);
    // ROS_INFO_STREAM("\nThe robot is located at " << state);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr fromOcuGrid =
        boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    for (int i = 0; i < width_x/resolution; i++) {
      for (int j = 0; j < length_y/resolution; j++) {
        pcl::PointXYZRGB p;
        uchar prob = map.at<uchar>(i, j);
        if (prob > 127) {
          p = pcl::PointXYZRGB();
          p.x = (i * resolution) - (width_x / 2);
          p.y = (j * resolution) - (length_y / 2);
          p.r = 0;
          p.g = static_cast<uint8_t>((prob - 127) * 2);
          p.b = 0;
          fromOcuGrid->points.push_back(p);
          //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
        } else if (prob < 127) {
          p = pcl::PointXYZRGB();
          p.x = (i * resolution) - (width_x / 2);
          p.y = (j * resolution) - (length_y / 2);
          p.r = 0;
          p.g = 0;
          p.b = static_cast<uint8_t>((127 - prob) * 2);
          fromOcuGrid->points.push_back(p);
          //          ROS_INFO_STREAM("(" << i << ", " << j << ") -> (" << p.x << ", " << p.y << ")");
        } else if (prob == 127) {
        }
        // Set x y coordinates as the center of the grid cell.
      }
    }
    fromOcuGrid->header.frame_id = "/odom";
    fromOcuGrid->header.stamp = stamp;
    //    ROS_INFO_STREAM("Size: " << fromOcuGrid->points.size() << " / " << (width_x * length_y));
    debug_pcl_pub.publish(fromOcuGrid);
  }
}

void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::ConstPtr &pc) {
  // Pass through filter to only keep ones closest to us
  pcl::PointCloud<pcl::PointXYZ>::Ptr small(new pcl::PointCloud<pcl::PointXYZ>);
  float distanceFromSphereCenterPoint;
  bool pointIsWithinSphere;
  for (int point_i = 0; point_i < pc->size(); ++point_i) {
    distanceFromSphereCenterPoint = pc->at(point_i).x * pc->at(point_i).x + pc->at(point_i).y * pc->at(point_i).y + pc->at(point_i).z * pc->at(point_i).z;
    pointIsWithinSphere = distanceFromSphereCenterPoint <= radius*radius*radius;
    if (pointIsWithinSphere) {
      small->push_back(pc->at(point_i));
    }
  }
  //  ROS_INFO("Got Pointcloud");
  // make transformed clouds
  pcl::PointCloud<pcl::PointXYZ>::Ptr transformed =
      pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>());

  // Check if static transform already exists for this topic.
  checkExistsStaticTransform(pc, "/scan/pointcloud");

  // Lookup transform form Ros Localization for position
  getOdomTransform(pc);

  // Apply transformation from lidar to base_link aka robot pose
  pcl_ros::transformPointCloud(*small, *transformed, transforms.at("/scan/pointcloud"));
  pcl_ros::transformPointCloud(*transformed, *transformed, state.transform);

  //  Eigen::Affine3f transform_to_odom = Eigen::Affine3f::Identity();
  //  // TODO: Is this backward?
  //  transform_to_odom.rotate(Eigen::AngleAxisf(state.yaw(), Eigen::Vector3f::UnitZ()));
  //  Eigen::Affine3d transform_to_odom;
  //  tf::transformTFToEigen(state.transform.inverse(), transform_to_odom);

  //  ROS_INFO_STREAM("State.transform: " << state.transform.getOrigin().x() << ", " << state.transform.getOrigin().y() << ", " << state.transform.getOrigin().z());
  //  ROS_INFO_STREAM("State.transform: " << temp.getOrigin().x() << ", " << temp.getOrigin().y() << ", " << temp.getOrigin().z());

  pcl::PointCloud<pcl::PointXYZ> ground;
  pcl::PointCloud<pcl::PointXYZ> nonground;

  //  pcl::transformPointCloud(*transformed, *transformed2, transform_to_odom);
//  octomapper->filter_ground_plane(*transformed, ground, nonground);
//
//  ground.header.frame_id = "/odom";
//  transformed->header.frame_id = "/odom";
//  nonground.header.frame_id = "/odom";
//  ground.header.stamp = pc->header.stamp;
//  transformed->header.stamp = pc->header.stamp;
//  nonground.header.stamp = pc->header.stamp;
//
//  nonground_pub.publish(nonground);
//  ground_pub.publish(ground);

  visualization_msgs::Marker points;
  points.header.frame_id = "/odom";
  points.header.stamp = ros::Time::now();
  points.pose.position.x = state2.transform.getOrigin().getX();
  points.pose.position.y = state2.transform.getOrigin().getY();
  points.pose.position.z = state2.transform.getOrigin().getZ();

  points.action = visualization_msgs::Marker::ADD;
  points.id = 0;
  points.type = visualization_msgs::Marker::CUBE;
  points.scale.x = 0.05;
  points.scale.y = 0.05;
  points.scale.z = 0.05;
  points.color.g = 1.0f;
  points.color.a = 1.0;

  sensor_pub.publish(points);

  octomapper->insert_scan(state2.transform.getOrigin(), pc_map_pair, *transformed);

  //  ROS_INFO("Publishing");
  // Publish map
  //  ROS_INFO_STREAM("octomap tree size1: " << pc_map_pair.octree->getNumLeafNodes());
  octomapper->get_updated_map(pc_map_pair);
  //  ROS_INFO_STREAM("octomap tree size2: " << pc_map_pair.octree->getNumLeafNodes());
  publish(*(pc_map_pair.map), pc->header.stamp);
}

void node_cleanup(int sig)
{
  published_map.reset();
  published_map.reset();
  octomapper.reset();
  tf_listener.reset();
  ros::shutdown();
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "mapper");
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");
  std::string topics;

  signal(SIGINT, node_cleanup);

  std::list<ros::Subscriber> subs;
  tf_listener = std::unique_ptr<tf::TransformListener>(new tf::TransformListener());

  octomapper = std::unique_ptr<Octomapper>(new Octomapper(pNh));
  octomapper->create_octree(pc_map_pair);

  // assumes all params inputted in meters
  igvc::getParam(pNh, "octree/resolution", resolution);
  igvc::getParam(pNh, "map/length", length_y);
  igvc::getParam(pNh, "map/width", width_x);
  igvc::getParam(pNh, "debug", debug);
  igvc::getParam(pNh, "sensor_model/max_range", radius);

  // convert from meters to grid
  ros::Subscriber pcl_sub = nh.subscribe<pcl::PointCloud<pcl::PointXYZ>>("/scan/pointcloud", 1, &pc_callback);

  published_map = std::unique_ptr<cv::Mat>(new cv::Mat(length_y, width_x, CV_8UC1));

  map_pub = nh.advertise<igvc_msgs::map>("/map", 1);

  if (debug) {
    debug_pub = nh.advertise<sensor_msgs::Image>("/map_debug", 1);
    debug_pcl_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/map_debug_pcl", 1);
    ground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/ground_pcl", 1);
    nonground_pub = nh.advertise<pcl::PointCloud<pcl::PointXYZ>>("/nonground_pcl", 1);
    sensor_pub = nh.advertise<visualization_msgs::Marker>("/sensor_pos", 1);
  }

  ros::spin();
}
