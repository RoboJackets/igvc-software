#ifndef PROJECT_ICP_H
#define PROJECT_ICP_H

#include <igvc_msgs/map.h>
#include <visualization_msgs/Marker.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <ros/publisher.h>
#include <ros/ros.h>

#include <boost/circular_buffer.hpp>

#include <math.h>
#include <stdlib.h>

#include <Eigen/Core>

#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>

#include <opencv2/core/eigen.hpp>
#include <opencv2/opencv.hpp>

#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

class ICP
{
public:
  ICP() : m_last_cloud(nullptr)
  {}
  void pc_callback(const pcl::PointCloud<pcl::PointXYZ>::Ptr& pc);
  ros::Publisher scanmatch_pub;                  // Publishes map
private:
  pcl::PointCloud<pcl::PointXYZ>::Ptr m_last_cloud;
  tf::TransformBroadcaster m_br;
};
#endif //PROJECT_ICP_H
