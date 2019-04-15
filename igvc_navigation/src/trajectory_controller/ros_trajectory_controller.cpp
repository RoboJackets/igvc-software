#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <igvc_utils/NodeUtils.hpp>
#include <igvc_navigation/signed_distance_field.h>
#include <igvc_msgs/velocity_pair.h>

#include "ros_trajectory_controller.h"
#include "trajectory_controller.h"

namespace ros_trajectory_controller
{

ROSTrajectoryController::ROSTrajectoryController() : nh{}, pNh{"~"}
{
  initSubscribeAndPublish();
  initController();

  ros::Rate rate(0.2);
  while (ros::ok())
  {
//    std::unique_ptr<cv::Mat> field = trajectory_controller.test();
//    publishAsPCL(signed_distance_field_pub_, *field, resolution, "/odom", pcl_conversions::toPCL(ros::Time::now()));
    rate.sleep();
  }
}

void ROSTrajectoryController::publishAsPCL(const ros::Publisher &pub, const cv::Mat &mat, double resolution,
                                           const std::string &frame_id, uint64_t stamp)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < mat.cols; i++)
  {
    for (int j = 0; j < mat.rows; j++)
    {
      pcl::PointXYZI p{};
      p.x = static_cast<float>((i - (mat.cols / 2.0)) * resolution);
      p.y = static_cast<float>(((mat.rows / 2.0) - j) * resolution);
      p.intensity = cos(mat.at<float>(j, i));
      pointcloud->points.push_back(p);
    }
  }
  pointcloud->header.frame_id = frame_id;
  pointcloud->header.stamp = stamp;
  pub.publish(pointcloud);
}

void ROSTrajectoryController::initSubscribeAndPublish()
{
  std::string topic_trajectory;
  std::string topic_path;
  std::string topic_odometry;
  std::string topic_motors;

  igvc::getParam(pNh, "topics/trajectory", topic_trajectory);
  igvc::getParam(pNh, "topics/path", topic_path);
  igvc::getParam(pNh, "topics/odometry", topic_odometry);
  igvc::getParam(pNh, "topics/motors", topic_motors);

  signed_distance_field_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic_trajectory, 1);
  motor_pub_ = nh.advertise<igvc_msgs::velocity_pair>(topic_motors, 1);
}

void ROSTrajectoryController::initController()
{
  float width;
  float height;
  float resolution;

  float timestep;
  float horizon;
  int samples;

  float max_velocity;


  igvc::getParam(pNh, "signed_distance_field/width", width);
  igvc::getParam(pNh, "signed_distance_field/height", height);
  igvc::getParam(pNh, "signed_distance_field/resolution", resolution);

  igvc::getParam(pNh, "controller/timestep", timestep);
  igvc::getParam(pNh, "controller/horizon", horizon);
  igvc::getParam(pNh, "controller/samples", samples);

  igvc::getParam(pNh, "cost_function/max_velocity", max_velocity);

  sdf_options_ = std::make_unique<SignedDistanceFieldOptions>(width, height, 0, 0, resolution);
  controller_ = std::make_unique<TrajectoryController>(sdf_options_, timestep, horizon, samples, max_velocity);
}
}  // namespace ros_trajectory_controller
