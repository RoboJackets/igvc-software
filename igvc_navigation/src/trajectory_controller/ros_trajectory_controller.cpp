#include <igvc_utils/NodeUtils.hpp>

#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include "ros_trajectory_controller.h"
#include <igvc_navigation/signed_distance_field.h>

using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;

ROSTrajectoryController::ROSTrajectoryController()
{
  ros::NodeHandle nh;
  ros::NodeHandle pNh("~");

  std::string topic_trajectory;

  igvc::getParam(pNh, "topics/trajectory", topic_trajectory);

  trajectory_pub_ = nh.advertise<pcl::PointCloud<pcl::PointXYZI>>(topic_trajectory, 1);

  ros::Rate rate(30);
  while (ros::ok())
  {
    testSignedDistanceField();
    rate.sleep();
  }
}

void ROSTrajectoryController::testSignedDistanceField()
{
  int rows = 101;
  int cols = 101;
  float x = 50.0;
  float y = 50.0;
  float resolution = 1.0;

  SignedDistanceFieldOptions options{rows, cols, x, y, resolution};
  SignedDistanceField field{options};

  nav_msgs::Path path;
  geometry_msgs::PoseStamped pose;
  pose.pose.position.x = 0;
  pose.pose.position.y = 0;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 50;
  pose.pose.position.y = 50;
  path.poses.emplace_back(pose);
  pose.pose.position.x = 0;
  pose.pose.position.y = 100;
  path.poses.emplace_back(pose);

  cv::Mat traversal_costs(options.grid_rows, options.grid_cols, CV_32F, 1.0f);
  field.calculate(path, 0, 2, traversal_costs);

  std::unique_ptr<cv::Mat> solution = field.toMat();

//  solver.printGrid();
  publishAsPCL(trajectory_pub_, *solution, 1.0, "/odom", pcl_conversions::toPCL(ros::Time::now()));
}

void ROSTrajectoryController::publishAsPCL(const ros::Publisher &pub, const cv::Mat &mat, double resolution, const std::string &frame_id,
                                           uint64_t stamp)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr pointcloud = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
  for (int i = 0; i < mat.cols; i++)
  {
    for (int j = 0; j < mat.rows; j++)
    {
      pcl::PointXYZI p{};
      p.x = static_cast<float>((i * resolution) - (mat.cols / 2.0));
      p.y = static_cast<float>((j * resolution) - (mat.rows / 2.0));
      p.intensity = mat.at<float>(j, i);
      pointcloud->points.push_back(p);
    }
  }
  pointcloud->header.frame_id = frame_id;
  pointcloud->header.stamp = stamp;
  pub.publish(pointcloud);
}
