#ifndef SRC_ROS_TRAJECTORY_CONTROLLER_H
#define SRC_ROS_TRAJECTORY_CONTROLLER_H
#include <ros/ros.h>
#include <opencv2/core/mat.hpp>
#include "trajectory_controller.h"

namespace ros_trajectory_controller
{
using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;
using trajectory_controller::TrajectoryController;

class ROSTrajectoryController
{
public:
  ROSTrajectoryController();

private:
  void initSubscribeAndPublish();
  void initController();
  void publishAsPCL(const ros::Publisher &pub, const cv::Mat &mat, double resolution, const std::string &frame_id,
                    uint64_t stamp);

  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Publisher signed_distance_field_pub_;
  ros::Publisher motor_pub_;

  std::unique_ptr<SignedDistanceFieldOptions> sdf_options_;
  std::unique_ptr<TrajectoryController> controller_;
};
}  // namespace ros_trajectory_controller

#endif  // SRC_ROS_TRAJECTORY_CONTROLLER_H
