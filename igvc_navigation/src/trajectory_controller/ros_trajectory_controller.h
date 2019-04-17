#ifndef SRC_ROS_TRAJECTORY_CONTROLLER_H
#define SRC_ROS_TRAJECTORY_CONTROLLER_H
#include <ros/ros.h>
#include <opencv2/core/mat.hpp>
#include <visualization_msgs/MarkerArray.h>

#include "trajectory_controller.h"

namespace ros_trajectory_controller
{
using signed_distance_field::SignedDistanceField;
using signed_distance_field::SignedDistanceFieldOptions;
using trajectory_controller::TrajectoryController;
using trajectory_controller::State;
using trajectory_controller::Model;

class ROSTrajectoryController
{
public:
  ROSTrajectoryController();

private:
  void initSubscribeAndPublish();
  void initController();
  void initMiscParams();
  void publishAsPCL(const ros::Publisher& pub, const cv::Mat& mat, double resolution, const std::string& frame_id,
                    uint64_t stamp, float cos_scaling = 1.0) const;

  void visualizeRollout(const std::unique_ptr<OptimizationResult<Model>>& optimization_result,
                        const ros::Time& stamp) const;
  void visualizeSignedDistanceField(const std::unique_ptr<cv::Mat>& signed_distance_field,
                                    const ros::Time& stamp) const;

  visualization_msgs::Marker toLineStrip(const std::vector<State>& states, int id, float width, float r, float g,
                                         float b, float a, const ros::Time& stamp) const;

  void getControls(const ros::Time& stamp);
  void pathCallback(const nav_msgs::PathConstPtr& path);
  void odomCallback(const nav_msgs::OdometryConstPtr& odom);
  void wheelOdomCallback(const igvc_msgs::velocity_pair& velocity_pair);

  void executeControls(igvc_msgs::velocity_pair controls, const ros::Time& stamp);

  void testController(float starting_yaw, float cos_scaling);

  bool debug_;

  double sampled_width_;
  double weighted_width_;

  ros::NodeHandle nh;
  ros::NodeHandle pNh;

  ros::Publisher motor_pub_;

  ros::Publisher debug_signed_distance_field_pub_;
  ros::Publisher debug_rollout_pub_;

  ros::Subscriber path_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber wheel_odom_sub_;

  std::unique_ptr<SignedDistanceFieldOptions> sdf_options_;
  std::unique_ptr<TrajectoryController> controller_;

  nav_msgs::PathConstPtr path_;
  std::optional<RobotState> state_;
};
}  // namespace ros_trajectory_controller

#endif  // SRC_ROS_TRAJECTORY_CONTROLLER_H
