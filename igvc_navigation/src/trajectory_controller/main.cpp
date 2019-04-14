#include <ros/ros.h>

#include "ros_trajectory_controller.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "trajectory_controller");
  ros::NodeHandle nh;

  using ros_trajectory_controller::ROSTrajectoryController;

  ROSTrajectoryController ros_trajectory_controller{};

  return 0;
}
