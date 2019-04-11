#include <ros/ros.h>

#include "trajectory_controller.h"

int main(int argc, char *argv[]) {
  ros::init(argc, argv, "trajectory_controller");
  ros::NodeHandle nh;

  TrajectoryController trajectory_controller{};

  return 0;
}
