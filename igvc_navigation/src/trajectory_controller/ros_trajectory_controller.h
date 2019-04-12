#ifndef SRC_ROS_TRAJECTORY_CONTROLLER_H
#define SRC_ROS_TRAJECTORY_CONTROLLER_H
#include <ros/ros.h>
#include <opencv2/core/mat.hpp>

class ROSTrajectoryController
{
public:
  ROSTrajectoryController();

private:
  void testSignedDistanceField();
  void publishAsPCL(const ros::Publisher &pub, const cv::Mat &mat, double resolution, const std::string &frame_id,
                    uint64_t stamp);

  ros::Publisher trajectory_pub_;
};

#endif  // SRC_ROS_TRAJECTORY_CONTROLLER_H
