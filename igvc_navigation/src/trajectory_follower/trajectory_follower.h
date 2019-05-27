/**
 * Class for following an igvc_mgs/Trajectory published by TrajectoryPlanner.
 * Currently, it's an open loop that simply runs the control command for whatever the TrajectoryPlanner is sent,
 * but can be changed to a closed control loop in the future.
 *
 * Author: Oswin So <oswiso@gmail.com>
 */
#ifndef SRC_TRAJECTORY_FOLLOWER_H
#define SRC_TRAJECTORY_FOLLOWER_H

#include <mutex>

#include <igvc_msgs/trajectory.h>
#include <igvc_msgs/trajectory_point.h>
#include <igvc_utils/robot_control.h>

class TrajectoryFollower
{
public:
  TrajectoryFollower();

private:
  igvc_msgs::trajectoryConstPtr trajectory_;
  ros::Publisher control_pub_;

  std::string path_topic_{};
  double loop_hz_{};
  double axle_length_{};
  double motor_loop_hz_{};
  double min_velocity_{};
  ros::Duration time_delta_{};

  std::mutex trajectory_mutex_;

  void trajectoryCallback(igvc_msgs::trajectoryConstPtr trajectory);

  /**
   * Called in trajectoryFollowLoop. Publishes the command to follow the trajectory, open loop style.
   */
  void followTrajectory();

  /**
   * Calculates the controls required to follow the trajectory, open loop style.
   * @return
   */
  RobotControl getControl();

  /**
   * Ensures that velocities sent to the motor are above the deadband minimum_velocity, otherwise
   * the motor just refuses to move, and may result in the motor oscillating between 0 and some nonzero velocity
   * very quickly
   * @param control
   */
  void ensureAboveDeadband(RobotControl& control);

  /**
   * Main loop for the trajectory following. Finds current time, interpolates curvature and velocity between points.
   */
  void trajectoryFollowLoop();
};

#endif  // SRC_TRAJECTORY_FOLLOWER_H
