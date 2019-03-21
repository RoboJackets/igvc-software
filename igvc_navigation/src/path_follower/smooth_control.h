/**
Implementation of smooth control law derived from:
A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment

Paper found here:

https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
*/

#ifndef SMOOTH_CONTROL_H
#define SMOOTH_CONTROL_H

#define _USE_MATH_DEFINES

#include <Eigen/Dense>
#include <cmath>
#include <optional>

#include <nav_msgs/Path.h>

#include <igvc_msgs/velocity_pair.h>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>

struct Action
{
  double v;
  double w;
  double dt;
};

class SmoothControl
{
public:
  SmoothControl(double k1, double k2, double axle_length, double simulation_frequency, double target_velocity,
                double m_lookahead_dist, double simulation_horizon, double target_reached_distance, double target_move_threshold);
  /**
   * Generate an immediate velocity command and visualize a smooth control trajectory
   * using the procedure described in 'A Smooth Control Law for Graceful Motion of
   * Differential Wheeled Mobile Robots in 2D Environment'. A radius of curvature is
   * generated and then combined with a target velocity to produce a control law for
   * the angular velocity of the robot (steering).
   *
   * @param[out] vel velocity_pair message to store command in
   * @param[in] path path to generate smooth control law for
   * @param[out] trajectory msg to store visualization trajectory in
   * @param[in] cur_pos current position of the robot
   * @param[out] target the target pose the controller is planning for
   */
  void getTrajectory(igvc_msgs::velocity_pair& vel, const nav_msgs::PathConstPtr& path, nav_msgs::Path& trajectory,
                     const RobotState& cur_pos, RobotState& target);

private:
  double k1_, k2_;
  double axle_length_;
  double simulation_frequency_;
  double target_velocity_;
  double lookahead_dist_;
  double simulation_horizon_;
  double target_reached_distance_;
  double target_move_threshold_;
  ros::Publisher target_pub_;
  ros::Publisher closest_point_pub_;
  std::optional<RobotState> target_;

  /**
   * Computes the radius of curvature to obtain a new angular velocity value.
   * @param[in] state current state of the robot
   * @param[in] target target state of the robot
   * @return A control command for the next iteration
   */
  Action getAction(const RobotState& state, const RobotState& target);

  /**
   * Propogates the current state given the current velocity and angular velocity
   * command for visualization purposes.
   * angles [delta, theta].
   *
   * Makes extensive use of differential drive odometry equations to calculate resultant
   * pose.
   *
   * Source: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
   *
   * @param[in] action Trajectory action to visualize
   * @param[in/out] state The state to use for state propogation
   */
  void propogateState(const Action& action, RobotState& state);

  /**
   * Returns if we have reached the target, given the current state and the target
   * @param state state to check for
   * @param target target to check for
   * @return whether or not we have reached the target
   */
  bool reachedTarget(const RobotState& state, const RobotState& target) const;

  /**
   * Find the furthest point along trajectory which is lookahead_dist_ away from the current position, interpolating
   * between points, which is the target position.
   *
   * @param[in] path path to get target position from
   * @param[in] path_index index of closest position along the path relative to current position
   * @param[in] state current state of the robot
   * @return the target position
   */
  RobotState getTargetPosition(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                              const std::optional<RobotState>& target) const;

  /**
   * Acquires a new target which is lookahead_dist_ away from the current position, interpolating between points
   * on the path
   * @param path path from which to find a target position
   * @param state current state of the robot
   * @return the newly acquired target position
   */
  RobotState acquireNewTarget(const nav_msgs::PathConstPtr& path, const RobotState& state) const;

  /**
   * Finds the point closest to the old target on the current path. If the distance is greater
   * than new_target_threshold_, std::nullopt is returned.
   * @return the found target if within the threshold, or std::nullopt
   */
  std::optional<RobotState> findTargetOnPath(const nav_msgs::PathConstPtr& path, const RobotState& target) const;

  /**
   * Finds the distance along the path provided from the current state to the target state
   * @param path the path to calculate the distance along
   * @param state the current state
   * @param target the target state
   * @return the distance from the current state to the target state along the given path
   */
  double distAlongPath(const nav_msgs::PathConstPtr& path, const RobotState& state, const RobotState& target) const;

  /**
   * Converts velocity and angular velocity to left and right wheel velocities
   * @param[in] vel_msg message to store wheel velocities in
   * @param[in/out] action Action to convert from
   */
  void getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, const Action& action) const;
};

#endif
