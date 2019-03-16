/**
Implementation of smooth control law derived from:
A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment

Paper found here:

https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
*/

#ifndef SMOOTH_CONTROL_H
#define SMOOTH_CONTROL_H

#define _USE_MATH_DEFINES

#include <igvc_msgs/velocity_pair.h>
#include <nav_msgs/Path.h>
#include <Eigen/Dense>
#include <cmath>
#include <igvc_utils/NodeUtils.hpp>
#include <igvc_utils/RobotState.hpp>

struct Action
{
  double v;
  double w;
  double dt;
};

struct Egocentric_angles
{
  double delta;
  double theta;
};

class SmoothControl
{
public:
  SmoothControl(double k1, double k2, double axle_length, double granularity, double target_velocity,
                double m_lookahead_dist, double simulation_horizon);
  /**
   * Generate an immediate velocity command and visualize a smooth control trajectory
   * using the procedure described in 'A Smooth Control Law for Graceful Motion of
   * Differential Wheeled Mobile Robots in 2D Environment'. A radius of curvature is
   * generated and then combined with a target velocity to produce a control law for
   * the angular velocity of the robot (steering).
   *
   * @param[in] vel velocity_pair message to store command in
   * @param[in] path path to generate smooth control law for
   * @param[out] trajectory msg to store visualization trajectory in
   * @param[in] cur_pos current position of the robot
   * @param[out] target the target pose the controller is planning for
   */
  void getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path, nav_msgs::Path& trajectory,
                     const RobotState& cur_pos, RobotState& target);

private:
  double k1_, k2_;
  double axle_length_;
  double granularity_;
  double target_velocity_;
  double lookahead_dist_;
  double simulation_horizon_;

  /**
   * Computes the radius of curvature to obtain a new angular velocity value.
   * @param[in] delta angle between current robot heading and line of sight
   * @param[in] theta angle betweeen line of sight and target heading
   * @param[in] state current state of the robot
   * @param[in] target target state of the robot
   * @return A control command for the next iteration
   */
  Action getAction(double delta, double theta, const RobotState& state, const RobotState& target);

  /**
   * Estimates the resultant pose given the current velocity and angular velocity
   * command for visualization purposes. Additionally, updates the egocentric polar
   * angles [delta, theta].
   *
   * Makes extensive use of differential drive odometry equations to calculate resultant
   * pose.
   *
   * Source: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
   *
   * @param[in] action Trajectory action to visualize
   * @param[in/out] state The state to use for state propopgation
   */
  void propogateState(const Action& action, RobotState& state);

  /**
   * Find the index of the closest point on the path relative to the current position.
   *
   * @param[in] path path to get closest position from
   */
  unsigned int getClosestPosition(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                  unsigned int start_index = 0);

  /**
   * Find the furthest point along trajectory that isn't further than the
   * lookahead distance. This is the target position.
   *
   * @param[in] path path to get target position from
   * @param[in] path_index index of closest position along the path relative to current position
   * @param[in] state current state of the robot
   * @return the target position
   */
  Eigen::Vector3d getTargetPosition(const nav_msgs::PathConstPtr& path, unsigned int path_index,
                                    const RobotState& state);

  /**
   * Calculate the line of sight (los) from the robot to the target position as
   * well as the current robot heading in vector format
   * @param[out] los line of sight
   * @param[out] heading current robot heading
   * @param[in] state current state of the robot
   * @param[in] target current target that the robot is pathing to
   */
  void getLOSandHeading(Eigen::Vector3d& los, Eigen::Vector3d& heading, const RobotState& state,
                        const RobotState& target);

  /**
   * Calculates egocentric polar angles for smooth control law calculations:
   *     - delta, the angle between the line of sight and the current robot heading.
   *     - theta, the angle between the line of sight and the target heading
   *
   * @param[in] path Graph search generate path to use when calculating angles
   * @param[in] path_index index of closest position in the path
   * @param[in] los line of sight
   * @param[in] heading current robot heading
   * @return a struct containing delta and theta
   */
  Egocentric_angles getEgocentricAngles(const nav_msgs::PathConstPtr& path, unsigned int path_index,
                                        const Eigen::Vector3d& los, const Eigen::Vector3d& heading);

  /**
   * Converts velocity and angular velocity to left and right wheel velocities
   * @param[in] vel_msg message to store wheel velocities in
   * @param[in/out] action Action to convert from
   */
  void getWheelVelocities(igvc_msgs::velocity_pair& vel_msg, const Action& action) const;
};

#endif
