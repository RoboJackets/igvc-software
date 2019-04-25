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

#include <igvc_msgs/trajectory.h>
#include <igvc_utils/robot_state.h>
#include <igvc_utils/NodeUtils.hpp>

struct SmoothControlOptions
{
  double k1;
  double k2;
};

struct PathGenerationOptions
{
  double simulation_horizon;
  double simulation_frequency;
  double simulation_velocity;

  int getNumSamples() const
  {
    return static_cast<int>(simulation_horizon * simulation_frequency);
  }
};

struct TargetSelectionOptions
{
  double lookahead_dist;
  double target_reached_distance;
  double target_move_threshold;
};

struct CurvatureBlendingOptions
{
  double blending_distance;
};

class SmoothControl
{
public:
  SmoothControl(SmoothControlOptions smooth_control_options, PathGenerationOptions path_generation_options,
                TargetSelectionOptions target_selection_options, CurvatureBlendingOptions curvature_blending_options,
                double axle_length);
  /**
   * Generate an immediate velocity command and visualize a smooth control trajectory
   * using the procedure described in 'A Smooth Control Law for Graceful Motion of
   * Differential Wheeled Mobile Robots in 2D Environment'. A radius of curvature is
   * generated and then combined with a target velocity to produce a control law for
   * the angular velocity of the robot (steering).
   *
   * @param[in] path path to generate smooth control law for
   * @param[out] trajectory msg to store visualization trajectory in
   * @param[in] start_pos current position of the robot
   * @param[out] target the target pose the controller is planning for
   */
  void getPath(const nav_msgs::PathConstPtr& path, const igvc_msgs::trajectoryPtr& trajectory_ptr,
               const RobotState& start_pos);

private:
  SmoothControlOptions smooth_control_options_;
  PathGenerationOptions path_generation_options_;
  TargetSelectionOptions target_selection_options_;
  CurvatureBlendingOptions curvature_blending_options_;
  double axle_length_;

  ros::Publisher target_pub_;
  ros::Publisher closest_point_pub_;
  std::optional<RobotState> target_;

  /**
   * Computes the control command to be executed for the next dt seconds
   * @param[in] state current state of the robot
   * @param[in] target target state of the robot
   * @param[in] second_target the target state after the next target
   * @param[in] dt the duration for which the generated command will be executed for
   * @return A control command for the next iteration
   */
  RobotControl getControl(const RobotState& state, const RobotState& target, const RobotState& second_target) const;

  /**
   * Returns the curvature of the path from the current state to the target
   * @param state starting state
   * @param target target state
   * @return intantaneous curvature of the path from state to target
   */
  double getCurvature(const RobotState& state, const RobotState& target) const;

  /**
   * Returns if we have reached the target, given the current state and the target
   * @param state state to check for
   * @param target target to check for
   * @return whether or not we have reached the target
   */
  bool reachedTarget(const RobotState& state, const RobotState& target) const;

  /**
   * Find the furthest point along trajectory which is at least lookahead_dist_ away from the current position.
   *
   * @param[in] path path to get target position from
   * @param[in] path_index index of closest position along the path relative to current position
   * @param[in] state current state of the robot
   * @param[in] path_index the index of the closest point on the path to the current state
   * @return the positions of the first and second targets.
   */
  std::pair<RobotState, RobotState> getTargetPosition(const nav_msgs::PathConstPtr& path, const RobotState& state,
                                                      const std::optional<RobotState>& target, size_t path_index) const;

  /**
   * Acquires a new target which is at least lookahead_dist_ away from the current position
   * @param[in] path path from which to find a target position
   * @param[in] state current state of the robot
   * @param[in] state_index the index of the closest point on the path to the current state
   * @return the newly acquired target position
   */
  RobotState acquireNewTarget(const nav_msgs::PathConstPtr& path, const RobotState& state, size_t state_index) const;

  /**
   * Finds the point closest to the old target on the current path. If the distance is greater
   * than new_target_threshold_, std::nullopt is returned.
   * @param path
   * @param target
   * @return the index of the closest point, std::nullopt otherwise
   */
  std::optional<size_t> findTargetOnPath(const nav_msgs::PathConstPtr& path, const RobotState& target) const;

  /**
   * Finds the distance along the path provided from the current state to the target state
   * @param path the path to calculate the distance along
   * @param path_index the index of the current state
   * @param target_index the index of the target state
   * @return the arc length from the current state to the target state
   */
  double distAlongPath(const nav_msgs::PathConstPtr& path, size_t path_index, size_t target_index) const;

  /**
   * Returns the closest point on the path to the given state
   * @param path path to find closest point on
   * @param state state to find closest point on path for
   * @return the index of the path which is closest to the state.
   */
  size_t getClosestIndex(const nav_msgs::PathConstPtr& path, const RobotState& state) const;

  /**
   * Calculates the curvature by blending the curvature by pathing to the current target and the curvature
   * by pathing to the next target, and taking a weighted sum based on distance.
   * @param[in] state current state, which includes current velocity
   * @param[in] k1 curvature of path to follow to the current target
   * @param[in] k2 curvature of path to follow to the next target
   * @return the blended curvature
   */
  double blendCurvature(double distance, double K1, double K2) const;
};

#endif
