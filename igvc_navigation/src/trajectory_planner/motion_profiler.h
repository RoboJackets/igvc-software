/**
 * A class which performs "motion profiling" for now,
 * implementing the algorithm described in
 * "A Smooth Control Law for Graceful Motion of Differential Wheeled Mobile Robots in 2D Environment"
 * https://web.eecs.umich.edu/~kuipers/papers/Park-icra-11.pdf
 */
#ifndef SRC_MOTION_PROFILER_H
#define SRC_MOTION_PROFILER_H

#include <igvc_msgs/trajectory.h>
#include <igvc_utils/robot_state.h>

struct WheelConstraint
{
  double velocity;
  double acceleration;
};

struct RobotConstraint
{
  double velocity;
  double acceleration;
};

struct MotionProfilerOptions
{
  double beta;  // Both are taken from the paper.
  double lambda;
};

class MotionProfiler
{
public:
  MotionProfiler(double axle_length, const WheelConstraint& wheel_constraint, const RobotConstraint& robot_constraint,
                 const MotionProfilerOptions& motion_profiler_options, double target_velocity,
                 double linear_acceleration_curvature_threshold_);

  /**
   * Performs motion profiling on the passed in trajectory, doing a constrained optimization on acceleration
   * @param trajectory_ptr the trajectory to perform motion profiling on
   */
  void profileTrajectory(const igvc_msgs::trajectoryPtr& trajectory_ptr, const RobotState& state);

  /**
   * Caps linear acceleration between the last and current trajectory_point by changing the current
   * trajectory_point's velocity, if the curvature of the control last executed is small enough
   * that it can be approximated by a straight line
   * @param last
   * @param cur
   */
  void capLinearAcceleration(const igvc_msgs::trajectory_point& last, igvc_msgs::trajectory_point& cur) const;

  /**
   * Calculates the velocity using the function described in the paper, where velocity is a function of curvature
   * @param curvature
   * @return
   */
  double calculateVelocity(double curvature) const;

private:
  WheelConstraint wheel_constraint_;
  RobotConstraint robot_constraint_;
  MotionProfilerOptions motion_profiler_options_;

  double axle_length_;
  double target_velocity_;
  double linear_acceleration_curvature_threshold_;
};

#endif  // SRC_MOTION_PROFILER_H
