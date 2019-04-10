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
  double beta;
  double lambda;
};

class MotionProfiler
{
public:
  MotionProfiler(double axle_length, const WheelConstraint& wheel_constraint, const RobotConstraint& robot_constraint,
                 const MotionProfilerOptions& motion_profiler_options, double target_velocity);

  /**
   * Performs motion profiling on the passed in trajectory, doing a constrained optimization on acceleration
   * @param trajectory_ptr the trajectory to perform motion profiling on
   */
  void profileTrajectory(const igvc_msgs::trajectoryPtr& trajectory_ptr, const RobotState& state);

private:
  WheelConstraint wheel_constraint_;
  RobotConstraint robot_constraint_;
  MotionProfilerOptions motion_profiler_options_;

  double axle_length_;
  double target_velocity_;
};

#endif  // SRC_MOTION_PROFILER_H
