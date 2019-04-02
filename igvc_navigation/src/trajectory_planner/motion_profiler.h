#ifndef SRC_MOTION_PROFILER_H
#define SRC_MOTION_PROFILER_H

#include <igvc_msgs/trajectory.h>

namespace motion_profiler
{
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

/**
 * Performs motion profiling on the passed in trajectory, doing a constrained optimization on acceleration
 * @param trajectory_ptr
 */
void profileTrajectory(igvc_msgs::trajectoryPtr trajectory_ptr, const WheelConstraint& wheel_constraint,
                       const RobotConstraint& robot_constraint, double axle_length);

double calculateSlope(double curvature, double axle_length);
}  // namespace motion_profiler

#endif  // SRC_MOTION_PROFILER_H
