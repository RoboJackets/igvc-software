#include "motion_profiler.h"
#include <tf/transform_datatypes.h>

MotionProfiler::MotionProfiler(double axle_length, const WheelConstraint& wheel_constraint,
                               const RobotConstraint& robot_constraint,
                               const MotionProfilerOptions& motion_profiler_options, double target_velocity,
                               double linear_acceleration_curvature_threshold_)
  : wheel_constraint_{ wheel_constraint }
  , robot_constraint_{ robot_constraint }
  , motion_profiler_options_{ motion_profiler_options }
  , axle_length_{ axle_length }
  , target_velocity_{ target_velocity }
  , linear_acceleration_curvature_threshold_{ linear_acceleration_curvature_threshold_ }
{
}

void MotionProfiler::profileTrajectory(const igvc_msgs::trajectoryPtr& trajectory_ptr, const RobotState& state)
{
  // For now, a simple "motion profile" where velocity is a function of curvature. ie. as curvature increases,
  // velocity decreases, as well as acceleration if the curvature is small
  trajectory_ptr->trajectory.front().header.stamp = trajectory_ptr->header.stamp;

  for (size_t i = 0; i < trajectory_ptr->trajectory.size(); i++)
  {
    igvc_msgs::trajectory_point& trajectory_point = trajectory_ptr->trajectory[i];

    double velocity = calculateVelocity(trajectory_point.curvature);

    // Constraint checking and limiting
    if (velocity > robot_constraint_.velocity)
    {
      velocity = robot_constraint_.velocity;
    }

    if (i == 0)
    {
      trajectory_point.velocity = state.linearVelocity();
    }
    else
    {
      trajectory_point.velocity = velocity;
    }

    if (i > 0)
    {
      const igvc_msgs::trajectory_point& last_point = trajectory_ptr->trajectory[i - 1];
      double distance = RobotState::getArcLength(last_point, trajectory_point);

      capLinearAcceleration(last_point, trajectory_point);

      double average_speed = (last_point.velocity + trajectory_point.velocity) / 2;
      ros::Duration move_duration = ros::Duration(distance / std::abs(average_speed));

      trajectory_point.header.stamp = last_point.header.stamp + move_duration;
    }
  }
}

void MotionProfiler::capLinearAcceleration(const igvc_msgs::trajectory_point& last,
                                           igvc_msgs::trajectory_point& cur) const
{
  if (std::abs(last.curvature) < linear_acceleration_curvature_threshold_)
  {
    double dv = cur.velocity - last.velocity;
    double dt = (cur.header.stamp - last.header.stamp).toSec();

    double a = dv / dt;

    if (std::abs(a) > robot_constraint_.acceleration)
    {
      double capped_velocity = last.velocity + std::copysign(robot_constraint_.acceleration, dv) * dt;
      ROS_WARN_STREAM_THROTTLE(1, "K: " << cur.curvature << ". Desired a: " << a << ", limiting to "
                                        << robot_constraint_.acceleration << ", desired velocity: " << cur.velocity
                                        << ", capping to " << capped_velocity);
      cur.velocity = capped_velocity;
    }
  }
}

double MotionProfiler::calculateVelocity(double curvature) const
{
  const auto [beta, lambda] = motion_profiler_options_;
  return target_velocity_ / (1 + beta * std::pow(std::abs(curvature), lambda));
}
