#include "motion_profiler.h"
#include <tf/transform_datatypes.h>

MotionProfiler::MotionProfiler(double axle_length, const WheelConstraint& wheel_constraint,
                               const RobotConstraint& robot_constraint,
                               const MotionProfilerOptions& motion_profiler_options, double target_velocity)
  : wheel_constraint_{ wheel_constraint }
  , robot_constraint_{ robot_constraint }
  , motion_profiler_options_{ motion_profiler_options }
  , axle_length_{ axle_length }
  , target_velocity_{ target_velocity }
{
}

void MotionProfiler::profileTrajectory(const igvc_msgs::trajectoryPtr& trajectory_ptr)
{
  // For now, a simple "motion profile" where velocity is a function of curvature. ie. as curvature increases,
  // velocity decreases.
  trajectory_ptr->trajectory.front().header.stamp = trajectory_ptr->header.stamp;

  for (size_t i = 0; i < trajectory_ptr->trajectory.size(); i++)
  {
    igvc_msgs::trajectory_point& trajectory_point = trajectory_ptr->trajectory[i];
    double beta = motion_profiler_options_.beta;
    double lambda = motion_profiler_options_.lambda;
    double velocity = target_velocity_ / (1 + beta * std::pow(std::abs(trajectory_point.curvature), lambda));

    // Constraint checking and limiting
    if (velocity > robot_constraint_.velocity) {
      velocity = robot_constraint_.velocity;
    }

    trajectory_point.velocity = velocity;

    // Caclulate time to next point
    if (i != trajectory_ptr->trajectory.size() - 1)
    {
      double distance;
      if (std::abs(trajectory_point.curvature) > 1e-8) {
        double R = 1/trajectory_point.curvature;
        double current_yaw = tf::getYaw(trajectory_point.pose.orientation);
        double next_yaw = tf::getYaw(trajectory_ptr->trajectory[i+1].pose.orientation);

        double d_theta = std::abs(next_yaw - current_yaw);
        distance = d_theta * R;

        ROS_INFO_STREAM("arclength: " << distance << ", velocity: " << velocity << ", duration: " << distance / trajectory_point.velocity);

      } else {
        double dx = trajectory_point.pose.position.x - trajectory_ptr->trajectory[i+1].pose.position.x;
        double dy = trajectory_point.pose.position.y - trajectory_ptr->trajectory[i+1].pose.position.y;
        distance = std::hypot(dx, dy);
      }
      ros::Duration move_duration = ros::Duration(distance / trajectory_point.velocity);

      trajectory_ptr->trajectory[i+1].header.stamp = trajectory_point.header.stamp + move_duration;
    }
  }
}
