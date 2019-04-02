#include <tf/transform_datatypes.h>
#include <Eigen/src/Core/Matrix.h>
#include "motion_profiler.h"

void motion_profiler::profileTrajectory(igvc_msgs::trajectoryPtr trajectory_ptr, const WheelConstraint& wheel_constraint,
    const RobotConstraint& robot_constraint, double axle_length)
{
  // Pass 1: Forward pass from beginning
  std::vector<double> slopes;
  slopes.reserve(trajectory_ptr->trajectory.size());
  slopes.emplace_back(calculateSlope(trajectory_ptr->trajectory.front().curvature, axle_length));

//  double max_velocity_delta = trajectory_ptr->time_resolution;

  for (size_t i = 0; i < trajectory_ptr->trajectory.size() - 1; i++)
  {
    // slope[i] calculated already. Calcualte slope[i + 1]
    slopes.emplace_back(calculateSlope(trajectory_ptr->trajectory[i + 1].curvature, axle_length));

    // Calculate ds for current segment
    double ds;
    if (std::abs(trajectory_ptr->trajectory[i].curvature) > 1e-10)
    {
      double R = 1/trajectory_ptr->trajectory[i].curvature;
      double yaw = tf::getYaw(trajectory_ptr->trajectory[i].pose.orientation);
      double x1 = trajectory_ptr->trajectory[i].pose.position.x;
      double y1 = trajectory_ptr->trajectory[i].pose.position.y;
      double x2 = trajectory_ptr->trajectory[i+1].pose.position.x;
      double y2 = trajectory_ptr->trajectory[i+1].pose.position.y;
      double ICCx = x1 - (R * sin(yaw));
      double ICCy = y1 + (R * cos(yaw));
      x1 -= ICCx;
      y1 -= ICCy;
      x2 -= ICCx;
      y2 -= ICCy;
      double theta = acos((x1 * x2 + y1 * y2) / (std::hypot(x1, x2) * std::hypot(y1, y2)));
      ds = R * theta;
    } else {
      double x1 = trajectory_ptr->trajectory[i].pose.position.x;
      double y1 = trajectory_ptr->trajectory[i].pose.position.y;
      double x2 = trajectory_ptr->trajectory[i+1].pose.position.x;
      double y2 = trajectory_ptr->trajectory[i+1].pose.position.y;
      ds = std::hypot(x1-x2, y1-y1);
    }

    // Calculate dt based on ds
    double dt = ds / trajectory_ptr->trajectory[i].velocity;

    // Get which wheel should go at max acceleration
    double dm = slopes[i+1] - slopes[i];
    double vr, vl;
    if (dm > 0) {
      // right wheel max acceleration
      vr = wheel_constraint.acceleration;
      vl = slopes[i + 1] * vr;
    } else {
      vl = wheel_constraint.acceleration;
      vr = vl / slopes[i + 1];
    }
    trajectory_ptr->trajectory[i+1].velocity = (vl + vr)/2;
  }
}

double motion_profiler::calculateSlope(double curvature, double axle_length)
{
  return (2 - axle_length * curvature)/(2 + axle_length * curvature);
}
