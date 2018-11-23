#include "SmoothControl.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory,
        Eigen::Vector3d cur_pos, Eigen::Vector3d target)
{
  double dt = rollOutTime / 10;
  for (int i = 0; i < 15; i++)
  {
    Eigen::Vector3d action;
    getAction(action, cur_pos, target);
    action[2] = dt;

    if (i == 0)
    {
      vel.left_velocity = action[0] - action[1] * axle_length / 2;
      vel.right_velocity = action[0] + action[1] * axle_length / 2;
    }

    Eigen::Vector3d end_pos;
    getResult(end_pos, cur_pos, action);

    if (i == 0)
    {
      geometry_msgs::PoseStamped start;
      start.pose.position.x = cur_pos[0];
      start.pose.position.y = cur_pos[1];
      trajectory.poses.push_back(start);
    }

    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = end_pos[0];
    pose.pose.position.y = end_pos[1];
    trajectory.poses.push_back(pose);
    cur_pos = end_pos;
  }
}

void SmoothControl::getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d target)
{
  /**
  Computes the radius of curvature to obtain a new angular velocity value.
  */
  double delta = cur_pos[2], theta = target[2];

  // adjust both angles to lie between -PI and PI
  // fitToPolar(delta);
  // fitToPolar(theta);

  // calculate the radius of curvature, K
  double d = sqrt(pow(target[0] - cur_pos[0], 2) + pow(target[1] - cur_pos[1], 2));
  double K = k2 * (delta - atan(-k1 * theta));
  K += (1 + (k1 / (1 + pow(k1 * theta, 2)))) * sin(delta);
  K /= -d;

  // calculate angular velocity using radius of curvature
  double w = K * v;
  result[0] = v;
  result[1] = w;
}

void SmoothControl::getResult(Eigen::Vector3d& result, Eigen::Vector3d cur_pos,
            Eigen::Vector3d action)
{
  /**
  Obtains the resultant pose given the current velocity and angular velocity command.
  */
  double v = action[0];
  double w = action[1];
  double dt = action[2];

  if (std::abs(w) > 1e-10)
  {
    double R = v / w;
    double ICCx = cur_pos[0] - (R * sin(cur_pos[2]));
    double ICCy = cur_pos[1] + (R * cos(cur_pos[2]));
    using namespace Eigen;
    Matrix3d T;
    double wdt = w * dt;
    T << cos(wdt), -sin(wdt), 0, sin(wdt), cos(wdt), 0, 0, 0, 1;
    Vector3d a(cur_pos[0] - ICCx, cur_pos[1] - ICCy, cur_pos[2]);
    Vector3d b = T * a;
    Vector3d c = b + Vector3d(ICCx, ICCy, wdt);

    result[0] = c[0];
    result[1] = c[1];
    result[2] = c[2];

    fitToPolar(result[2]);
  }
  else
  {
    result[0] = cur_pos[0] + (cos(cur_pos[2]) * v * dt);
    result[1] = cur_pos[1] + (sin(cur_pos[2]) * v * dt);
    result[2] = cur_pos[2];
  }
}

void SmoothControl::fitToPolar(double& angle) {
    /**
    Adjust angle to lie within the polar range [-PI, PI]
    */
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
}
