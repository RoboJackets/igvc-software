#include "SmoothControl.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos,
                                  Eigen::Vector3d target)
{
  double dt = rollOutTime / 10;
  for (int i = 0; i < 15; i++)
  {
    // std::cout << "cur_pos = " << cur_pos[0] << ", " << cur_pos[1] << ", " << cur_pos[2] << std::endl;
    // std::cout << "target = " << target[0] << ", " << target[1] << ", " << target[2] << std::endl;
    Eigen::Vector3d action;
    getAction(action, cur_pos, target);
    action[2] = dt;
    // std::cout << "action = " << action[0] << ", " << action[1] << ", " << action[2] << std::endl;

    if (i == 0)
    {
      vel.left_velocity = action[0] - action[1] * axle_length / 2;
      vel.right_velocity = action[0] + action[1] * axle_length / 2;
    }

    Eigen::Vector3d end_pos;
    getResult(end_pos, cur_pos, action);
    // std::cout << "end_pos = " << end_pos[0] << ", " << end_pos[1] << ", " << end_pos[2] << std::endl;

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
    // std::cout << std::endl;
    cur_pos = end_pos;
  }
  // std::cout << "\n end" << std::endl;
}

void SmoothControl::getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d target)
{
  double delta, theta;
  Eigen::Vector3d slope = target - cur_pos;
  // std::cout << "atan2 = " << atan2(slope[1], slope[0]) << std::endl;
  if (slope[0] == 0)
  {
    delta = slope[1] > 0 ? -M_PI / 2 : M_PI / 2;
    delta += cur_pos[2];
  }
  else
  {
    delta = -atan2(slope[1], slope[0]) + cur_pos[2];
  }
  while (delta > M_PI)
  {
    delta -= 2 * M_PI;
  }
  while (delta < -M_PI)
  {
    delta += 2 * M_PI;
  }

  slope = cur_pos - target;
  if (slope[0] == 0)
  {
    theta = slope[1] > 0 ? M_PI / 2 : -M_PI / 2;
    theta -= target[2];
  }
  else
  {
    theta = atan2(slope[1], slope[0]) - target[2];
  }
  theta += M_PI;
  theta = -theta;
  while (theta > M_PI)
  {
    theta -= 2 * M_PI;
  }
  while (theta < -M_PI)
  {
    theta += 2 * M_PI;
  }

  // std::cout << "slope = "<< slope[0] << ", " << slope[1] << std::endl;
  // std::cout << "theta = " << theta << " delta = " << delta << std::endl;

  double d = sqrt(pow(target[0] - cur_pos[0], 2) + pow(target[1] - cur_pos[1], 2));

  double K = k2 * (delta - atan(k1 * theta));
  K += (1 + (k1 / (1 + pow(k1 * theta, 2)))) * sin(delta);
  K /= -d;
  double w = K * v;
  result[0] = v;
  result[1] = w;
}

void SmoothControl::getResult(Eigen::Vector3d& result, Eigen::Vector3d cur_pos, Eigen::Vector3d action)
{
  // std::cout << "get result v = " << action[0] << " w = " << action[1] << std::endl;
  double v = action[0];
  double w = action[1];
  double dt = action[2];
  // std::cout << "abs(w)" << std::abs(w) << " < " << 1e-10 << std::endl;
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
    // std::cout << "result theta = " << c[2] << std::endl;
    while (result[2] < -M_PI)
      result[2] += 2 * M_PI;
    while (result[2] > M_PI)
      result[2] -= 2 * M_PI;
  }
  else
  {
    // std::cout << "straight" << std::endl;
    result[0] = cur_pos[0] + (cos(cur_pos[2]) * v * dt);
    result[1] = cur_pos[1] + (sin(cur_pos[2]) * v * dt);
    result[2] = cur_pos[2];
  }
}
