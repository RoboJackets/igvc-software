#include "SmoothControl.h"

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::Path& trajectory,
        Eigen::Vector3d cur_pos, Eigen::Vector3d target, Eigen::Vector2d egocentric_heading)
{
  double dt = rollOutTime / 10;
  for (int i = 0; i < 15; i++)
  {
    Eigen::Vector3d action;
    getAction(action, cur_pos, target, egocentric_heading);
    action[2] = dt;

    if (i == 0)
    {
      vel.left_velocity = action[0] - action[1] * axle_length / 2;
      vel.right_velocity = action[0] + action[1] * axle_length / 2;
    }

    Eigen::Vector3d end_pos;
    getResult(end_pos, egocentric_heading, cur_pos, action);

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

void SmoothControl::getTrajectory(igvc_msgs::velocity_pair& vel, nav_msgs::PathConstPtr path,
    nav_msgs::Path& trajectory, Eigen::Vector3d cur_pos)
{
    this->cur_pos << cur_pos;

    // TODO: Create intiialize method
    double pathIndex;
    pathIndex = getClosestPosition(path);
    getTargetPosition(path, path_index);
    getLineOfSightAndHeading();
    getEgocentricAngles(path);

    double dt = rollOutTime/10;

    for (int i = 0; i < 15; i++)
    {
        Eigen::Vector3d action;
        getAction(action);
        action[2] = dt;

        if (i == 0)
        {
          vel.left_velocity = action[0] - action[1] * axle_length / 2;
          vel.right_velocity = action[0] + action[1] * axle_length / 2;
        }
    }
}

void SmoothControl::getAction(Eigen::Vector3d& result)
{
    // get egocentric polar coordinates for delta and theta
    double delta = cur_theta, theta = tar_theta;

    // adjust both angles to lie between -PI and PI
    fitToPolar(delta);
    fitToPolar(theta);

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

/**
Calculate cur_theta, the angle between the los and the current robot heading.

Calculate target theta (tar_theta), the angle between the los and the target
pose.
*/
void SmoothControl::getEgocentricAngles(nav_msgs::PathConstPtr path)
{
    //get egocentric polar angle of los relative to heading
    float cur_theta;
    compute_angle(cur_theta, los, heading);
    this->cur_theta = cur_theta;

    // get i and j components of target orientation vector (res_orientation)
    double distance = 0;
    geometry_msgs::Point point1, point2;
    int path_idx = 0;
    while (path_idx < path->poses.size() - 1)
    {
      point1 = path->poses[path_idx].pose.position;
      point2 = path->poses[path_idx + 1].pose.position;
      double increment = get_distance(
                              point1.x,
                              point1.y,
                              point2.x,
                              point2.y
                          );

      if (distance + increment > lookahead_dist) { break; }

      path_idx++;
      distance += increment;
    }

    double pose_x = point2.x - point1.x;
    double pose_y = point2.y - point1.y;

    Eigen::Vector3d tar_orientation(pose_x, pose_y, 0); // target orientation
    tar_orientation.normalize();

    // get egocentric polar angle of los relative to target orientation
    float tar_theta;
    compute_angle(tar_theta, los, tar_orientation);

    this->tar_theta = tar_theta;

}

/**
Calculate the line of sight (los) from the robot to the target position as
well as the current robot heading in vector format
*/
void SmoothControl::getLineOfSightAndHeading()
{
    double slope_x = target[0] - cur_pos[0];
    double slope_y = target[1] - cur_pos[1];

    Eigen::Vector3d los(slope_x, slope_y, 0); // line of sight
    los.normalize();
    this->los << los;

    // get current robot heading in vector format
    Eigen::Vector3d heading(std::cos(cur_pos[2]), std::sin(cur_pos[2]), 0);
    this->heading << heading;
}


/**
Find the furthest point along trajectory that isn't further than the
lookahead distance. This is the target position.
*/
void SmoothControl::getTargetPosition(nav_msgs::PathConstPtr path, double path_index)
{
    // target position
    float tar_x, tar_y;

    geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;

    if (get_distance(cur_pos[0], cur_pos[1], end.x, end.y) > lookahead_dist)
    {
      double distance = 0;
      bool cont = true;

      while (cont && path_index < path->poses.size() - 1)
      {
        geometry_msgs::Point point1, point2;
        point1 = path->poses[path_index].pose.position;
        point2 = path->poses[path_index + 1].pose.position;
        double increment = get_distance(
                              point1.x,
                              point1.y,
                              point2.x,
                              point2.y
                          );

        if (distance + increment > lookahead_dist)
        {
          cont = false;
          Eigen::Vector3d first(point1.x, point1.y, 0);
          Eigen::Vector3d second(point2.x, point2.y, 0);
          Eigen::Vector3d slope = second - first;

          slope /= increment;
          slope *= (distance - lookahead_dist) + increment;

          slope += first;
          tar_x = slope[0];
          tar_y = slope[1];
        }
        else
        {
          path_index++;
          distance += increment;
        }
      }
    }
    else
    {
      tar_x = end.x;
      tar_y = end.y;
    }

    // load target position and arbitrary angle into the motion target vector
    target << tar_x, tar_y, 0;
}

/**
Find the index of the closest point on the path.
*/
int SmoothControl::getClosestPosition(nav_msgs::PathConstPtr path)
{
    geometry_msgs::Point end = path->poses[path->poses.size() - 1].pose.position;
    int path_index = 0;
    double closest = get_distance(
                         cur_pos[0],
                         cur_pos[1],
                         path->poses[0].pose.position.x,
                         path->poses[0].pose.position.y
                      );

    double temp = get_distance(
                      cur_pos[0],
                      cur_pos[1],
                      path->poses[path_index].pose.position.x,
                      path->poses[path_index].pose.position.y
                  );

    while (path_index < path->poses.size() && temp <= closest)
    {
      if (temp < closest)
      {
        closest = temp;
      }
      path_index++;
      temp = get_distance(
                 cur_pos[0],
                 cur_pos[1],
                 path->poses[path_index].pose.position.x,
                 path->poses[path_index].pose.position.y
             );
    }

    return path_index;
}

/**
Computes the radius of curvature to obtain a new angular velocity value.
*/
void SmoothControl::getAction(Eigen::Vector3d& result, Eigen::Vector3d cur_pos,
        Eigen::Vector3d target, Eigen::Vector2d egocentric_heading)
{
  // get egocentric polar coordinates for delta and theta
  double delta = egocentric_heading[0], theta = egocentric_heading[1];

  // adjust both angles to lie between -PI and PI
  fitToPolar(delta);
  fitToPolar(theta);

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

/**
Obtains the resultant pose given the current velocity and angular velocity command.
Makes extensive use of differential drive odometry equations to calculate resultant
pose

Source: http://www8.cs.umu.se/kurser/5DV122/HT13/material/Hellstrom-ForwardKinematics.pdf
*/
void SmoothControl::getResult(Eigen::Vector3d& result, Eigen::Vector2d& egocentric_heading,
        Eigen::Vector3d cur_pos, Eigen::Vector3d action)
{
  double v = action[0];
  double w = action[1];
  double dt = action[2];

  if (std::abs(w) > 1e-10)
  {
    // calculate instantaneous center of curvature
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


    // TODO correct delta and theta updates to implement same heading calculation as in path_follower
    egocentric_heading[0] += wdt; // update delta

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

/* TODO: move these methods into NodeUtils */

/**
Adjust angle to lie within the polar range [-PI, PI]
*/
void SmoothControl::fitToPolar(double& angle) {
    while (angle > M_PI)
      angle -= 2 * M_PI;
    while (angle < -M_PI)
      angle += 2 * M_PI;
}

/**
Calculates euclidian distance between two points
*/
double SmoothControl::get_distance(double x1, double y1, double x2, double y2)
{

  return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

void SmoothControl::compute_angle(float& angle, Eigen::Vector3d vec2, Eigen::Vector3d vec1)
{
  double dot = vec2[0]*vec1[0] + vec2[1]*vec1[1]; // dot product - proportional to cos
  double det = vec2[0]*vec1[1] - vec2[1]*vec1[0]; // determinant - proportional to sin

  angle = atan2(det, dot);
}
