#include <ros/ros.h>

#include <igvc_navigation/signed_distance_field.h>

namespace signed_distance_field
{
SignedDistanceFieldOptions::SignedDistanceFieldOptions(int rows, int cols, double x, double y, double resolution)
  : grid_rows{ rows }, grid_cols{ cols }, grid_x{ x }, grid_y{ y }, grid_resolution{ resolution }
{
  if (grid_rows % 2 == 0 || grid_cols % 2 == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1, "SignedDistanceFieldOptions_dimensionsMustBeOdd", "Grid dimensions must be odd");
  }
}

cv::Mat getSignedDistanceField(const nav_msgs::Path& path, int path_start, int path_end,
                               const SignedDistanceFieldOptions& options, const cv::Mat& traversal_costs,
                               fast_sweep::FastSweep& solver)
{
  cv::Mat field(options.grid_rows, options.grid_cols, CV_32F, cvScalar(std::numeric_limits<float>::max()));
  std::vector<fast_sweep::Node> gamma_points;  // TODO: Reserve?
  for (int i = 0; i < path.poses.size() - 1; i++)
  {
    double start_x = path.poses[i].pose.position.x;
    double start_y = path.poses[i].pose.position.y;
    double end_x = path.poses[i + 1].pose.position.x;
    double end_y = path.poses[i + 1].pose.position.y;

    std::vector<fast_sweep::Node> line =
        getNodesBetweenWaypoints(toNode(start_x, start_y, options), toNode(end_x, end_y, options));
    gamma_points.insert(gamma_points.end(), line.begin(), line.end());
  }

  std::vector<float> solution = solver.solveEikonal(gamma_points, toVector<float>(traversal_costs));

  return toMat(solution, options.grid_rows);
}

std::vector<fast_sweep::Node> getNodesBetweenWaypoints(const fast_sweep::Node& start, const fast_sweep::Node& end)
{
  std::vector<fast_sweep::Node> out;

  int m_new = 2 * (end.y - start.y);
  int slope_error_new = m_new - (end.x - start.x);
  for (int x = start.x, y = start.y; x <= end.x; x++)
  {
    out.emplace_back(fast_sweep::Node{ x, y });
    slope_error_new += m_new;

    if (slope_error_new >= 0)
    {
      y++;
      slope_error_new -= 2 * (end.x - start.x);
    }
  }
  return out;
}

}  // namespace signed_distance_field
