#include <ros/ros.h>

#include <igvc_navigation/signed_distance_field.h>

namespace signed_distance_field
{
SignedDistanceFieldOptions::SignedDistanceFieldOptions(int rows, int cols, float x, float y, float resolution)
  : grid_rows{ rows }, grid_cols{ cols }, grid_x{ x }, grid_y{ y }, grid_resolution{ resolution }
{
  if (grid_rows % 2 == 0 || grid_cols % 2 == 0)
  {
    ROS_ERROR_THROTTLE_NAMED(1, "SignedDistanceFieldOptions_dimensionsMustBeOdd", "Grid dimensions must be odd");
  }
}

void SignedDistanceField::calculate(const nav_msgs::Path& path, size_t path_start, size_t path_end,
                                    const cv::Mat& traversal_costs)
{
  std::vector<fast_sweep::Node> gamma_points;  // TODO: Reserve?
  if (path_end - path_start > 0)
  {
    for (size_t i = path_start; i < path_end; i++)
    {
      double start_x = path.poses[i].pose.position.x;
      double start_y = path.poses[i].pose.position.y;
      double end_x = path.poses[i + 1].pose.position.x;
      double end_y = path.poses[i + 1].pose.position.y;

      fast_sweep::Node start = toNode(start_x, start_y);
      fast_sweep::Node end = toNode(end_x, end_y);

      std::vector<fast_sweep::Node> line = getNodesBetweenWaypoints(start, end);
      gamma_points.insert(gamma_points.end(), line.begin(), line.end());
    }
  }
  else if (path_start - path_end == 0)
  {
    double x = path.poses.front().pose.position.x;
    double y = path.poses.front().pose.position.y;
    gamma_points.emplace_back(toNode(x, y));
  }

  field_ = solver_.solveEikonal(gamma_points, toVector<float>(traversal_costs));
}

std::vector<fast_sweep::Node> SignedDistanceField::getNodesBetweenWaypoints(const fast_sweep::Node& start,
                                                                            const fast_sweep::Node& end)
{
  int x1 = start.x;
  int y1 = start.y;
  int x2 = end.x;
  int y2 = end.y;

  std::vector<fast_sweep::Node> out;

  const bool steep = std::abs(y2 - y1) > std::abs(x2 - x1);
  if (steep)
  {
    std::swap(x1, y1);
    std::swap(x2, y2);
  }
  if (x1 > x2)
  {
    std::swap(x1, x2);
    std::swap(y1, y2);
  }

  const float dx = x2 - x1;
  const float dy = std::abs(y2 - y1);

  float error = dx / 2.0f;
  const int ystep = (y1 < y2) ? 1 : -1;
  int y = y1;

  for (int x = x1; x <= x2; x++)
  {
    if (steep)
    {
      out.emplace_back(fast_sweep::Node{ y, x });
    }
    else
    {
      out.emplace_back(fast_sweep::Node{ x, y });
    }
    error -= dy;
    if (error < 0)
    {
      y += ystep;
      error += dx;
    }
  }
  return out;
}

SignedDistanceField::SignedDistanceField(const SignedDistanceFieldOptions& options)
  : options_{ options }, solver_{ options.grid_rows, options.grid_cols, options.grid_resolution }
{
}

std::optional<float> SignedDistanceField::getValue(float x, float y)
{
  fast_sweep::Node node = toNode(x, y);
  if (node.isValid(options_.grid_rows, options_.grid_cols))
  {
    return solver_.getCell(node);
  }
  return std::nullopt;
}

std::unique_ptr<cv::Mat> SignedDistanceField::toMat() const
{
  std::unique_ptr<cv::Mat> mat = std::make_unique<cv::Mat>(field_, true);
  *mat = mat->reshape(0, options_.grid_rows);
  return mat;
}
}  // namespace signed_distance_field
