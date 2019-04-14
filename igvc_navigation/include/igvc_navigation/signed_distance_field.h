#ifndef SRC_TRAJECTORY_UTILS_H
#define SRC_TRAJECTORY_UTILS_H
#include <nav_msgs/Path.h>
#include <memory>
#include <opencv2/opencv.hpp>

#include <igvc_navigation/fast_sweep.h>

namespace signed_distance_field
{
struct SignedDistanceFieldOptions
{
  int grid_rows;  // in cells, centered
  int grid_cols;  // in cells, centered
  float grid_x;   // in m
  float grid_y;   // in m
  float grid_resolution;

public:
  SignedDistanceFieldOptions(int rows, int cols, float x, float y, float resolution);
};

class SignedDistanceField
{
public:
  explicit SignedDistanceField(const SignedDistanceFieldOptions& options);

  std::optional<float> getValue(float x, float y);
  /*
   * Calculates the signedDistanceField for the path passed in, creating a grid of size grid_width, grid_height for
   * waypoints from path_start_idx to path_end_idx
   * @param path path to calculate signedDistanceField for.
   * @param options options for creating the signedDistanceField
   * @param traversal_costs the traversal costs used (ie. obstacles)
   * @param solver the solver to be used to get the SignedDistanceField
   * @return the created signedDistanceField
   */
  void calculate(const nav_msgs::Path& path, size_t path_start, size_t path_end, const cv::Mat& traversal_costs);

  /**
   * Returns a vector of Nodes between start (inclusive) and end (exclusive) using Bressenham's line algorithm,
   * with weights initialized to 0.
   * @param start start node
   * @param end end node
   * @return vector of nodes between start and end exclusive.
   */
  std::vector<fast_sweep::Node> getNodesBetweenWaypoints(const fast_sweep::Node& start, const fast_sweep::Node& end);

  /**
   * Converts from coordinates in odom to a Node given grid start_x, start_y and resolution, passed through the options.
   * In node coordinates, (0, 0) correponds to the top left, with rows and columns growing to the bottom right.
   * @tparam T
   * @param x x coord in odom
   * @param y y coord in odom
   * @param options options used for signedDistanceField
   * @return Node representing the same coordinates
   */
  template <class T>
  inline fast_sweep::Node toNode(T x, T y) const;

  template <class T>
  std::vector<T> toVector(const cv::Mat& mat);

  std::unique_ptr<cv::Mat> toMat() const;

private:
  std::vector<float> field_;
  SignedDistanceFieldOptions options_;
  fast_sweep::FastSweep solver_;
};

template <class T>
inline fast_sweep::Node SignedDistanceField::toNode(T x, T y) const
{
  int half_width = (options_.grid_cols - 1) / 2;
  int half_height = (options_.grid_rows - 1) / 2;
  int node_x = static_cast<int>(std::round((x - options_.grid_x + half_width) / options_.grid_resolution));
  int node_y = static_cast<int>(std::round(((options_.grid_y - y) + half_height) /
                                           options_.grid_resolution));  // Flip, since Node has y increasing downwards
  return { node_x, node_y };
}

template <class T>
std::vector<T> SignedDistanceField::toVector(const cv::Mat& mat)
{
  std::vector<T> vec;
  if (mat.isContinuous())
  {
    vec.assign((T*)mat.datastart, (T*)mat.dataend);
  }
  else
  {
    for (int i = 0; i < mat.rows; ++i)
    {
      vec.insert(vec.end(), mat.ptr<T>(i), mat.ptr<T>(i) + mat.cols);
    }
  }
  return vec;
}

}  // namespace signed_distance_field

#endif  // SRC_TRAJECTORY_UTILS_H
