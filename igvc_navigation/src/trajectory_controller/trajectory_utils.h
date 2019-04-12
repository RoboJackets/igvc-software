#ifndef SRC_TRAJECTORY_UTILS_H
#define SRC_TRAJECTORY_UTILS_H
#include <nav_msgs/Path.h>
#include <memory>
#include <opencv2/opencv.hpp>

struct SignedDistanceFieldOptions
{
  int grid_rows;  // in cells, centered
  int grid_cols;
  int cv_type;
  int grid_x;  // in cells
  int grid_y;
  int max_iterations;
};

namespace trajectory_utils
{
struct Node
{
  int row;
  int col;
  int weight;

  friend std::ostream& operator<<(std::ostream& out, const Node& node);
};

/**
 * Compares lhs and rhs
 * used in the priority queue so that smallest weight is at the top
 */
struct CompareNode
{
  inline bool operator()(const Node& lhs, const Node& rhs) const
  {
    return lhs.weight > rhs.weight;
  }
};

/**
 * Calculates the signedDistanceField for the path passed in, creating a grid of size grid_width, grid_height for
 * waypoints from path_start_idx to path_end_idx
 * @param path path to calculate signedDistanceField for.
 * @param options options for creating the signedDistanceField
 * @return the created signedDistanceField
 */
cv::Mat getSignedDistanceField(const nav_msgs::Path& path, int path_start, int path_end,
                               SignedDistanceFieldOptions options, cv::Mat traversal_costs, double grid_resolution);

/**
 * Returns a vector of Nodes between start (inclusive) and end (exclusive) using Bressenham's line algorithm,
 * with weights initialized to 0.
 * @param start start node
 * @param end end node
 * @return vector of nodes between start and end exclusive.
 */
std::vector<Node> getNodesBetweenWaypoints(const Node& start, const Node& end);

std::vector<Node> getAdjacentNodes(const Node& node, int grid_rows, int grid_cols);

template <class T>
inline int toGrid(T coord, T grid_resolution)
{
  return std::round(coord / grid_resolution);
}
}  // namespace trajectory_utils

#endif  // SRC_TRAJECTORY_UTILS_H
