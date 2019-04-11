#ifndef SRC_TRAJECTORY_UTILS_H
#define SRC_TRAJECTORY_UTILS_H
#include <memory>
#include <opencv2/opencv.hpp>
#include <nav_msgs/Path.h>

struct SignedBitFieldOptions {
  int grid_rows; // in cells, centered
  int grid_cols;
  int cv_type;
  int grid_x;
  int grid_y;
  int path_start_idx;
  int path_end_idx;
  int max_iterations{50000};
};

namespace trajectory_utils {
  struct Node {
    int row;
    int col;
    int weight;
  };

  struct CompareNode {
    inline bool operator()(const Node& lhs, const Node& rhs) const {
      return lhs.weight - rhs.weight;
    }
  };

  /**
   * Calculates the signedDistanceField for the path passed in, creating a grid of size grid_width, grid_height for
   * waypoints from path_start_idx to path_end_idx
   * @param path path to calculate signedDistanceField for.
   * @param options options for creating the signedDistanceField
   * @return the created signedDistanceField
   */
  cv::Mat getSignedDistanceField(const nav_msgs::Path& path, SignedBitFieldOptions options);

  /**
   * Returns a vector of Nodes between start (inclusive) and end (exclusive) using Bressenham's line algorithm,
   * with weights initialized to 0.
   * @param start start node
   * @param end end node
   * @return vector of nodes between start and end exclusive.
   */
  std::vector<Node> getNodesBetweenWaypoints(Node&& start, Node&& end);

  std::vector<Node> getAdjacentNodes(const Node& node, int grid_rows, int grid_cols);

  template <class T>
  inline int toGrid(T coord, T grid_resolution) {
    return std::round(coord / grid_resolution);
  }
}

#endif //SRC_TRAJECTORY_UTILS_H
