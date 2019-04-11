#include "trajectory_utils.h"

namespace trajectory_utils {
  cv::Mat getSignedDistanceField(const nav_msgs::Path& path, SignedBitFieldOptions options, double grid_resolution) {
    cv::Mat field(options.grid_rows, options.grid_cols, options.cv_type, cvScalar(0));

    std::vector<bool> visited(options.grid_cols * options.grid_rows);
    std::priority_queue<Node, std::vector<Node>, CompareNode> priority_queue;

    for (int i = options.path_start_idx; i < options.path_end_idx - 1; i++) {
      // Bressenham's line algorithm to add all nodes into the grid
      geometry_msgs::PoseStamped start = path.poses[i];
      geometry_msgs::PoseStamped end = path.poses[i + 1];

      int start_x = toGrid(start.pose.position.x, grid_resolution) - options.grid_x;
      int start_y = toGrid(start.pose.position.y, grid_resolution) - options.grid_y;
      int end_x = toGrid(end.pose.position.x, grid_resolution) - options.grid_x;
      int end_y = toGrid(end.pose.position.y, grid_resolution) - options.grid_y;

      for (auto &node : getNodesBetweenWaypoints({start_x, start_y, 0}, {end_x, end_y, 0})) {
        priority_queue.emplace(node);
      }
    }

    int iterations = 0;
    while (!priority_queue.empty() && iterations <= options.max_iterations) {
      Node cur = priority_queue.top();
      priority_queue.pop();
      if (visited[cur.row * options.grid_rows + cur.col])
        continue;
      visited[cur.row * options.grid_rows + cur.col] = true;

      for (Node& node : getAdjacentNodes(cur, options.grid_rows, options.grid_cols)) {
        if (!visited[node.row * options.grid_rows + node.col] && (cur.weight + 1) < field.at<int>(node.row, node.col)) {
          field.at<int>(node.row, node.col) = cur.weight + 1;
          node.weight = cur.weight + 1;
          priority_queue.emplace(node);
        }
      }
    }
    return field;
  }

  std::vector<Node> getNodesBetweenWaypoints(Node&& start, Node&& end) {
    std::vector<Node> out;

    int m_new = 2 * (end.row - start.row);
    int slope_error_new = m_new - (end.col - start.col);
    for (int x = start.col, y = end.col; x < end.col; x++)
    {
      out.emplace_back(x, y, 0);
      slope_error_new += m_new;

      if (slope_error_new >= 0)
      {
        y++;
        slope_error_new  -= 2 * (end.col - start.col);
      }
    }
    return out;
  }

  std::vector<Node> getAdjacentNodes(const Node& node, int grid_rows, int grid_cols) {
    std::vector<Node> adjacent_nodes;
    if (node.row + 1 < grid_rows) {
      adjacent_nodes.emplace_back(node.row + 1, node.col, 0);
    }
    if (node.row - 1 > 0) {
      adjacent_nodes.emplace_back(node.row - 1, node.col, 0);
    }
    if (node.col + 1 < grid_cols) {
      adjacent_nodes.emplace_back(node.row, node.col + 1, 0);
    }
    if (node.col - 1 > 0) {
      adjacent_nodes.emplace_back(node.row, node.col - 1, 0);
    }
    return adjacent_nodes;
  }
}
