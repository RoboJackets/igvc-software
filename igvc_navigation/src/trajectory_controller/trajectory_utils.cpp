#include "trajectory_utils.h"
#include <ros/ros.h>

namespace trajectory_utils
{
cv::Mat getSignedDistanceField(const nav_msgs::Path& path, int path_start, int path_end,
                               SignedDistanceFieldOptions options, cv::Mat traversal_costs, double grid_resolution)
{
  cv::Mat field(options.grid_rows, options.grid_cols, options.cv_type, cvScalar(255u));

  std::vector<bool> visited(options.grid_cols * options.grid_rows);
  std::priority_queue<Node, std::vector<Node>, CompareNode> priority_queue(field);

  if (path_end - path_start == 0)
  {
    geometry_msgs::PoseStamped start = path.poses.front();

    int start_x = toGrid(start.pose.position.x, grid_resolution) + (options.grid_x - options.grid_cols / 2);
    int start_y = toGrid(start.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows / 2);
    priority_queue.emplace(Node{ start_x, start_y });
    field.at<uchar>(start_y, start_x) = 0;
  }
  else
  {
    for (int i = path_start; i < path_end; i++)
    {
      // Bressenham's line algorithm to add all nodes into the grid
      geometry_msgs::PoseStamped start = path.poses.at(i);
      geometry_msgs::PoseStamped end = path.poses.at(i + 1);

      int start_x = toGrid(start.pose.position.x, grid_resolution) + (options.grid_y - options.grid_rows / 2);
      int start_y = toGrid(start.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows / 2);
      int end_x = toGrid(end.pose.position.x, grid_resolution) + (options.grid_y - options.grid_rows / 2);
      int end_y = toGrid(end.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows / 2);

      for (auto& node : getNodesBetweenWaypoints({ start_x, start_y }, { end_x, end_y }))
      {
        ROS_INFO_STREAM("node in line: " << node.col << ", " << node.row);
        priority_queue.emplace(node);
        field.at<uchar>(node.row, node.col) = 0;
      }
    }
  }

  int iterations = 0;
  while (!priority_queue.empty() && iterations <= options.max_iterations)
  {
    ROS_INFO_STREAM("pq size: " << priority_queue.size());
    ROS_INFO_STREAM("distances: \n" << field);
    Node cur = priority_queue.top();
    priority_queue.pop();
    ROS_INFO_STREAM("");
    ROS_INFO_STREAM("top is " << cur);
    if (visited[cur.row * options.grid_rows + cur.col])
    {
      ROS_INFO_STREAM("\talready visited, next");
      continue;
    }
    visited[cur.row * options.grid_rows + cur.col] = true;
    ROS_INFO_STREAM("\tmarking as visited");

    for (Node& node : getAdjacentNodes(cur, options.grid_rows, options.grid_cols))
    {
      ROS_INFO_STREAM("\t\tadjacent is " << node);
      ROS_INFO_STREAM("\t\t\tvisited: " << visited[node.row * options.grid_rows + node.col]);
      ROS_INFO_STREAM("\t\t\tfield distance: " << static_cast<unsigned int>(field.at<uchar>(node.row, node.col)));
      if (!visited[node.row * options.grid_rows + node.col])
      {
        updateWeightsFromNeighbours(node, field, traversal_costs.at<uchar>(node.row, node.col));
        priority_queue.emplace(node);
      }
    }
  }
  //  ROS_INFO_STREAM(">>Ending<<");
  return field;
}

void updateWeightsFromNeighbours(const Node& node, cv::Mat field, uchar traversal_cost)
{
  uchar up = 255u;
  uchar down = 255u;
  uchar left = 255u;
  uchar right = 255u;

  if (node.row + 1 < field.rows)
  {
    down = field.at<uchar>(node.row + 1, node.col);
  }
  if (node.row - 1 >= 0)
  {
    up = field.at<uchar>(node.row - 1, node.col);
  }
  if (node.col + 1 < field.cols)
  {
    right = field.at<uchar>(node.row, node.col + 1);
  }
  if (node.col - 1 >= 0)
  {
    left = field.at<uchar>(node.row, node.col - 1);
  }
  int dx = std::min(left, right);
  int dy = std::min(up, down);

//  field.at<uchar>(node.row, node.col) = std::min(dx + traversal_cost, dy + traversal_cost);
  double delta = 2 * traversal_cost - (dx - dy) * (dx - dy);
  if (delta >= 0) {
    field.at<uchar>(node.row, node.col) = (dx + dy + sqrt(delta))/2;
  } else {
    field.at<uchar>(node.row, node.col) = std::min(dx + traversal_cost, dy + traversal_cost);
  }
}

std::vector<Node> getNodesBetweenWaypoints(const Node& start, const Node& end)
{
  std::vector<Node> out;

  int m_new = 2 * (end.row - start.row);
  int slope_error_new = m_new - (end.col - start.col);
  for (int x = start.col, y = start.row; x <= end.col; x++)
  {
    out.emplace_back(Node{ x, y });
    slope_error_new += m_new;

    if (slope_error_new >= 0)
    {
      y++;
      slope_error_new -= 2 * (end.col - start.col);
    }
  }
  return out;
}

std::vector<Node> getAdjacentNodes(const Node& node, int grid_rows, int grid_cols)
{
  std::vector<Node> adjacent_nodes;
  if (node.row + 1 < grid_rows)
  {
    adjacent_nodes.emplace_back(Node{ node.row + 1, node.col });
  }
  if (node.row - 1 >= 0)
  {
    adjacent_nodes.emplace_back(Node{ node.row - 1, node.col });
  }
  if (node.col + 1 < grid_cols)
  {
    adjacent_nodes.emplace_back(Node{ node.row, node.col + 1 });
  }
  if (node.col - 1 >= 0)
  {
    adjacent_nodes.emplace_back(Node{ node.row, node.col - 1 });
  }
  return adjacent_nodes;
}

std::ostream& operator<<(std::ostream& out, const Node& node)
{
  out << "(" << node.col << ", " << node.row << ")";
  return out;
}
}  // namespace trajectory_utils
