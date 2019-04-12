#include "trajectory_utils.h"
#include <ros/ros.h>

namespace trajectory_utils
{
cv::Mat getSignedDistanceField(const nav_msgs::Path& path, int path_start, int path_end,
                               SignedDistanceFieldOptions options, cv::Mat traversal_costs, double grid_resolution)
{
  cv::Mat field(options.grid_rows, options.grid_cols, options.cv_type, cvScalar(255u));

  std::vector<bool> visited(options.grid_cols * options.grid_rows);
  std::priority_queue<Node, std::vector<Node>, CompareNode> priority_queue;

  if (path_end - path_start == 0)
  {
    geometry_msgs::PoseStamped start = path.poses.front();

    int start_x = toGrid(start.pose.position.x, grid_resolution) + (options.grid_x - options.grid_cols/2);
    int start_y = toGrid(start.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows/2);
    priority_queue.emplace(Node{ start_x, start_y, 0 });
    field.at<uchar>(start_y, start_x) = 0;
  }
  else
  {
    for (int i = path_start; i < path_end; i++)
    {
      // Bressenham's line algorithm to add all nodes into the grid
      geometry_msgs::PoseStamped start = path.poses.at(i);
      geometry_msgs::PoseStamped end = path.poses.at(i + 1);

      int start_x = toGrid(start.pose.position.x, grid_resolution) + (options.grid_y - options.grid_rows/2);
      int start_y = toGrid(start.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows/2);
      int end_x = toGrid(end.pose.position.x, grid_resolution) + (options.grid_y - options.grid_rows/2);
      int end_y = toGrid(end.pose.position.y, grid_resolution) + (options.grid_y - options.grid_rows/2);

      for (auto& node : getNodesBetweenWaypoints({ start_x, start_y, 0 }, { end_x, end_y, 0 }))
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
    if (visited[cur.row * options.grid_rows + cur.col]) {
      ROS_INFO_STREAM("\talready visited, next");
      continue;
    }
    visited[cur.row * options.grid_rows + cur.col] = true;
    ROS_INFO_STREAM("\tmarking as visited");
    ROS_INFO_STREAM("\tcur weight: " << cur.weight);

    for (Node& node : getAdjacentNodes(cur, options.grid_rows, options.grid_cols))
    {
      ROS_INFO_STREAM("\t\tadjacent is " << node);
      ROS_INFO_STREAM("\t\t\tvisited: " << visited[node.row * options.grid_rows + node.col]);
      ROS_INFO_STREAM("\t\t\tfield distance: " << static_cast<unsigned int>(field.at<uchar>(node.row, node.col)));
      // TODO: this has to scale with grid resolution
      uchar next_cost = cur.weight + traversal_costs.at<uchar>(node.row, node.col);
      if (!visited[node.row * options.grid_rows + node.col] && next_cost < field.at<uchar>(node.row, node.col))
      {
        field.at<uchar>(node.row, node.col) = next_cost;
        ROS_INFO_STREAM("\t\t\tUpdated node weight to " << next_cost);
        node.weight = next_cost;
        priority_queue.emplace(node);
      }
    }
  }
//  ROS_INFO_STREAM(">>Ending<<");
  return field;
}

std::vector<Node> getNodesBetweenWaypoints(const Node& start, const Node& end)
{
  std::vector<Node> out;

  int m_new = 2 * (end.row - start.row);
  int slope_error_new = m_new - (end.col - start.col);
  for (int x = start.col, y = start.row; x <= end.col; x++)
  {
    out.emplace_back(Node{ x, y, 0 });
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
    adjacent_nodes.emplace_back(Node{ node.row + 1, node.col, 0 });
  }
  if (node.row - 1 >= 0)
  {
    adjacent_nodes.emplace_back(Node{ node.row - 1, node.col, 0 });
  }
  if (node.col + 1 < grid_cols)
  {
    adjacent_nodes.emplace_back(Node{ node.row, node.col + 1, 0 });
  }
  if (node.col - 1 >= 0)
  {
    adjacent_nodes.emplace_back(Node{ node.row, node.col - 1, 0 });
  }
  return adjacent_nodes;
}

std::ostream& operator<<(std::ostream& out, const Node& node)
{
  out << "(" << node.col << ", " << node.row << ")";
  return out;
}
}  // namespace trajectory_utils
