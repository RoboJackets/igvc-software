#include "DLitePlanner.h"

void DLitePlanner::setGoalDistance(float goal_dist)
{
  this->goal_dist_ = goal_dist;
}

Key DLitePlanner::calculateKey(Node s)
{
  // obtain g-values and rhs-values for node s
  float g = getG(s);
  float rhs = getRHS(s);
  return Key(std::min(g, rhs) + node_grid_.euclidianHeuristic(s) + node_grid_.key_modifier_, std::min(g, rhs));
}

void DLitePlanner::initializeSearch()
{
  expanded_map_.insert(std::make_pair(node_grid_.start_, std::make_tuple(std::numeric_limits<float>::infinity(),
                                                                         std::numeric_limits<float>::infinity())));
  expanded_map_.insert(std::make_pair(node_grid_.goal_, std::make_tuple(std::numeric_limits<float>::infinity(), 0)));

  priority_queue_.insert(node_grid_.goal_, calculateKey(node_grid_.goal_));
}

void DLitePlanner::reInitializeSearch()
{
  expanded_map_.clear();
  priority_queue_.clear();
  node_grid_.updated_cells_.clear();

  this->initializeSearch();
}

void DLitePlanner::updateNode(Node s)
{
  if (expanded_map_.find(s) == expanded_map_.end())  // s never visited before, add to unordered map
    expanded_map_.insert(std::make_pair(
        s, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

  /**
  looks for a node in the priority queue and removes it if found
  same as calling: if priority_queue_.contains(s) priority_queue_.remove(s);
  */
  priority_queue_.remove(s);

  // update rhs value of Node s
  if (s != node_grid_.goal_)
  {
    float min_rhs = std::numeric_limits<float>::infinity();
    for (Node succ : node_grid_.nbrs(s))
    {
      if (expanded_map_.find(succ) == expanded_map_.end())  // node never visited before, add to unordered map
        expanded_map_.insert(std::make_pair(
            succ, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

      min_rhs = std::min(min_rhs, node_grid_.getTraversalCost(s, succ) + getG(succ));
    }

    insertOrAssign(s, getG(s), min_rhs);
  }

  // insert node into priority queue if it is locally inconsistent
  if (getG(s) != getRHS(s))
    priority_queue_.insert(s, calculateKey(s));
}

int DLitePlanner::computeShortestPath()
{
  // if the start node is occupied, return immediately. By definition, a path
  // does not exist if the start node is occupied.
  if (node_grid_.getMinTraversalCost(node_grid_.start_) == std::numeric_limits<float>::infinity())
    return 0;

  int num_nodes_expanded = 0;

  while (((priority_queue_.topKey() < calculateKey(node_grid_.start_)) ||
          (getRHS(node_grid_.start_) != getG(node_grid_.start_))) &&
         (priority_queue_.size() > 0))
  {
    Node top_node = priority_queue_.topNode();
    Key top_key = priority_queue_.topKey();
    priority_queue_.pop();
    num_nodes_expanded++;

    if (top_key < calculateKey(top_node))
    {
      priority_queue_.insert(top_node, calculateKey(top_node));
    }
    else if (getG(top_node) > getRHS(top_node))
    {
      // locally overconsistent case. This node is now more favorable.
      // make node locally consistent by setting g = rhs
      insertOrAssign(top_node, getRHS(top_node), getRHS(top_node));  // update node's standing in unordered map
      // propagate changes to neighboring nodes
      for (Node nbr : node_grid_.nbrs(top_node))
        updateNode(nbr);
    }
    else
    {
      // locally underconsistent case. This node is now less favorable
      // than it was before

      // make node locally consistent or overconsistent by setting g = inf
      insertOrAssign(top_node, std::numeric_limits<float>::infinity(), getRHS(top_node));

      // propagate changes to neighbors and to top_node
      std::vector<Node> to_propagate = node_grid_.nbrs(top_node);
      to_propagate.push_back(top_node);

      for (Node n : to_propagate)
        updateNode(n);
    }
  }

  return num_nodes_expanded;
}

int DLitePlanner::updateNodesAroundUpdatedCells()
{
  int num_nodes_updated = 0;

  for (Cell cell_update : node_grid_.updated_cells_)
  {
    for (Node n : node_grid_.getNodesAroundCellWithConfigurationSpace(cell_update))
    {
      if (expanded_map_.find(n) == expanded_map_.end())  // node hasn't been explored yet. Leave alone
        continue;

      updateNode(n);
      num_nodes_updated++;
    }
  }

  return num_nodes_updated;
}

void DLitePlanner::constructOptimalPath()
{
  path_.clear();

  Node curr_node = node_grid_.start_;

  float min_cost;
  float temp_cost;
  Node temp_node;

  int max_steps = static_cast<int>(500.00f / (this->node_grid_.resolution_));
  int curr_steps = 0;

  // greedily move from one node to the next, selecting the node s' that
  // minimizes: g(s') + c(s,s')
  // if no valid neighbor nodes can be traveled to (all have cost inf), then
  // an empty path is returned.
  do
  {
    path_.push_back(curr_node.getIndex());

    min_cost = std::numeric_limits<float>::infinity();
    for (Node nbr : node_grid_.nbrs(curr_node))
    {
      if (expanded_map_.find(nbr) == expanded_map_.end())  // node has not been explored yet
        continue;

      temp_cost = node_grid_.getTraversalCost(curr_node, nbr) + getG(nbr);
      if (temp_cost <= min_cost)
      {
        min_cost = temp_cost;
        temp_node = nbr;
      }
    }
    curr_node = temp_node;
    curr_steps += 1;

  } while (!isWithinRangeOfGoal(curr_node) && (min_cost != std::numeric_limits<float>::infinity()) &&
           (curr_steps < max_steps));

  // no valid path found. Return empty path
  if ((min_cost == std::numeric_limits<float>::infinity()) || (curr_steps >= max_steps))
    path_.clear();
}

void DLitePlanner::insertOrAssign(Node s, float g, float rhs)
{
  // re-assigns value of node in unordered map or inserts new entry if
  // node not found
  if (expanded_map_.find(s) != expanded_map_.end())
    expanded_map_.erase(s);

  expanded_map_.insert(std::make_pair(s, std::make_tuple(g, rhs)));
}

bool DLitePlanner::isWithinRangeOfGoal(Node s)
{
  int goal_x, goal_y;
  std::tie(goal_x, goal_y) = node_grid_.goal_.getIndex();

  int x, y;
  std::tie(x, y) = s.getIndex();

  int sep = static_cast<int>(goal_dist_ / node_grid_.resolution_);

  bool satisfies_x_bounds = (x >= (goal_x - sep)) && (x <= (goal_x + sep));
  bool satisfies_y_bounds = (y >= (goal_y - sep)) && (y <= (goal_y + sep));

  return satisfies_x_bounds && satisfies_y_bounds;
}

float DLitePlanner::getG(Node s)
{
  // return g value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (expanded_map_.find(s) != expanded_map_.end())
    return std::get<0>(expanded_map_.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

float DLitePlanner::getRHS(Node s)
{
  // return rhs value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (expanded_map_.find(s) != expanded_map_.end())
    return std::get<1>(expanded_map_.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

std::vector<std::tuple<int, int>> DLitePlanner::getExplored()
{
  std::vector<std::tuple<int, int>> explored;
  for (std::pair<Node, std::tuple<float, float>> e : expanded_map_)
  {
    explored.push_back(e.first.getIndex());
  }

  return explored;
}
