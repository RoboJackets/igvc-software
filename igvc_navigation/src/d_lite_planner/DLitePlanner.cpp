#include "DLitePlanner.h"

void DLitePlanner::setGoalDistance(float goalDist)
{
  this->GoalDist = goalDist;
}

Key DLitePlanner::calculateKey(Node s)
{
  // obtain g-values and rhs-values for node s
  float g = getG(s);
  float rhs = getRHS(s);
  return Key(std::min(g, rhs) + NodeGrid.euclidianHeuristic(s) + NodeGrid.KeyModifier, std::min(g, rhs));
}

void DLitePlanner::initializeSearch()
{
  unorderedMap.insert(std::make_pair(
      NodeGrid.Start, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));
  unorderedMap.insert(std::make_pair(NodeGrid.Goal, std::make_tuple(std::numeric_limits<float>::infinity(), 0)));

  PQ.insert(NodeGrid.Goal, calculateKey(NodeGrid.Goal));
}

void DLitePlanner::reInitializeSearch()
{
  unorderedMap.clear();
  PQ.clear();
  NodeGrid.updatedCells.clear();

  this->initializeSearch();
}

void DLitePlanner::updateNode(Node s)
{
  if (unorderedMap.find(s) == unorderedMap.end())  // s never visited before, add to unordered map
    unorderedMap.insert(std::make_pair(
        s, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

  /**
  looks for a node in the priority queue and removes it if found
  same as calling: if PQ.contains(s) PQ.remove(s);
  */
  PQ.remove(s);

  // update rhs value of Node s
  if (s != NodeGrid.Goal)
  {
    float minRHS = std::numeric_limits<float>::infinity();
    for (Node succ : NodeGrid.nbrs(s))
    {
      if (unorderedMap.find(succ) == unorderedMap.end())  // node never visited before, add to unordered map
        unorderedMap.insert(std::make_pair(
            succ, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

      minRHS = std::min(minRHS, NodeGrid.getTraversalCost(s, succ) + getG(succ));
    }

    insertOrAssign(s, getG(s), minRHS);
  }

  // insert node into priority queue if it is locally inconsistent
  if (getG(s) != getRHS(s))
    PQ.insert(s, calculateKey(s));
}

int DLitePlanner::computeShortestPath()
{
  // if the start node is occupied, return immediately. By definition, a path
  // does not exist if the start node is occupied.
  if (NodeGrid.getMinTraversalCost(NodeGrid.Start) == std::numeric_limits<float>::infinity())
    return 0;

  int numNodesExpanded = 0;

  while (((PQ.topKey() < calculateKey(NodeGrid.Start)) || (getRHS(NodeGrid.Start) != getG(NodeGrid.Start))) &&
         (PQ.size() > 0))
  {
    Node topNode = PQ.topNode();
    Key topKey = PQ.topKey();
    PQ.pop();
    numNodesExpanded++;

    if (topKey < calculateKey(topNode))
    {
      PQ.insert(topNode, calculateKey(topNode));
    }
    else if (getG(topNode) > getRHS(topNode))
    {
      // locally overconsistent case. This node is now more favorable.
      // make node locally consistent by setting g = rhs
      insertOrAssign(topNode, getRHS(topNode), getRHS(topNode));  // update node's standing in unordered map
      // propagate changes to neighboring nodes
      for (Node nbr : NodeGrid.nbrs(topNode))
        updateNode(nbr);
    }
    else
    {
      // locally underconsistent case. This node is now less favorable
      // than it was before

      // make node locally consistent or overconsistent by setting g = inf
      insertOrAssign(topNode, std::numeric_limits<float>::infinity(), getRHS(topNode));

      // propagate changes to neighbors and to topNode
      std::vector<Node> toPropagate = NodeGrid.nbrs(topNode);
      toPropagate.push_back(topNode);

      for (Node n : toPropagate)
        updateNode(n);
    }
  }

  return numNodesExpanded;
}

int DLitePlanner::updateNodesAroundUpdatedCells()
{
  int numNodesUpdated = 0;

  for (Cell cellUpdate : NodeGrid.updatedCells)
  {
    for (Node n : NodeGrid.getNodesAroundCellWithConfigurationSpace(cellUpdate))
    {
      if (unorderedMap.find(n) == unorderedMap.end())  // node hasn't been explored yet. Leave alone
        continue;

      updateNode(n);
      numNodesUpdated++;
    }
  }

  return numNodesUpdated;
}

void DLitePlanner::constructOptimalPath()
{
  Path.clear();
  std::vector<std::tuple<int, int>>::iterator start_it;

  Node currNode = NodeGrid.Start;

  float minCost;
  float tempCost;
  Node tempNode;

  int maxSteps = static_cast<int>(500.00f / (this->NodeGrid.Resolution));
  int currSteps = 0;

  // greedily move from one node to the next, selecting the node s' that
  // minimizes: g(s') + c(s,s')
  // if no valid neighbor nodes can be traveled to (all have cost inf), then
  // an empty path is returned.
  do
  {
    start_it = Path.begin();
    Path.insert(start_it, currNode.getIndex());

    minCost = std::numeric_limits<float>::infinity();
    for (Node nbr : NodeGrid.nbrs(currNode))
    {
      if (unorderedMap.find(nbr) == unorderedMap.end())  // node has not been explored yet
        continue;

      tempCost = NodeGrid.getTraversalCost(currNode, nbr) + getG(nbr);
      if (tempCost <= minCost)
      {
        minCost = tempCost;
        tempNode = nbr;
      }
    }
    currNode = tempNode;
    currSteps += 1;

  } while (!isWithinRangeOfGoal(currNode) && (minCost != std::numeric_limits<float>::infinity()) &&
           (currSteps < maxSteps));

  // no valid path found. Return empty path
  if ((minCost == std::numeric_limits<float>::infinity()) || (currSteps >= maxSteps))
    Path.clear();
}

void DLitePlanner::insertOrAssign(Node s, float g, float rhs)
{
  // re-assigns value of node in unordered map or inserts new entry if
  // node not found
  if (unorderedMap.find(s) != unorderedMap.end())
    unorderedMap.erase(s);

  unorderedMap.insert(std::make_pair(s, std::make_tuple(g, rhs)));
}

bool DLitePlanner::isWithinRangeOfGoal(Node s)
{
  int goal_x, goal_y;
  std::tie(goal_x, goal_y) = NodeGrid.Goal.getIndex();

  int x, y;
  std::tie(x, y) = s.getIndex();

  int sep = static_cast<int>(GoalDist / NodeGrid.Resolution);

  bool satisfiesXBounds = (x >= (goal_x - sep)) && (x <= (goal_x + sep));
  bool satisfiesYBounds = (y >= (goal_y - sep)) && (y <= (goal_y + sep));

  return satisfiesXBounds && satisfiesYBounds;
}

float DLitePlanner::getG(Node s)
{
  // return g value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (unorderedMap.find(s) != unorderedMap.end())
    return std::get<0>(unorderedMap.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

float DLitePlanner::getRHS(Node s)
{
  // return rhs value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (unorderedMap.find(s) != unorderedMap.end())
    return std::get<1>(unorderedMap.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

std::vector<std::tuple<int, int>> DLitePlanner::getExplored()
{
  std::vector<std::tuple<int, int>> explored;
  for (std::pair<Node, std::tuple<float, float>> e : unorderedMap)
  {
    explored.push_back(e.first.getIndex());
  }

  return explored;
}
