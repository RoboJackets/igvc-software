#include "DLitePlanner.h"

DLitePlanner::DLitePlanner()
{
}

DLitePlanner::~DLitePlanner()
{
}

Key DLitePlanner::calculateKey(Node s)
{
  // obtain g-values and rhs-values for node s
  float g = getG(s);
  float rhs = getRHS(s);
  return Key(std::min(g, rhs) + graph.euclidian_heuristic(s) + graph.K_M, std::min(g, rhs));
}

void DLitePlanner::initialize()
{
  umap.insert(std::make_pair(
      graph.Start, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));
  umap.insert(std::make_pair(graph.Goal, std::make_tuple(std::numeric_limits<float>::infinity(), 0)));

  PQ.insert(graph.Goal, calculateKey(graph.Goal));
}

void DLitePlanner::reinitialize()
{
  umap.clear();
  PQ.clear();
  graph.updatedCells.clear();

  this->initialize();
}

void DLitePlanner::updateNode(Node s)
{
  if (umap.find(s) == umap.end())  // s never visited before, add to unordered map
    umap.insert(std::make_pair(
        s, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

  /**
  looks for a node in the priority queue and removes it if found
  same as calling: if PQ.contains(s) PQ.remove(s);
  */
  PQ.remove(s);

  // update rhs value of Node s
  if (s != graph.Goal)
  {
    float minRHS = std::numeric_limits<float>::infinity();
    for (Node succ : graph.nbrs(s))
    {
      if (umap.find(succ) == umap.end())  // node never visited before, add to unordered map
        umap.insert(std::make_pair(
            succ, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

      minRHS = std::min(minRHS, graph.getTraversalCost(s, succ) + getG(succ));
    }

    insert_or_assign(s, getG(s), minRHS);
  }

  // insert node into priority queue if it is locally inconsistent
  if (getG(s) != getRHS(s))
    PQ.insert(s, calculateKey(s));
}

int DLitePlanner::computeShortestPath()
{
  // if the start node is occupied, return immediately. By definition, a path
  // does not exist if the start node is occupied.
  if (graph.getMinTraversalCost(graph.Start) == std::numeric_limits<float>::infinity())
    return 0;

  int numNodesExpanded = 0;

  while ((PQ.topKey() < calculateKey(graph.Start)) || (getRHS(graph.Start) != getG(graph.Start)))
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
      insert_or_assign(topNode, getRHS(topNode), getRHS(topNode));  // update node's standing in unordered map
      // propagate changes to neighboring nodes
      for (Node nbr : graph.nbrs(topNode))
        updateNode(nbr);
    }
    else
    {
      // locally underconsistent case. This node is now less favorable
      // than it was before

      // make node locally consistent or overconsistent by setting g = inf
      insert_or_assign(topNode, std::numeric_limits<float>::infinity(), getRHS(topNode));

      // propagate changes to neighbors and to topNode
      std::vector<Node> toPropagate = graph.nbrs(topNode);
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

  for (std::tuple<int, int> cellUpdate : graph.updatedCells)
  {
    for (Node n : graph.getNodesAroundCellWithCSpace(cellUpdate))
    {
      if (umap.find(n) == umap.end())  // node hasn't been explored yet. Leave alone
        continue;

      updateNode(n);
      numNodesUpdated++;
    }
  }

  return numNodesUpdated;
}

void DLitePlanner::constructOptimalPath()
{
  path.clear();
  std::vector<std::tuple<int, int>>::iterator start_it;

  Node currNode = graph.Start;

  float minCost;
  float tempCost;
  Node tempNode;

  // greedily move from one node to the next, selecting the node s' that
  // minimizes: g(s') + c(s,s')
  // if no valid neighbor nodes can be traveled to (all have cost inf), then
  // an empty path is returned.
  do
  {
    start_it = path.begin();
    path.insert(start_it, currNode.getIndex());

    minCost = std::numeric_limits<float>::infinity();
    for (Node nbr : graph.nbrs(currNode))
    {
      if (umap.find(nbr) == umap.end())  // node has not been explored yet
        continue;

      tempCost = graph.getTraversalCost(currNode, nbr) + getG(nbr);
      if (tempCost <= minCost)
      {
        minCost = tempCost;
        tempNode = nbr;
      }
    }
    currNode = tempNode;

  } while (!isWithinRangeOfGoal(currNode) && (minCost != std::numeric_limits<float>::infinity()));

  // no valid path found. Return empty path
  if (minCost == std::numeric_limits<float>::infinity())
    path.clear();
}

void DLitePlanner::insert_or_assign(Node s, float g, float rhs)
{
  // re-assigns value of node in unordered map or inserts new entry if
  // node not found
  if (umap.find(s) != umap.end())
    umap.erase(s);

  umap.insert(std::make_pair(s, std::make_tuple(g, rhs)));
}

bool DLitePlanner::isWithinRangeOfGoal(Node s)
{
  int goal_x, goal_y;
  std::tie(goal_x, goal_y) = graph.Goal.getIndex();

  int x, y;
  std::tie(x, y) = s.getIndex();

  int sep = static_cast<int>(GOAL_DIST / graph.Resolution);

  bool satisfiesXBounds = (x >= (goal_x - sep)) && (x <= (goal_x + sep));
  bool satisfiesYBounds = (y >= (goal_y - sep)) && (y <= (goal_y + sep));

  return satisfiesXBounds && satisfiesYBounds;
}

float DLitePlanner::getG(Node s)
{
  // return g value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (umap.find(s) != umap.end())
    return std::get<0>(umap.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

float DLitePlanner::getRHS(Node s)
{
  // return rhs value if node has been looked at before (is in unordered map)
  // otherwise, return infinity
  if (umap.find(s) != umap.end())
    return std::get<1>(umap.at(s));
  else
    return std::numeric_limits<float>::infinity();
}

std::vector<std::tuple<int, int>> DLitePlanner::getExplored()
{
  std::vector<std::tuple<int, int>> explored;
  for (std::pair<Node, std::tuple<float, float>> e : umap)
  {
    explored.push_back(e.first.getIndex());
  }

  return explored;
}
