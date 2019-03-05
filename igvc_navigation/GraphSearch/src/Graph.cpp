#include "Graph.h"

void Graph::setConfigurationSpace(float ConfigurationSpace)
{
  this->ConfigurationSpace = ConfigurationSpace;
}

void Graph::setGoal(std::tuple<int, int> Goal)
{
  this->Goal.setIndex(Goal);
}

void Graph::initializeGraph(const igvc_msgs::mapConstPtr& msg)
{
  this->updatedCells.clear();
  this->Length = msg->length;
  this->Width = msg->width;
  this->Resolution = msg->resolution;
  this->Start.setIndex(msg->x, msg->y);
  this->KeyModifier = 0;  // reset the key modifuer

  // set the current map equal to the input message's map
  Map = cv_bridge::toCvCopy(msg->image, "mono8");
}

void Graph::updateGraph(igvc_msgs::mapConstPtr& msg)
{
  // Update the start position and KeyModifier
  std::tuple<float, float> newStart = std::make_tuple(msg->x, msg->y);
  std::tuple<float, float> oldStart = this->Start.getIndex();

  // update the key modifier value to account for the robot's new pos
  this->KeyModifier += igvc::get_distance(oldStart, newStart);
  this->Start.setIndex(static_cast<std::tuple<int, int>>(newStart));

  this->updatedCells.clear();  // clear the vector of cells that need updating

  // get the most recently observed occupancy grid
  cv_bridge::CvImagePtr currMap = cv_bridge::toCvCopy(msg->image, "mono8");

  // gather the cells that need updating if the new map differs
  if (!std::equal(Map->image.begin<uchar>(), Map->image.end<uchar>(), currMap->image.begin<uchar>()))
  {
    auto start_it = Map->image.begin<uchar>();

    auto it_new = currMap->image.begin<uchar>();
    auto it_map = Map->image.begin<uchar>();

    auto it_new_end = currMap->image.end<uchar>();
    auto it_map_end = Map->image.end<uchar>();

    // if cells differ, get (x,y) index of occupancy grid cell
    // and add it to the list of updated cells
    while ((it_new != it_new_end) && (it_map != it_map_end))
    {
      uchar curr_val = *it_new;
      uchar map_val = *it_map;
      // store index of updated cell if occupancy grid values differ
      if (curr_val != map_val)
      {
        int pos = it_map - start_it;
        int row = pos / (this->Width);
        int col = pos % (this->Width);
        this->updatedCells.push_back(Cell(row, col));
        // update the value of Map with the value of the new map
        *it_map = curr_val;
      }

      it_new++;
      it_map++;
    }
  }
}

bool Graph::isValidNode(const Node& s)
{
  int x, y;
  std::tie(x, y) = s.getIndex();
  // with [x,y] indexing
  return (x <= Length) && (y <= Width) && (x >= 0) && (y >= 0);
}

bool Graph::isValidPosition(const Position& p)
{
  // with [x,y] indexing
  return (p.x <= static_cast<float>(Length)) && (p.y <= static_cast<float>(Width)) && (p.x >= 0.0f) && (p.y >= 0.0f);
}

bool Graph::isValidCell(const std::tuple<int, int>& ind)
{
  int x, y;
  std::tie(x, y) = ind;
  return (x < Length) && (y < Width) && (x >= 0) && (y >= 0);
}

bool Graph::isDiagonal(const Node& s, const Node& s_prime)
{
  int x1, y1;
  std::tie(x1, y1) = s.getIndex();

  int x2, y2;
  std::tie(x2, y2) = s_prime.getIndex();

  int x_diff = std::abs(x2 - x1);
  int y_diff = std::abs(y2 - y1);

  return (x_diff == 1) && (y_diff == 1);
}

bool Graph::isDiagonalContinuous(const Position& p, const Position& p_prime)
{
  return ((p.x != p_prime.x) && (p.y != p_prime.y));
}

std::vector<Node> Graph::nbrs(const Node& s, bool include_invalid)
{
  std::vector<Node> neighbors;
  int x, y;
  std::tie(x, y) = s.getIndex();

  // right
  Node r(x + 1, y);
  if (include_invalid || isValidNode(r))
    neighbors.push_back(std::move(r));

  // top right
  Node tr(x + 1, y + 1);
  if (include_invalid || isValidNode(tr))
    neighbors.push_back(std::move(tr));

  // above
  Node t(x, y + 1);
  if (include_invalid || isValidNode(t))
    neighbors.push_back(std::move(t));

  // top left
  Node tl(x - 1, y + 1);
  if (include_invalid || isValidNode(tl))
    neighbors.push_back(std::move(tl));

  // left
  Node l(x - 1, y);
  if (include_invalid || isValidNode(l))
    neighbors.push_back(std::move(l));

  // bottom left
  Node bl(x - 1, y - 1);
  if (include_invalid || isValidNode(bl))
    neighbors.push_back(std::move(bl));

  // bottom
  Node b(x, y - 1);
  if (include_invalid || isValidNode(b))
    neighbors.push_back(std::move(b));

  // bottom right
  Node br(x + 1, y - 1);
  if (include_invalid || isValidNode(br))
    neighbors.push_back(std::move(br));

  return neighbors;
}

std::vector<std::pair<Position, Position>>
Graph::nbrsContinuous(const Position& p)
{
  std::vector<Position> neighbors;
  std::vector<std::pair<Position, Position>> connbrs;

  // there are 8 consecutive neighbors for an edge node.
  neighbors.push_back(std::make_tuple(p.x + 1.0f, p.y));         // right
  neighbors.push_back(std::make_tuple(p.x + 1.0f, p.y + 1.0f));  // top right
  neighbors.push_back(std::make_tuple(p.x, p.y + 1.0f));         // top
  neighbors.push_back(std::make_tuple(p.x - 1.0f, p.y + 1.0f));  // top left
  neighbors.push_back(std::make_tuple(p.x - 1.0f, p.y));         // left
  neighbors.push_back(std::make_tuple(p.x - 1.0f, p.y - 1.0f));  // bottom left
  neighbors.push_back(std::make_tuple(p.x, p.y - 1.0f));         // bottom
  neighbors.push_back(std::make_tuple(p.x + 1.0f, p.y - 1.0f));  // bottom right

  // first 7 connbrs
  for (size_t i = 0; i < neighbors.size() - 1; i++)
  {
    if (isValidPosition(neighbors[i]) && isValidPosition(neighbors[i + 1]))
      connbrs.push_back(std::make_pair(neighbors[i], neighbors[i + 1]));
  }

  // last connbrs pair [s8->s1]
  if (isValidPosition(neighbors[neighbors.size() - 1]) && isValidPosition(neighbors[0]))
    connbrs.push_back(std::make_pair(neighbors[neighbors.size() - 1], neighbors[0]));

  return connbrs;
}

Node Graph::counterClockwiseNeighbor(Node s, Node s_prime)
{
  int x, y;
  std::tie(x, y) = s.getIndex();

  int x_prime, y_prime;
  std::tie(x_prime, y_prime) = s_prime.getIndex();

  int x_diff = x_prime - x;
  int y_diff = y_prime - y;

  Node counterClockwiseNeighbor;  // counter-clockwise neighbor

  if (x_diff == 1 && y_diff == 0)
  {
    counterClockwiseNeighbor.setIndex(x + 1, y + 1);
  }
  else if (x_diff == 1 && y_diff == 1)
  {
    counterClockwiseNeighbor.setIndex(x, y + 1);
  }
  else if (x_diff == 0 && y_diff == 1)
  {
    counterClockwiseNeighbor.setIndex(x - 1, y + 1);
  }
  else if (x_diff == -1 && y_diff == 1)
  {
    counterClockwiseNeighbor.setIndex(x - 1, y);
  }
  else if (x_diff == -1 && y_diff == 0)
  {
    counterClockwiseNeighbor.setIndex(x - 1, y - 1);
  }
  else if (x_diff == -1 && y_diff == -1)
  {
    counterClockwiseNeighbor.setIndex(x, y - 1);
  }
  else if (x_diff == 0 && y_diff == -1)
  {
    counterClockwiseNeighbor.setIndex(x + 1, y - 1);
  }
  else if (x_diff == 1 && y_diff == -1)
  {
    counterClockwiseNeighbor.setIndex(x + 1, y);
  }

  // if counter-clockwise neighbor node is valid (within bounds), then return
  // it. Otherwise, return a node with validity set to false.
  if (isValidNode(counterClockwiseNeighbor))
    return counterClockwiseNeighbor;
  else
    return Node(false);
}

Node Graph::clockwiseNeighbor(Node s, Node s_prime)
{
  int x, y;
  std::tie(x, y) = s.getIndex();

  int x_prime, y_prime;
  std::tie(x_prime, y_prime) = s_prime.getIndex();

  int x_diff = x_prime - x;
  int y_diff = y_prime - y;

  Node clockwiseNeighbor;  // clockwise neighbor

  if (x_diff == 1 && y_diff == 0)
  {
    clockwiseNeighbor.setIndex(x + 1, y - 1);
  }
  else if (x_diff == 1 && y_diff == 1)
  {
    clockwiseNeighbor.setIndex(x + 1, y);
  }
  else if (x_diff == 0 && y_diff == 1)
  {
    clockwiseNeighbor.setIndex(x + 1, y + 1);
  }
  else if (x_diff == -1 && y_diff == 1)
  {
    clockwiseNeighbor.setIndex(x, y + 1);
  }
  else if (x_diff == -1 && y_diff == 0)
  {
    clockwiseNeighbor.setIndex(x - 1, y + 1);
  }
  else if (x_diff == -1 && y_diff == -1)
  {
    clockwiseNeighbor.setIndex(x - 1, y);
  }
  else if (x_diff == 0 && y_diff == -1)
  {
    clockwiseNeighbor.setIndex(x - 1, y - 1);
  }
  else if (x_diff == 1 && y_diff == -1)
  {
    clockwiseNeighbor.setIndex(x, y - 1);
  }

  // if clockwise neighbor node is valid (within bounds), then return
  // it. Otherwise, return a node with validity set to false.
  if (isValidNode(clockwiseNeighbor))
    return clockwiseNeighbor;
  else
    return Node(false);
}

std::vector<std::tuple<Node, Node>> Graph::consecutiveNeighbors(const Node& s)
{
  // get neighbors of current node, including invalid nodes
  std::vector<Node> neighbors = nbrs(s, true);
  std::vector<std::tuple<Node, Node>> consecutiveNeighbors;

  // first 7 consecutive neighbor pairs
  for (size_t i = 0; i < neighbors.size() - 1; i++)
  {
    assert(!isDiagonal(neighbors[i], neighbors[i + 1]));
    // if both consecutiveNeighbors valid, make a tuple and put it in the list
    if (isValidNode(neighbors[i]) && isValidNode(neighbors[i + 1]))
      consecutiveNeighbors.push_back(std::make_tuple(neighbors[i], neighbors[i + 1]));
  }

  // last consecutive neighbor pair [s8->s1]
  if (isValidNode(neighbors[neighbors.size() - 1]) && isValidNode(neighbors[0]))
    consecutiveNeighbors.push_back(std::make_tuple(neighbors[neighbors.size() - 1], neighbors[0]));

  return consecutiveNeighbors;
}

float Graph::getC(const Node& s, const Node& s_prime)
{
  // index of cell between s and s_prime. s and s_prime assumed to be
  // diagonal neighbors
  std::tuple<int, int> cellInd;
  float cellVal;

  int x1, y1;  // indices of s
  std::tie(x1, y1) = s.getIndex();

  int x2, y2;  // indices of s'
  std::tie(x2, y2) = s_prime.getIndex();

  // get orientation of s_prime relative to s. Used to locate containing cell
  int x_diff = x2 - x1;
  int y_diff = y2 - y1;

  assert((std::abs(x_diff) == 1) && (std::abs(y_diff) == 1));

  if ((x_diff == 1) && (y_diff == 1))
  {
    cellInd = std::make_tuple(x1, y1);  // top right cell
  }
  else if ((x_diff == -1) && (y_diff == 1))
  {
    cellInd = std::make_tuple(x1 - 1, y1);  // top left cell
  }
  else if ((x_diff == -1) && (y_diff == -1))
  {
    cellInd = std::make_tuple(x1 - 1, y1 - 1);  // bottom left cell
  }
  else if ((x_diff == 1) && (y_diff == -1))
  {
    cellInd = std::make_tuple(x1, y1 - 1);  // bottom right cell
  }

  // return inf cost if cell is occupied, otherwise return constant traversal cost (1)
  cellVal = getValWithConfigurationSpace(cellInd);
  return (cellVal > 150) ? std::numeric_limits<float>::infinity() : TraversalCost;  // #TODO get rid of magic number
}

float Graph::getB(const Node& s, const Node& s_prime)
{
  // each edge has 2 neighboring cells
  std::tuple<int, int> cellInd1;
  std::tuple<int, int> cellInd2;

  float maxCellVal;  // maximum occupied status of both neighboring cells

  int x1, y1;
  std::tie(x1, y1) = s.getIndex();

  int x2, y2;
  std::tie(x2, y2) = s_prime.getIndex();

  int x_diff = x2 - x1;
  int y_diff = y2 - y1;

  assert((std::abs(x_diff) == 1 && y_diff == 0) || (x_diff == 0 && std::abs(y_diff) == 1));

  if ((x_diff == 1) && (y_diff == 0))
  {
    cellInd1 = std::make_tuple(x1, y1);      // top right cell
    cellInd2 = std::make_tuple(x1, y1 - 1);  // bottom right cell
  }
  else if ((x_diff == 0) && (y_diff == 1))
  {
    cellInd1 = std::make_tuple(x1 - 1, y1);  // top left cell
    cellInd2 = std::make_tuple(x1, y1);      // top right cell
  }
  else if ((x_diff == -1) && (y_diff == 0))
  {
    cellInd1 = std::make_tuple(x1 - 1, y1);      // top left cell
    cellInd2 = std::make_tuple(x1 - 1, y1 - 1);  // bottom left cell
  }
  else if ((x_diff == 0) && (y_diff == -1))
  {
    cellInd1 = std::make_tuple(x1 - 1, y1 - 1);  // bottom left cell
    cellInd2 = std::make_tuple(x1, y1 - 1);      // bottom right cell
  }

  // return inf cost if cell is occupied, otherwise return constant traversal cost (1)
  maxCellVal = std::max(getValWithConfigurationSpace(cellInd1), getValWithConfigurationSpace(cellInd2));
  return (maxCellVal > 150) ? std::numeric_limits<float>::infinity() : TraversalCost;
}

float Graph::getValWithConfigurationSpace(const std::tuple<int, int>& ind)
{
  // invalid cells have infinite travel cost
  if (!isValidCell(ind))
    return 255.0f;

  int x, y;
  std::tie(x, y) = ind;

  int sep = ConfigurationSpace / Resolution + 1;  // number of cells accounted for with ConfigurationSpace

  // get a slice around the cell (ind) with the desired ConfigurationSpace
  cv::Mat subsection = Map->image(cv::Range(std::max(x - sep, 0), std::min(x + sep + 1, Map->image.size().height)),
                                  cv::Range(std::max(y - sep, 0), std::min(y + sep + 1, Map->image.size().width)));

  // get the value of the most occupied cell in the slice
  double min_val;
  double max_val;
  cv::minMaxLoc(subsection, &min_val, &max_val);

  return static_cast<float>(max_val);
}

float Graph::getTraversalCost(const Node& s, const Node& s_prime)
{
  if (isDiagonal(s, s_prime))
    return getC(s, s_prime) * DiagonalDistance;
  else
    return getB(s, s_prime) * EdgeDistance;
}

float Graph::getContinuousTraversalCost(const Position& p, const Position& p_prime)
{
  // round p to get a reference node
  Node s(std::make_tuple(static_cast<int>(roundf(p.x)), static_cast<int>(roundf(p.y))));

  // get relative distance between p_prime and p. Note: ceil0 is biased away from 0
  // i.e. ceil0(-0.35) = -1.00
  float x_diff = igvc::ceil0(p_prime.x - p.x);
  float y_diff = igvc::ceil0(p_prime.y - p.y);

  assert((x_diff != 0) || (y_diff != 0));
  assert((std::fabs(x_diff) <= 1) && (std::fabs(y_diff) <= 1));

  Node s_prime(std::make_tuple(static_cast<int>(roundf(p.x) + x_diff), static_cast<int>(roundf(p.y) + y_diff)));

  return (isDiagonal(s, s_prime) ? getC(s, s_prime) : getB(s, s_prime));
}

float Graph::getMinTraversalCost(const Node& s)
{
  float min_cost = std::numeric_limits<float>::infinity();
  for (Node nbr : this->nbrs(s))
    min_cost = std::min(min_cost, this->getTraversalCost(s, nbr));
  return min_cost;
}

float Graph::euclidianHeuristic(const Node& s)
{
  return this->euclidianHeuristic(s.getIndex());
}

float Graph::euclidianHeuristic(const std::tuple<int, int>& ind)
{
  std::tuple<float, float> start = Start.getIndex();
  std::tuple<float, float> s = ind;

  return igvc::get_distance(start, s);
}

std::vector<Node> Graph::getNodesAroundCellWithConfigurationSpace(const Cell& cell)
{
  std::queue<Node> openList;           // nodes to evaluate
  std::unordered_set<Node> closedSet;  // evaluated nodes
  std::vector<Node> cellNodes;         // nodes that require update

  Node startNode(cell.x, cell.y);

  openList.push(startNode);
  closedSet.insert(startNode);
  cellNodes.push_back(startNode);
  // number of cells on all sides that constitute C-space
  int separationDist = static_cast<int>(ceil(ConfigurationSpace / Resolution));

  // perform a simple breadth-first search to radially look for nodes around C_space
  while (!openList.empty())
  {
    Node currNode = openList.front();
    openList.pop();

    for (Node n : this->nbrs(currNode))
    {
      // node already considered
      if (closedSet.find(n) != closedSet.end())
        continue;
      else
        closedSet.insert(n);

      // node is within the configuration space
      if (startNode.distTo(static_cast<std::tuple<float, float>>(n.getIndex())) < separationDist)
      {
        openList.push(n);
        cellNodes.push_back(std::move(n));
      }
    }
  }
  return cellNodes;
}
