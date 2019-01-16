#include "FieldDPlanner.h"

FieldDPlanner::FieldDPlanner() {}
FieldDPlanner::~FieldDPlanner() {}

Key FieldDPlanner::calculateKey(Node s)
{
    // obtain g-values and rhs-values for node s
    float g = getG(s);
    float rhs = getRHS(s);
    return Key(std::min(g, rhs) + graph.euclidian_heuristic(s) + graph.K_M, std::min(g, rhs));
}

void FieldDPlanner::initialize()
{
    umap.insert(std::make_pair(graph.Start, std::make_tuple(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity())));
    umap.insert(std::make_pair(graph.Goal, std::make_tuple(
        std::numeric_limits<float>::infinity(),
        0)));

    PQ.insert(graph.Goal, calculateKey(graph.Goal));
}

void FieldDPlanner::reinitialize()
{
    // reinitialize clears the unordered map, Ø, updated cells,
    // and initializes the search problem from scratch
    umap.clear();
    PQ.clear();
    graph.updatedCells.clear();

    this->initialize();
}

void FieldDPlanner::updateNode(Node s)
{
    if (umap.find(s) == umap.end()) // s never visited before, add to unordered map
        umap.insert(std::make_pair(s, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

    /**
    looks for node s in the Ø and removes it if found
    -> same as calling: if PQ.contains(s) PQ.remove(s);
    */
    PQ.remove(s);

    if (s != graph.Goal)
    {
        float minRHS = std::numeric_limits<float>::infinity();
        Node s_a, s_b; // consecutive neighbors of s

        for (std::tuple<Node, Node> connbr : graph.connbrs(s))
        {
            std::tie(s_a, s_b) = connbr;
            if ((umap.find(s_a) == umap.end()) || (umap.find(s_b) == umap.end()))
                continue; // continue  if either connbr was never visited before
            float tempCost;
            std::tie(tempCost, std::ignore, std::ignore) = this->computeCostContinuous(static_cast<std::tuple<float,float>>(s.getIndex()),
                                                                              static_cast<std::tuple<float,float>>(s_a.getIndex()),
                                                                              static_cast<std::tuple<float,float>>(s_b.getIndex()));
            minRHS = std::min(minRHS, tempCost);
        }

        std::cout << minRHS << ", " << getG(s);
        // update Node s' RHS value
        insert_or_assign(s, getG(s), minRHS);
    }

    // insert node into priority queue if it is locally inconsistent
    if (getG(s) != getRHS(s))
        PQ.insert(s, calculateKey(s));
}

int FieldDPlanner::computeShortestPath()
{
    int numNodesExpanded = 0;

    while((PQ.topKey() < calculateKey(graph.Start)) || (getRHS(graph.Start) != getG(graph.Start)))
    {
        Node topNode = PQ.topNode();
        Key topKey = PQ.topKey();
        PQ.pop();
        numNodesExpanded++;

        float g = getG(topNode);
        float rhs = getRHS(topNode);

        if (topKey < calculateKey(topNode))
        {
            PQ.insert(topNode, calculateKey(topNode));
        }
        else if (g > rhs)
        {
            // locally overconsistent case. This node is now more favorable.
            insert_or_assign(topNode, rhs, rhs); // make node locally consistent by setting g = rhs
            for (Node nbr : graph.nbrs(topNode)) updateNode(nbr);
        }
        else
        {
            // locally underconsistent case. This node is now less favorable
            // make node locally consistent or overconsistent by setting g = inf
            insert_or_assign(topNode, std::numeric_limits<float>::infinity(), rhs);
            // propagate changes to neighbors and to topNode
            std::vector<Node> toPropagate = graph.nbrs(topNode);
            toPropagate.push_back(topNode);

            for (Node n : toPropagate) updateNode(n);
        }
    }
    return numNodesExpanded;
}

int FieldDPlanner::updateNodesAroundUpdatedCells()
{
    int numNodesUpdated = 0;
    std::cout << "Nodes with upated cell values: {" << std::endl;
    for (std::tuple<int,int> cellUpdate : graph.updatedCells)
    {
        for (Node n : graph.getNodesAroundCellWithCSpace(cellUpdate))
        {

            if (umap.find(n) == umap.end()) // Update node if it's already been expanded
                continue;

            std::cout << "\t[" << n << "], new rhs: ,";
            updateNode(n);
            std::cout << std::endl;
            numNodesUpdated++;
        }
    }
    std::cout << "}" << std::endl;
    return numNodesUpdated;
}

void FieldDPlanner::constructOptimalPath()
{
    path.clear();

    std::tuple<float,float> curr_pos = graph.Start.getIndex();
    path.push_back(curr_pos);

    int MAX_SIZE = 50;
    int curr_size = 0;
    float min_cost;
    path_additions pa;

    do
    {
        pa = getPathAdditions(curr_pos);
        // append new positions to the end of path
        path.insert(path.end(), pa.first.begin(), pa.first.end());
        min_cost = pa.second;
        // std::cout << "[" << std::get<0>(curr_pos) << ", " << std::get<1>(curr_pos) << "] " << min_cost << std::endl;
        curr_pos = path.back();
        curr_size += 1;
    } while (!isWithinRangeOfGoal(curr_pos) && (min_cost != std::numeric_limits<float>::infinity()) && (curr_size < MAX_SIZE));

    if (min_cost == std::numeric_limits<float>::infinity())
        path.clear();
}

bool FieldDPlanner::isVertex(std::tuple<float,float> p)
{
    float x,y;
    std::tie(x,y) = p;

    bool is_vertex = (ceilf(x) == x) && (ceilf(y) == y);
    bool satisfies_bounds = (x >= 0) && (x <= graph.length) && (y >= 0) && (y <= graph.width);

    return is_vertex && satisfies_bounds;
}

FieldDPlanner::path_additions FieldDPlanner::getPathAdditions(std::tuple<float,float> p)
{
    std::vector<std::tuple<float,float>> positions; // positions to add to path

    float cost = std::numeric_limits<float>::infinity();
    float x,y;
    std::tuple<float,float> p1,p2; // p1 - nearest neighbor; p2 - diagonal

    // temp variables
    float temp_cost, temp_x, temp_y;

    if (isVertex(p)) // planning from vertex
    {
        // node to search from
        Node s(static_cast<std::tuple<int,int>>(p));
        Node s_a, s_b; // temp nodes
        for (std::tuple<Node, Node> connbr : graph.connbrs(s))
        {
            std::tie(s_a, s_b) = connbr;

            // continue if either connbr was never visited before
            if ((umap.find(s_a) == umap.end()) || (umap.find(s_b) == umap.end()))
                continue;

            std::tie(temp_cost, temp_x, temp_y) = this->computeCostContinuous(static_cast<std::tuple<float,float>>(s.getIndex()),
                                                                              static_cast<std::tuple<float,float>>(s_a.getIndex()),
                                                                              static_cast<std::tuple<float,float>>(s_b.getIndex()));

            if (temp_cost < cost)
            {
                cost = temp_cost;
                x = temp_x;
                y = temp_y;
                if (graph.isDiagonal(s, s_a))
                    std::tie(p1,p2) = std::make_tuple(static_cast<std::tuple<float,float>>(s_b.getIndex()),
                                                        static_cast<std::tuple<float,float>>(s_a.getIndex()));
                else
                    std::tie(p1,p2) = std::make_tuple(static_cast<std::tuple<float,float>>(s_a.getIndex()),
                                                        static_cast<std::tuple<float,float>>(s_b.getIndex()));

            }
        }
    }
    else
    {
        std::tuple<float,float> p_a,p_b;
        for (std::pair<std::tuple<float,float>,std::tuple<float,float>> connbr : getEdgeConnbrs(p))
        {
            std::tie(p_a, p_b) = connbr;

            std::tie(temp_cost, temp_x, temp_y) = this->computeCostContinuous(p, p_a, p_b);

            if (temp_cost < cost)
            {
                cost = temp_cost;
                x = temp_x;
                y = temp_y;
                if ((std::get<0>(p_a) != std::get<0>(p)) && (std::get<1>(p_a) != std::get<1>(p))) // p_b is nearest neighbor
                    std::tie(p1,p2) = std::make_tuple(p_b,p_a);
                else // p_a is nearest neighbor
                    std::tie(p1,p2) = std::make_tuple(p_a,p_b);
            }
        }
    }


    // CASE 0: no valid path found (infinite cost/no consecutive neighbors)
    if (cost == std::numeric_limits<float>::infinity())
        return std::make_pair(positions, cost);

    // calculate the multiplier for the positions to be added to the path. This
    // step is required because x and y calculations are peformed independently of
    // the consecutive neighbors used to obtain these values. As such, x_multiplier
    // and y_multiplier account for this.
    float p_x, p_y;
    std::tie(p_x, p_y) = p;
    float p1_x, p1_y;
    std::tie(p1_x, p1_y) = p1;
    float p2_x, p2_y;
    std::tie(p2_x, p2_y) = p2;

    float x_multiplier,y_multiplier;
    bool flip = false; // path additions must be flipped to account for relative orientation

    if (p1_x != p_x) // nearest neighbor lies to left or right of s
    {
        x_multiplier = (p1_x > p_x) ? 1.0f : -1.0f;
        y_multiplier = (p2_y > p_y) ? 1.0f : -1.0f;
    }
    else // nearest neighbor lies above or below s
    {
        flip = true;
        y_multiplier = (p1_y > p_y) ? 1.0f : -1.0f;
        x_multiplier = (p2_x > p_x) ? 1.0f : -1.0f;
    }

    // CASE 1(1/2): travel along x(1/2) then cut to s2(2/2)
    // CASE 2: travel directly to diagonal node (s2)
    // CASE 3: travel to nearest node
    // CASE 4: travel to point along edge
    if (flip)
        std::tie(x,y) = std::make_tuple(y,x);

    x *= x_multiplier;
    y *= y_multiplier;
    positions.insert(positions.begin(), std::make_tuple(p_x + x, p_y + y));

    // CASE 1 (2/2): travel along x(2/2) then cut to s2(2/2)
    if ((x > 0.0f) && (x < 1.0f) &&  (y == 0.0f))
        positions.push_back(std::make_tuple(p2_x, p2_y));

    return std::make_pair(positions, cost);
}

std::tuple<float,float,float> FieldDPlanner::computeCostContinuous(std::tuple<float,float> p, std::tuple<float,float> p_a, std::tuple<float,float> p_b)
{
    std::tuple<float,float> p1; // nearest neighbor
    std::tuple<float,float> p2; // diagonal neighbor

    if ((std::get<0>(p_a) != std::get<0>(p)) && (std::get<1>(p_a) != std::get<1>(p))) // p_b is nearest neighbor
        std::tie(p1,p2) = std::make_tuple(p_b,p_a);
    else // p_a is nearest neighbor
        std::tie(p1,p2) = std::make_tuple(p_a,p_b);

    // traversal cost of position p and a diagonal position p2
    // in units of (cost/distance)
    float c = graph.getContinuousTraversalCost(p, p2);
    // traversal cost of position p and p1, a non-diaginal neighbor of p
    // in units of (cost/distance)
    float b = graph.getContinuousTraversalCost(p,p1);

    float g_p1 = getEdgePositionCost(p1); // path cost of edge neighbor
    float g_p2 = getEdgePositionCost(p2); // path cost of diagonal neighbor

    float d_p1 = igvc::get_distance(p,p1); // distance to nearest neighbor
    float d_p2 = igvc::get_distance(p,p2); // distance to diagonal
    float d_n = igvc::get_distance(p1,p2); // distance between consecutive neighbors (edge length)

    // travel distances
    float x = 0.0f;
    float y = 0.0f;

    // path cost of node s
    float v_s;

    if (std::min(c,b) == std::numeric_limits<float>::infinity())
    {
        // infinite traversal cost, cells likely occupied
        v_s =  std::numeric_limits<float>::infinity();
    }
    else if (g_p1 <= g_p2)
    {
        // cheapest to travel directly to nearest neighbor (non-diagonal)
        x = d_p1;
        v_s =  std::min(c,b) + g_p1;
    }
    else
    {
        float f = g_p1 - g_p2; // cost of going from s1 to s2

        if (f <= b)
        {
            if (c <= f)
            {
                // cheapest to go directly to diagonal cell
                x = d_p1;
                y = d_n;
                v_s =  c * d_p2 + g_p2;
            }
            else
            {
                // travel along diagonal to point along edge
                float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                x = d_p1;
                y = std::min(toComp, d_n);
                v_s =  c * sqrt(std::pow(d_p1,2.0f) + std::pow(y,2.0f)) + (f * (d_n - y)) + g_p2;
            }
        }
        else
        {
            if (c <= b)
            {
                // cheapest to go directly to diagonal cell
                x = d_p1;
                y = d_n;
                v_s =  c * d_p2 + g_p2;
            }
            else
            {
                // travel along edge then to s2
                float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                x = d_p1 - std::min(toComp, d_p1);
                v_s =  c * sqrt(std::pow(d_n, 2.0f) + std::pow(d_p1 - x, 2.0f)) + (b * x) + g_p2;
            }
        }
    }

    return std::make_tuple(v_s, x, y);
}

float FieldDPlanner::getEdgePositionCost(std::tuple<float,float> p)
{
    if (isVertex(p))
        return getG(Node(static_cast<std::tuple<int,int>>(p)));

    float x,y;
    std::tie(x,y) = p;

    std::tuple<float,float> p_a = std::make_tuple(ceilf(x), ceilf(y)); // get position of first neighbor
    std::tuple<float,float> p_b = std::make_tuple(floorf(x), floorf(y)); // get position of second neighbor

    float d = igvc::get_distance(p_a,p_b); // distance between neighbors
    float d_a = igvc::get_distance(p,p_a); // distance to first neighbor
    float d_b = igvc::get_distance(p,p_b); // distance to second neighbor

    float g_a = getG(Node(static_cast<std::tuple<int,int>>(p_a))); // path cost of p_a
    float g_b = getG(Node(static_cast<std::tuple<int,int>>(p_b))); // path cost of p_b

    return (d - d_a) * g_a + (d - d_b) * g_b; // return linearly interpolated path cost
}

std::vector<std::pair<std::tuple<float,float>,std::tuple<float,float>>> FieldDPlanner::getEdgeConnbrs(std::tuple<float,float> p)
{
    std::vector<std::tuple<float,float>> neighbors;
    std::vector<std::pair<std::tuple<float,float>,std::tuple<float,float>>> connbrs;

    float x,y;
    std::tie(x,y) = p;

    float horizontal_dist_pos, horizontal_dist_neg; // positive and negative distances in the horizontal direction
    float vertical_dist_pos, vertical_dist_neg; // positive and negative distances in the vertical direction

    if (y != floorf(y)) // p lies on an edge along the y axis
    {
        horizontal_dist_pos = 1.0f;
        horizontal_dist_neg = -1.0f;
        vertical_dist_pos = igvc::get_distance(p,std::make_tuple(x, ceilf(y)));
        vertical_dist_neg = -igvc::get_distance(p,std::make_tuple(x, floorf(y)));
    }
    else // p lies on an edge along the x axis
    {
        horizontal_dist_pos = igvc::get_distance(p,std::make_tuple(ceilf(x), y));
        horizontal_dist_neg = -igvc::get_distance(p,std::make_tuple(floorf(x), y));
        vertical_dist_pos = 1.0f;
        vertical_dist_neg = -1.0f;
    }

    // there are 8 consecutive neighbors for an edge node.
    neighbors.push_back(std::make_tuple(x + horizontal_dist_pos, y)); // right
    neighbors.push_back(std::make_tuple(x + horizontal_dist_pos, y + vertical_dist_pos)); // top right
    neighbors.push_back(std::make_tuple(x, y + vertical_dist_pos)); // top
    neighbors.push_back(std::make_tuple(x + horizontal_dist_neg, y + vertical_dist_pos)); // top left
    neighbors.push_back(std::make_tuple(x + horizontal_dist_neg, y)); // left
    neighbors.push_back(std::make_tuple(x + horizontal_dist_neg, y + vertical_dist_neg)); // bottom left
    neighbors.push_back(std::make_tuple(x, y + vertical_dist_neg)); // bottom
    neighbors.push_back(std::make_tuple(x + horizontal_dist_pos, y + vertical_dist_neg)); // bottom right

    std::tuple<float,float> sp;
    std::tuple<float,float> spp;
    // first 7 connbrs
    for (size_t i = 0; i < neighbors.size() - 1; i++)
    {
        if (graph.isValidPosition(neighbors[i]) && graph.isValidPosition(neighbors[i+1]))
            connbrs.push_back(std::make_pair(neighbors[i],neighbors[i+1]));
    }

    // last connbrs pair [s8->s1]
    if (graph.isValidPosition(neighbors[neighbors.size() - 1]) && graph.isValidPosition(neighbors[0]))
        connbrs.push_back(std::make_pair(neighbors[neighbors.size() - 1],neighbors[0]));

    return connbrs;
}

void FieldDPlanner::insert_or_assign(Node s, float g, float rhs)
{
    // re-assigns value of node in unordered map or inserts new entry if
    // node not found
    if (umap.find(s) != umap.end())
        umap.erase(s);

    umap.insert(std::make_pair(s, std::make_tuple(g, rhs)));
}

bool FieldDPlanner::isWithinRangeOfGoal(std::tuple<float,float> p)
{
    float goal_x, goal_y;
    std::tie(goal_x, goal_y) = graph.Goal.getIndex();

    float x, y;
    std::tie(x,y) = p;
    // calculate number of nodes that lie within the goal distance
    float sep = GOAL_DIST/graph.Resolution;

    bool satisfies_x_bounds = (x >= (goal_x - sep)) && (x <= (goal_x + sep));
    bool satisfies_y_bounds = (y >= (goal_y - sep)) && (y <= (goal_y + sep));

    return satisfies_x_bounds && satisfies_y_bounds;
}

float FieldDPlanner::getG(Node s)
{
    // return g value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (umap.find(s) != umap.end())
        return std::get<0>(umap.at(s));
    else
        return std::numeric_limits<float>::infinity();
}

float FieldDPlanner::getRHS(Node s)
{
    // return rhs value if node has been looked at before (is in unordered map)
    // otherwise, return infinity
    if (umap.find(s) != umap.end())
        return std::get<1>(umap.at(s));
    else
        return std::numeric_limits<float>::infinity();
}

std::vector<std::tuple<int,int>> FieldDPlanner::getExplored()
{
    std::vector<std::tuple<int,int>> explored;
    for (std::pair<Node, std::tuple<float,float>> e : umap)
        explored.push_back(e.first.getIndex());

    return explored;
}
