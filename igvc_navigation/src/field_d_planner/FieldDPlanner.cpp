#include "FieldDPlanner.h"

FieldDPlanner::FieldDPlanner() {}
FieldDPlanner::~FieldDPlanner() {}

Key FieldDPlanner::calculateKey(Node s)
{
    // obtain g-values and rhs-values for node s
    float min_cost = std::min(getG(s), getRHS(s));
    return Key(min_cost + graph.euclidian_heuristic(s) + graph.K_M, min_cost);
}

float FieldDPlanner::computeCost(Node s, Node s_a, Node s_b)
{
    Node s1; // non-diagonal neighbor
    Node s2; // diagonal neighbor

    // return the path cost of node s

    if (graph.isDiagonal(s, s_a))
    {
        s1 = s_b;
        s2 = s_a;
    }
    else
    {
        s1 = s_a;
        s2 = s_b;
    }
    // traversal cost of node s and a diagonal node s2
    // in units of (cost/distance)
    float c = graph.getTraversalCost(s,s2);
    // traversal cost of node s and s1, a non-diaginal neighbor of s
    // in units of (cost/distance)
    float b = graph.getTraversalCost(s,s1);
    if (std::min(c,b) == std::numeric_limits<float>::infinity())
    {
        // infinite traversal cost, cells likely occupied
        return std::numeric_limits<float>::infinity();
    }
    else if (getG(s1) <= getG(s2))
    {
        // cheapest to travel directly to nearest neighbor (non-diagonal)
        return std::min(c,b) + getG(s1);
    }
    else
    {
        float f = getG(s1) - getG(s2); // cost of going from s1 to s2

        if (f <= b)
        {
            if (c <= f)
            {
                // cheapest to go directly to diagonal cell
                return c * sqrt(2.0f) + getG(s2);
            }
            else
            {
                // travel along diagonal to point along edge
                float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                float y = std::min(toComp, 1.0f);
                return c * sqrt(1 + std::pow(y,2.0f)) + (f * (1 - y)) + getG(s2);
            }
        }
        else
        {
            if (c <= b)
            {
                // cheapest to go directly to diagonal cell
                return c * sqrt(2.0f) + getG(s2);
            }
            else
            {
                // travel along edge then to s2
                float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                float x = 1 - std::min(toComp, 1.0f);
                return c * sqrt(1 + std::pow(1 - x, 2.0f)) + (b * x) + getG(s2);
            }
        }
    }
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
    looks for a node in the priority queue and removes it if found
    same as calling: if PQ.contains(s) PQ.remove(s);
    */
    PQ.remove(s);

    if (s != graph.Goal)
    {
        float minRHS = std::numeric_limits<float>::infinity();

        Node s_a, s_b;

        for (std::tuple<Node, Node> connbr : graph.connbrs(s))
        {
            std::tie(s_a, s_b) = connbr;

            if (!(umap.find(s_a) == umap.end()) && !(umap.find(s_b) == umap.end()))
                minRHS = std::min(minRHS, this->computeCost(s, s_a, s_b));
            else
                continue;
        }
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
        PQ.pop();
        numNodesExpanded++;

        float g = getG(topNode);
        float rhs = getRHS(topNode);

        if (g > rhs)
        {
            // locally overconsistent case. This node is now more favorable.
            // make node locally consistent by setting g = rhs
            insert_or_assign(topNode, rhs, rhs);
            for (Node nbr : graph.nbrs(topNode)) updateNode(nbr);
        }
        else
        {
            // locally underconsistent case. This node is now less favorable
            // than it was before

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

    for (std::tuple<int,int> cellUpdate : graph.updatedCells)
    {
        for (Node n : graph.getNodesAroundCellWithCSpace(cellUpdate))
        {
            if (umap.find(n) == umap.end()) // node hasn't been explored yet. Leave alone
                continue;

            updateNode(n);
            numNodesUpdated++;
        }
    }
    return numNodesUpdated;
}

void FieldDPlanner::constructOptimalPath()
{
    path.clear();
    // iterator to the start of the path vector
    std::vector<std::tuple<float,float>>::iterator start_it;

    std::tuple<float,float> curr_pos = graph.Start.getIndex();

    float minCost;

    do
    {
        start_it = path.begin();


    } while(!isWithinRangeOfGoal(curr_pos) && (minCost != std::numeric_limits<float>::infinity()));

    if (minCost == std::numeric_limits<float>::infinity())
        path.clear();
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

    bool satisfiesXBounds = (x >= (goal_x - sep)) && (x <= (goal_x + sep));
    bool satisfiesYBounds = (y >= (goal_y - sep)) && (y <= (goal_y + sep));

    return satisfiesXBounds && satisfiesYBounds;
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
    {
        explored.push_back(e.first.getIndex());
    }

    return explored;
}
