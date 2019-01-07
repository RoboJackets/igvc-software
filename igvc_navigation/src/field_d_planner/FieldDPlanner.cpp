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

    float g_s1 = getG(s1);
    float g_s2 = getG(s2);

    if (std::min(c,b) == std::numeric_limits<float>::infinity())
    {
        // infinite traversal cost, cells likely occupied
        return std::numeric_limits<float>::infinity();
    }
    else if (g_s1 <= g_s2)
    {
        // cheapest to travel directly to nearest neighbor (non-diagonal)
        return std::min(c,b) + g_s1;
    }
    else
    {
        float f = g_s1 - g_s2; // cost of going from s1 to s2

        if (f <= b)
        {
            if (c <= f)
            {
                // cheapest to go directly to diagonal cell
                return c * sqrt(2.0f) + g_s2;
            }
            else
            {
                // travel along diagonal to point along edge
                float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                float y = std::min(toComp, 1.0f);
                return c * sqrt(1 + std::pow(y,2.0f)) + (f * (1 - y)) + g_s2;
            }
        }
        else
        {
            if (c <= b)
            {
                // cheapest to go directly to diagonal cell
                return c * sqrt(2.0f) + g_s2;
            }
            else
            {
                // travel along edge then to s2
                float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                float x = 1 - std::min(toComp, 1.0f);
                return c * sqrt(1 + std::pow(1 - x, 2.0f)) + (b * x) + g_s2;
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

    float min_cost;

    if (isVertex(curr_pos))
    {
        path_additions pa = getNextPositionsFromVertex(curr_pos);
        path.insert(path.end(), pa.first.begin(), pa.first.end());
        min_cost = pa.second;
    }

    // do
    // {
    //     start_it = path.begin();
    // } while(!isWithinRangeOfGoal(curr_pos) && (minCost != std::numeric_limits<float>::infinity()));

    if (min_cost == std::numeric_limits<float>::infinity())
        path.clear();
}

bool FieldDPlanner::isVertex(std::tuple<float,float> p)
{
    float x,y;
    std::tie(x,y) = p;

    bool bothInts = (ceilf(x) == x) && (ceilf(y) == y);
    bool satisfiesXBounds = (x >= 0) && (x <= graph.length);
    bool satisfiesYBounds = (y >= 0) && (y <= graph.width);

    return bothInts && satisfiesXBounds && satisfiesYBounds;
}

FieldDPlanner::path_additions FieldDPlanner::getNextPositionsFromVertex(std::tuple<float,float> p)
{
    std::vector<std::tuple<float,float>> positions; // positions to add to path

    Node s = Node(static_cast<std::tuple<int,int>>(p)); // start node

    // temporary variables to hold during search.
    float temp_x, temp_y;
    float temp_cost;

    // variables corresponding to minimum cost path from s
    Node min_s1, min_s2;
    float min_x, min_y;
    float min_cost = std::numeric_limits<float>::infinity();

    for (std::tuple<Node,Node> connbr : graph.connbrs(s))
    {
        // consecutive neighbors
        Node s_a, s_b;
        std::tie(s_a, s_b) = connbr;

        Node s1, s2;
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
        float c = graph.getTraversalCost(s,s2);
        // traversal cost of node s and s1, a non-diaginal neighbor of s
        float b = graph.getTraversalCost(s,s1);

        float g_s1 = getG(s1);
        float g_s2 = getG(s2);

        if (std::min(c,b) == std::numeric_limits<float>::infinity())
        {
            // infinite traversal cost, cells likely occupied
            temp_cost = std::numeric_limits<float>::infinity();
            temp_x = temp_y = 0.0f;
        }
        else if (g_s1 <= g_s2)
        {
            // cheapest to travel directly to nearest neighbor (non-diagonal)
            temp_cost = std::min(c,b) + g_s1;
            temp_x = 1.0f;
            temp_y = 0.0f;
        }
        else
        {
            float f = g_s1 - g_s2; // cost of going from s1 to s2

            if (f <= b)
            {
                if (c <= f)
                {
                    // cheapest to go directly to diagonal cell
                    temp_cost = c * sqrt(2.0f) + g_s2;
                    temp_x = temp_y = 1.0f;
                }
                else
                {
                    // travel along diagonal to point along edge
                    float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                    temp_x = 1.0f;
                    temp_y = std::min(toComp, 1.0f);
                    temp_cost = c * sqrt(1 + std::pow(temp_y,2.0f)) + (f * (1 - temp_y)) + g_s2;
                }
            }
            else
            {
                if (c <= b)
                {
                    // cheapest to go directly to diagonal cell
                    temp_cost = c * sqrt(2.0f) + g_s2;
                    temp_x = temp_y = 1.0f;
                }
                else
                {
                    // travel along edge then to s2. In practice, this results
                    // in the addition of two points to the path. one for
                    // (orig_x + x, orig_y) and another for (orig_x + 1, orig_y + 1)
                    float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                    temp_x = 1 - std::min(toComp, 1.0f);
                    temp_y = 0.0f;
                    temp_cost = c * sqrt(1 + std::pow(1 - temp_x, 2.0f)) + (b * temp_x) + g_s2;
                }
            }
        }

        // obtain the minimum cost traversal
        if (temp_cost < min_cost)
        {
            min_cost = temp_cost;
            min_x = temp_x;
            min_y = temp_y;
            min_s1 = s1;
            min_s2 = s2;
        }
    }

    // CASE 0: no valid path found (infinite cost/no consecutive neighbors)
    if (min_cost == std::numeric_limits<float>::infinity())
    {
        return std::make_pair(positions, min_cost);
    }

    // calculate the multiplier for the positions to be added to the path. This
    // step is required because x and y calculations are peformed independently of
    // the consecutive neighbors used to obtain these values. As such, x_multiplier
    // and y_multiplier account for this.
    int s_x, s_y;
    std::tie(s_x, s_y) = s.getIndex();
    int s1_x, s1_y;
    std::tie(s1_x, s1_y) = min_s1.getIndex();
    int s2_x, s2_y;
    std::tie(s2_x, s2_y) = min_s2.getIndex();

    int x_multiplier,y_multiplier;
    bool flip = false;

    if (s1_x != s_x) // nearest neighbor lies to left or right of s
    {
        x_multiplier = ((s1_x - s_x) > 0) ? 1 : -1;
        y_multiplier = ((s2_y - s_y) > 0) ? 1 : -1;
    }
    else // nearest neighbor lies above of below s
    {
        flip = true;
        y_multiplier = ((s1_y - s_y) > 0) ? 1 : -1;
        x_multiplier = ((s2_x - s_x) > 0) ? 1 : -1;
    }

    if ((min_x > 0) && (min_x < 1) && (min_y == 0.0f))
    {
        // CASE 1: travel along x then cut to s2
        if (flip)
        {
            float temp = min_x;
            min_x = min_y;
            min_y = temp;
        }
        min_x *= x_multiplier;
        min_y *= y_multiplier;
        positions.push_back(std::make_tuple(s_x + min_x, s_y + min_y));
        positions.push_back(std::make_tuple(s_x + x_multiplier, s_y + y_multiplier));
    }
    else
    {
        // CASE 2: travel directly to diagonal node (s2)
        // CASE 3: travel to nearest node
        // CASE 4: travel to point along edge
        if (flip)
        {
            float temp = min_x;
            min_x = min_y;
            min_y = temp;
        }
        min_x *= x_multiplier;
        min_y *= y_multiplier;
        positions.push_back(std::make_tuple(s_x + min_x, s_y + min_y));
    }

    return std::make_pair(positions, min_cost);
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
