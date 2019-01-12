#include "FieldDPlanner.h"

FieldDPlanner::FieldDPlanner() {}
FieldDPlanner::~FieldDPlanner() {}

Key FieldDPlanner::calculateKey(Node s)
{
    // calculate the key for node s. The key is used to order nodes in Ø
    float g_value = std::min(getG(s), getRHS(s));
    float f_value = g_value + graph.euclidian_heuristic(s) + graph.K_M;
    return Key(f_value, g_value);
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
    float c = graph.getC(s, s2);
    // traversal cost of node s and s1, a non-diaginal neighbor of s
    // in units of (cost/distance)
    float b = graph.getB(s,s1);

    float g_s1 = getG(s1); // path cost of edge neighbor
    float g_s2 = getG(s2); // path cost of diagonal neighbor

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
                return c * graph.DIAGONAL_DISTANCE + g_s2;
            }
            else
            {
                // travel along diagonal to point along edge
                float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                float y = std::min(toComp, 1.0f);
                return c * sqrt(1 + std::pow(y,2.0f)) + (f * (1.0f - y)) + g_s2;
            }
        }
        else
        {
            if (c <= b)
            {
                // cheapest to go directly to diagonal cell
                return c *  graph.DIAGONAL_DISTANCE + g_s2;
            }
            else
            {
                // travel along edge then to s2
                float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                float x = 1 - std::min(toComp, 1.0f);
                return c * sqrt(1.0f + std::pow(1.0f - x, 2.0f)) + (b * x) + g_s2;
            }
        }
    }
}

void FieldDPlanner::initialize()
{
    // g(s_start) = rhs(s_start) = inf
    umap.insert(std::make_pair(graph.Start, std::make_tuple(
        std::numeric_limits<float>::infinity(),
        std::numeric_limits<float>::infinity())));
    // g(s_goal) = inf; rhs(s_goal) = 0;
    umap.insert(std::make_pair(graph.Goal, std::make_tuple(
        std::numeric_limits<float>::infinity(),
        0.0f)));

    // insert s_goal into Ø with key(s_goal)
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
    // s never visited before, add to unordered map with g(s) = rhs(s) = inf
    if (umap.find(s) == umap.end())
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

            // add connbrs to map if they were never visited before
            if (umap.find(s_a) == umap.end())
                umap.insert(std::make_pair(s_a, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));
            if (umap.find(s_b) == umap.end())
                umap.insert(std::make_pair(s_b, std::make_tuple(std::numeric_limits<float>::infinity(), std::numeric_limits<float>::infinity())));

            minRHS = std::min(minRHS, this->computeCost(s, s_a, s_b));

        }
        // update Node s' RHS value
        insert_or_assign(s, getG(s), minRHS);
    }

    // insert node into Ø if it is locally inconsistent
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

    std::tuple<float,float> curr_pos = graph.Start.getIndex();
    path.push_back(curr_pos);

    float min_cost;
    path_additions pa;

    std::cout << "Starting search: Traversed {";

    int MAX_SIZE = 50;
    int curr_step = 0;

    do
    {
        if (isVertex(curr_pos))
            pa = getNextPositionsFromVertex(curr_pos);
        else
            pa = getNextPositionsFromEdge(curr_pos);

        // append new positions to the end of path
        path.insert(path.end(), pa.first.begin(), pa.first.end());
        min_cost = pa.second;

        curr_pos = path.back();
        curr_step += 1;

        std::cout<< "<" << std::get<0>(curr_pos) << ", " << std::get<1>(curr_pos) << ">, " ;
    } while (!isWithinRangeOfGoal(curr_pos) && (min_cost != std::numeric_limits<float>::infinity()) && (curr_step < MAX_SIZE));

    if (min_cost == std::numeric_limits<float>::infinity())
        path.clear();

    std::cout << "} search ended" << std::endl;

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
        float c = graph.getC(s,s2);
        // traversal cost of node s and s1, a non-diaginal neighbor of s
        float b = graph.getB(s,s1);

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

FieldDPlanner::path_additions FieldDPlanner::getNextPositionsFromEdge(std::tuple<float,float> p)
{
    std::vector<ContinuousNeighbors> CNs = getContinuousNeighbors(p);

    // temporary variables to hold during search.
    std::vector<std::tuple<float,float>> temp_positions;
    float temp_cost;

    // variables corresponding to minimum cost path from p
    std::vector<std::tuple<float,float>> min_positions;
    float min_cost = std::numeric_limits<float>::infinity();

    for (ContinuousNeighbors CN : CNs)
    {
        temp_positions.clear();

        std::tuple<float,float> s1; // non-diagonal neighbor
        std::tuple<float,float> s2; // diagonal neighbor

        float g_s1, g_s2;

        // determine which neighbor lies along the edge
        if ((std::get<0>(p) == std::get<0>(CN.s_a)) || (std::get<1>(p) == std::get<1>(CN.s_a)))
        {
            s1 = CN.s_a;
            g_s1 = CN.cost_s_a;
            s2 = CN.s_b;
            g_s2 = CN.cost_s_b;
        }
        else
        {
            s1 = CN.s_b;
            g_s1 = CN.cost_s_b;
            s2 = CN.s_a;
            g_s2 = CN.cost_s_a;
        }

        // these variables are used to calculate how the linear interpolation
        // calculations (temp_x, temp_y) should be oriented as to add positions
        // to the path.
        bool neighbors_horizontal = (std::get<1>(s1) == std::get<1>(s2)) ? true : false;
        int x_scaler,y_scaler;

        if (neighbors_horizontal)
        {
            x_scaler = ((std::get<1>(s1) - std::get<1>(p)) > 0) ? 1 : -1;
            y_scaler = ((std::get<0>(s2) - std::get<0>(s1)) > 0) ? 1 : -1;
        }
        else
        {
            x_scaler = ((std::get<0>(s1) - std::get<0>(p)) > 0) ? 1 : -1;
            y_scaler = ((std::get<1>(s2) - std::get<1>(s1)) > 0) ? 1 : -1;
        }

        // get traversal costs to both neighbors
        float c = graph.getContinuousTraversalCost(p, s2);
        float b = graph.getContinuousTraversalCost(p, s1);

        float dist_s1 = igvc::get_distance(p, s1); // distance to nearest neighbor
        float dist_s2 = igvc::get_distance(p, s2); // distance to diagonal neighbor
        float dist_neighbors = igvc::get_distance(s1, s2); // distance between neighbors

        if (std::min(c,b) == std::numeric_limits<float>::infinity())
        {
            // infinite traversal cost, cells likely occupied
            temp_cost = std::numeric_limits<float>::infinity();
        }
        else if (g_s1 <= g_s2)
        {
            // cheapest to travel directly to nearest neighbor (non-diagonal, s1)
            temp_cost = std::min(c,b) + g_s1;
            temp_positions.push_back(s1);
        }
        else
        {
            float f = g_s1 - g_s2; // cost of going from s1 to s2

            if (f <= b)
            {
                if (c <= f)
                {
                    // cheapest to go directly to diagonal neighbor, s2
                    temp_cost = c * dist_s2 + g_s2;
                    temp_positions.push_back(s2);
                }
                else
                {
                    // travel along diagonal to point along edge
                    float toComp = f / sqrt(std::pow(c,2.0f) - std::pow(f,2.0f));
                    float temp_y = std::min(toComp, dist_neighbors);
                    temp_cost = c * sqrt(std::pow(dist_s1,2.0f) + std::pow(temp_y,2.0f)) + (f * (dist_neighbors - temp_y)) + g_s2;

                    float x, y;
                    if (neighbors_horizontal)
                    {
                        y = std::get<1>(s1);
                        x = std::get<0>(s1) + y_scaler * temp_y;
                    }
                    else
                    {
                        x = std::get<0>(s1);
                        y = std::get<1>(s1) + y_scaler * temp_y;
                    }

                    temp_positions.push_back(std::make_pair(x, y));
                }
            }
            else
            {
                if (c <= b)
                {
                    // cheapest to go directly to diagonal cell
                    temp_cost = c * dist_s2 + g_s2;
                    temp_positions.push_back(s2);
                }
                else
                {
                    // travel along edge then to s2. In practice, this results
                    // in the addition of two points to the path.
                    float toComp = b / sqrt(std::pow(c,2.0f) - std::pow(b,2.0f));
                    float temp_x = dist_s1 - std::min(toComp, dist_s1);
                    temp_cost = c * sqrt(std::pow(dist_neighbors,2.0f) + std::pow(dist_s1 - temp_x, 2.0f)) + (b * temp_x) + g_s2;

                    if (neighbors_horizontal)
                        temp_positions.push_back(std::make_tuple(std::get<0>(p), std::get<1>(p) + x_scaler * temp_x));
                    else
                        temp_positions.push_back(std::make_tuple(std::get<0>(p) + x_scaler * temp_x, std::get<1>(p)));

                    temp_positions.push_back(s2);
                }
            }
        }

        if (temp_cost < min_cost)
        {
            min_cost = temp_cost;
            min_positions = temp_positions;
        }
    }

    return std::make_pair(temp_positions, min_cost);
}

std::vector<ContinuousNeighbors> FieldDPlanner::getContinuousNeighbors(std::tuple<float,float> p)
{
    std::vector<ContinuousNeighbors> CNs;

    // determine whether p lies on an edge along the x axis (y is an integer)
    // or on an edge along the y axis (x is an integer)
    float x,y;
    std::tie(x,y) = p;

    ContinuousNeighbors CN;
    std::tuple<float,float> s_a;
    std::tuple<float,float> s_b;
    float cost_s_a;
    float cost_s_b;

    // if y is an integer this means p lies along an x edge
    if (ceilf(y) == y)
    {
        // imaginary nodes and their linearly interpolated path cost
        float scaler = fmodf(std::abs(x),floorf(std::abs(x))); // how far along the edge p lies relative to neighbors
        if (x < 0)
            scaler = (1 - scaler);
        std::tuple<float,float> in1 = std::make_tuple(x,y+1); // imaginary node 1
        float cost_in1 = scaler * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(ceilf(x),y+1)))) \
                         + (1 - scaler) * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(floorf(x),y+1)))); // linear interpolation happens here
        std::tuple<float,float> in2 = std::make_tuple(x,y-1); // imaginary node 2
        float cost_in2 = scaler * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(ceilf(x),y-1)))) \
                         + (1 - scaler) * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(floorf(x),y-1)))); // linear interpolation happens here

        // 8 possible pairs of continuous neighbors when node lies along an edge
        s_a = std::make_tuple(ceilf(x),y);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(ceilf(x),y+1);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(ceilf(x),y+1);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        CN.setValues(s_a, in1, cost_s_a, cost_in1);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_b = std::make_tuple(floorf(x), y+1);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(in1, s_b, cost_in1, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(floorf(x),y+1);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(floorf(x),y);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(floorf(x),y);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(floorf(x),y-1);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(floorf(x),y-1);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        CN.setValues(s_a, in2, cost_s_a, cost_in2);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_b = std::make_tuple(ceilf(x), y-1);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(in2, s_b, cost_in2, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(ceilf(x),y-1);
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(ceilf(x),y);
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);
    }
    else
    {
        // imaginary nodes and their linearly interpolated path cost
        float scaler = fmodf(std::abs(y),floorf(std::abs(y))); // how far along the edge p lies relative to neighbors
        if (y < 0)
            scaler = (1 - scaler);
        std::tuple<float,float> in1 = std::make_tuple(x+1,y); // imaginary node 1
        float cost_in1 = scaler * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(x+1,ceilf(y))))) \
                         + (1 - scaler) * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(x+1,floorf(y))))); // linear interpolation happens here
        std::tuple<float,float> in2 = std::make_tuple(x-1,y); // imaginary node 2
        float cost_in2 = scaler * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(x-1,ceilf(y))))) \
                         + (1 - scaler) * getG(Node(static_cast<std::tuple<int,int>>(std::make_tuple(x-1,floorf(y))))); // linear interpolation happens here

        s_a = std::make_tuple(x,floorf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(x+1,floorf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(x+1,floorf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        CN.setValues(s_a, in1, cost_s_a, cost_in1);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_b = std::make_tuple(x+1, ceilf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(in1, s_b, cost_in1, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(x+1, ceilf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(x, ceilf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(x,ceilf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(x-1,ceilf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(x-1,ceilf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        CN.setValues(s_a, in2, cost_s_a, cost_in2);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_b = std::make_tuple(x-1, floorf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(in2, s_b, cost_in2, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);

        s_a = std::make_tuple(x-1,floorf(y));
        cost_s_a = getG(Node(static_cast<std::tuple<int,int>>(s_a)));
        s_b = std::make_tuple(x,floorf(y));
        cost_s_b = getG(Node(static_cast<std::tuple<int,int>>(s_b)));
        CN.setValues(s_a, s_b, cost_s_a, cost_s_b);
        if (isValidContinuousNeighbors(CN))
            CNs.push_back(CN);
    }

    return CNs;
}

bool FieldDPlanner::isValidContinuousNeighbors(ContinuousNeighbors CN)
{
    return graph.isValidPosition(CN.s_a) && graph.isValidPosition(CN.s_b);
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

std::ostream& operator<<(std::ostream& stream, const ContinuousNeighbors& CN)
{
    stream << "[<" << CN.s_a << ", " << CN.s_b << ">, c: <" << CN.cost_s_a << ", " << CN.cost_s_b << ">]";
    return stream;
}
