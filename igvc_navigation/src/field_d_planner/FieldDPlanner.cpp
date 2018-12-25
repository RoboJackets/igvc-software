#include "FieldDPlanner.h"

std::tuple<float, float> FieldDPlanner::key(Node s)
{
    float f_value = std::min(s.getCost(), s.getRHS()) + graph.euclidian_heuristic(s) + graph.K_M;
    float g_value = std::min(s.getCost(), s.getRHS());

    return std::make_tuple(f_value, g_value);
}

float FieldDPlanner::computeCost(Node s, Node s_prime, Node s_pp)
{
    Node s1;
    Node s2;

    float v_s; // cost

    if (graph.isDiagonal(s, s_prime))
    {
        s1 = s_pp;
        s2 = s_prime;
    }
    else
    {
        s1 = s_prime;
        s2 = s_pp;
    }

    float c = graph.getC(s, s2); // diagonal traversal cost
    float b = graph.getB(s, s1); // edge traversal cost

    if (std::min(c,b) == std::numeric_limits<float>::infinity())
    {
        v_s = std::numeric_limits<float>::infinity();
    }
    else if (s1.getCost() <= s2.getCost())
    {
        v_s = std::min(c,b) + s1.getCost();
    }
    else
    {
        float f = s1.getCost() - s2.getCost();
        if (f <= b)
        {
            if (c <= f)
            {
                v_s = c * ((float) sqrt(2)) + s2.getCost();
            }
            else
            {
                float arg1 = f / sqrt(pow(c,2) - pow(f,2));
                float y = std::min(arg1, (float) 1);
                v_s = c * sqrt(1 + pow(y,2)) + f * (1-y) + s2.getCost();
            }
        }
        else
        {
            if (c <= b)
            {
                v_s = c * ((float) sqrt(2)) + s2.getCost();
            }
            else
            {
                float arg1 = b / sqrt(pow(c,2) - pow(b,2));
                float x = 1 - std::min(arg1, (float) 1);
                v_s = c * sqrt(1 + pow((1-x),2)) + b * x + s2.getCost();
            }
        }
    }

    return v_s;
}

void FieldDPlanner::initialize()
{
    Node start = graph.Start;
    Node goal  = graph.Goal;

    start.setCost(std::numeric_limits<float>::infinity());
    start.setRHS(std::numeric_limits<float>::infinity());
    goal.setCost(std::numeric_limits<float>::infinity());

    goal.setRHS(0);

    float goal_f, goal_g;
    std::tie(goal_f, goal_g) = this->key(goal);

    // insert locally inconsistent state into priority queue
    PQ.insert(State(goal.getIndex(), goal_f, goal_g));
    // insert start and goal into HashSet
    US.insert(start);
    US.insert(goal);
}

void FieldDPlanner::getNode(Node &to_get, Node ref_node)
{
    if (US.find(ref_node) == US.end())
        to_get = Node(std::get<0>(ref_node.getIndex()), std::get<1>(ref_node.getIndex()));
    else
        to_get = *US.find(ref_node);
}

void FieldDPlanner::updateState(Node s)
{
    this->getNode(s,s);

    // calculate
    float minRHS = std::numeric_limits<float>::infinity();
    if (s.getIndex() != graph.Goal.getIndex())
    {
        Node connbr1;
        Node connbr2;
        for (std::tuple<Node,Node> connbr : graph.connbrs(s))
        {
            // retreive values of neighbors if they already exist
            this->getNode(connbr1, std::get<0>(connbr));
            this->getNode(connbr2, std::get<1>(connbr));
            float currRHS = this->computeCost(s, connbr1, connbr2);
            if (currRHS <= minRHS) minRHS = currRHS;
        }
    }
    s.setRHS(minRHS);
    /**
    Searches for a state and removes it if found. Does nothing if state not found.
    Same thing as calling:
    if PQ.contains(s.getIndex())
        PQ.remove(s.getIndex())
    */
    PQ.remove(s.getIndex());

    // state is locally inconsistent
    if (s.getCost() != s.getRHS())
    {
        float f, g;
        std::tie(f, g) = this->key(s);
        PQ.insert(State(s.getIndex(), f, g));
    }

    // add/update node in hashSet
    US.erase(s);
    US.insert(s);
}

void FieldDPlanner::computeShortestPath()
{
    // get the top key and popped node from the priority queue
    State topState = PQ.top();
    std::tuple<float,float> topKey = std::make_tuple(topState.f, topState.g);
    Node poppedNode;
    this->getNode(poppedNode, Node(std::get<0>(topState.ind), std::get<1>(topState.ind)));


    std::tuple<int,int> startInd = graph.Start.getIndex();
    Node start;
    this->getNode(start, Node(std::get<0>(startInd) , std::get<1>(startInd)));

    while(graph.keyLessThanEq(topKey, this->key(start)) || (start.getRHS() != start.getCost()))
    {
        PQ.pop(); // remove lowest cost state from priority queue

        if (poppedNode.getCost() > poppedNode.getRHS())
        {
            // state has become favorable.
            poppedNode.setCost(poppedNode.getRHS());

            // updates nodes values in hashed set
            US.erase(poppedNode);
            US.insert(poppedNode);

            // update all neighboring nodes to make them consistent
            for (Node neighbor : graph.nbrs(poppedNode))
                this->updateState(neighbor);
        }
        else
        {
            // node is locally inconsistent
            poppedNode.setCost(std::numeric_limits<float>::infinity());

            // update nodes values in hashed set
            US.erase(poppedNode);
            US.insert(poppedNode);

            // update state of the popped node and its neighbors
            for (Node neighbor : graph.nbrs(poppedNode))
                this->updateState(neighbor);
            this->updateState(poppedNode);
        }
    }
}
