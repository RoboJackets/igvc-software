#ifndef GRAPHSEARCH_HPP_INCLUDED
#define GRAPHSEARCH_HPP_INCLUDED

#include <set>
#include <list>
#include <stack>
#include <queue>
#include <iostream>
#include <algorithm>

#include "SearchProblem.hpp"

using namespace std;

template <class StateType, class ActionType>
class PathComparator
{
private:
    SearchProblem<StateType, ActionType>* _problem;
public:
    PathComparator(SearchProblem<StateType, ActionType>* problem)
    {
        _problem = problem;
    }

    bool operator () (Path<StateType, ActionType>& p1, Path<StateType, ActionType>& p2)
    {
        double c1 = _problem->getPathCost(&p1);
        c1 += _problem->getHeuristicCost(p1.getLastState());

        double c2 = _problem->getPathCost(&p2);
        c2 += _problem->getHeuristicCost(p2.getLastState());

        return ( c1 > c2 );
    }
};

class GraphSearch
{

public:
    /** Runs Depth-First graph search on the given search problem */
    template <class StateType, class ActionType>
    static Path<StateType, ActionType> DFS(SearchProblem<StateType, ActionType> &problem)
    {
        set<StateType> expanded;
        stack< Path<StateType, ActionType> > frontier;

        {
            Path<StateType, ActionType> p;
            p.addState(problem.getStartState());
            frontier.push(p);
        }

        while(!frontier.empty())
        {
            Path<StateType, ActionType> path = frontier.top();
            frontier.pop();

            if( expanded.find(path.getLastState()) == expanded.end() )// expanded does not contain path's last state
            {
                expanded.insert(path.getLastState());

                if(problem.isGoal(path.getLastState()))
                {
                    return path;
                }
                list<ActionType> legalActions = problem.getActions(path.getLastState());
                StateType last = path.getLastState();

                for( typename list<ActionType>::iterator it = legalActions.begin(); it != legalActions.end(); it++)
                {
                    ActionType action = (*it);
                    StateType result = problem.getResult(last, action);

                    Path<StateType, ActionType> newPath(path);
                    newPath.addAction(action);
                    newPath.addState(result);
                    frontier.push(newPath);
                }
            }
        }

        cout << __func__ << " Error: Could not find a solution." << endl;
        Path<StateType, ActionType> empty;
        return empty;
    }

    /** Runs Breadth-First graph on the given search problem */
    template <class StateType, class ActionType>
    static Path<StateType, ActionType> BFS(SearchProblem<StateType, ActionType> &problem)
    {
        set<StateType> expanded;
        queue< Path<StateType, ActionType> > frontier;

        {
            Path<StateType, ActionType> p;
            p.addState(problem.getStartState());
            frontier.push(p);
        }

        while(!frontier.empty())
        {
            Path<StateType, ActionType> path = frontier.front();
            frontier.pop();

            if( expanded.find(path.getLastState()) == expanded.end() ) // expanded does not contain path's last state
            {

                expanded.insert(path.getLastState());

                if(problem.isGoal(path.getLastState()))
                {
                    return path;
                }
                list<ActionType> legalActions = problem.getActions(path.getLastState());
                StateType last = path.getLastState();

                for( typename list<ActionType>::iterator it = legalActions.begin(); it != legalActions.end(); it++)
                {
                    ActionType action = (*it);
                    StateType result = problem.getResult(last, action);

                    Path<StateType, ActionType> newPath(path);
                    newPath.addAction(action);
                    newPath.addState(result);
                    frontier.push(newPath);
                }
            }
        }

        cout << __func__ << " Error: Could not find a solution." << endl;
        Path<StateType, ActionType> empty;
        return empty;
    }

    /** Runs A* graph search on the given search problem */
    template <class StateType, class ActionType>
    static Path<StateType, ActionType> AStar(SearchProblem<StateType, ActionType> &problem)
    {

        set<StateType> expanded;
        priority_queue< Path<StateType, ActionType>, vector<Path<StateType, ActionType> >, PathComparator<StateType, ActionType> > frontier((PathComparator<StateType,ActionType>(&problem)));

        {
            Path<StateType, ActionType> p;
            p.addState(problem.getStartState());
            frontier.push(p);
        }

        while(!frontier.empty())
        {
            Path<StateType, ActionType> path = frontier.top();
            frontier.pop();

            if( expanded.find(path.getLastState()) == expanded.end() ) // expanded does not contain path's last state
            {
                StateType last = path.getLastState();
                expanded.insert(last);

                if(problem.isGoal(last))
                {
                    return path;
                }
                list<ActionType> legalActions = problem.getActions(last);

                for( typename list<ActionType>::iterator it = legalActions.begin(); it != legalActions.end(); it++)
                {
                    ActionType action = (*it);
                    StateType result = problem.getResult(last, action);
                    Path<StateType, ActionType> newPath(path);
                    newPath.addAction(action);
                    newPath.addState(result);
                    frontier.push(newPath);
                }
            }
        }

//        cout << __func__ << " Error: Could not find a solution." << endl;
        Path<StateType, ActionType> empty;
        return empty;
    }
};

#endif // GRAPHSEARCH_HPP_INCLUDED
