#ifndef GRAPHSEARCH_HPP_INCLUDED
#define GRAPHSEARCH_HPP_INCLUDED

#include <algorithm>
#include <iostream>
#include <list>
#include <queue>
#include <set>
#include <stack>

#include "SearchProblem.hpp"

using namespace std;

template <class StateType, class ActionType>
class PathComparator
{
private:
  SearchProblem<StateType, ActionType> *_problem;

public:
  PathComparator(SearchProblem<StateType, ActionType> *problem)
  {
    _problem = problem;
  }

  bool operator()(Path<StateType, ActionType> &p1, Path<StateType, ActionType> &p2)
  {
    double c1 = _problem->getPathCost(&p1);
    c1 += _problem->getHeuristicCost(p1.getLastState());

    double c2 = _problem->getPathCost(&p2);
    c2 += _problem->getHeuristicCost(p2.getLastState());

    return (c1 > c2);
  }
};

template <class StateType, class ActionType, template <typename...> class FrontierType>
struct FrontierTraits
{
  using frontier_type = FrontierType<Path<StateType, ActionType>>;
  using value_type = typename frontier_type::value_type;

  static auto PopNextElement(frontier_type &frontier) -> value_type
  {
    auto next = frontier.top();
    frontier.pop();
    return next;
  }
};

template <class StateType, class ActionType>
struct FrontierTraits<StateType, ActionType, std::queue>
{
  using frontier_type = std::queue<Path<StateType, ActionType>>;
  using value_type = typename frontier_type::value_type;

  static auto PopNextElement(frontier_type &frontier) -> value_type
  {
    auto next = frontier.front();
    frontier.pop();
    return next;
  }
};

class GraphSearch
{
public:
  // Runs a generic search that DFS and BSF can call.
  // Getting this to work with the current implementation of A* is possible, but difficult.
  template <class StateType, class ActionType, template <typename...> class FrontierType>
  static auto GenericSearch(SearchProblem<StateType, ActionType> &problem) -> Path<StateType, ActionType>
  {
    using frontier_traits = FrontierTraits<StateType, ActionType, FrontierType>;
    using frontier_type = typename frontier_traits::frontier_type;

    set<StateType> expanded;
    frontier_type frontier;

    {
      Path<StateType, ActionType> p;
      p.addState(problem.getStartState());
      frontier.push(p);
    }

    while (!frontier.empty())
    {
      Path<StateType, ActionType> path = frontier_traits::PopNextElement(frontier);
      auto const lastState = path.getLastState();

      // expanded does not contain path's last state
      if (expanded.find(lastState) == expanded.cend())
      {
        expanded.insert(lastState);

        if (problem.isGoal(lastState))
        {
          return path;
        }

        auto legalActions = problem.getActions(lastState);

        for (auto it = legalActions.cbegin(); it != legalActions.cend(); ++it)
        {
          ActionType action = (*it);
          StateType result = problem.getResult(lastState, action);

          Path<StateType, ActionType> newPath(path);
          newPath.addAction(action);
          newPath.addState(result);
          frontier.push(newPath);
        }
      }
    }

    cerr << __func__ << " Error: Could not find a solution." << endl;
    Path<StateType, ActionType> empty;
    return empty;
  }

  /** Runs Depth-First graph search on the given search problem */
  template <class StateType, class ActionType>
  static auto DFS(SearchProblem<StateType, ActionType> &problem) -> Path<StateType, ActionType>
  {
    return GenericSearch<StateType, ActionType, std::stack>(problem);
  }

  /** Runs Breadth-First graph on the given search problem */
  template <class StateType, class ActionType>
  static auto BFS(SearchProblem<StateType, ActionType> &problem) -> Path<StateType, ActionType>
  {
    return GenericSearch<StateType, ActionType, std::queue>(problem);
  }

  /** Runs A* graph search on the given search problem */
  template <class StateType, class ActionType>
  static Path<StateType, ActionType> AStar(SearchProblem<StateType, ActionType> &problem,
                                           void (*expandedCallback)(const StateType &))
  {
    set<StateType> expanded;
    priority_queue<Path<StateType, ActionType>, vector<Path<StateType, ActionType>>,
                   PathComparator<StateType, ActionType>>
        frontier((PathComparator<StateType, ActionType>(&problem)));

    {
      Path<StateType, ActionType> p;
      p.addState(problem.getStartState());
      frontier.push(p);
    }

    auto iteration = 0;
    while (!frontier.empty() && iteration < 4000)
    {
      Path<StateType, ActionType> path = frontier.top();
      frontier.pop();

      auto last = path.getLastState();
      if (expanded.insert(last).second)  // expanded does not contain path's last state
      {
        if (problem.isGoal(last))
        {
          return path;
        }
        list<ActionType> legalActions = problem.getActions(last, problem.getStartState());

        for (typename list<ActionType>::iterator it = legalActions.begin(); it != legalActions.end(); it++)
        {
          ActionType action = (*it);
          StateType result = problem.getResult(last, action);
          Path<StateType, ActionType> newPath(path);
          newPath.addAction(action);
          newPath.addState(result);
          frontier.push(newPath);
        }
      }
      iteration++;
      expandedCallback(last);
    }

    cout << __func__ << " Error: Could not find a solution after " << iteration << " iterations" << endl;
    Path<StateType, ActionType> empty;
    return empty;
  }
};

#endif  // GRAPHSEARCH_HPP_INCLUDED
