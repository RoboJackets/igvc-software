#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include <math.h>
#include <pcl/octree/octree_search.h>
#include <functional>
#include <vector>
#include <opencv2/core/core.hpp>
#include <cv_bridge/cv_bridge.h>
#include "GraphSearch.hpp"
#include "searchlocation.h"
#include "searchmove.h"

class IGVCSearchProblemDiscrete : public SearchProblem<SearchLocation, SearchMove>
{
public:
  cv_bridge::CvImageConstPtr Map;
  SearchLocation Start;
  SearchLocation Goal;
  double Resolution;
  double CSpace;
  double GoalThreshold;
  double ProbabilityThreshold;
  bool PointTurnsEnabled;
  bool ReverseEnabled;
  double DistanceToGoal;

  SearchLocation getStartState()
  {
    return Start;
  }

  std::list<SearchMove> getActions(SearchLocation state, SearchLocation robot_position);

  SearchLocation getResult(SearchLocation state, SearchMove action);

  bool isGoal(SearchLocation state)
  {
    return sqrt(pow(state.X - Goal.X, 2) + pow(state.Y - Goal.Y, 2)) * Resolution < GoalThreshold || state.distTo(Start, Resolution) > 10;
  }

  double getStepCost(SearchLocation location, SearchMove action)
  {
    if(abs(action.X) > 0 && abs(action.Y) > 0) {
      return sqrt(pow(action.X, 2) + pow(action.Y, 2)) * Resolution;
    } else if(action.X != 0) {
      return abs(action.X) * Resolution;
    } else if(action.Y != 0) {
      return abs(action.Y) * Resolution;
    } else {
      return 0;
    }
  }

  double getHeuristicCost(SearchLocation state)
  {
    double theta;
    double x = Goal.X - state.X;
    double y = Goal.Y - state.Y;
    if(x > 0) {
      theta = atan(y/x);
    } else if(y > 0) {
      theta = atan(y/x) + M_PI;
    } else if(y < 0) {
      theta = atan(y / x) - M_PI;
    } else {
      theta = y > 0 ? M_PI / 2 : -M_PI / 2;
    }
    return state.distTo(Goal, Resolution) + (abs(theta - state.Theta) / M_PI) * 10 * Resolution;
  }

  bool isActionValid(SearchMove& move, SearchLocation start_state);

};

#endif  // IGVCSEARCHPROBLEM_H
