#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include <math.h>
#include <pcl/octree/octree_search.h>
#include <functional>
#include <vector>
#include <opencv2/core/core.hpp>
#include "GraphSearch.hpp"
#include "searchlocation.h"
#include "searchmove.h"

class IGVCSearchProblemDiscrete : public SearchProblem<SearchLocation, SearchMove>
{
public:
  cv::Mat Map;
  SearchLocation Start;
  SearchLocation Goal;
  double Threshold;
  double GoalThreshold;
  bool PointTurnsEnabled;
  bool ReverseEnabled;

  SearchLocation getStartState()
  {
    return Start;
  }

  std::list<SearchMove> getActions(SearchLocation state, SearchLocation robot_position);

  SearchLocation getResult(SearchLocation state, SearchMove action);

  bool isGoal(SearchLocation state)
  {
    return state == Goal;
  }

  double getStepCost(SearchLocation location, SearchMove action)
  {
    if(abs(action.X) == 1 && abs(action.Y) == 1) {
      return sqrt(2);
    } else if(action.X != 0 || action.Y != 0) {
      return 1;
    } else {
      return 0;
    }
  }

  double getHeuristicCost(SearchLocation state)
  {
    return state.distTo(Goal);
  }

  bool isActionValid(SearchMove& move, SearchLocation start_state);
};

#endif  // IGVCSEARCHPROBLEM_H
