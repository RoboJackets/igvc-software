#ifndef IGVCSEARCHPROBLEM_H
#define IGVCSEARCHPROBLEM_H

#include <cv_bridge/cv_bridge.h>
#include <math.h>
#include <pcl/octree/octree_search.h>
#include <functional>
#include <opencv2/core/core.hpp>
#include <vector>
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
  double MaxJumpSize;
  double ThetaFilter;
  double MaxThetaChange;
  double ThetaChangeWindow;

  SearchLocation getStartState()
  {
    return Start;
  }

  std::list<SearchMove> getActions(SearchLocation state, SearchLocation robot_position);

  SearchLocation getResult(SearchLocation state, SearchMove action);

  bool isGoal(SearchLocation state)
  {
    return state.distTo(Goal, Resolution) < GoalThreshold || state.distTo(Start, Resolution) > 7;
  }

  double getStepCost(SearchLocation location, SearchMove action)
  {
    SearchLocation result;
    result.X = location.X + action.X;
    result.Y = location.Y + action.Y;
    return result.distTo(location, Resolution);
  }

  double getHeuristicCost(SearchLocation state)
  {
    double theta;
    double x = Goal.X - state.X;
    double y = Goal.Y - state.Y;
    if (x > 0)
    {
      theta = atan(y / x);
    }
    else if (y > 0)
    {
      theta = atan(y / x) + M_PI;
    }
    else if (y < 0)
    {
      theta = atan(y / x) - M_PI;
    }
    else
    {
      theta = y > 0 ? M_PI / 2 : -M_PI / 2;
    }

    return state.distTo(Goal, Resolution) + (abs(theta - state.Theta) / M_PI) * ThetaFilter * Resolution;
  }

  bool isActionValid(SearchMove& move, SearchLocation start_state);
};

#endif  // IGVCSEARCHPROBLEM_H
