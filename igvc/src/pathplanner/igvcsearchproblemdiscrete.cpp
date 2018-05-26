#include "igvcsearchproblemdiscrete.h"
#include <math.h>

bool IGVCSearchProblemDiscrete::isActionValid(SearchMove& move, SearchLocation start_state)
{
  //std::cout << start_state.X + move.X << "," << start_state.Y + move.Y;
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  if(x < 0 || y < 0 || x >= Map->image.size().width || y >= Map->image.size().height) {
    return false;
  }
  double min_val;
  double max_val;
  double sep = CSpace / Resolution;
  cv::Mat subsection = Map->image(cv::Range(max(x - sep, 0.0), min(static_cast<int>(std::round(x + sep)) + 1, Map->image.size().height)),
                                  cv::Range(max(y - sep, 0.0), min(static_cast<int>(std::round(y + sep)) + 1, Map->image.size().width)));
  cv::minMaxLoc(subsection, &min_val, &max_val);
  if(max_val > ProbabilityThreshold * 255) {
    return false;
  }
  return true;
}

std::list<SearchMove> IGVCSearchProblemDiscrete::getActions(SearchLocation state, SearchLocation robot_position)
{
  std::list<SearchMove> acts;
  double theta = state.Theta;
  double thetaThreshold = 0.1;
  double jump_size = (DistanceToGoal - state.distTo(Goal, Resolution)) / DistanceToGoal;
  if(jump_size > 0.5) {
    jump_size = 1 - jump_size;
  }
  int dist = max(std::round(jump_size * MaxJumpSize), 1.0);
  if(abs(theta) < thetaThreshold) {
    acts.push_back(SearchMove(dist, dist));
    acts.push_back(SearchMove(dist, 0));
    acts.push_back(SearchMove(dist, -dist));
  } else if(abs(theta - M_PI / 4) < thetaThreshold) {
    acts.push_back(SearchMove(0, dist));
    acts.push_back(SearchMove(dist, dist));
    acts.push_back(SearchMove(dist, 0));
  } else if(abs(theta - M_PI / 2) < thetaThreshold) {
    acts.push_back(SearchMove(-dist, dist));
    acts.push_back(SearchMove(0, dist));
    acts.push_back(SearchMove(dist, dist));
  } else if(abs(theta - 3 * M_PI / 4) < thetaThreshold) {
    acts.push_back(SearchMove(-dist, 0));
    acts.push_back(SearchMove(-dist, dist));
    acts.push_back(SearchMove(0, dist));
  } else if(abs(abs(theta) - M_PI) < thetaThreshold) {
    acts.push_back(SearchMove(-dist, -dist));
    acts.push_back(SearchMove(-dist, 0));
    acts.push_back(SearchMove(-dist, dist));
  } else if(abs(theta + 3 * M_PI / 4) < thetaThreshold) {
    acts.push_back(SearchMove(0, -dist));
    acts.push_back(SearchMove(-dist, -dist));
    acts.push_back(SearchMove(-dist, 0));
  } else if(abs(theta + M_PI / 2) < thetaThreshold) {
    acts.push_back(SearchMove(dist, -dist));
    acts.push_back(SearchMove(0, -dist));
    acts.push_back(SearchMove(-dist, -dist));
  } else if(abs(theta + M_PI / 4) < thetaThreshold) {
    acts.push_back(SearchMove(dist, 0));
    acts.push_back(SearchMove(dist, -dist));
    acts.push_back(SearchMove(0, -dist));
  } else {
    std::cerr << "\n\n\n\n\nfail" << theta << "\n\n\n\n\n" << std::endl;
  }
  acts.remove_if([this, state](SearchMove m){ return !isActionValid(m, state); });
  return acts;
}

SearchLocation IGVCSearchProblemDiscrete::getResult(SearchLocation state, SearchMove action)
{
  SearchLocation result;
  result.X = state.X;
  result.Y = state.Y;
  result.X += action.X;
  result.Y += action.Y;
  if(action.X > 0 && action.Y == 0) {
    result.Theta = 0;
  } else if(action.X > 0 && action.Y > 0) {
    result.Theta = M_PI/4;
  } else if(action.X == 0 && action.Y > 0) {
    result.Theta = M_PI/2;
  } else if(action.X < 0 && action.Y > 0) {
    result.Theta = 3 * M_PI/4;
  } else if(action.X < 0 && action.Y == 0) {
    result.Theta = M_PI;
  } else if(action.X < 0 && action.Y < 0) {
    result.Theta = -3*M_PI/4;
  } else if(action.X == 0 && action.Y < 0) {
    result.Theta = -M_PI/2;
  } else if(action.X > 0 && action.Y < 0) {
    result.Theta = -M_PI/4;
  } else {
    std::cerr << "\n\n\n\n\nfail" << action << "\n\n\n\n\n" << std::endl;
  }
  return result;
}
