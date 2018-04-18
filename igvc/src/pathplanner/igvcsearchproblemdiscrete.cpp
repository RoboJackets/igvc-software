#include "igvcsearchproblemdiscrete.h"
#include <math.h>

bool IGVCSearchProblemDiscrete::isActionValid(SearchMove& move, SearchLocation start_state)
{
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  if(x < 0 || y < 0 || x >= Map->image.size().width || y >= Map->image.size().height) {
    return false;
  }
  double min_val;
  double max_val;
  double sep = CSpace / Resolution;
  cv::Mat subsection = Map->image(cv::Range(max(x - sep, 0.0), min(static_cast<int>(x + sep) + 1, Map->image.size().height)), cv::Range(max(y - sep, 0.0), min(static_cast<int>(y + sep) + 1, Map->image.size().width)));
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
  if(theta == 0) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
  } else if(theta - M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(0, 1));
  } else if(theta - M_PI / 2 < thetaThreshold) {
    acts.push_back(SearchMove(1, 1));
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
  } else if(theta - 3 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(0, 1));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, 0));
  } else if(theta - M_PI < thetaThreshold) {
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(-1, 1));
    acts.push_back(SearchMove(-1, -1));
  } else if(theta - 5 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(-1, 0));
    acts.push_back(SearchMove(0, -1));
  } else if(theta - 3 * M_PI / 2 < thetaThreshold) {
    acts.push_back(SearchMove(-1, -1));
    acts.push_back(SearchMove(0, -1));
    acts.push_back(SearchMove(1, -1));
  } else if(theta - 7 * M_PI / 4 < thetaThreshold) {
    acts.push_back(SearchMove(1, 0));
    acts.push_back(SearchMove(1, -1));
    acts.push_back(SearchMove(0, -1));
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
  if(result.X != 0)  {
    result.Theta = tan(result.Y / result.X);
  } else {
    result.Theta = result.Y == 1 ? M_PI / 4 : 3 * M_PI / 2;
  }
  return result;
}
