#include "igvcsearchproblem.h"
#include <math.h>

// TODO update to include orientation when calculting CSpace
// TODO what if I jump over an obstacle
bool IGVCSearchProblem::isActionValid(const SearchMove& move, const SearchLocation& start_state)
{
  SearchLocation result = getResult(start_state, move);
  if (std::abs(result.ThetaChange) > MaxThetaChange)
  {
    return false;
  }
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  if (x < 0 || y < 0 || x >= Map->image.size().width || y >= Map->image.size().height)
  {
    return false;
  }
  double min_val;
  double max_val;
  double sep = CSpace / Resolution;

  // TODO how does this function worik
  // implicit cast, TODO check a thing we want to do
  cv::Mat subsection =
      Map->image(cv::Range(max(x - sep, 0.0), min(static_cast<int>(std::round(x + sep)) + 1, Map->image.size().height)),
                 cv::Range(max(y - sep, 0.0), min(static_cast<int>(std::round(y + sep)) + 1, Map->image.size().width)));
  cv::minMaxLoc(subsection, &min_val, &max_val);
  if (max_val > ProbabilityThreshold * 255)
  {
    return false;
  }
  return true;
}

std::list<SearchMove> IGVCSearchProblem::getActions(const SearchLocation& state)
{
  std::list<SearchMove> acts;
  double theta = state.Theta;
  // TODO this should not be needed
  double thetaThreshold = 0.1;
  // TODO is this an issue
  double jump_size = (DistanceToGoal - state.distTo(Goal, Resolution)) / DistanceToGoal;
  if (jump_size > 0.5)
  {
    jump_size = 1 - jump_size;
  }
  int dist = max(std::round(jump_size * MaxJumpSize), 1.0);
  if (abs(theta) < thetaThreshold)
  {
    acts.push_back(SearchMove(dist, dist));
    acts.push_back(SearchMove(dist, 0));
    acts.push_back(SearchMove(dist, -dist));
  }
  else if (abs(theta - M_PI / 4) < thetaThreshold)
  {
    acts.push_back(SearchMove(0, dist));
    acts.push_back(SearchMove(dist, dist));
    acts.push_back(SearchMove(dist, 0));
  }
  else if (abs(theta - M_PI / 2) < thetaThreshold)
  {
    acts.push_back(SearchMove(-dist, dist));
    acts.push_back(SearchMove(0, dist));
    acts.push_back(SearchMove(dist, dist));
  }
  else if (abs(theta - 3 * M_PI / 4) < thetaThreshold)
  {
    acts.push_back(SearchMove(-dist, 0));
    acts.push_back(SearchMove(-dist, dist));
    acts.push_back(SearchMove(0, dist));
  }
  else if (abs(abs(theta) - M_PI) < thetaThreshold)
  {
    acts.push_back(SearchMove(-dist, -dist));
    acts.push_back(SearchMove(-dist, 0));
    acts.push_back(SearchMove(-dist, dist));
  }
  else if (abs(theta + 3 * M_PI / 4) < thetaThreshold)
  {
    acts.push_back(SearchMove(0, -dist));
    acts.push_back(SearchMove(-dist, -dist));
    acts.push_back(SearchMove(-dist, 0));
  }
  else if (abs(theta + M_PI / 2) < thetaThreshold)
  {
    acts.push_back(SearchMove(dist, -dist));
    acts.push_back(SearchMove(0, -dist));
    acts.push_back(SearchMove(-dist, -dist));
  }
  else if (abs(theta + M_PI / 4) < thetaThreshold)
  {
    acts.push_back(SearchMove(dist, 0));
    acts.push_back(SearchMove(dist, -dist));
    acts.push_back(SearchMove(0, -dist));
  }
  else
  {
    // TODO handle error
    // std::cerr << "\n\n\n\n\nfail" << theta << "\n\n\n\n\n" << std::endl;
  }
  acts.remove_if([this, state](SearchMove m) { return !isActionValid(m, state); });
  return acts;
}

SearchLocation IGVCSearchProblem::getResult(const SearchLocation& state, const SearchMove& action)
{
  SearchLocation result;
  result.X = state.X;
  result.Y = state.Y;
  result.X += action.X;
  result.Y += action.Y;
  result.cost = state.cost + getStepCost(state, action);
  if (action.X > 0 && action.Y == 0)
  {
    result.Theta = 0;
  }
  else if (action.X > 0 && action.Y > 0)
  {
    result.Theta = M_PI / 4;
  }
  else if (action.X == 0 && action.Y > 0)
  {
    result.Theta = M_PI / 2;
  }
  else if (action.X < 0 && action.Y > 0)
  {
    result.Theta = 3 * M_PI / 4;
  }
  else if (action.X < 0 && action.Y == 0)
  {
    result.Theta = M_PI;
  }
  else if (action.X < 0 && action.Y < 0)
  {
    result.Theta = -3 * M_PI / 4;
  }
  else if (action.X == 0 && action.Y < 0)
  {
    result.Theta = -M_PI / 2;
  }
  else if (action.X > 0 && action.Y < 0)
  {
    result.Theta = -M_PI / 4;
  }
  else
  {
    // TODO handle case
    // std::cerr << "\n\n\n\n\nfail" << action << "\n\n\n\n\n" << std::endl;
  }
  result.PrevTheta.resize(ThetaChangeWindow);
  double thetaDiff;
  if (std::abs(state.Theta - M_PI) < 0.1)
  {
    thetaDiff = result.Theta > 0 ? result.Theta - M_PI : result.Theta + M_PI;
  }
  else if (std::abs(result.Theta - M_PI) < 0.1)
  {
    thetaDiff = state.Theta > 0 ? M_PI - state.Theta : -M_PI - state.Theta;
  }
  else if (state.Theta < 0 && result.Theta > 0)
  {
    thetaDiff = state.Theta + M_PI - result.Theta;
  }
  else if (state.Theta > 0 && result.Theta < 0)
  {
    thetaDiff = result.Theta + M_PI - state.Theta;
  }
  else
  {
    thetaDiff = result.Theta - state.Theta;
  }
  if (state.PrevTheta.size() < ThetaChangeWindow)
  {
    std::copy(state.PrevTheta.begin(), state.PrevTheta.end(), result.PrevTheta.begin());
    result.ThetaChange = state.ThetaChange + thetaDiff;
    result.PrevTheta.push_back(thetaDiff);
  }
  else
  {
    double theta = state.PrevTheta.front();
    // copy without the first element
    std::copy(++state.PrevTheta.begin(), state.PrevTheta.end(), result.PrevTheta.begin());
    result.ThetaChange = state.ThetaChange - theta + thetaDiff;
    result.PrevTheta.push_back(thetaDiff);
  }

  return result;
}
