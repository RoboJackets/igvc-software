#include "igvcsearchproblemdiscrete.h"
#include <math.h>

bool IGVCSearchProblemDiscrete::isActionValid(SearchMove& move, SearchLocation start_state)
{
  double x = start_state.X + move.X;
  double y = start_state.Y + move.Y;
  if((x < 0 || y < 0) || (x >= Map.size().width && y >= Map.size().height)) {
    return false;
  }
  return true;
}

std::list<SearchMove> IGVCSearchProblemDiscrete::getActions(SearchLocation state, SearchLocation robot_position)
{
  std::list<SearchMove> acts;
  double x = state.X;
  double y = state.Y;
  double theta = state.Theta;
  if(theta == 0) {
    acts.push_back(SearchMove(x + 1, y + 1));
    acts.push_back(SearchMove(x + 1, y));
    acts.push_back(SearchMove(x + 1, y - 1));
  } else if(theta == M_PI / 4) {
    acts.push_back(SearchMove(x + 1, y + 1));
    acts.push_back(SearchMove(x + 1, y));
    acts.push_back(SearchMove(x, y + 1));
  } else if(theta == M_PI / 2) {
    acts.push_back(SearchMove(x + 1, y + 1));
    acts.push_back(SearchMove(x, y + 1));
    acts.push_back(SearchMove(x - 1, y + 1));
  } else if(theta == 3 * M_PI / 4) {
    acts.push_back(SearchMove(x, y + 1));
    acts.push_back(SearchMove(x - 1, y + 1));
    acts.push_back(SearchMove(x - 1, y));
  } else if(theta == M_PI) {
    acts.push_back(SearchMove(x - 1, y));
    acts.push_back(SearchMove(x - 1, y + 1));
    acts.push_back(SearchMove(x - 1, y - 1));
  } else if(theta == 5 * M_PI / 4) {
    acts.push_back(SearchMove(x - 1, y - 1));
    acts.push_back(SearchMove(x - 1, y));
    acts.push_back(SearchMove(x, y - 1));
  } else if(theta == 3 * M_PI / 2) {
    acts.push_back(SearchMove(x - 1, y - 1));
    acts.push_back(SearchMove(x, y - 1));
    acts.push_back(SearchMove(x + 1, y - 1));
  } else if(theta == 7 * M_PI / 4) {
    acts.push_back(SearchMove(x + 1, y));
    acts.push_back(SearchMove(x + 1, y - 1));
    acts.push_back(SearchMove(x, y - 1));
  }
  return acts;
}

SearchLocation IGVCSearchProblemDiscrete::getResult(SearchLocation state, SearchMove action)
{
  SearchLocation result;
  result.X += action.X;
  result.Y += action.Y;
  if(result.X != 0)  {
    result.Theta = tan(result.Y / result.X);
  } else {
    result.Theta = result.Y == 1 ? M_PI / 4 : 3 * M_PI / 2;
  }
  return result;
}
