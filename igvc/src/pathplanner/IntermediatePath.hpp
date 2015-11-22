#include <iostream>
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include "SearchProblem.hpp"


using namespace std;

class IntermediatePath {
private :
    static Path<class SearchLocation, class SearchMove> oldPath;
    static int counter;
public :
    template <class StateType, class ActionType>
            static Path<StateType, ActionType> selectOptimalPath(Path<StateType, 		 ActionType> newPath, SearchProblem<StateType, ActionType> &_problem)
            {
                    if(counter > 0 && counter < 10) {
                        counter++;
						
                        double oldPathCost = _problem.getPathCost(&oldPath);
                        oldPathCost += _problem.getHeuristicCost(oldPath.getLastState());

                        double newPathCost = _problem.getPathCost(&newPath);
                        newPathCost += _problem.getHeuristicCost(newPath.getLastState());
                        if (newPathCost < oldPathCost) {
                            return newPath;
                        } else {
                            return oldPath;
                        }
                    } else {
                        counter  = 1;
                        oldPath = newPath;
			return newPath;
                    }

            }
};
int IntermediatePath::counter = 0;
Path<class SearchLocation, class SearchMove> IntermediatePath::oldPath;
